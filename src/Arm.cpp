#include <florid/usb/Arm.hpp>

#include <ProtocolStack.hpp>
#include <RPL/Serializer.hpp>
#include <astrial.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <optional>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;

namespace florid::usb {

// ──────────────────────────────────────────────
//  ProtocolStack instantiation (host side)
// ──────────────────────────────────────────────

using HostStack = ProtocolStack<std::chrono::steady_clock,
    AckPacket,
    SetMitLimitRequestPacket, SetMitLimitResponsePacket,
    SetMotorStateRequestPacket, SetMotorStateResponsePacket,
    SetMotorControlModeRequestPacket, SetMotorControlModeResponsePacket,
    SetZeroRequestPacket, SetZeroResponsePacket,
    ClearErrorRequestPacket, ClearErrorResponsePacket,
    HomeAllRequestPacket, HomeAllResponsePacket,
    RunCalibrationRequestPacket, RunCalibrationResponsePacket,
    ClearFaultsRequestPacket, ClearFaultsResponsePacket,
    SetSafetyRequestPacket, SetSafetyResponsePacket,
    SdkClientConnectedRequestPacket, SdkClientConnectedResponsePacket,
    SdkClientDisconnectedRequestPacket, SdkClientDisconnectedResponsePacket,
    HomeDoneRequestPacket, HomeDoneResponsePacket,
    UsbSessionStartRequestPacket, UsbSessionStartResponsePacket,
    UsbSessionStopRequestPacket, UsbSessionStopResponsePacket,
    GetMotorFeedbackRequestPacket, GetMotorFeedbackResponsePacket,
    JointCommandPacket, EmergencyStopPacket, GripperCommandPacket,
    JointPosVelCommandPacket, JointVelocityCommandPacket, JointHybridCommandPacket,
    ArmStatus, MotorFeedbackArray, GripperStatus>;

// ──────────────────────────────────────────────
//  PIMPL implementation
// ──────────────────────────────────────────────

class Arm::Impl
{
public:
    explicit Impl(Config cfg) : m_cfg(std::move(cfg))
    {
    }

    ~Impl()
    {
        disconnect();
    }

    bool connect()
    {
        if (m_running.load(std::memory_order_acquire))
            return true;

        // Recreate ProtocolStack for a clean session
        m_stack = std::make_unique<HostStack>(make_cfg());
        register_handlers();

        auto result = Serial::builder()
            .buad_rate(m_cfg.baud_rate)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .open(m_cfg.device);
        if (!result) return false;
        m_serial = std::move(result.value());

        m_serial->on_data([this](std::span<const uint8_t> data) {
            m_stack->isr_feed(data.data(), data.size());
        });

        m_stack->set_send_frame([this](const uint8_t* frame, size_t len) -> bool {
            if (!m_serial.has_value()) return false;
            auto ec = m_serial->write(std::span(frame, len));
            return ec.has_value();
        });

        m_running.store(true, std::memory_order_release);
        m_worker = std::thread(&Impl::worker_loop, this);

        return true;
    }

    void disconnect()
    {
        m_running.store(false, std::memory_order_release);
        m_op_cv.notify_all();

        if (m_worker.joinable())
            m_worker.join();

        m_serial.reset();
    }

    bool isConnected() const
    {
        if (!m_serial.has_value()) return false;

        std::lock_guard lk(m_status_mutex);
        auto elapsed = std::chrono::steady_clock::now() - m_last_status_time;
        return elapsed < 1s;
    }

    // ── 会话 ──

    bool startSession(std::chrono::milliseconds timeout)
    {
        std::cerr << "[Arm] startSession: sending UsbSessionStart...\n";
        auto p = std::make_shared<std::promise<bool>>();
        auto f = p->get_future();
        post([this, p]() {
            m_stack->send<UsbSessionStartRequestPacket, UsbSessionStartResponsePacket>(
                UsbSessionStartCommand{.dummy = 0},
                [p](const uint8_t*, size_t) {
                    std::cerr << "[Arm] startSession: response OK\n";
                    p->set_value(true);
                },
                [p](SessionError e) {
                    std::cerr << "[Arm] startSession: error " << static_cast<int>(e) << "\n";
                    p->set_value(false);
                },
                m_cfg.session_timeout,
                static_cast<size_t>(m_cfg.max_retries));
        });
        auto status = f.wait_for(timeout);
        if (status != std::future_status::ready) {
            std::cerr << "[Arm] startSession: timeout after " << timeout.count() << "ms\n";
            return false;
        }
        return f.get();
    }

    bool stopSession(std::chrono::milliseconds timeout)
    {
        auto p = std::make_shared<std::promise<bool>>();
        auto f = p->get_future();
        post([this, p]() {
            m_stack->send<UsbSessionStopRequestPacket, UsbSessionStopResponsePacket>(
                UsbSessionStopCommand{.dummy = 0},
                [p](const uint8_t*, size_t) { p->set_value(true); },
                [p](SessionError) { p->set_value(false); },
                m_cfg.session_timeout,
                static_cast<size_t>(m_cfg.max_retries));
        });
        return f.wait_for(timeout) == std::future_status::ready && f.get();
    }

    // ── MIT 控制 ──

    void sendMitCommand(const JointCommandPacket& cmd)
    {
        JointCommandPacket jc = cmd;
        stamp(jc);
        std::lock_guard lk(m_jc_mutex);
        m_next_jc = jc;
        m_pending_control = PendingControl::Mit;
        m_op_cv.notify_one();
    }

    void sendMitCommand(const float q[6], const float dq[6], const float tau[6],
                          const float kp[6], const float kd[6],
                          uint8_t control_mode)
    {
        JointCommandPacket jc{};
        for (size_t i = 0; i < 6; ++i)
        {
            jc.q[i] = q[i];
            jc.dq[i] = dq[i];
            jc.tau[i] = tau[i];
            jc.kp[i] = kp[i];
            jc.kd[i] = kd[i];
        }
        jc.control_mode = control_mode;
        stamp(jc);
        std::lock_guard lk(m_jc_mutex);
        m_next_jc = jc;
        m_pending_control = PendingControl::Mit;
        m_op_cv.notify_one();
    }

    bool setMotorControlMode(uint8_t joint_id,
                             MotorControlMode mode,
                             std::chrono::milliseconds timeout)
    {
        auto p = std::make_shared<std::promise<bool>>();
        auto f = p->get_future();
        post([this, p, joint_id, mode]() {
            m_stack->send<SetMotorControlModeRequestPacket, SetMotorControlModeResponsePacket>(
                SetMotorControlModeCommand{.joint_id = joint_id, .mode = mode},
                [p](const uint8_t* bytes, size_t size) {
                    bool ok = false;
                    if (size >= sizeof(SetMotorControlModeResponsePacket)) {
                        SetMotorControlModeResponsePacket rsp{};
                        std::memcpy(&rsp, bytes, sizeof(rsp));
                        ok = rsp.payload.status == SetMotorControlModeStatus::Ok;
                    }
                    p->set_value(ok);
                },
                [p](SessionError) { p->set_value(false); },
                m_cfg.session_timeout,
                static_cast<size_t>(m_cfg.max_retries));
        });
        return f.wait_for(timeout) == std::future_status::ready && f.get();
    }

    void sendPosVelCommand(const float q[6], const float dq[6], uint8_t enabled_mask)
    {
        JointPosVelCommandPacket pkt{};
        for (size_t i = 0; i < 6; ++i)
        {
            pkt.q[i] = q[i];
            pkt.dq[i] = dq[i];
        }
        pkt.enabled_mask = enabled_mask & 0x3fU;
        pkt.seq = m_seq++;
        std::lock_guard lk(m_jc_mutex);
        m_next_posvel = pkt;
        m_pending_control = PendingControl::PosVel;
        m_op_cv.notify_one();
    }

    void sendVelocityCommand(const float dq[6], uint8_t enabled_mask)
    {
        JointVelocityCommandPacket pkt{};
        for (size_t i = 0; i < 6; ++i)
            pkt.dq[i] = dq[i];
        pkt.enabled_mask = enabled_mask & 0x3fU;
        pkt.seq = m_seq++;
        std::lock_guard lk(m_jc_mutex);
        m_next_velocity = pkt;
        m_pending_control = PendingControl::Velocity;
        m_op_cv.notify_one();
    }

    void sendHybridCommand(const float q[6], const float dq_limit[6],
                           const float current_limit_norm[6], uint8_t enabled_mask)
    {
        JointHybridCommandPacket pkt{};
        for (size_t i = 0; i < 6; ++i)
        {
            pkt.q[i] = q[i];
            pkt.dq_limit[i] = std::clamp(dq_limit[i], 0.0f, 100.0f);
            pkt.current_limit_norm[i] = std::clamp(current_limit_norm[i], 0.0f, 1.0f);
        }
        pkt.enabled_mask = enabled_mask & 0x3fU;
        pkt.seq = m_seq++;
        std::lock_guard lk(m_jc_mutex);
        m_next_hybrid = pkt;
        m_pending_control = PendingControl::Hybrid;
        m_op_cv.notify_one();
    }

private:
    static constexpr auto kTrajectoryGap = std::chrono::milliseconds{200};

    void stamp(JointCommandPacket& jc)
    {
        const auto now = std::chrono::steady_clock::now();
        const bool new_trajectory =
            m_last_send_time.time_since_epoch().count() == 0 ||
            (now - m_last_send_time) > kTrajectoryGap;

        if (new_trajectory)
        {
            jc.dt_us = 1000;
        }
        else
        {
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                now - m_last_send_time);
            jc.dt_us = static_cast<uint32_t>(std::clamp<int64_t>(
                elapsed.count(), 1000, 50000));
        }
        m_last_send_time = now;
        jc.seq = m_seq++;
    }

    void stamp_gripper(GripperCommandPacket& gc)
    {
        const auto now = std::chrono::steady_clock::now();
        const bool new_trajectory =
            m_last_gripper_time.time_since_epoch().count() == 0 ||
            (now - m_last_gripper_time) > kTrajectoryGap;

        if (new_trajectory)
            gc.dt_us = 1000;
        else
        {
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - m_last_gripper_time);
            gc.dt_us = static_cast<uint32_t>(std::clamp<int64_t>(elapsed.count(), 1000, 50000));
        }
        m_last_gripper_time = now;
        gc.seq = m_gripper_seq++;
    }

public:
    // ── 安全 ──

    void emergencyStop()
    {
        post([this]() {
            fire_and_forget(EmergencyStopPacket{.dummy = 0});
        });
    }

    // ── 状态 ──

    ArmStatus getArmStatus() const
    {
        std::lock_guard lk(m_status_mutex);
        return m_arm_status;
    }

    GripperStatus getGripperStatus() const
    {
        std::lock_guard lk(m_status_mutex);
        return m_gripper_status;
    }

    MotorFeedbackArray getMotorFeedback(std::chrono::milliseconds timeout)
    {
        auto p = std::make_shared<std::promise<MotorFeedbackArray>>();
        auto f = p->get_future();
        post([this, p]() {
            m_stack->send<GetMotorFeedbackRequestPacket, GetMotorFeedbackResponsePacket>(
                GetMotorFeedbackCommand{.dummy = 0},
                [p](const uint8_t* bytes, size_t size) {
                    if (size >= sizeof(GetMotorFeedbackResponsePacket)) {
                        GetMotorFeedbackResponsePacket rsp{};
                        std::memcpy(&rsp, bytes, sizeof(rsp));
                        p->set_value(rsp.payload.motors);
                    } else { p->set_value(MotorFeedbackArray{}); }
                },
                [p](SessionError) { p->set_value(MotorFeedbackArray{}); },
                m_cfg.session_timeout,
                static_cast<size_t>(m_cfg.max_retries));
        });
        if (f.wait_for(timeout) == std::future_status::ready)
            return f.get();
        return MotorFeedbackArray{};
    }

    // ── 夹爪控制 ──

    void sendGripperCommand(float q, float dq, float tau,
                            float kp, float kd, uint8_t control_mode)
    {
        GripperCommandPacket jc{};
        jc.q = q;
        jc.dq = dq;
        jc.tau = tau;
        jc.kp = kp;
        jc.kd = kd;
        jc.control_mode = control_mode;
        stamp_gripper(jc);
        post([this, jc]() { fire_and_forget(jc); });
    }

    // ── 杂项 ──

    bool homeAll(std::chrono::milliseconds timeout)
    {
        auto p = std::make_shared<std::promise<bool>>();
        auto f = p->get_future();
        post([this, p]() {
            m_stack->send<HomeAllRequestPacket, HomeAllResponsePacket>(
                HomeAllCommand{.dummy = 0},
                [p](const uint8_t*, size_t) { p->set_value(true); },
                [p](SessionError) { p->set_value(false); },
                m_cfg.session_timeout,
                static_cast<size_t>(m_cfg.max_retries));
        });
        return f.wait_for(timeout) == std::future_status::ready && f.get();
    }

    bool clearFaults(std::chrono::milliseconds timeout)
    {
        auto p = std::make_shared<std::promise<bool>>();
        auto f = p->get_future();
        post([this, p]() {
            m_stack->send<ClearFaultsRequestPacket, ClearFaultsResponsePacket>(
                ClearFaultsCommand{.dummy = 0},
                [p](const uint8_t*, size_t) { p->set_value(true); },
                [p](SessionError) { p->set_value(false); },
                m_cfg.session_timeout,
                static_cast<size_t>(m_cfg.max_retries));
        });
        return f.wait_for(timeout) == std::future_status::ready && f.get();
    }

    bool setZero(uint8_t joint_id, std::chrono::milliseconds timeout)
    {
        auto p = std::make_shared<std::promise<bool>>();
        auto f = p->get_future();
        post([this, p, joint_id]() {
            m_stack->send<SetZeroRequestPacket, SetZeroResponsePacket>(
                SetZeroCommand{.joint_id = joint_id},
                [p](const uint8_t* bytes, size_t size) {
                    bool ok = false;
                    if (size >= sizeof(SetZeroResponsePacket)) {
                        SetZeroResponsePacket rsp{};
                        std::memcpy(&rsp, bytes, sizeof(rsp));
                        ok = rsp.payload.status == SetZeroStatus::Ok;
                    }
                    p->set_value(ok);
                },
                [p](SessionError) { p->set_value(false); },
                m_cfg.session_timeout,
                static_cast<size_t>(m_cfg.max_retries));
        });
        return f.wait_for(timeout) == std::future_status::ready && f.get();
    }

private:
    HostStack::Config make_cfg() const
    {
        HostStack::Config cfg{};
        cfg.session.window_size = 1;
        cfg.session.timeout = m_cfg.session_timeout;
        cfg.session.max_retries = static_cast<size_t>(m_cfg.max_retries);
        cfg.session.device_cache_capacity = 64;
        cfg.rx_ring_capacity = 4096;
        cfg.tx_queue_capacity = 16;
        return cfg;
    }

    void register_handlers()
    {
        m_stack->register_handler(
            RPL::Meta::PacketTraits<ArmStatus>::cmd,
            [this](uint16_t, TransactionId, const uint8_t* data, size_t size) {
                if (size >= sizeof(ArmStatus))
                {
                    std::lock_guard lk(m_status_mutex);
                    std::memcpy(&m_arm_status, data, sizeof(ArmStatus));
                    m_gripper_status = m_arm_status.gripper;
                    m_last_status_time = std::chrono::steady_clock::now();
                    ++m_arm_status_count;
                }
            });

        m_stack->register_handler(
            RPL::Meta::PacketTraits<MotorFeedbackArray>::cmd,
            [this](uint16_t, TransactionId, const uint8_t* data, size_t size) {
                if (size >= sizeof(MotorFeedbackArray))
                {
                    std::lock_guard lk(m_status_mutex);
                    std::memcpy(&m_motor_fb, data, sizeof(MotorFeedbackArray));
                }
            });

        m_stack->register_handler(
            RPL::Meta::PacketTraits<GripperStatus>::cmd,
            [this](uint16_t, TransactionId, const uint8_t* data, size_t size) {
                if (size >= sizeof(GripperStatus))
                {
                    std::lock_guard lk(m_status_mutex);
                    std::memcpy(&m_gripper_status, data, sizeof(GripperStatus));
                }
            });

        m_stack->register_handler(
            RPL::Meta::PacketTraits<HomeDoneRequestPacket>::cmd,
            [](uint16_t, TransactionId, const uint8_t*, size_t) {
                // HomeDone notification from firmware — informational only on host side
            });
    }

    void post(std::function<void()> op)
    {
        {
            std::lock_guard lk(m_op_mutex);
            m_op_queue.push_back(std::move(op));
        }
        m_op_cv.notify_one();
    }

    template <typename Pkt>
    bool fire_and_forget(const Pkt& pkt)
    {
        RPL::Serializer<Pkt> ser;
        uint8_t buf[512];
        auto result = ser.serialize(buf, sizeof(buf), pkt);
        if (!result.has_value()) return false;
        if (!m_serial.has_value()) return false;
        return m_serial->write(std::span(buf, *result)).has_value();
    }

    void worker_loop()
    {
        m_stack->bind_to_current_thread();

        while (m_running.load(std::memory_order_acquire))
        {
            // 1. Process queued operations
            for (;;)
            {
                std::function<void()> op;
                {
                    std::lock_guard lk(m_op_mutex);
                    if (m_op_queue.empty()) break;
                    op = std::move(m_op_queue.front());
                    m_op_queue.pop_front();
                }
                op();
            }

            // 2. Fire-and-forget: send latest JointCommand if pending
            {
                std::lock_guard lk(m_jc_mutex);
                switch (m_pending_control)
                {
                case PendingControl::None:
                    break;
                case PendingControl::Mit:
                    fire_and_forget(m_next_jc);
                    break;
                case PendingControl::PosVel:
                    fire_and_forget(m_next_posvel);
                    break;
                case PendingControl::Velocity:
                    fire_and_forget(m_next_velocity);
                    break;
                case PendingControl::Hybrid:
                    fire_and_forget(m_next_hybrid);
                    break;
                }
                m_pending_control = PendingControl::None;
            }

            // 3. Drive ProtocolStack (RX drain → session outbound → retries → TX drain)
            m_stack->run_one_cycle();

            // 4. Wait for work or short timeout
            {
                std::unique_lock lk(m_op_mutex);
                m_op_cv.wait_for(lk, 1ms);
            }
        }
    }

    Config m_cfg;
    std::unique_ptr<HostStack> m_stack;
    std::optional<Serial> m_serial;
    std::thread m_worker;
    std::atomic<bool> m_running{false};

    // Operation queue
    std::mutex m_op_mutex;
    std::deque<std::function<void()>> m_op_queue;
    std::condition_variable m_op_cv;

    enum class PendingControl : uint8_t
    {
        None,
        Mit,
        PosVel,
        Velocity,
        Hybrid,
    };

    // Joint control staging (latest wins)
    std::mutex m_jc_mutex;
    JointCommandPacket m_next_jc{};
    JointPosVelCommandPacket m_next_posvel{};
    JointVelocityCommandPacket m_next_velocity{};
    JointHybridCommandPacket m_next_hybrid{};
    PendingControl m_pending_control{PendingControl::None};

    // Telemetry cache
    mutable std::mutex m_status_mutex;
    ArmStatus m_arm_status{};
    MotorFeedbackArray m_motor_fb{};
    std::chrono::steady_clock::time_point m_last_status_time{};
    unsigned m_arm_status_count = 0;

    uint16_t m_seq = 0;
    std::chrono::steady_clock::time_point m_last_send_time{};

    GripperStatus m_gripper_status{};
    uint16_t m_gripper_seq = 0;
    std::chrono::steady_clock::time_point m_last_gripper_time{};

};

// ──────────────────────────────────────────────
//  Arm public API (delegates to Impl)
// ──────────────────────────────────────────────

Arm::Arm(Config cfg) : m_impl(std::make_unique<Impl>(std::move(cfg))) {}

Arm::~Arm() = default;

Arm::Arm(Arm&&) noexcept = default;
Arm& Arm::operator=(Arm&&) noexcept = default;

bool Arm::connect() { return m_impl->connect(); }
void Arm::disconnect() { m_impl->disconnect(); }
bool Arm::isConnected() const { return m_impl->isConnected(); }

bool Arm::startSession(std::chrono::milliseconds timeout) { return m_impl->startSession(timeout); }
bool Arm::stopSession(std::chrono::milliseconds timeout) { return m_impl->stopSession(timeout); }

void Arm::sendMitCommand(const JointCommandPacket& cmd) { m_impl->sendMitCommand(cmd); }
void Arm::sendMitCommand(const float q[6], const float dq[6], const float tau[6],
                           const float kp[6], const float kd[6],
                           uint8_t control_mode)
{
    m_impl->sendMitCommand(q, dq, tau, kp, kd, control_mode);
}

bool Arm::setMotorControlMode(uint8_t joint_id,
                              MotorControlMode mode,
                              std::chrono::milliseconds timeout)
{
    return m_impl->setMotorControlMode(joint_id, mode, timeout);
}

void Arm::sendPosVelCommand(const float q[6], const float dq[6], uint8_t enabled_mask)
{
    m_impl->sendPosVelCommand(q, dq, enabled_mask);
}

void Arm::sendVelocityCommand(const float dq[6], uint8_t enabled_mask)
{
    m_impl->sendVelocityCommand(dq, enabled_mask);
}

void Arm::sendHybridCommand(const float q[6], const float dq_limit[6],
                            const float current_limit_norm[6], uint8_t enabled_mask)
{
    m_impl->sendHybridCommand(q, dq_limit, current_limit_norm, enabled_mask);
}

void Arm::sendGripperCommand(float q, float dq, float tau,
                              float kp, float kd, uint8_t control_mode)
{
    m_impl->sendGripperCommand(q, dq, tau, kp, kd, control_mode);
}

void Arm::emergencyStop() { m_impl->emergencyStop(); }

ArmStatus Arm::getArmStatus() const { return m_impl->getArmStatus(); }
GripperStatus Arm::getGripperStatus() const { return m_impl->getGripperStatus(); }
MotorFeedbackArray Arm::getMotorFeedback(std::chrono::milliseconds timeout) { return m_impl->getMotorFeedback(timeout); }

bool Arm::homeAll(std::chrono::milliseconds timeout) { return m_impl->homeAll(timeout); }
bool Arm::clearFaults(std::chrono::milliseconds timeout) { return m_impl->clearFaults(timeout); }
bool Arm::setZero(uint8_t joint_id, std::chrono::milliseconds timeout) { return m_impl->setZero(joint_id, timeout); }

} // namespace florid::usb
