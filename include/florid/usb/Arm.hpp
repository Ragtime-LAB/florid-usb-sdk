#ifndef FLORID_USB_SDK_ARM_HPP
#define FLORID_USB_SDK_ARM_HPP

#include <ArmCommand.hpp>
#include <ArmStatus.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace florid::usb {

class Arm
{
public:
    struct Config
    {
        Config() = default;
        std::string device = "/dev/ttyACM0";
        int baud_rate = 115200;
        std::chrono::milliseconds session_timeout{500};
        int max_retries = 3;
    };

    explicit Arm(Config cfg);
    ~Arm();

    Arm(const Arm&) = delete;
    Arm& operator=(const Arm&) = delete;
    Arm(Arm&&) noexcept;
    Arm& operator=(Arm&&) noexcept;

    // ── 连接管理 ──
    bool connect();
    void disconnect();
    bool isConnected() const;

    // ── 会话管理（阻塞等待 UsbSessionStart / UsbSessionStop 响应） ──
    bool startSession(std::chrono::milliseconds timeout = std::chrono::milliseconds{500});
    bool stopSession(std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    // ── MIT 直接控制（fire-and-forget，非阻塞） ──
    // Firmware provides built-in gravity compensation via CasADi.
    // The `tau` parameter is an additional feed-forward torque on top of gravity.
    // dt_us is auto-computed from wall-clock elapsed since last send.
    // Gap > 200ms is treated as a new trajectory (dt_us = 1000 µs).
    // control_mode: bit[1:0]=type(0=hold,1=MIT,3=torque), bit[2]=gravity_enable.
    //   e.g. 0x05 = MIT+gravity, 0x07 = torque+gravity.
    void sendMitCommand(const JointCommandPacket& cmd);
    void sendMitCommand(const float q[6], const float dq[6], const float tau[6],
                        const float kp[6], const float kd[6],
                        uint8_t control_mode = 0x01);

    bool setMotorControlMode(uint8_t joint_id,
                             MotorControlMode mode,
                             std::chrono::milliseconds timeout = std::chrono::milliseconds{500});
    void sendPosVelCommand(const float q[6], const float dq[6], uint8_t enabled_mask = 0x3f);
    void sendVelocityCommand(const float dq[6], uint8_t enabled_mask = 0x3f);
    void sendHybridCommand(const float q[6], const float dq_limit[6],
                           const float current_limit_norm[6],
                           uint8_t enabled_mask = 0x3f);

    // ── 夹爪直接控制（fire-and-forget，非阻塞） ──
    // No gravity compensation — pure MIT torque on the gripper motor.
    // dt_us / seq auto-computed same as sendMitCommand.
    void sendGripperCommand(float q, float dq, float tau,
                            float kp, float kd,
                            uint8_t control_mode = 0x01);

    // ── 安全 ──
    void emergencyStop();

    // ── 状态查询（返回缓存副本） ──
    ArmStatus getArmStatus() const;
    GripperStatus getGripperStatus() const;

    // ── 电机反馈（请求-响应，阻塞） ──
    MotorFeedbackArray getMotorFeedback(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

    // ── 杂项（阻塞） ──
    bool homeAll(std::chrono::milliseconds timeout = std::chrono::milliseconds{500});
    bool clearFaults(std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace florid::usb

#endif
