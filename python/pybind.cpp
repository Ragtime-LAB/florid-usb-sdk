#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>

#include <florid/usb/Arm.hpp>

#include <cstring>
#include <chrono>

namespace py = pybind11;
using namespace florid::usb;
using namespace std::chrono_literals;

namespace {

float* extract_6f(py::array_t<float, py::array::c_style | py::array::forcecast> arr,
                  const char* name)
{
    if (arr.ndim() != 1 || arr.shape(0) < 6)
        throw std::runtime_error(std::string(name) + " must be a 1D array of length >= 6");
    return static_cast<float*>(arr.request().ptr);
}

MotorControlMode parse_motor_control_mode(const py::object& obj)
{
    if (py::isinstance<py::str>(obj))
    {
        const std::string mode = obj.cast<std::string>();
        if (mode == "mit") return MotorControlMode::Mit;
        if (mode == "posvel") return MotorControlMode::PosVel;
        if (mode == "vel") return MotorControlMode::Vel;
        if (mode == "velocity") return MotorControlMode::Vel;
        if (mode == "hybrid") return MotorControlMode::Hybrid;
        throw std::runtime_error("mode must be one of: mit, posvel, vel, hybrid");
    }

    const int mode = obj.cast<int>();
    switch (mode)
    {
    case 1:
        return MotorControlMode::Mit;
    case 2:
        return MotorControlMode::PosVel;
    case 3:
        return MotorControlMode::Vel;
    case 4:
        return MotorControlMode::Hybrid;
    default:
        throw std::runtime_error("mode must be 1(MIT), 2(POSVEL), 3(VEL), or 4(HYBRID)");
    }
}

py::dict arm_status_to_dict(const ArmStatus& s)
{
    py::array_t<float> q({6});
    py::array_t<float> dq({6});
    py::array_t<float> tau({6});
    std::memcpy(q.mutable_data(), s.status.q.data(), 6 * sizeof(float));
    std::memcpy(dq.mutable_data(), s.status.dq.data(), 6 * sizeof(float));
    std::memcpy(tau.mutable_data(), s.status.tau.data(), 6 * sizeof(float));

    py::dict d;
    d["mode"] = static_cast<int>(s.mode);
    d["seq"] = s.seq;
    d["timestamp_us"] = s.timestamp_us;
    d["q"] = q;
    d["dq"] = dq;
    d["tau"] = tau;
    py::dict gripper;
    gripper["q"] = s.gripper.q;
    gripper["dq"] = s.gripper.dq;
    gripper["tau"] = s.gripper.tau;
    gripper["temp_c"] = s.gripper.temp_c;
    gripper["enabled"] = (s.gripper.enabled != 0);
    d["gripper"] = gripper;
    return d;
}

} // namespace

PYBIND11_MODULE(florid_usb, m)
{
    m.doc() = "Ragtime Florid USB SDK — MIT direct control over USB CDC";

    // ── Config struct (must be registered before Arm) ──

    py::class_<Arm::Config>(m, "Config")
        .def(py::init<>())
        .def_readwrite("device", &Arm::Config::device,
                       "Serial device path (default '/dev/ttyACM0')")
        .def_readwrite("baud_rate", &Arm::Config::baud_rate,
                       "Baud rate (default 115200)")
        .def_readwrite("session_timeout_ms", &Arm::Config::session_timeout,
                       "Per-request timeout in milliseconds (default 500)")
        .def_readwrite("max_retries", &Arm::Config::max_retries,
                       "Max retries per request (default 3)");

    // ── Arm class ──

    py::class_<Arm>(m, "Arm")
        .def(py::init([]() { return Arm(Arm::Config{}); }),
             "Create an Arm client with default config")
        .def(py::init<Arm::Config>(),
             py::arg("config"),
             "Create an Arm client. Config fields: device='/dev/ttyACM0', baud_rate=115200, "
             "session_timeout_ms=500, max_retries=3")

        .def("connect", &Arm::connect,
             "Open the serial port and start background communication")
        .def("disconnect", &Arm::disconnect,
             "Close the serial port and stop the worker thread")
        .def("is_connected", &Arm::isConnected,
             "Return whether USB is connected and receiving data")

        // ── Session ──

        .def("start_session", [](Arm& self, double timeout_s) {
            return self.startSession(std::chrono::milliseconds(static_cast<int>(timeout_s * 1000.0)));
        }, py::arg("timeout") = 0.5,
             "Start a USB control session (sends UsbSessionStart). "
             "Returns True on success. timeout is in seconds.")
        .def("stop_session", [](Arm& self, double timeout_s) {
            return self.stopSession(std::chrono::milliseconds(static_cast<int>(timeout_s * 1000.0)));
        }, py::arg("timeout") = 0.5,
             "Stop the USB control session (sends UsbSessionStop). "
             "Returns True on success. timeout is in seconds.")

        // ── MIT direct control ──

        .def("send_mit_command",
             [](Arm& self,
                py::array_t<float, py::array::c_style | py::array::forcecast> q,
                py::array_t<float, py::array::c_style | py::array::forcecast> dq,
                py::array_t<float, py::array::c_style | py::array::forcecast> tau,
                py::array_t<float, py::array::c_style | py::array::forcecast> kp,
                py::array_t<float, py::array::c_style | py::array::forcecast> kd,
                uint8_t control_mode) {
                 self.sendMitCommand(
                     extract_6f(q, "q"), extract_6f(dq, "dq"), extract_6f(tau, "tau"),
                     extract_6f(kp, "kp"), extract_6f(kd, "kd"), control_mode);
             },
             py::arg("q"), py::arg("dq"), py::arg("tau"),
             py::arg("kp"), py::arg("kd"), py::arg("control_mode") = 1,
             "Send an MIT control command. Firmware provides built-in gravity compensation; "
             "`tau` is additional feed-forward torque on top of gravity.\n"
             "control_mode: bit[1:0]=type(0=hold,1=MIT,3=torque), bit[2]=gravity_enable.\n"
             "  Examples: 0x01=MIT, 0x03=torque, 0x05=MIT+gravity, 0x07=torque+gravity.\n"
             "Each arg must be a numpy array (or list) of 6 floats.\n"
             "dt_us and seq are auto-computed from wall-clock time between calls.")

        .def("set_motor_control_mode",
             [](Arm& self, uint8_t joint_id, py::object mode, double timeout_s) {
                 return self.setMotorControlMode(
                     joint_id,
                     parse_motor_control_mode(mode),
                     std::chrono::milliseconds(static_cast<int>(timeout_s * 1000.0)));
             },
             py::arg("joint_id"), py::arg("mode"), py::arg("timeout") = 0.5,
             "Switch one arm joint motor control mode. mode may be 'mit', 'posvel', 'vel', "
             "'hybrid' or integer 1..4. Returns True on firmware ACK.")

        .def("send_posvel_command",
             [](Arm& self,
                py::array_t<float, py::array::c_style | py::array::forcecast> q,
                py::array_t<float, py::array::c_style | py::array::forcecast> dq,
                uint8_t enabled_mask) {
                 self.sendPosVelCommand(extract_6f(q, "q"), extract_6f(dq, "dq"), enabled_mask);
             },
             py::arg("q"), py::arg("dq"), py::arg("enabled_mask") = 0x3f,
             "Send a POSVEL command to joints selected by enabled_mask. "
             "Motors must already be switched to posvel mode.")

        .def("send_velocity_command",
             [](Arm& self,
                py::array_t<float, py::array::c_style | py::array::forcecast> dq,
                uint8_t enabled_mask) {
                 self.sendVelocityCommand(extract_6f(dq, "dq"), enabled_mask);
             },
             py::arg("dq"), py::arg("enabled_mask") = 0x3f,
             "Send a velocity command to joints selected by enabled_mask. "
             "Motors must already be switched to vel mode.")

        .def("send_hybrid_command",
             [](Arm& self,
                py::array_t<float, py::array::c_style | py::array::forcecast> q,
                py::array_t<float, py::array::c_style | py::array::forcecast> dq_limit,
                py::array_t<float, py::array::c_style | py::array::forcecast> current_limit_norm,
                uint8_t enabled_mask) {
                 self.sendHybridCommand(
                     extract_6f(q, "q"),
                     extract_6f(dq_limit, "dq_limit"),
                     extract_6f(current_limit_norm, "current_limit_norm"),
                     enabled_mask);
             },
             py::arg("q"), py::arg("dq_limit"), py::arg("current_limit_norm"),
             py::arg("enabled_mask") = 0x3f,
             "Send a HYBRID force-position command to joints selected by enabled_mask. "
             "q is rad, dq_limit is rad/s, current_limit_norm is 0..1.")

        // ── Gripper control ──

        .def("send_gripper_command",
             [](Arm& self, float q, float dq, float tau,
                float kp, float kd, uint8_t control_mode) {
                 self.sendGripperCommand(q, dq, tau, kp, kd, control_mode);
             },
             py::arg("q"), py::arg("dq"), py::arg("tau"),
             py::arg("kp"), py::arg("kd"), py::arg("control_mode") = 1,
             "Send a gripper command. No gravity compensation — pure MIT torque.")

        // ── Safety ──

        .def("emergency_stop", &Arm::emergencyStop,
             "Send an emergency stop (fire-and-forget)")

        // ── Status ──

        .def("get_arm_status", [](Arm& self) {
            return arm_status_to_dict(self.getArmStatus());
        }, "Return latest cached ArmStatus as a dict with keys: mode, seq, timestamp_us, q, dq, tau")

        .def("get_gripper_status", [](Arm& self) -> py::dict {
            auto gs = self.getGripperStatus();
            const float q = gs.q;
            const float dq = gs.dq;
            const float tau = gs.tau;
            const float temp_c = gs.temp_c;
            const bool enabled = (gs.enabled != 0);
            py::dict d;
            d["q"] = q;
            d["dq"] = dq;
            d["tau"] = tau;
            d["temp_c"] = temp_c;
            d["enabled"] = enabled;
            return d;
        }, "Return latest cached GripperStatus as a dict with keys: q, dq, tau, temp_c, enabled")

        .def("get_motor_feedback", [](Arm& self, double timeout_s) -> py::dict {
            auto fb = self.getMotorFeedback(
                std::chrono::milliseconds(static_cast<int>(timeout_s * 1000.0)));
            py::list motors;
            for (int i = 0; i < 7; ++i) {
                const auto m = fb.motors[i];
                py::dict md;
                md["joint_id"] = static_cast<int>(m.joint_id);
                md["device_status"] = static_cast<int>(m.device_status);
                md["enabled"] = (m.enabled != 0);
                md["position_rad"] = m.position_rad;
                md["speed_rad_s"] = m.speed_rad_s;
                md["torque_nm"] = m.torque_nm;
                md["temp_c"] = m.temp_c;
                motors.append(md);
            }
            py::dict d;
            d["motors"] = motors;
            return d;
        }, py::arg("timeout") = 0.5,
             "Request motor feedback from firmware (request-response, blocking). timeout in seconds.")

        // ── Miscellaneous ──

        .def("home_all", [](Arm& self, double timeout_s) {
            return self.homeAll(std::chrono::milliseconds(static_cast<int>(timeout_s * 1000.0)));
        }, py::arg("timeout") = 0.5,
             "Request home-all. Returns True on ack. timeout in seconds.")
        .def("clear_faults", [](Arm& self, double timeout_s) {
            return self.clearFaults(std::chrono::milliseconds(static_cast<int>(timeout_s * 1000.0)));
        }, py::arg("timeout") = 0.5,
             "Clear all faults. Returns True on ack. timeout in seconds.");
}
