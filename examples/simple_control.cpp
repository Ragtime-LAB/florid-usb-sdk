#include <florid/usb/Arm.hpp>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

using namespace florid::usb;
using namespace std::chrono_literals;

namespace {
std::atomic<bool> g_running{true};
void sig_handler(int) { g_running.store(false); }
} // namespace

static void print_arm_status(const ArmStatus &s) {
  std::cout << std::fixed << std::setprecision(3)
            << "[status] mode=" << static_cast<int>(s.mode) << " seq=" << s.seq
            << "  q=[" << s.status.q[0] << "," << s.status.q[1] << ","
            << s.status.q[2] << "," << s.status.q[3] << "," << s.status.q[4]
            << "," << s.status.q[5] << "]"
            << "  dq=[" << s.status.dq[0] << "," << s.status.dq[1] << ","
            << s.status.dq[2] << "," << s.status.dq[3] << "," << s.status.dq[4]
            << "," << s.status.dq[5] << "]"
            << "  tau=[" << s.status.tau[0] << "," << s.status.tau[1] << ","
            << s.status.tau[2] << "," << s.status.tau[3] << ","
            << s.status.tau[4] << "," << s.status.tau[5] << "]\n";
}

int main(int argc, char **argv) {
  std::signal(SIGINT, sig_handler);

  const std::string device = (argc > 1) ? argv[1] : "/dev/ttyACM0";
  std::cout << "[demo] opening " << device << "\n";

  Arm::Config cfg;
  cfg.device = device;
  cfg.baud_rate = 115200;
  cfg.session_timeout = 500ms;
  cfg.max_retries = 3;

  Arm arm(std::move(cfg));

  if (!arm.connect()) {
    std::cerr << "[demo] failed to open serial port\n";
    return 1;
  }
  std::cout << "[demo] serial connected\n";

  // ── 1. Wait for ArmStatus to verify firmware is broadcasting ──
  std::cout << "[demo] waiting for ArmStatus (firmware should broadcast every "
               "2ms)...\n";
  bool got_status = false;
  for (int i = 0; i < 200; ++i) {
    auto s = arm.getArmStatus();
    if (s.seq != 0) {
      // std::cout << "[demo] ArmStatus received!\n";
      //  print_arm_status(s);
      got_status = true;
      break;
    }
    std::this_thread::sleep_for(10ms);
  }
  if (!got_status) {
    std::cerr << "[demo] no ArmStatus received in 2s — is firmware running "
                 "USB_ONLY?\n";
    std::cerr << "[demo] trying startSession anyway...\n";
  }

  // ── 2. Start USB session ──
  std::cout << "[demo] starting USB session...\n";
  if (!arm.startSession(1s)) {
    std::cerr << "[demo] UsbSessionStart failed\n";
    return 1;
  }
  std::cout << "[demo] USB session active\n";

  // ── 3. Read current position and send JointCommand ramp ──
  std::cout << "[demo] sending JointCommand ramp on J5...\n";
  float q[6]{}, dq[6]{}, tau[6]{};
  float kp[6]{}, kd[6]{};

  // Read current position
  auto initial = arm.getArmStatus();
  for (size_t i = 0; i < 6; ++i)
    q[i] = initial.status.q[i];

  constexpr float kTargetDelta = 0.5f;
  const float q0_j5 = q[5];
  const float q1_j5 = q0_j5 + kTargetDelta;

  constexpr auto kPeriod = 10ms;
  constexpr int kSteps = 100; // 1 second = 0.5 rad/s
  auto next_tick = std::chrono::steady_clock::now();

  for (int step = 0; step < kSteps && g_running.load(); ++step) {
    float frac = static_cast<float>(step) / kSteps;
    q[5] = q0_j5 + frac * (q1_j5 - q0_j5);

    kp[5] = 8.0f;
    kd[5] = 0.7f;

    arm.sendMitCommand(q, dq, tau, kp, kd);

    next_tick += kPeriod;
    std::this_thread::sleep_until(next_tick);
  }

  std::cout << "[demo] ramp done, status: ";
  print_arm_status(arm.getArmStatus());

  // Ramp back
  std::cout << "[demo] ramping J5 back...\n";
  {
    auto cur = arm.getArmStatus();
    const float cur_q5 = cur.status.q[5];
    next_tick = std::chrono::steady_clock::now();

    for (int step = 0; step < kSteps && g_running.load(); ++step) {
      float frac = static_cast<float>(step) / kSteps;
      q[5] = cur_q5 + frac * (q0_j5 - cur_q5);

      kp[5] = 8.0f;
      kd[5] = 0.7f;

      arm.sendMitCommand(q, dq, tau, kp, kd);

      next_tick += kPeriod;
      std::this_thread::sleep_until(next_tick);
    }
  }

  std::cout << "[demo] ramp back done, status: ";
  print_arm_status(arm.getArmStatus());

  std::cout << "[demo] stopping USB session...\n";
  arm.stopSession(1s);

  std::cout << "[demo] done.\n";
  return 0;
}
