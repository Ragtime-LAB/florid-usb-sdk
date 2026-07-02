#!/usr/bin/env python3
"""
Simple MIT control example at 500 Hz.

Usage:
    PYTHONPATH=build/python python examples/python/example_simple.py [/dev/ttyACM0]
"""

import sys
import time
import numpy as np

import florid_usb


def main():
    device = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    print(f"[demo] opening {device}")

    cfg = florid_usb.Config()
    cfg.device = device
    arm = florid_usb.Arm(cfg)

    if not arm.connect():
        print("[demo] failed to open serial port")
        return 1
    print("[demo] serial connected")

    print("[demo] starting USB session...")
    if not arm.start_session(timeout=1.0):
        print("[demo] start_session failed")
        return 1
    print("[demo] USB session active")

    # Read initial position
    status = arm.get_arm_status()
    q = status["q"].copy()
    dq = np.zeros(6, dtype=np.float32)
    tau = np.zeros(6, dtype=np.float32)
    kp = np.zeros(6, dtype=np.float32)
    kd = np.zeros(6, dtype=np.float32)

    q0_j5 = float(q[5])
    q1_j5 = q0_j5 + 0.5

    # 500 Hz = 2 ms period
    period_s = 0.002
    n_steps = 500  # 1 second
    next_tick = time.perf_counter()

    print(f"[demo] ramp J5: {q0_j5:.3f} -> {q1_j5:.3f} @ 500 Hz")
    for step in range(n_steps):
        frac = step / n_steps
        q[5] = q0_j5 + frac * (q1_j5 - q0_j5)

        kp[5] = 6.5
        kd[5] = 0.3

        arm.send_mit_command(q, dq, tau, kp, kd)

        next_tick += period_s
        while time.perf_counter() < next_tick:
            pass  # busy-spin for precise 500 Hz timing

    arm.get_arm_status()
    status = arm.get_arm_status()
    print(f"[demo] ramp done, J5={status['q'][5]:.3f}")

    # Ramp back
    cur_q5 = float(status["q"][5])
    next_tick = time.perf_counter()

    print(f"[demo] ramp J5 back: {cur_q5:.3f} -> {q0_j5:.3f}")
    for step in range(n_steps):
        frac = step / n_steps
        q[5] = cur_q5 + frac * (q0_j5 - cur_q5)

        kp[5] = 6.5
        kd[5] = 0.3

        arm.send_mit_command(q, dq, tau, kp, kd)

        next_tick += period_s
        while time.perf_counter() < next_tick:
            pass

    status = arm.get_arm_status()
    print(f"[demo] ramp back done, J5={status['q'][5]:.3f}")

    print("[demo] stopping session...")
    arm.stop_session(1.0)
    print("[demo] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
