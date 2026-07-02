#!/usr/bin/env python3
"""
Gripper control test.

Usage:
    python python/example_gripper.py [/dev/ttyACM0]
"""

import sys
import time

import numpy as np

import florid_usb


def main():
    device = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    print(f"[gripper] opening {device}")

    cfg = florid_usb.Config()
    cfg.device = device
    arm = florid_usb.Arm(cfg)

    if not arm.connect():
        print("[gripper] failed to open serial port")
        return 1
    print("[gripper] serial connected")

    print("[gripper] starting USB session...")
    if not arm.start_session(timeout=1.0):
        print("[gripper] start_session failed")
        return 1
    print("[gripper] USB session active")

    # Read initial gripper state
    print("[gripper] reading initial status...")
    gs = arm.get_gripper_status()
    print(f"  Gripper: q={gs['q']:.4f}  dq={gs['dq']:.4f}  tau={gs['tau']:.4f}  "
          f"temp={gs['temp_c']:.1f}  enabled={gs['enabled']}")

    # Read motor feedback to confirm gripper is alive
    print("[gripper] requesting motor feedback...")
    fb = arm.get_motor_feedback(1.0)
    for m in fb["motors"]:
        if m["joint_id"] == 6:
            print(f"  Motor 6 (gripper): pos={m['position_rad']:.4f}  "
                  f"enabled={m['enabled']}  status={m['device_status']}")

    # Control loop: slowly open and close the gripper
    period_s = 0.01  # 100 Hz
    n_steps = 200     # 2 seconds per move

    # First get current position
    pos_now = float(gs["q"])
    print(f"\n[gripper] current position: {pos_now:.4f} rad")
    print("[gripper] opening (+0.5 rad) ...")

    next_tick = time.perf_counter()
    for step in range(n_steps):
        frac = step / n_steps
        target = pos_now + 0.5 * frac
        arm.send_gripper_command(
            q=target, dq=0.0, tau=0.0,
            kp=10.0, kd=1.0, control_mode=1,
        )
        next_tick += period_s
        while time.perf_counter() < next_tick:
            pass

    gs = arm.get_gripper_status()
    print(f"  Gripper: q={gs['q']:.4f}  enabled={gs['enabled']}")

    # Close back
    print("[gripper] closing (back to start) ...")
    next_tick = time.perf_counter()
    for step in range(n_steps):
        frac = step / n_steps
        target = (pos_now + 0.5) - 0.5 * frac
        arm.send_gripper_command(
            q=target, dq=0.0, tau=0.0,
            kp=10.0, kd=1.0, control_mode=1,
        )
        next_tick += period_s
        while time.perf_counter() < next_tick:
            pass

    gs = arm.get_gripper_status()
    print(f"  Gripper: q={gs['q']:.4f}  enabled={gs['enabled']}")

    print("[gripper] stopping session...")
    arm.stop_session(1.0)
    print("[gripper] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
