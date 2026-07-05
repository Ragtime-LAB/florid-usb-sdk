#!/usr/bin/env python3
"""
USB_ONLY gravity hold — firmware-side gravity compensation via control_mode=0x05.

Puts the arm into a compliant gravity-compensated standby state using the
firmware's built-in CasADi gravity model.  No host-side Pinocchio needed.

Usage:
    PYTHONPATH=python python3 python/usb_gravity_hold.py
    PYTHONPATH=python python3 python/usb_gravity_hold.py /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

import numpy as np

import florid_usb


STOP = False
JOINT_COUNT = 6
DEFAULT_DEVICE = "/dev/ttyACM0"
DEFAULT_PERIOD_S = 0.004
DEFAULT_PRINT_INTERVAL_S = 0.2
DEFAULT_KP = 5.0
DEFAULT_KD = 0.3


def _signal_handler(signum, _frame) -> None:
    global STOP
    STOP = True
    print(f"\n[hold] signal {signum}, stopping...")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="USB_ONLY gravity hold — firmware-side gravity compensation"
    )
    parser.add_argument("device", nargs="?", default=DEFAULT_DEVICE)
    parser.add_argument(
        "--period",
        type=float,
        default=DEFAULT_PERIOD_S,
        help=f"control period in seconds, default {DEFAULT_PERIOD_S} ({1/DEFAULT_PERIOD_S:.0f} Hz)",
    )
    parser.add_argument(
        "--print-interval",
        type=float,
        default=DEFAULT_PRINT_INTERVAL_S,
        help="status print interval in seconds",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=DEFAULT_KP,
        help=f"uniform stiffness for all joints, default {DEFAULT_KP}",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=DEFAULT_KD,
        help=f"uniform damping for all joints, default {DEFAULT_KD}",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    cfg = florid_usb.Config()
    cfg.device = args.device
    arm = florid_usb.Arm(cfg)

    print(f"[hold] opening {args.device}")
    if not arm.connect():
        print("[hold] failed to open serial port")
        return 1
    print("[hold] serial connected")

    session_started = False
    try:
        print("[hold] starting USB session...")
        if not arm.start_session(timeout=1.0):
            print("[hold] start_session failed")
            return 1
        session_started = True
        print("[hold] USB session active")

        status = arm.get_arm_status()
        q_cmd = np.asarray(status["q"], dtype=np.float32)
        dq_cmd = np.zeros(JOINT_COUNT, dtype=np.float32)
        tau_cmd = np.zeros(JOINT_COUNT, dtype=np.float32)
        kp = np.full(JOINT_COUNT, args.kp, dtype=np.float32)
        kd = np.full(JOINT_COUNT, args.kd, dtype=np.float32)
        control_mode = 0x05

        print(f"[hold] running with firmware gravity (control_mode=0x{control_mode:02x})")
        print(f"[hold] kp={args.kp}, kd={args.kd}  — Ctrl+C to stop")

        next_tick = time.perf_counter()
        last_print = 0.0

        while not STOP:
            status = arm.get_arm_status()
            q = np.asarray(status["q"], dtype=np.float32)
            dq = np.asarray(status["dq"], dtype=np.float32)
            tau_meas = np.asarray(status["tau"], dtype=np.float32)

            q_cmd[:] = q
            tau_cmd.fill(0.0)

            arm.send_mit_command(q_cmd, dq_cmd, tau_cmd, kp, kd, control_mode)

            now = time.perf_counter()
            if now - last_print >= args.print_interval:
                print(
                    "[hold] "
                    f"q={np.array2string(q, precision=3, suppress_small=True)} "
                    f"dq={np.array2string(dq, precision=3, suppress_small=True)} "
                    f"tau_meas={np.array2string(tau_meas, precision=3, suppress_small=True)} "
                    f"kp={args.kp:.1f} kd={args.kd:.2f}"
                )
                last_print = now

            next_tick += args.period
            while not STOP and time.perf_counter() < next_tick:
                pass

            if time.perf_counter() - next_tick > 0.2:
                next_tick = time.perf_counter()

    finally:
        if session_started:
            print("[hold] stopping session...")
            arm.stop_session(1.0)
        arm.disconnect()
        print("[hold] done")

    return 0


if __name__ == "__main__":
    sys.exit(main())
