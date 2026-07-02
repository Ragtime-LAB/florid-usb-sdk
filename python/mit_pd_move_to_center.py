#!/usr/bin/env python3
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
TARGET_POS = np.array([0.0, 0.8, 1.0, -0.5, 0.0, 0.0], dtype=np.float32)
MIT_KP = np.array([8.0, 15.0, 20.0, 5.0, 3.0, 1.0], dtype=np.float32)
MIT_KD = np.array([0.8, 1.5, 2.0, 0.6, 0.3, 0.1], dtype=np.float32)
ZERO_DQ = np.zeros(JOINT_COUNT, dtype=np.float32)
ZERO_TAU = np.zeros(JOINT_COUNT, dtype=np.float32)


def _signal_handler(signum, _frame) -> None:
    global STOP
    STOP = True
    print(f"\n[pd] received signal {signum}, stopping...")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move robot to center pose using MIT PD")
    parser.add_argument("device", nargs="?", default=DEFAULT_DEVICE)
    parser.add_argument("--period", type=float, default=0.002, help="control period in seconds")
    parser.add_argument("--print-interval", type=float, default=0.5, help="status print interval in seconds")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    cfg = florid_usb.Config()
    cfg.device = args.device
    arm = florid_usb.Arm(cfg)

    print(f"[pd] opening {args.device}")
    if not arm.connect():
        print("[pd] failed to open serial port")
        return 1
    print("[pd] serial connected")

    session_started = False
    try:
        print("[pd] starting USB session...")
        if not arm.start_session(timeout=1.0):
            print("[pd] start_session failed")
            return 1
        session_started = True
        print("[pd] USB session active")
        print(f"[pd] target: {TARGET_POS}")
        print(f"[pd] kp: {MIT_KP}")
        print(f"[pd] kd: {MIT_KD}")

        next_tick = time.perf_counter()
        last_print = 0.0

        while not STOP:
            arm.send_mit_command(TARGET_POS, ZERO_DQ, ZERO_TAU, MIT_KP, MIT_KD)

            now = time.perf_counter()
            if now - last_print >= args.print_interval:
                status = arm.get_arm_status()
                q = np.asarray(status["q"], dtype=np.float32)
                dq = np.asarray(status["dq"], dtype=np.float32)
                err = TARGET_POS - q
                print(
                    "[pd] "
                    f"q={np.array2string(q, precision=3, suppress_small=True)} "
                    f"err={np.array2string(err, precision=3, suppress_small=True)} "
                    f"dq={np.array2string(dq, precision=3, suppress_small=True)}"
                )
                last_print = now

            next_tick += args.period
            while not STOP and time.perf_counter() < next_tick:
                pass
            if time.perf_counter() - next_tick > 0.2:
                next_tick = time.perf_counter()

        final_status = arm.get_arm_status()
        q_final = np.asarray(final_status["q"], dtype=np.float32)
        print(f"[pd] final q={q_final}")
        print(f"[pd] final err={TARGET_POS - q_final}")
    finally:
        if session_started:
            arm.stop_session(1.0)
        arm.disconnect()
        print("[pd] done")

    return 0


if __name__ == "__main__":
    sys.exit(main())
