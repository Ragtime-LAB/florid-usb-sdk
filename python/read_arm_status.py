#!/usr/bin/env python3
from __future__ import annotations

import argparse
import signal
import sys
import time

import numpy as np

import florid_usb


STOP = False
DEFAULT_ACM_INDEX = 0


def _signal_handler(signum, _frame) -> None:
    global STOP
    STOP = True
    print(f"\n[status] received signal {signum}, stopping...")


def resolve_device(device_arg: str | None, acm_index: int | None) -> str:
    if device_arg:
        if device_arg.startswith("/dev/"):
            return device_arg
        if device_arg.upper().startswith("ACM"):
            suffix = device_arg[3:]
            if not suffix.isdigit():
                raise ValueError(f"invalid ACM device: {device_arg}")
            return f"/dev/ttyACM{int(suffix)}"
        if device_arg.isdigit():
            return f"/dev/ttyACM{int(device_arg)}"
        raise ValueError(f"unsupported device argument: {device_arg}")

    index = DEFAULT_ACM_INDEX if acm_index is None else acm_index
    return f"/dev/ttyACM{index}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read and print robotic arm status over USB")
    parser.add_argument(
        "device",
        nargs="?",
        help="device selector: 0, 1, ACM0, ACM1, or full path like /dev/ttyACM0",
    )
    parser.add_argument(
        "--acm",
        type=int,
        help="ACM index to use, for example --acm 0 or --acm 1",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="print interval in seconds",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="number of samples to print, 0 means run until Ctrl+C",
    )
    return parser.parse_args()


def print_status(sample_idx: int, status: dict) -> None:
    q = np.asarray(status["q"], dtype=np.float32)
    dq = np.asarray(status["dq"], dtype=np.float32)
    tau = np.asarray(status["tau"], dtype=np.float32)
    mode = int(status["mode"])
    seq = int(status["seq"])
    timestamp_us = int(status["timestamp_us"])

    gripper = status.get("gripper", {})
    gq = float(gripper.get("q", 0.0))
    gdq = float(gripper.get("dq", 0.0))
    gtau = float(gripper.get("tau", 0.0))
    gtemp = float(gripper.get("temp_c", 0.0))
    genabled = bool(gripper.get("enabled", False))

    print(
        f"[status] sample={sample_idx} mode={mode} seq={seq} timestamp_us={timestamp_us}\n"
        f"         q   = {np.array2string(q, precision=4, suppress_small=True)}\n"
        f"         dq  = {np.array2string(dq, precision=4, suppress_small=True)}\n"
        f"         tau = {np.array2string(tau, precision=4, suppress_small=True)}\n"
        f"         gripper  q={gq:.4f}  dq={gdq:.4f}  tau={gtau:.4f}  temp={gtemp:.1f}°C  enabled={genabled}"
    )


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        device = resolve_device(args.device, args.acm)
    except ValueError as exc:
        print(f"[status] {exc}")
        return 2

    cfg = florid_usb.Config()
    cfg.device = device
    arm = florid_usb.Arm(cfg)

    print(f"[status] opening {device}")
    if not arm.connect():
        print("[status] failed to open serial port")
        return 1
    print("[status] serial connected")

    sample_idx = 0
    try:
        while not STOP:
            sample_idx += 1
            print_status(sample_idx, arm.get_arm_status())

            if args.count > 0 and sample_idx >= args.count:
                break

            time.sleep(args.interval)
    finally:
        arm.disconnect()
        print("[status] disconnected")

    return 0


if __name__ == "__main__":
    sys.exit(main())
