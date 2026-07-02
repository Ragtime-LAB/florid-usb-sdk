#!/usr/bin/env python3
from __future__ import annotations

import argparse
import signal
import sys
import time

import numpy as np

import florid_usb


STOP = False
DEFAULT_LEADER_DEVICE = "/dev/ttyACM0"
DEFAULT_FOLLOWER_DEVICE = "/dev/ttyACM1"


def _signal_handler(signum, _frame) -> None:
    global STOP
    STOP = True
    print(f"\n[status] received signal {signum}, stopping...")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read two Florid USB arm statuses without sending control commands"
    )
    parser.add_argument(
        "device",
        nargs="?",
        default=None,
        help="single device shorthand, e.g. ACM0, ACM1, /dev/ttyACM0",
    )
    parser.add_argument("--leader-device", default=DEFAULT_LEADER_DEVICE)
    parser.add_argument("--follower-device", default=DEFAULT_FOLLOWER_DEVICE)
    parser.add_argument(
        "--only",
        choices=["leader", "follower"],
        default=None,
        help="read only one arm",
    )
    parser.add_argument("--period", type=float, default=0.2, help="print period in seconds")
    return parser.parse_args()


def normalize_device(device: str) -> str:
    if device.startswith("/dev/"):
        return device
    if device.startswith("tty"):
        return f"/dev/{device}"
    if device.startswith("ACM") or device.startswith("USB"):
        return f"/dev/tty{device}"
    return device


def make_arm(device: str) -> florid_usb.Arm:
    cfg = florid_usb.Config()
    cfg.device = device
    return florid_usb.Arm(cfg)


def connect_and_start(arm: florid_usb.Arm, label: str, device: str) -> None:
    print(f"[status] opening {label} on {device}")
    if not arm.connect():
        raise RuntimeError(f"failed to open {label} serial port: {device}")
    try:
        print(f"[status] {label} serial connected")
        if not arm.start_session(timeout=1.0):
            raise RuntimeError(f"failed to start {label} USB session")
        print(f"[status] {label} USB session active")
    except Exception:
        arm.disconnect()
        raise


def stop_and_disconnect(arm: florid_usb.Arm, label: str) -> None:
    try:
        arm.stop_session(1.0)
    except Exception:
        pass
    arm.disconnect()
    print(f"[status] {label} disconnected")


def format_status(label: str, status: dict) -> str:
    q = np.asarray(status["q"], dtype=np.float64)
    dq = np.asarray(status["dq"], dtype=np.float64)
    tau = np.asarray(status["tau"], dtype=np.float64)
    return (
        f"{label}_seq={status['seq']} "
        f"{label}_q={np.array2string(q, precision=3, suppress_small=True)} "
        f"{label}_dq={np.array2string(dq, precision=3, suppress_small=True)} "
        f"{label}_tau={np.array2string(tau, precision=3, suppress_small=True)}"
    )


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    if args.device is not None:
        args.leader_device = normalize_device(args.device)
        args.only = "leader"

    use_leader = args.only in (None, "leader")
    use_follower = args.only in (None, "follower")

    leader = make_arm(args.leader_device) if use_leader else None
    follower = make_arm(args.follower_device) if use_follower else None
    leader_started = False
    follower_started = False

    try:
        if leader is not None:
            connect_and_start(leader, "leader", args.leader_device)
            leader_started = True
        if follower is not None:
            connect_and_start(follower, "follower", args.follower_device)
            follower_started = True

        print("[status] reading only, no control commands will be sent")
        while not STOP:
            parts: list[str] = []
            if leader is not None:
                leader_status = leader.get_arm_status()
                parts.append(format_status("leader", leader_status))
            if follower is not None:
                follower_status = follower.get_arm_status()
                parts.append(format_status("follower", follower_status))
            print("[status] " + " ".join(parts))
            time.sleep(args.period)

        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"[status] error: {exc}")
        return 1
    finally:
        if follower_started and follower is not None:
            stop_and_disconnect(follower, "follower")
        if leader_started and leader is not None:
            stop_and_disconnect(leader, "leader")


if __name__ == "__main__":
    sys.exit(main())
