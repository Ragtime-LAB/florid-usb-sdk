#!/usr/bin/env python3
"""
Gravity compensation control for Bernie arm using host-side Pinocchio
computation via BernieDescription.urdf.

Reads joint status from USB, computes gravity feedforward torques via
Pinocchio, and sends MIT frames with the torques.

Usage:
    PYTHONPATH=python python3 python/bernie_gravity_compensation.py
    PYTHONPATH=python python3 python/bernie_gravity_compensation.py /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

import numpy as np

import florid_usb


STOP = False
JOINT_COUNT = 6
DEFAULT_DEVICE = "/dev/ttyACM0"
DEFAULT_PERIOD_S = 0.002  # 500 Hz
DEFAULT_PRINT_INTERVAL_S = 0.2
DEFAULT_KP = 0.0
DEFAULT_KD = 0.0
DEFAULT_SCALE = 1.0
TAU_LIMIT_NM = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0], dtype=np.float32)

URDF_PATH = (
    Path(__file__).resolve().parent
    / ".." / ".." / ".."
    / "3_Host" / "Cubie" / "core" / "config" / "urdf" / "BernieDescription.urdf"
).resolve()

ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def _signal_handler(signum, _frame) -> None:
    global STOP
    STOP = True
    print(f"\n[bernie_grav] received signal {signum}, stopping...")


class BernieDynamics:
    def __init__(self, urdf_path: Path) -> None:
        import pinocchio as pin

        self.pin = pin
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        if self.model.nq != JOINT_COUNT:
            raise RuntimeError(
                f"model.nq={self.model.nq} != {JOINT_COUNT}"
            )
        self.q_indices: dict[str, int] = {}
        self.v_indices: dict[str, int] = {}
        for name in ARM_JOINT_NAMES:
            jid = self.model.getJointId(name)
            joint = self.model.joints[jid]
            self.q_indices[name] = int(joint.idx_q() if callable(joint.idx_q) else joint.idx_q)
            self.v_indices[name] = int(joint.idx_v() if callable(joint.idx_v) else joint.idx_v)
        self.arm_vidx = np.array(
            [self.v_indices[name] for name in ARM_JOINT_NAMES], dtype=np.int32
        )

    def gravity(self, q_arm: np.ndarray) -> np.ndarray:
        q_pin = self.pin.neutral(self.model)
        for idx, name in enumerate(ARM_JOINT_NAMES):
            q_pin[self.q_indices[name]] = q_arm[idx]
        tau = self.pin.computeGeneralizedGravity(self.model, self.data, q_pin)
        return np.asarray(tau[self.arm_vidx], dtype=np.float64)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bernie gravity compensation via Pinocchio"
    )
    parser.add_argument("device", nargs="?", default=DEFAULT_DEVICE)
    parser.add_argument(
        "--period",
        type=float,
        default=DEFAULT_PERIOD_S,
        help=f"control period in seconds, default {DEFAULT_PERIOD_S}",
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
        help="uniform MIT kp for all 6 joints, default: 0.0",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=DEFAULT_KD,
        help="uniform MIT kd for all 6 joints, default: 0.4",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=DEFAULT_SCALE,
        help="gravity compensation scale factor 0.0–1.0, default: 0.3 (30%%). "
        "Start with --scale 0.2 for safe testing.",
    )
    return parser.parse_args()


def as_f32(vec: np.ndarray) -> np.ndarray:
    return np.asarray(vec, dtype=np.float32)


def main() -> int:
    args = parse_args()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    if not URDF_PATH.is_file():
        print(f"[bernie_grav] URDF not found: {URDF_PATH}")
        return 1

    try:
        dynamics = BernieDynamics(URDF_PATH)
    except ImportError:
        print("[bernie_grav] pinocchio not installed")
        return 1
    except Exception as exc:
        print(f"[bernie_grav] failed to load URDF: {exc}")
        return 1

    print(f"[bernie_grav] URDF: {URDF_PATH}")
    print(f"[bernie_grav] joints: {ARM_JOINT_NAMES}")

    cfg = florid_usb.Config()
    cfg.device = args.device
    arm = florid_usb.Arm(cfg)

    print(f"[bernie_grav] opening {args.device}")
    if not arm.connect():
        print("[bernie_grav] failed to open serial port")
        return 1
    print("[bernie_grav] serial connected")

    session_started = False
    try:
        print("[bernie_grav] starting USB session...")
        if not arm.start_session(timeout=1.0):
            print("[bernie_grav] start_session failed")
            return 1
        session_started = True
        print("[bernie_grav] USB session active")

        status = arm.get_arm_status()
        q_cmd = as_f32(status["q"])
        dq_cmd = np.zeros(JOINT_COUNT, dtype=np.float32)
        kp = np.full(JOINT_COUNT, args.kp, dtype=np.float32)
        kd = np.full(JOINT_COUNT, args.kd, dtype=np.float32)
        tau_cmd = np.zeros(JOINT_COUNT, dtype=np.float32)

        print(f"[bernie_grav] gravity scale: {args.scale:.2f}")
        print("[bernie_grav] running host-side gravity compensation, Ctrl+C to stop")
        next_tick = time.perf_counter()
        last_print = 0.0

        while not STOP:
            status = arm.get_arm_status()
            q = as_f32(status["q"])
            dq = as_f32(status["dq"])
            tau_meas = as_f32(status["tau"])

            q_cmd[:] = q

            host_gravity = np.asarray(dynamics.gravity(q), dtype=np.float32) * args.scale
            host_gravity = np.clip(host_gravity, -TAU_LIMIT_NM, TAU_LIMIT_NM)
            tau_cmd[:] = host_gravity

            arm.send_mit_command(q_cmd, dq_cmd, tau_cmd, kp, kd, control_mode=1)

            now = time.perf_counter()
            if now - last_print >= args.print_interval:
                print(
                    "[bernie_grav] "
                    f"scale={args.scale:.2f} "
                    f"q={np.array2string(q, precision=3, suppress_small=True)} "
                    f"dq={np.array2string(dq, precision=3, suppress_small=True)} "
                    f"tau_meas={np.array2string(tau_meas, precision=3, suppress_small=True)} "
                    f"tau_cmd={np.array2string(tau_cmd, precision=3, suppress_small=True)} "
                    f"kp={args.kp:.3f} kd={args.kd:.3f}"
                )
                last_print = now

            next_tick += args.period
            while not STOP and time.perf_counter() < next_tick:
                pass

            if time.perf_counter() - next_tick > 0.2:
                next_tick = time.perf_counter()

    finally:
        if session_started:
            print("[bernie_grav] stopping session...")
            arm.stop_session(1.0)
        arm.disconnect()
        print("[bernie_grav] done")

    return 0


if __name__ == "__main__":
    sys.exit(main())
