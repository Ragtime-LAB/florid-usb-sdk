#!/usr/bin/env python3
"""
Gravity compensation control example for Ragtime Willow.

This script uses the existing USB MIT control path:
1. Connect to the arm
2. Start a USB session
3. Read joint state continuously
4. Send MIT commands at a fixed rate

Important:
- This script computes gravity on the host and sends it explicitly through
  MIT `tau`.

Optional:
- If `pinocchio` is installed, the script can load the local URDF and print a
  host-side gravity estimate for debugging or sending.

Usage:
    PYTHONPATH=python python3 python/gravity_compensation_control.py
    PYTHONPATH=python python3 python/gravity_compensation_control.py /dev/ttyACM0
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
DEFAULT_KD = 0.4
TAU_LIMIT_NM = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0], dtype=np.float32)


def _signal_handler(signum, _frame) -> None:
    global STOP
    STOP = True
    print(f"\n[grav] received signal {signum}, stopping...")


class HostGravityEstimator:
    def __init__(self) -> None:
        self._pin = None
        self._model = None
        self._data = None
        self._urdf_path = (
            Path(__file__).resolve().parent
            / "Ragtime_Willow_description"
            / "urdf"
            / "Ragtime_Willow_description.urdf"
        )

    def available(self) -> bool:
        try:
            import pinocchio as pin  # type: ignore

            self._pin = pin
        except ImportError:
            return False

        if self._model is None:
            self._model = self._pin.buildModelFromUrdf(str(self._urdf_path))
            self._data = self._model.createData()
            if self._model.nq != JOINT_COUNT:
                raise RuntimeError(
                    f"unexpected model.nq={self._model.nq}, expected {JOINT_COUNT}"
                )
        return True

    def compute(self, q: np.ndarray) -> np.ndarray:
        if not self.available():
            raise RuntimeError("pinocchio is not installed")
        gravity = self._pin.computeGeneralizedGravity(
            self._model, self._data, np.asarray(q, dtype=np.float64)
        )
        return np.asarray(gravity, dtype=np.float32)

    @property
    def urdf_path(self) -> Path:
        return self._urdf_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Ragtime Willow gravity compensation example"
    )
    parser.add_argument("device", nargs="?", default=DEFAULT_DEVICE)
    parser.add_argument(
        "--period",
        type=float,
        default=DEFAULT_PERIOD_S,
        help="control period in seconds, default: 0.002 (500 Hz)",
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
        "--print-host-gravity",
        action="store_true",
        help="load URDF and print host-side gravity estimate from pinocchio",
    )
    return parser.parse_args()


def as_f32(vec: np.ndarray) -> np.ndarray:
    return np.asarray(vec, dtype=np.float32)


def print_motor_feedback(arm: florid_usb.Arm) -> None:
    feedback = arm.get_motor_feedback()
    motors = feedback["motors"]
    for motor in motors[:JOINT_COUNT]:
        print(
            "[grav] motor "
            f"j{motor['joint_id']} enabled={motor['enabled']} "
            f"status={motor['device_status']} "
            f"pos={motor['position_rad']:.3f} "
            f"vel={motor['speed_rad_s']:.3f} "
            f"tau={motor['torque_nm']:.3f}"
        )


def main() -> int:
    args = parse_args()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    estimator = HostGravityEstimator()
    if not estimator.available():
        print("[grav] pinocchio not installed, host gravity estimation unavailable")
        return 1
    print(f"[grav] host gravity URDF: {estimator.urdf_path}")

    cfg = florid_usb.Config()
    cfg.device = args.device
    arm = florid_usb.Arm(cfg)

    print(f"[grav] opening {args.device}")
    if not arm.connect():
        print("[grav] failed to open serial port")
        return 1
    print("[grav] serial connected")

    session_started = False
    try:
        print("[grav] starting USB session...")
        if not arm.start_session(timeout=1.0):
            print("[grav] start_session failed")
            return 1
        session_started = True
        print("[grav] USB session active")

        status = arm.get_arm_status()
        q_cmd = as_f32(status["q"])
        dq_cmd = np.zeros(JOINT_COUNT, dtype=np.float32)
        kp = np.full(JOINT_COUNT, args.kp, dtype=np.float32)
        kd = np.full(JOINT_COUNT, args.kd, dtype=np.float32)
        tau_cmd = np.zeros(JOINT_COUNT, dtype=np.float32)
        print_motor_feedback(arm)

        print("[grav] running gravity compensation, Ctrl+C to stop")
        next_tick = time.perf_counter()
        last_print = 0.0

        while not STOP:
            status = arm.get_arm_status()
            q = as_f32(status["q"])
            dq = as_f32(status["dq"])
            tau_meas = as_f32(status["tau"])

            # Keep the current joint targets equal to the measured pose so the
            # MIT packet only maintains a compliant gravity-balanced state.
            q_cmd[:] = q
            tau_cmd.fill(0.0)

            host_gravity = np.clip(
                estimator.compute(q), -TAU_LIMIT_NM, TAU_LIMIT_NM
            )
            tau_cmd[:] = host_gravity

            arm.send_mit_command(q_cmd, dq_cmd, tau_cmd, kp, kd)

            now = time.perf_counter()
            if now - last_print >= args.print_interval:
                print(
                    "[grav] "
                    f"q={np.array2string(q, precision=3, suppress_small=True)} "
                    f"dq={np.array2string(dq, precision=3, suppress_small=True)} "
                    f"tau_meas={np.array2string(tau_meas, precision=3, suppress_small=True)} "
                    f"tau_cmd={np.array2string(tau_cmd, precision=3, suppress_small=True)} "
                    f"kp={args.kp:.3f} kd={args.kd:.3f}"
                )
                if args.print_host_gravity:
                    print(
                        "[grav] "
                        f"tau_host={np.array2string(host_gravity, precision=3, suppress_small=True)}"
                    )
                last_print = now

            next_tick += args.period
            while not STOP and time.perf_counter() < next_tick:
                pass

            if time.perf_counter() - next_tick > 0.2:
                next_tick = time.perf_counter()

    finally:
        if session_started:
            print("[grav] stopping session...")
            arm.stop_session(1.0)
        arm.disconnect()
        print("[grav] done")

    return 0


if __name__ == "__main__":
    sys.exit(main())
