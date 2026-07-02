#!/usr/bin/env python3
"""Shared utilities for 4-arm teleoperation scripts (MIT / PosVel / Hybrid)."""

from __future__ import annotations

import signal
import time
from pathlib import Path

import numpy as np

import florid_usb

try:
    import pin as pin
except Exception:
    import pinocchio as pin


STOP = False
JOINT_COUNT = 6
DEFAULT_PERIOD_S = 0.002
DEFAULT_PRINT_INTERVAL_S = 0.5
DEFAULT_SYNC_DURATION_S = 4.0
DEFAULT_STALE_TIMEOUT_S = 0.1
DEFAULT_NONZERO_EPS = 1.0e-4

LEADER_KP = np.zeros(JOINT_COUNT, dtype=np.float64)
TAU_LIMIT_NM = np.array([30.0, 30.0, 30.0, 30.0, 30.0, 10.0], dtype=np.float64)

ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
BASE_DIR = Path(__file__).resolve().parent
URDF_PATH = BASE_DIR / "Ragtime_Willow_description" / "urdf" / "Ragtime_Willow_description.urdf"


def signal_handler(signum, _frame) -> None:
    global STOP
    STOP = True
    print(f"\n[teleop] received signal {signum}, stopping...")


def as_f32(vec: np.ndarray) -> np.ndarray:
    return np.asarray(vec, dtype=np.float32)


def clip_tau(tau: np.ndarray) -> np.ndarray:
    return np.clip(np.asarray(tau, dtype=np.float64), -TAU_LIMIT_NM, TAU_LIMIT_NM)


def scaled_gravity(
    gravity: "HostGravityEstimator",
    q: np.ndarray,
    scale: np.ndarray,
) -> np.ndarray:
    return clip_tau(np.asarray(scale, dtype=np.float64) * gravity.gravity(q))


class HostGravityEstimator:
    def __init__(self, urdf_path: Path) -> None:
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.q_index: dict[str, int] = {}
        self.v_index: dict[str, int] = {}
        for name in ARM_JOINT_NAMES:
            jid = self.model.getJointId(name)
            joint = self.model.joints[jid]
            self.q_index[name] = int(joint.idx_q() if callable(joint.idx_q) else joint.idx_q)
            self.v_index[name] = int(joint.idx_v() if callable(joint.idx_v) else joint.idx_v)
        self.arm_vidx = np.array([self.v_index[name] for name in ARM_JOINT_NAMES], dtype=np.int32)

    def gravity(self, q_arm: np.ndarray) -> np.ndarray:
        q_pin = pin.neutral(self.model)
        for idx, name in enumerate(ARM_JOINT_NAMES):
            q_pin[self.q_index[name]] = q_arm[idx]
        tau = pin.computeGeneralizedGravity(self.model, self.data, q_pin)
        return np.asarray(tau[self.arm_vidx], dtype=np.float64)


def make_arm(device: str) -> florid_usb.Arm:
    cfg = florid_usb.Config()
    cfg.device = device
    return florid_usb.Arm(cfg)


def connect_and_start(arm: florid_usb.Arm, label: str, device: str) -> None:
    print(f"[teleop] opening {label} on {device}")
    if not arm.connect():
        raise RuntimeError(f"failed to open {label} serial port: {device}")
    try:
        print(f"[teleop] {label} serial connected")
        if not arm.start_session(timeout=1.0):
            raise RuntimeError(f"failed to start {label} USB session")
        print(f"[teleop] {label} USB session active")
    except Exception:
        arm.disconnect()
        raise


def send_no_data(arm: florid_usb.Arm) -> None:
    zero = np.zeros(JOINT_COUNT, dtype=np.float32)
    arm.send_mit_command(zero, zero, zero, zero, zero, control_mode=0)


def send_release(arm: florid_usb.Arm, mode: str = "mit") -> None:
    if mode == "mit":
        send_no_data(arm)
        return
    dq_zero = np.zeros(JOINT_COUNT, dtype=np.float32)
    try:
        status = arm.get_arm_status()
        q = np.asarray(status["q"], dtype=np.float32)
    except Exception:
        q = np.zeros(JOINT_COUNT, dtype=np.float32)
    if mode == "posvel":
        arm.send_posvel_command(q, dq_zero)
    elif mode == "hybrid":
        arm.send_hybrid_command(q, dq_zero, dq_zero)


def stop_and_disconnect(arm: florid_usb.Arm, label: str, mode: str = "mit") -> None:
    for _ in range(5):
        try:
            send_release(arm, mode)
        except Exception as exc:
            print(f"[teleop] {label} release failed: {exc}")
            break
        time.sleep(0.002)
    try:
        stopped = arm.stop_session(1.0)
        print(f"[teleop] {label} stop_session={stopped}")
    except Exception as exc:
        print(f"[teleop] {label} stop_session error: {exc}")
    arm.disconnect()
    print(f"[teleop] {label} disconnected")


class FreshnessMonitor:
    def __init__(self, timeout_s: float) -> None:
        self.timeout_s = timeout_s
        self.last_seq: dict[str, int] = {}
        self.last_change: dict[str, float] = {}

    def check(self, label: str, status: dict, now: float) -> None:
        seq = int(status["seq"])
        if label not in self.last_seq or seq != self.last_seq[label]:
            self.last_seq[label] = seq
            self.last_change[label] = now
            return
        age_s = now - self.last_change[label]
        if age_s > self.timeout_s:
            raise RuntimeError(
                f"{label} status stale for {age_s:.3f}s "
                f"(seq={seq}, timeout={self.timeout_s:.3f}s)"
            )


def get_status(arm: florid_usb.Arm, label: str, freshness: FreshnessMonitor) -> dict:
    status = arm.get_arm_status()
    freshness.check(label, status, time.perf_counter())
    return status


def state_from_status(status: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    q = np.asarray(status["q"], dtype=np.float64)
    dq = np.asarray(status["dq"], dtype=np.float64)
    tau = np.asarray(status["tau"], dtype=np.float64)
    return q, dq, tau


def status_is_nonzero(status: dict, eps: float) -> bool:
    q, dq, tau = state_from_status(status)
    return (
        np.max(np.abs(q)) > eps
        or np.max(np.abs(dq)) > eps
        or np.max(np.abs(tau)) > eps
    )


def format_status(label: str, status: dict, device: str | None = None) -> str:
    q, dq, tau = state_from_status(status)
    prefix = f"{label}({device})" if device is not None else label
    return (
        f"{prefix}_mode={status['mode']} "
        f"{prefix}_seq={status['seq']} "
        f"{prefix}_q={np.array2string(q, precision=3, suppress_small=True)} "
        f"{prefix}_dq={np.array2string(dq, precision=3, suppress_small=True)} "
        f"{prefix}_tau={np.array2string(tau, precision=3, suppress_small=True)}"
    )


def get_state(
    arm: florid_usb.Arm,
    label: str,
    freshness: FreshnessMonitor,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    return state_from_status(get_status(arm, label, freshness))


def wait_for_fresh_status(
    arms: list[tuple[str, florid_usb.Arm, str]],
    freshness: FreshnessMonitor,
    timeout_s: float,
    nonzero_eps: float,
) -> None:
    deadline = time.perf_counter() + timeout_s
    initial_seq = {label: int(arm.get_arm_status()["seq"]) for label, arm, _ in arms}
    seen: set[str] = set()
    latest: dict[str, dict] = {}
    while len(seen) < len(arms):
        if STOP:
            return
        for label, arm, _device in arms:
            status = arm.get_arm_status()
            latest[label] = status
            freshness.check(label, status, time.perf_counter())
            if int(status["seq"]) != initial_seq[label] and status_is_nonzero(status, nonzero_eps):
                seen.add(label)
        if time.perf_counter() > deadline:
            missing = [label for label, _arm, _device in arms if label not in seen]
            details = "; ".join(
                format_status(label, latest.get(label, arm.get_arm_status()), device)
                for label, arm, device in arms
                if label in missing
            )
            raise RuntimeError(
                f"timed out waiting for nonzero fresh status: {', '.join(missing)}; {details}"
            )
        time.sleep(0.001)


def send_leader_damping(
    leader: florid_usb.Arm,
    q: np.ndarray,
    dq_zero: np.ndarray,
    tau_ff: np.ndarray,
    leader_kd: np.ndarray,
) -> None:
    leader.send_mit_command(
        as_f32(q),
        as_f32(dq_zero),
        as_f32(tau_ff),
        as_f32(LEADER_KP),
        as_f32(leader_kd),
    )


def switch_joints_to_mode(arm: florid_usb.Arm, label: str, mode: str) -> None:
    for jid in range(JOINT_COUNT):
        if not arm.set_motor_control_mode(jid, mode, timeout=1.0):
            raise RuntimeError(f"failed to switch {label} joint {jid} to {mode}")
    print(f"[teleop] {label} switched to {mode} mode")


import threading


class TunableParams:
    """Thread-safe container for runtime-tunable control parameters."""

    def __init__(
        self,
        follower_kp: np.ndarray,
        follower_kd: np.ndarray,
        leader_kd: np.ndarray,
        teleop_alpha: float,
        leader_gravity_scale: np.ndarray,
        follower_gravity_scale: np.ndarray,
        gripper_kp: float,
        gripper_kd: float,
        gripper_tau: float,
    ) -> None:
        self._lock = threading.Lock()
        self._follower_kp = follower_kp.copy()
        self._follower_kd = follower_kd.copy()
        self._leader_kd = leader_kd.copy()
        self._teleop_alpha = teleop_alpha
        self._leader_gravity_scale = leader_gravity_scale.copy()
        self._follower_gravity_scale = follower_gravity_scale.copy()
        self._gripper_kp = gripper_kp
        self._gripper_kd = gripper_kd
        self._gripper_tau = gripper_tau
        self._loop_counter = 0
        self._freq_prev_count = 0
        self._freq_prev_time = time.perf_counter()
        self.real_freq_hz = 0.0

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "follower_kp": self._follower_kp.copy(),
                "follower_kd": self._follower_kd.copy(),
                "leader_kd": self._leader_kd.copy(),
                "teleop_alpha": self._teleop_alpha,
                "leader_gravity_scale": self._leader_gravity_scale.copy(),
                "follower_gravity_scale": self._follower_gravity_scale.copy(),
                "gripper_kp": self._gripper_kp,
                "gripper_kd": self._gripper_kd,
                "gripper_tau": self._gripper_tau,
            }

    def update_follower_kp(self, values: np.ndarray) -> None:
        with self._lock:
            self._follower_kp[:] = values

    def update_follower_kd(self, values: np.ndarray) -> None:
        with self._lock:
            self._follower_kd[:] = values

    def update_leader_kd(self, values: np.ndarray) -> None:
        with self._lock:
            self._leader_kd[:] = values

    def update_teleop_alpha(self, value: float) -> None:
        with self._lock:
            self._teleop_alpha = value

    def update_leader_gravity_scale(self, values: np.ndarray) -> None:
        with self._lock:
            self._leader_gravity_scale[:] = values

    def update_follower_gravity_scale(self, values: np.ndarray) -> None:
        with self._lock:
            self._follower_gravity_scale[:] = values

    def update_gripper_kp(self, value: float) -> None:
        with self._lock:
            self._gripper_kp = value

    def update_gripper_kd(self, value: float) -> None:
        with self._lock:
            self._gripper_kd = value

    def update_gripper_tau(self, value: float) -> None:
        with self._lock:
            self._gripper_tau = value

    def increment_loop(self) -> None:
        with self._lock:
            self._loop_counter += 1

    def poll_freq(self) -> float:
        now = time.perf_counter()
        with self._lock:
            count = self._loop_counter
            delta = count - self._freq_prev_count
            self._freq_prev_count = count
            dt = now - self._freq_prev_time
            self._freq_prev_time = now
        if dt > 0:
            self.real_freq_hz = delta / dt
        return self.real_freq_hz
