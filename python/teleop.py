#!/usr/bin/env python3
"""
4-arm teleoperation: 2 leaders (damped) + 2 followers (mirror leaders).

Usage:
    PYTHONPATH=python python3 python/teleop.py
    PYTHONPATH=python python3 python/teleop.py \
        --leader1-device /dev/ttyACM0 \
        --leader2-device /dev/ttyACM1 \
        --follower1-device /dev/ttyACM2 \
        --follower2-device /dev/ttyACM3
"""

from __future__ import annotations

import argparse
import signal
import sys
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
LEADER_KD = np.zeros(JOINT_COUNT, dtype=np.float64)
FOLLOWER_KP = np.array([30.0, 80.0, 70.0, 70.0, 60.0, 20.0], dtype=np.float64)
FOLLOWER_KD = np.array([0.7, 1.2, 1.1, 1.0, 1.0, 0.5], dtype=np.float64)
TAU_LIMIT_NM = np.array([30.0, 30.0, 30.0, 30.0, 30.0, 10.0], dtype=np.float64)

GRIPPER_KP = 10.0
GRIPPER_KD = 1.0
GRIPPER_TAU = 0.0

ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
BASE_DIR = Path(__file__).resolve().parent
URDF_PATH = BASE_DIR / "Ragtime_Willow_description" / "urdf" / "Ragtime_Willow_description.urdf"


def _signal_handler(signum, _frame) -> None:
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="4-arm teleoperation: 2 leaders (damped) + 2 followers"
    )
    parser.add_argument("--leader1-device", default="/dev/ttyACM0")
    parser.add_argument("--leader2-device", default="/dev/ttyACM1")
    parser.add_argument("--follower1-device", default="/dev/ttyACM2")
    parser.add_argument("--follower2-device", default="/dev/ttyACM3")
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
        "--sync-duration",
        type=float,
        default=DEFAULT_SYNC_DURATION_S,
        help="seconds used to move followers to leader poses before teleop",
    )
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=DEFAULT_STALE_TIMEOUT_S,
        help="exit if any arm status seq does not update within this many seconds",
    )
    parser.add_argument(
        "--nonzero-eps",
        type=float,
        default=DEFAULT_NONZERO_EPS,
        help="startup rejects an arm if q, dq, and tau all stay below this magnitude",
    )
    parser.add_argument(
        "--leader-kd",
        type=float,
        nargs=6,
        default=LEADER_KD.tolist(),
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="leader damping gains",
    )
    parser.add_argument(
        "--follower-kp",
        type=float,
        nargs=6,
        default=FOLLOWER_KP.tolist(),
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="follower MIT kp gains",
    )
    parser.add_argument(
        "--follower-kd",
        type=float,
        nargs=6,
        default=FOLLOWER_KD.tolist(),
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="follower MIT kd gains",
    )
    parser.add_argument(
        "--teleop-alpha",
        type=float,
        default=0.2,
        help="leader pose low-pass factor in (0, 1], lower is smoother",
    )
    parser.add_argument(
        "--leader-gravity-scale",
        type=float,
        nargs=6,
        default=[1.0] * JOINT_COUNT,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="per-joint leader gravity scale; negative values are allowed",
    )
    parser.add_argument(
        "--follower-gravity-scale",
        type=float,
        nargs=6,
        default=[1.0] * JOINT_COUNT,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="per-joint follower gravity scale; negative values are allowed",
    )
    parser.add_argument(
        "--gripper-kp",
        type=float,
        default=GRIPPER_KP,
        help="follower gripper proportional gain (leader gripper is always kp=0)",
    )
    parser.add_argument(
        "--gripper-kd",
        type=float,
        default=GRIPPER_KD,
        help="follower gripper derivative gain",
    )
    parser.add_argument(
        "--gripper-tau",
        type=float,
        default=GRIPPER_TAU,
        help="follower gripper constant feed-forward torque",
    )
    return parser.parse_args()


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


def stop_and_disconnect(arm: florid_usb.Arm, label: str) -> None:
    for _ in range(5):
        try:
            send_no_data(arm)
        except Exception as exc:
            print(f"[teleop] {label} no-data release failed: {exc}")
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


def sync_followers_to_leaders(
    leader1: florid_usb.Arm,
    leader2: florid_usb.Arm,
    follower1: florid_usb.Arm,
    follower2: florid_usb.Arm,
    gravity: HostGravityEstimator,
    freshness: FreshnessMonitor,
    leader_kd: np.ndarray,
    follower_kp: np.ndarray,
    follower_kd: np.ndarray,
    leader_gravity_scale: np.ndarray,
    follower_gravity_scale: np.ndarray,
    duration_s: float,
    period_s: float,
) -> None:
    l1_q, _, _ = get_state(leader1, "leader1", freshness)
    l2_q, _, _ = get_state(leader2, "leader2", freshness)
    f1_start_q, _, _ = get_state(follower1, "follower1", freshness)
    f2_start_q, _, _ = get_state(follower2, "follower2", freshness)

    print("[teleop] syncing followers to current leader poses")
    print(f"[teleop] follower1 target={np.array2string(l1_q, precision=3, suppress_small=True)}")
    print(f"[teleop] follower2 target={np.array2string(l2_q, precision=3, suppress_small=True)}")

    dq_zero = np.zeros(JOINT_COUNT, dtype=np.float64)
    steps = max(2, int(duration_s / period_s))
    next_tick = time.perf_counter()

    for step in range(steps):
        if STOP:
            return

        frac = (step + 1) / steps
        smooth = frac * frac * (3.0 - 2.0 * frac)
        f1_q_des = f1_start_q + smooth * (l1_q - f1_start_q)
        f2_q_des = f2_start_q + smooth * (l2_q - f2_start_q)

        l1_now_q, _, _ = get_state(leader1, "leader1", freshness)
        l2_now_q, _, _ = get_state(leader2, "leader2", freshness)
        f1_actual_q, _, _ = get_state(follower1, "follower1", freshness)
        f2_actual_q, _, _ = get_state(follower2, "follower2", freshness)

        l1_tau_ff = scaled_gravity(gravity, l1_now_q, leader_gravity_scale)
        l2_tau_ff = scaled_gravity(gravity, l2_now_q, leader_gravity_scale)
        f1_tau_ff = scaled_gravity(gravity, f1_actual_q, follower_gravity_scale)
        f2_tau_ff = scaled_gravity(gravity, f2_actual_q, follower_gravity_scale)

        send_leader_damping(leader1, l1_now_q, dq_zero, l1_tau_ff, leader_kd)
        send_leader_damping(leader2, l2_now_q, dq_zero, l2_tau_ff, leader_kd)
        follower1.send_mit_command(
            as_f32(f1_q_des),
            as_f32(dq_zero),
            as_f32(f1_tau_ff),
            as_f32(follower_kp),
            as_f32(follower_kd),
        )
        follower2.send_mit_command(
            as_f32(f2_q_des),
            as_f32(dq_zero),
            as_f32(f2_tau_ff),
            as_f32(follower_kp),
            as_f32(follower_kd),
        )

        next_tick += period_s
        while not STOP and time.perf_counter() < next_tick:
            pass
        if time.perf_counter() - next_tick > 0.2:
            next_tick = time.perf_counter()


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    if not (0.0 < args.teleop_alpha <= 1.0):
        raise SystemExit("--teleop-alpha must be in (0, 1]")
    if args.sync_duration <= 0.0:
        raise SystemExit("--sync-duration must be > 0")
    if args.stale_timeout <= 0.0:
        raise SystemExit("--stale-timeout must be > 0")
    if args.nonzero_eps < 0.0:
        raise SystemExit("--nonzero-eps must be >= 0")

    gravity = HostGravityEstimator(URDF_PATH)
    freshness = FreshnessMonitor(args.stale_timeout)

    leader1 = make_arm(args.leader1_device)
    leader2 = make_arm(args.leader2_device)
    follower1 = make_arm(args.follower1_device)
    follower2 = make_arm(args.follower2_device)

    leader1_started = False
    leader2_started = False
    follower1_started = False
    follower2_started = False

    leader_kd = np.asarray(args.leader_kd, dtype=np.float64)
    follower_kp = np.asarray(args.follower_kp, dtype=np.float64)
    follower_kd = np.asarray(args.follower_kd, dtype=np.float64)
    leader_gravity_scale = np.asarray(args.leader_gravity_scale, dtype=np.float64)
    follower_gravity_scale = np.asarray(args.follower_gravity_scale, dtype=np.float64)

    try:
        connect_and_start(leader1, "leader1", args.leader1_device)
        leader1_started = True
        connect_and_start(leader2, "leader2", args.leader2_device)
        leader2_started = True
        connect_and_start(follower1, "follower1", args.follower1_device)
        follower1_started = True
        connect_and_start(follower2, "follower2", args.follower2_device)
        follower2_started = True

        wait_for_fresh_status(
            [
                ("leader1", leader1, args.leader1_device),
                ("leader2", leader2, args.leader2_device),
                ("follower1", follower1, args.follower1_device),
                ("follower2", follower2, args.follower2_device),
            ],
            freshness,
            timeout_s=max(1.0, args.stale_timeout * 3.0),
            nonzero_eps=args.nonzero_eps,
        )

        dq_zero = np.zeros(JOINT_COUNT, dtype=np.float64)

        sync_followers_to_leaders(
            leader1=leader1,
            leader2=leader2,
            follower1=follower1,
            follower2=follower2,
            gravity=gravity,
            freshness=freshness,
            leader_kd=leader_kd,
            follower_kp=follower_kp,
            follower_kd=follower_kd,
            leader_gravity_scale=leader_gravity_scale,
            follower_gravity_scale=follower_gravity_scale,
            duration_s=args.sync_duration,
            period_s=args.period,
        )
        if STOP:
            return 0

        q1_filt: np.ndarray | None = None
        q2_filt: np.ndarray | None = None
        last_print = 0.0
        next_tick = time.perf_counter()

        print("[teleop] 4-arm teleop running, Ctrl+C to stop")
        print(f"[teleop] leader_kd={leader_kd.tolist()}")
        print(f"[teleop] follower_kp={follower_kp.tolist()} kd={follower_kd.tolist()}")
        print(
            f"[teleop] leader_gravity_scale={leader_gravity_scale.tolist()} "
            f"follower_gravity_scale={follower_gravity_scale.tolist()}"
        )
        print(
            f"[teleop] teleop_alpha={args.teleop_alpha} "
            f"stale_timeout={args.stale_timeout}"
        )
        print(
            f"[teleop] gripper: leader kp=0 kd=0 (free), "
            f"follower kp={args.gripper_kp} kd={args.gripper_kd} tau={args.gripper_tau}"
        )

        while not STOP:
            l1_status = get_status(leader1, "leader1", freshness)
            l2_status = get_status(leader2, "leader2", freshness)
            f1_status = get_status(follower1, "follower1", freshness)
            f2_status = get_status(follower2, "follower2", freshness)
            l1_q, _, _ = state_from_status(l1_status)
            l2_q, _, _ = state_from_status(l2_status)
            f1_q, _, _ = state_from_status(f1_status)
            f2_q, _, _ = state_from_status(f2_status)

            # Low-pass filter leader positions for smooth follower commands
            if q1_filt is None:
                q1_filt = l1_q.copy()
            else:
                q1_filt = (1.0 - args.teleop_alpha) * q1_filt + args.teleop_alpha * l1_q

            if q2_filt is None:
                q2_filt = l2_q.copy()
            else:
                q2_filt = (1.0 - args.teleop_alpha) * q2_filt + args.teleop_alpha * l2_q

            # Feed-forward gravity is based on each arm's own current posture.
            l1_tau_ff = scaled_gravity(gravity, l1_q, leader_gravity_scale)
            l2_tau_ff = scaled_gravity(gravity, l2_q, leader_gravity_scale)
            f1_tau_ff = scaled_gravity(gravity, f1_q, follower_gravity_scale)
            f2_tau_ff = scaled_gravity(gravity, f2_q, follower_gravity_scale)

            send_leader_damping(leader1, l1_q, dq_zero, l1_tau_ff, leader_kd)
            send_leader_damping(leader2, l2_q, dq_zero, l2_tau_ff, leader_kd)

            # Send follower commands: track leader position with torque feed-forward
            follower1.send_mit_command(
                as_f32(q1_filt),
                as_f32(dq_zero),
                as_f32(f1_tau_ff),
                as_f32(follower_kp),
                as_f32(follower_kd),
            )
            follower2.send_mit_command(
                as_f32(q2_filt),
                as_f32(dq_zero),
                as_f32(f2_tau_ff),
                as_f32(follower_kp),
                as_f32(follower_kd),
            )

            # ── Gripper ──────────────────────────────────────────────────────
            # Leaders: free MIT frame (kp=kd=0) using actual position as setpoint,
            # matching firmware dragMode pattern. Followers track leader position.
            l1_gs = leader1.get_gripper_status()
            l2_gs = leader2.get_gripper_status()

            leader1.send_gripper_command(
                q=l1_gs["q"], dq=0.0, tau=0.0, kp=0.0, kd=0.0, control_mode=1
            )
            leader2.send_gripper_command(
                q=l2_gs["q"], dq=0.0, tau=0.0, kp=0.0, kd=0.0, control_mode=1
            )

            follower1.send_gripper_command(
                q=l1_gs["q"], dq=0.0, tau=args.gripper_tau,
                kp=args.gripper_kp, kd=args.gripper_kd, control_mode=1,
            )
            follower2.send_gripper_command(
                q=l2_gs["q"], dq=0.0, tau=args.gripper_tau,
                kp=args.gripper_kp, kd=args.gripper_kd, control_mode=1,
            )

            now = time.perf_counter()
            if now - last_print >= args.print_interval:
                print(
                    "[teleop] "
                    f"{format_status('L1', l1_status)} | "
                    f"{format_status('F1', f1_status)} | "
                    f"{format_status('L2', l2_status)} | "
                    f"{format_status('F2', f2_status)}"
                )
                f1_gripper_status = follower1.get_gripper_status()
                f2_gripper_status = follower2.get_gripper_status()
                print(
                    "[teleop] "
                    f"grip L1={l1_gs['q']:.4f} F1={f1_gripper_status['q']:.4f} | "
                    f"grip L2={l2_gs['q']:.4f} F2={f2_gripper_status['q']:.4f}"
                )
                last_print = now

            next_tick += args.period
            while not STOP and time.perf_counter() < next_tick:
                pass
            if time.perf_counter() - next_tick > 0.2:
                next_tick = time.perf_counter()

        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"[teleop] error: {exc}")
        return 1
    finally:
        if follower2_started:
            stop_and_disconnect(follower2, "follower2")
        if follower1_started:
            stop_and_disconnect(follower1, "follower1")
        if leader2_started:
            stop_and_disconnect(leader2, "leader2")
        if leader1_started:
            stop_and_disconnect(leader1, "leader1")


if __name__ == "__main__":
    sys.exit(main())
