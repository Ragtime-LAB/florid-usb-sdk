#!/usr/bin/env python3
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
DEFAULT_LEADER_DEVICE = "/dev/ttyACM0"
DEFAULT_FOLLOWER_DEVICE = "/dev/ttyACM1"
DEFAULT_PERIOD_S = 0.002
DEFAULT_PRINT_INTERVAL_S = 0.5

LEADER_KP = np.zeros(JOINT_COUNT, dtype=np.float64)
LEADER_KD = np.array([0.8, 1.5, 1.5, 0.6, 0.3, 0.1], dtype=np.float64)
FOLLOWER_KP = np.array([10.0, 21.0, 21.0, 16.0, 13.0, 1.0], dtype=np.float64)
FOLLOWER_KD = np.array([1.0, 2.0, 2.0, 0.9, 0.8, 0.1], dtype=np.float64)
TAU_LIMIT_NM = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0], dtype=np.float64)
FORCE_REFLECT_THRESHOLD_NM = np.array([0.5, 1.0, 1.0, 0.5, 0.3, 0.3], dtype=np.float64)
MAX_DQ_RAD_S = np.array([2.5, 2.5, 2.5, 3.0, 4.0, 4.0], dtype=np.float64)
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
        description="Master-slave teleoperation example for two Florid USB arms"
    )
    parser.add_argument("--leader-device", default=DEFAULT_LEADER_DEVICE)
    parser.add_argument("--follower-device", default=DEFAULT_FOLLOWER_DEVICE)
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
        default=2.5,
        help="seconds used to move follower to leader pose before teleop",
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
        "--force-feedback-gain",
        type=float,
        default=0.0,
        help="reflect follower measured joint torque back to leader, default disabled",
    )
    parser.add_argument(
        "--force-threshold",
        type=float,
        nargs=6,
        default=FORCE_REFLECT_THRESHOLD_NM.tolist(),
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="joint torque deadband for force reflection",
    )
    parser.add_argument(
        "--skip-sync",
        action="store_true",
        help="skip follower move-to-leader phase before teleop",
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


def stop_and_disconnect(arm: florid_usb.Arm, label: str) -> None:
    try:
        arm.stop_session(1.0)
    except Exception:
        pass
    arm.disconnect()
    print(f"[teleop] {label} disconnected")


def get_state(arm: florid_usb.Arm) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    status = arm.get_arm_status()
    q = np.asarray(status["q"], dtype=np.float64)
    dq = np.asarray(status["dq"], dtype=np.float64)
    tau = np.asarray(status["tau"], dtype=np.float64)
    return q, dq, tau


def get_status(arm: florid_usb.Arm) -> dict:
    return arm.get_arm_status()


def hold_follower_pose(
    follower: florid_usb.Arm,
    gravity: HostGravityEstimator,
    q_target: np.ndarray,
    kp: np.ndarray,
    kd: np.ndarray,
    seconds: float,
    period_s: float,
) -> None:
    dq_zero = np.zeros(JOINT_COUNT, dtype=np.float64)
    end_time = time.perf_counter() + seconds
    while not STOP and time.perf_counter() < end_time:
        actual_q, _, _ = get_state(follower)
        tau_ff = clip_tau(gravity.gravity(actual_q))
        follower.send_mit_command(
            as_f32(q_target),
            as_f32(dq_zero),
            as_f32(tau_ff),
            as_f32(kp),
            as_f32(kd),
        )
        time.sleep(period_s)


def sync_follower_to_leader(
    leader: florid_usb.Arm,
    follower: florid_usb.Arm,
    gravity: HostGravityEstimator,
    kp: np.ndarray,
    kd: np.ndarray,
    duration_s: float,
    period_s: float,
) -> None:
    leader_q, _, _ = get_state(leader)
    follower_q, _, _ = get_state(follower)
    dq_zero = np.zeros(JOINT_COUNT, dtype=np.float64)

    print(
        "[teleop] syncing follower to leader pose "
        f"{np.array2string(leader_q, precision=3, suppress_small=True)}"
    )
    steps = max(2, int(duration_s / period_s))
    next_tick = time.perf_counter()

    for step in range(steps):
        if STOP:
            return
        frac = (step + 1) / steps
        smooth = frac * frac * (3.0 - 2.0 * frac)
        q_des = follower_q + smooth * (leader_q - follower_q)
        follower_actual_q, _, _ = get_state(follower)
        tau_ff = clip_tau(gravity.gravity(follower_actual_q))
        follower.send_mit_command(
            as_f32(q_des),
            as_f32(dq_zero),
            as_f32(tau_ff),
            as_f32(kp),
            as_f32(kd),
        )
        next_tick += period_s
        while not STOP and time.perf_counter() < next_tick:
            pass

    hold_follower_pose(
        follower,
        gravity,
        leader_q,
        kp,
        kd,
        seconds=0.3,
        period_s=period_s,
    )


def teleop_loop(
    leader: florid_usb.Arm,
    follower: florid_usb.Arm,
    gravity: HostGravityEstimator,
    period_s: float,
    print_interval_s: float,
    leader_kd: np.ndarray,
    follower_kp: np.ndarray,
    follower_kd: np.ndarray,
    teleop_alpha: float,
    force_feedback_gain: float,
    force_threshold: np.ndarray,
) -> None:
    dq_zero = np.zeros(JOINT_COUNT, dtype=np.float64)
    q_des_filt: np.ndarray | None = None
    last_print = 0.0
    next_tick = time.perf_counter()

    print("[teleop] teleop running, Ctrl+C to stop")
    print(f"[teleop] leader={leader_kd.tolist()} kd-only, follower kp={follower_kp.tolist()} kd={follower_kd.tolist()}")
    print(f"[teleop] force_feedback_gain={force_feedback_gain:.3f}")

    while not STOP:
        leader_status = get_status(leader)
        follower_status = get_status(follower)

        leader_q = np.asarray(leader_status["q"], dtype=np.float64)
        leader_dq = np.asarray(leader_status["dq"], dtype=np.float64)
        leader_tau_meas = np.asarray(leader_status["tau"], dtype=np.float64)
        follower_q = np.asarray(follower_status["q"], dtype=np.float64)
        follower_dq = np.asarray(follower_status["dq"], dtype=np.float64)
        follower_tau_meas = np.asarray(follower_status["tau"], dtype=np.float64)

        if q_des_filt is None:
            q_des_filt = leader_q.copy()
        else:
            q_des_filt = (1.0 - teleop_alpha) * q_des_filt + teleop_alpha * leader_q
        follower_q_des = q_des_filt
        follower_dq_des = dq_zero

        leader_tau_ff = clip_tau(gravity.gravity(leader_q))
        follower_tau_ff = clip_tau(gravity.gravity(follower_q))
        if force_feedback_gain > 0.0:
            reflected_tau = -force_feedback_gain * follower_tau_meas
            reflected_tau[np.abs(reflected_tau) < force_threshold] = 0.0
            leader_tau_ff = clip_tau(leader_tau_ff + reflected_tau)

        leader.send_mit_command(
            as_f32(leader_q),
            as_f32(dq_zero),
            as_f32(leader_tau_ff),
            as_f32(LEADER_KP),
            as_f32(leader_kd),
        )
        follower.send_mit_command(
            as_f32(follower_q_des),
            as_f32(follower_dq_des),
            as_f32(follower_tau_ff),
            as_f32(follower_kp),
            as_f32(follower_kd),
        )

        now = time.perf_counter()
        if now - last_print >= print_interval_s:
            q_err = leader_q - follower_q
            print(
                "[teleop] "
                f"leader_seq={leader_status['seq']} "
                f"follower_seq={follower_status['seq']} "
                f"leader_q={np.array2string(leader_q, precision=3, suppress_small=True)} "
                f"follower_q={np.array2string(follower_q, precision=3, suppress_small=True)} "
                f"q_err={np.array2string(q_err, precision=3, suppress_small=True)}"
            )
            last_print = now

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

    leader = make_arm(args.leader_device)
    follower = make_arm(args.follower_device)
    gravity = HostGravityEstimator(URDF_PATH)

    leader_started = False
    follower_started = False

    leader_kd = np.asarray(args.leader_kd, dtype=np.float64)
    follower_kp = np.asarray(args.follower_kp, dtype=np.float64)
    follower_kd = np.asarray(args.follower_kd, dtype=np.float64)
    force_threshold = np.asarray(args.force_threshold, dtype=np.float64)

    try:
        connect_and_start(leader, "leader", args.leader_device)
        leader_started = True
        connect_and_start(follower, "follower", args.follower_device)
        follower_started = True

        if not args.skip_sync:
            sync_follower_to_leader(
                leader,
                follower,
                gravity,
                follower_kp,
                follower_kd,
                duration_s=args.sync_duration,
                period_s=args.period,
            )
            if STOP:
                return 0

        teleop_loop(
            leader=leader,
            follower=follower,
            gravity=gravity,
            period_s=args.period,
            print_interval_s=args.print_interval,
            leader_kd=leader_kd,
            follower_kp=follower_kp,
            follower_kd=follower_kd,
            teleop_alpha=args.teleop_alpha,
            force_feedback_gain=args.force_feedback_gain,
            force_threshold=force_threshold,
        )
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"[teleop] error: {exc}")
        return 1
    finally:
        if follower_started:
            stop_and_disconnect(follower, "follower")
        if leader_started:
            stop_and_disconnect(leader, "leader")


if __name__ == "__main__":
    sys.exit(main())
