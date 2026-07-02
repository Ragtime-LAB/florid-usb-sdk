#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import florid_usb

try:
    import pin as pin
except Exception:
    import pinocchio as pin


STOP = False
JOINT_COUNT = 6
DEFAULT_DEVICE = "/dev/ttyACM0"
CONTROL_RATE = 500.0
SETTLE_TIME = 1.0
FREQUENCY = 1.2
DURATION = 10.0
TAU_LIMIT = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0], dtype=np.float64)
JOINT_LIMITS = np.array(
    [
        [-np.pi, np.pi],
        [0.0, np.pi],
        [0.0, np.pi],
        [-1.3, 1.3],
        [-np.pi / 2.0, np.pi / 2.0],
        [-np.pi, np.pi],
    ],
    dtype=np.float64,
)
CENTER_POS = np.array([0.0, 0.8, 0.8, -0.3, 0.0, 0.0], dtype=np.float64)
# CENTER_POS = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
PRESET_AMPLITUDES = np.array([0.3, 0.4, 0.3, 0.4, 0.4, 0.0], dtype=np.float64)
# PRESET_AMPLITUDES = np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
PHASE_OFFSETS = np.zeros(JOINT_COUNT, dtype=np.float64)
# CT_KP = np.array([200.0, 230.0, 350.0, 200.0, 200.0, 0.0], dtype=np.float64)
# CT_KD = np.array([20.0, 23.0, 35.0, 20.0, 20.0, 0.0], dtype=np.float64)
CT_KP = np.array([500.0, 230.0, 550.0, 400.0, 300.0, 50.0], dtype=np.float64)
CT_KD = np.array([2.0, 4.0, 3.0, 2.0, 2.0, 0.50], dtype=np.float64)
MIT_KP = np.array([20.0, 20.0, 20.0, 20.0, 10.0, 20.0], dtype=np.float64)
MIT_KD = np.array([2.0, 2.0, 2.0, 2.0, 1.0, 2.0], dtype=np.float64)
MIT_SETTLE_KP = np.array([20.0, 21.0, 21.0, 6.0, 5.0, 1.0], dtype=np.float64)
MIT_SETTLE_KD = np.array([2.0, 2.0, 2.0, 0.9, 0.7, 0.1], dtype=np.float64)

BASE_DIR = Path(__file__).resolve().parent
URDF_PATH = BASE_DIR / "Ragtime_Willow_description" / "urdf" / "Ragtime_Willow_description.urdf"
PLOT_DIR = BASE_DIR / "plots"
ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def _signal_handler(signum, _frame) -> None:
    global STOP
    STOP = True
    print(f"\n[ct] received signal {signum}, stopping trajectory...")


class RobotDynamics:
    def __init__(self, urdf_path: Path) -> None:
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.q_index: dict[str, int] = {}
        self.v_index: dict[str, int] = {}
        for name in ARM_JOINT_NAMES:
            jid = self.model.getJointId(name)
            if not (0 < jid < len(self.model.names) and str(self.model.names[jid]) == name):
                raise RuntimeError(f"joint '{name}' not found in URDF")
            joint = self.model.joints[jid]
            self.q_index[name] = int(joint.idx_q() if callable(joint.idx_q) else joint.idx_q)
            self.v_index[name] = int(joint.idx_v() if callable(joint.idx_v) else joint.idx_v)
        self.arm_vidx = np.array([self.v_index[name] for name in ARM_JOINT_NAMES], dtype=np.int32)

    def qv(self, q_arm: np.ndarray, dq_arm: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        q_pin = pin.neutral(self.model)
        v_pin = np.zeros(self.model.nv, dtype=np.float64)
        for idx, name in enumerate(ARM_JOINT_NAMES):
            q_pin[self.q_index[name]] = q_arm[idx]
            v_pin[self.v_index[name]] = dq_arm[idx]
        return q_pin, v_pin

    def gravity(self, q_arm: np.ndarray) -> np.ndarray:
        q_pin, _ = self.qv(q_arm, np.zeros(JOINT_COUNT, dtype=np.float64))
        tau = pin.computeGeneralizedGravity(self.model, self.data, q_pin)
        return np.asarray(tau[self.arm_vidx], dtype=np.float64)

    def coriolis(self, q_arm: np.ndarray, dq_arm: np.ndarray) -> np.ndarray:
        q_pin, v_pin = self.qv(q_arm, dq_arm)
        tau_dyn = pin.rnea(
            self.model, self.data, q_pin, v_pin, np.zeros(self.model.nv, dtype=np.float64)
        )
        tau_g = pin.computeGeneralizedGravity(self.model, self.data, q_pin)
        return np.asarray((tau_dyn - tau_g)[self.arm_vidx], dtype=np.float64)

    def mass_matrix(self, q_arm: np.ndarray) -> np.ndarray:
        q_pin, _ = self.qv(q_arm, np.zeros(JOINT_COUNT, dtype=np.float64))
        mass = np.asarray(pin.crba(self.model, self.data, q_pin), dtype=np.float64)
        return mass[np.ix_(self.arm_vidx, self.arm_vidx)]

    def inertia_terms(self, q_arm: np.ndarray, ddq_arm: np.ndarray) -> np.ndarray:
        return self.mass_matrix(q_arm) @ np.asarray(ddq_arm, dtype=np.float64)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Computed torque sin trajectory example")
    parser.add_argument("device", nargs="?", default=DEFAULT_DEVICE)
    parser.add_argument("--duration", type=float, default=DURATION)
    parser.add_argument("--frequency", type=float, default=FREQUENCY)
    parser.add_argument("--control-rate", type=float, default=CONTROL_RATE)
    parser.add_argument("--print-interval", type=float, default=0.1)
    parser.add_argument("--start-duration", type=float, default=3.0)
    return parser.parse_args()


def as_f32(vec: np.ndarray) -> np.ndarray:
    return np.asarray(vec, dtype=np.float32)


def build_trajectory() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    lower_limits = JOINT_LIMITS[:, 0]
    upper_limits = JOINT_LIMITS[:, 1]
    safe_amplitudes = np.minimum(upper_limits - CENTER_POS, CENTER_POS - lower_limits) * 0.8
    amplitudes = np.minimum(safe_amplitudes, PRESET_AMPLITUDES)
    return lower_limits, upper_limits, amplitudes


def print_torque_summary(name: str, data: np.ndarray) -> None:
    abs_max = np.max(np.abs(data), axis=0)
    mean_abs = np.mean(np.abs(data), axis=0)
    print(f"\n{name}:")
    for idx in range(data.shape[1]):
        print(f"  J{idx + 1}: max_abs={abs_max[idx]:7.3f} Nm, mean_abs={mean_abs[idx]:7.3f} Nm")


def move_mit(arm: florid_usb.Arm, dynamics: RobotDynamics, q_target: np.ndarray, duration: float) -> None:
    status = arm.get_arm_status()
    q_start = np.asarray(status["q"], dtype=np.float64)
    dq_zero = np.zeros(JOINT_COUNT, dtype=np.float64)
    steps = max(2, int(duration * CONTROL_RATE))
    period = 1.0 / CONTROL_RATE
    next_tick = time.perf_counter()

    for step in range(steps):
        if STOP:
            return
        frac = (step + 1) / steps
        smooth = frac * frac * (3.0 - 2.0 * frac)
        q_des = q_start + smooth * (q_target - q_start)
        dq_des = np.zeros(JOINT_COUNT, dtype=np.float64)
        tau_ff = np.clip(dynamics.gravity(q_des), -TAU_LIMIT, TAU_LIMIT)
        arm.send_mit_command(
            as_f32(q_des),
            as_f32(dq_des),
            as_f32(tau_ff),
            as_f32(MIT_SETTLE_KP),
            as_f32(MIT_SETTLE_KD),
        )
        next_tick += period
        while not STOP and time.perf_counter() < next_tick:
            pass


def settle_in_mit(arm: florid_usb.Arm, dynamics: RobotDynamics) -> None:
    status = arm.get_arm_status()
    hold_pos = np.asarray(status["q"], dtype=np.float64)
    hold_vel = np.zeros(JOINT_COUNT, dtype=np.float64)
    hold_tau = np.clip(dynamics.gravity(hold_pos), -TAU_LIMIT, TAU_LIMIT)
    print(
        "[ct] settle hold | "
        f"hold={np.round(hold_pos[:4], 3)}, G_ff={np.round(hold_tau[:4], 3)}"
    )

    end_time = time.time() + SETTLE_TIME
    while time.time() < end_time and not STOP:
        arm.send_mit_command(
            as_f32(hold_pos),
            as_f32(hold_vel),
            as_f32(hold_tau),
            as_f32(MIT_SETTLE_KP),
            as_f32(MIT_SETTLE_KD),
        )
        time.sleep(0.01)


def save_results(result: dict[str, np.ndarray | list | float]) -> None:
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    json_path = PLOT_DIR / f"computed_torque_j12345_{timestamp}.json"
    png_path = PLOT_DIR / f"computed_torque_j12345_{timestamp}.png"

    payload = {
        key: value.tolist() if isinstance(value, np.ndarray) else value for key, value in result.items()
    }
    with json_path.open("w", encoding="utf-8") as file:
        json.dump(payload, file, ensure_ascii=False, indent=2)

    joint_count = result["desired_pos"].shape[1]
    fig, axes = plt.subplots(joint_count, 4, figsize=(24, 2.7 * joint_count), sharex="col")
    if joint_count == 1:
        axes = np.array([axes])
    pos_err = result["desired_pos"] - result["actual_pos"]

    for idx in range(joint_count):
        axes[idx, 0].plot(result["time"], result["desired_pos"][:, idx], label="desired", linewidth=1.5)
        axes[idx, 0].plot(result["time"], result["actual_pos"][:, idx], label="actual", linewidth=1.1)
        axes[idx, 0].set_ylabel(f"J{idx + 1} (rad)")
        axes[idx, 0].grid(True, linestyle="--", alpha=0.35)
        axes[idx, 0].legend(loc="upper right")

        axes[idx, 1].plot(result["time"], pos_err[:, idx], label="pos_error", linewidth=1.0)
        axes[idx, 1].grid(True, linestyle="--", alpha=0.35)
        axes[idx, 1].legend(loc="upper right")

        axes[idx, 2].plot(result["time"], result["gravity_torque"][:, idx], label="gravity", linewidth=1.0)
        axes[idx, 2].plot(result["time"], result["coriolis_torque"][:, idx], label="coriolis", linewidth=1.0)
        axes[idx, 2].plot(result["time"], result["inertia_ff_torque"][:, idx], label="M*ddq_des", linewidth=1.0)
        axes[idx, 2].plot(
            result["time"], result["feedback_torque"][:, idx], label="M*(Kp*e+Kd*de)", linewidth=1.0
        )
        axes[idx, 2].grid(True, linestyle="--", alpha=0.35)
        axes[idx, 2].legend(loc="upper right")

        axes[idx, 3].plot(result["time"], result["total_torque"][:, idx], label="total", linewidth=1.1)
        axes[idx, 3].plot(result["time"], result["commanded_torque"][:, idx], label="commanded", linewidth=1.1)
        axes[idx, 3].grid(True, linestyle="--", alpha=0.35)
        axes[idx, 3].legend(loc="upper right")

    axes[-1, 0].set_xlabel("Time (s)")
    axes[-1, 1].set_xlabel("Time (s)")
    axes[-1, 2].set_xlabel("Time (s)")
    axes[-1, 3].set_xlabel("Time (s)")
    axes[0, 0].set_title("Desired vs Actual")
    axes[0, 1].set_title("Tracking Error")
    axes[0, 2].set_title("Torque Components")
    axes[0, 3].set_title("Total vs Commanded")
    fig.suptitle("Computed Torque Sin Trajectory J1/J2/J3/J4/J5/J6")
    fig.tight_layout()
    fig.savefig(png_path, dpi=180, bbox_inches="tight")
    plt.close(fig)

    print(f"\n[ct] saved data:   {json_path}")
    print(f"[ct] saved figure: {png_path}")


def run_experiment(
    arm: florid_usb.Arm,
    dynamics: RobotDynamics,
    center_pos: np.ndarray,
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
    amplitudes: np.ndarray,
    duration: float,
    frequency: float,
    control_rate: float,
    print_interval: float,
) -> dict[str, np.ndarray | list | float]:
    print("\n[ct] start experiment: sin trajectory with computed torque")
    print(f"[ct] CT kp={CT_KP.tolist()}, kd={CT_KD.tolist()}")
    print(f"[ct] MIT kp={MIT_KP.tolist()}, kd={MIT_KD.tolist()}")

    dt = 1.0 / control_rate
    omega = 2.0 * np.pi * frequency
    t_log = []
    desired_pos_log = []
    actual_pos_log = []
    desired_vel_log = []
    actual_vel_log = []
    desired_acc_log = []
    gravity_log = []
    coriolis_log = []
    inertia_ff_log = []
    feedback_log = []
    total_log = []
    commanded_log = []

    start_time = time.time()
    next_tick = time.perf_counter()
    last_print = 0.0

    while (time.time() - start_time) < duration and not STOP:
        current_time = time.time() - start_time
        desired_pos = center_pos + amplitudes * np.sin(omega * current_time + PHASE_OFFSETS)
        desired_vel = amplitudes * omega * np.cos(omega * current_time + PHASE_OFFSETS)
        desired_acc = -amplitudes * (omega**2) * np.sin(omega * current_time + PHASE_OFFSETS)
        desired_pos = np.clip(desired_pos, lower_limits, upper_limits)

        status = arm.get_arm_status()
        actual_pos = np.asarray(status["q"], dtype=np.float64)
        actual_vel = np.asarray(status["dq"], dtype=np.float64)

        pos_err = desired_pos - actual_pos
        vel_err = desired_vel - actual_vel
        gravity_torque = dynamics.gravity(actual_pos)
        coriolis_torque = dynamics.coriolis(actual_pos, actual_vel)
        inertia_ff_torque = dynamics.inertia_terms(actual_pos, desired_acc)
        feedback_torque = dynamics.inertia_terms(actual_pos, CT_KP * pos_err + CT_KD * vel_err)
        total_torque = gravity_torque + coriolis_torque + inertia_ff_torque + feedback_torque
        commanded_torque = np.clip(total_torque, -TAU_LIMIT, TAU_LIMIT)

        # Track the desired joint trajectory while adding computed-torque feedforward.
        q_cmd = desired_pos
        dq_cmd = desired_vel
        kp_cmd = MIT_KP
        kd_cmd = MIT_KD

        arm.send_mit_command(
            as_f32(q_cmd),
            as_f32(dq_cmd),
            as_f32(commanded_torque),
            as_f32(kp_cmd),
            as_f32(kd_cmd),
        )

        t_log.append(current_time)
        desired_pos_log.append(desired_pos.copy())
        actual_pos_log.append(actual_pos.copy())
        desired_vel_log.append(desired_vel.copy())
        actual_vel_log.append(actual_vel.copy())
        desired_acc_log.append(desired_acc.copy())
        gravity_log.append(gravity_torque.copy())
        coriolis_log.append(coriolis_torque.copy())
        inertia_ff_log.append(inertia_ff_torque.copy())
        feedback_log.append(feedback_torque.copy())
        total_log.append(total_torque.copy())
        commanded_log.append(commanded_torque.copy())

        now = time.time()
        if now - last_print >= print_interval:
            print(
                "[ct] "
                f"t={current_time:5.2f}s | "
                f"J1 {desired_pos[0]: .3f}/{actual_pos[0]: .3f} "
                f"J2 {desired_pos[1]: .3f}/{actual_pos[1]: .3f} "
                f"J3 {desired_pos[2]: .3f}/{actual_pos[2]: .3f} "
                f"J4 {desired_pos[3]: .3f}/{actual_pos[3]: .3f} "
                f"| tau_j1={commanded_torque[0]: .3f} "
                f"max|tau|={np.max(np.abs(commanded_torque)):.3f}"
            )
            last_print = now

        next_tick += dt
        while not STOP and time.perf_counter() < next_tick:
            pass
        if time.perf_counter() - next_tick > 0.2:
            next_tick = time.perf_counter()

    print()
    return {
        "time": np.array(t_log),
        "desired_pos": np.array(desired_pos_log),
        "actual_pos": np.array(actual_pos_log),
        "desired_vel": np.array(desired_vel_log),
        "actual_vel": np.array(actual_vel_log),
        "desired_acc": np.array(desired_acc_log),
        "gravity_torque": np.array(gravity_log),
        "coriolis_torque": np.array(coriolis_log),
        "inertia_ff_torque": np.array(inertia_ff_log),
        "feedback_torque": np.array(feedback_log),
        "total_torque": np.array(total_log),
        "commanded_torque": np.array(commanded_log),
        "ct_kp": CT_KP.copy(),
        "ct_kd": CT_KD.copy(),
        "mit_kp": MIT_KP.copy(),
        "mit_kd": MIT_KD.copy(),
    }


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    dynamics = RobotDynamics(URDF_PATH)
    lower_limits, upper_limits, amplitudes = build_trajectory()
    print(f"[ct] URDF: {URDF_PATH}")
    print(f"[ct] center: {CENTER_POS}")
    print(f"[ct] amplitudes: {amplitudes}")
    print(f"[ct] frequency: {args.frequency} Hz, duration: {args.duration} s")

    cfg = florid_usb.Config()
    cfg.device = args.device
    arm = florid_usb.Arm(cfg)

    print(f"[ct] opening {args.device}")
    if not arm.connect():
        print("[ct] failed to open serial port")
        return 1

    session_started = False
    try:
        print("[ct] starting USB session...")
        if not arm.start_session(timeout=1.0):
            print("[ct] start_session failed")
            return 1
        session_started = True
        print("[ct] USB session active")

        print("[ct] moving to zero pose...")
        move_mit(arm, dynamics, np.zeros(JOINT_COUNT, dtype=np.float64), args.start_duration)
        time.sleep(SETTLE_TIME)
        print("[ct] moving to center pose...")
        move_mit(arm, dynamics, CENTER_POS, args.start_duration)
        time.sleep(SETTLE_TIME)

        result = run_experiment(
            arm,
            dynamics,
            CENTER_POS,
            lower_limits,
            upper_limits,
            amplitudes,
            args.duration,
            args.frequency,
            args.control_rate,
            args.print_interval,
        )
        if not STOP:
            settle_in_mit(arm, dynamics)

        print_torque_summary("total torque summary", result["total_torque"])
        print_torque_summary("feedback torque summary", result["feedback_torque"])
        save_results(result)

        if not STOP:
            print("[ct] return to center pose...")
            move_mit(arm, dynamics, CENTER_POS, args.start_duration)
            time.sleep(SETTLE_TIME)
            print("[ct] return to zero pose...")
            move_mit(arm, dynamics, np.zeros(JOINT_COUNT, dtype=np.float64), args.start_duration)
            time.sleep(SETTLE_TIME)
    finally:
        if session_started:
            arm.stop_session(1.0)
        arm.disconnect()
        print("[ct] done")

    return 0


if __name__ == "__main__":
    sys.exit(main())
