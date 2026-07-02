#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
import select
import signal
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

import florid_usb

try:
    import pin as pin
except Exception:
    import pinocchio as pin

try:
    from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler
except ModuleNotFoundError as exc:
    raise SystemExit(
        "missing dependency: scservo_sdk\n"
        "install with: python -m pip install feetech-servo-sdk"
    ) from exc


STOP = False
JOINT_COUNT = 6
DEFAULT_ARM_DEVICE = "/dev/ttyACM0"
DEFAULT_GELLO_PORT = "/dev/ttyACM1"
CONTROL_RATE = 400.0
PRINT_INTERVAL = 0.5
RAD_PER_TICK = 2.0 * math.pi / 4096.0
FEETECH_BAUD = 1_000_000
FEETECH_TICK_JUMP_TH = 800
FEETECH_BAD_FRAME_HOLD = 30
TELEOP_POS_ALPHA = 0.12
TELEOP_POS_DEADBAND = 0.008

ARM_IDS = [2, 3, 4, 5, 1, 6]
GRIPPER_ID = 7
SERVO_IDS = ARM_IDS + [GRIPPER_ID]
ARM_MAP = {
    1: {"offset": 2048, "sign": +1, "gear": 1.0},
    2: {"offset": 2419, "sign": -1, "gear": 1.0},
    3: {"offset": 5, "sign": -1, "gear": 1.0},
    4: {"offset": 2030, "sign": +1, "gear": 1.0},
    5: {"offset": 2048, "sign": -1, "gear": 1.0},
    6: {"offset": 2048, "sign": -1, "gear": 1.0},
}

TAU_LIMIT = np.array([26.90, 26.90, 26.90, 6.90, 6.90, 6.90], dtype=np.float64)
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
CT_KP = np.array([200.0, 180.0, 250.0, 200.0, 200.0, 0.0], dtype=np.float64)
CT_KD = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 0.0], dtype=np.float64)
MIT_KP = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
MIT_KD = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

BASE_DIR = Path(__file__).resolve().parent
URDF_PATH = BASE_DIR / "Ragtime_Willow_description" / "urdf" / "Ragtime_Willow_description.urdf"
ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

CALIBRATED_ARM_OFFSET: dict[int, int] = {}
CALIBRATED_GRIPPER_ZERO: int = 0


def _signal_handler(signum, _frame) -> None:
    global STOP
    STOP = True
    print(f"\n[gello] received signal {signum}, stopping...")


def wrap_ticks(dt: int) -> int:
    return ((dt + 2048) % 4096) - 2048


def as_f32(vec: np.ndarray) -> np.ndarray:
    return np.asarray(vec, dtype=np.float32)


class ServoController:
    PRESENT_POSITION_ADDR = 56

    def __init__(self, servo_ids: list[int], port: str, baudrate: int) -> None:
        self.servo_ids = list(servo_ids)
        self.port = PortHandler(port)
        self.packet_handler = PacketHandler(0)
        self.baudrate = baudrate

    def connect(self) -> None:
        if not self.port.openPort():
            raise RuntimeError(f"failed to open port: {self.port.getPortName()}")
        if not self.port.setBaudRate(self.baudrate):
            raise RuntimeError(f"failed to set baudrate: {self.baudrate}")

    def disconnect(self) -> None:
        if self.port is not None and self.port.is_open:
            self.port.closePort()

    def read_positions(self, servo_ids: list[int]) -> dict[int, int]:
        ticks: dict[int, int] = {}
        for sid in servo_ids:
            pos, result, error = self.packet_handler.read2ByteTxRx(
                self.port,
                int(sid),
                self.PRESENT_POSITION_ADDR,
            )
            if result != COMM_SUCCESS:
                raise RuntimeError(self.packet_handler.getTxRxResult(result))
            if error != 0:
                raise RuntimeError(self.packet_handler.getRxPacketError(error))
            ticks[int(sid)] = int(pos)
        return ticks


class FeetechUnwrapper:
    def __init__(self, servo_ids: list[int]) -> None:
        self.servo_ids = list(servo_ids)
        self._inited = False
        self.raw_prev: dict[int, int] = {}
        self.tick_unwrapped: dict[int, int] = {}

    def reset(self) -> None:
        self._inited = False
        self.raw_prev.clear()
        self.tick_unwrapped.clear()

    def init_from_frame(self, ticks: dict[int, int]) -> None:
        for sid in self.servo_ids:
            t = int(ticks[sid])
            self.raw_prev[sid] = t
            self.tick_unwrapped[sid] = t
        self._inited = True

    def update(self, ticks: dict[int, int]) -> None:
        if not self._inited:
            self.init_from_frame(ticks)
            return
        for sid in self.servo_ids:
            t = int(ticks[sid])
            dt = wrap_ticks(t - int(self.raw_prev[sid]))
            self.tick_unwrapped[sid] = int(self.tick_unwrapped[sid]) + int(dt)
            self.raw_prev[sid] = t


class FeetechRobustReader:
    def __init__(self, servo_ids: list[int]) -> None:
        self.servo_ids = list(servo_ids)
        self.prev_raw: dict[int, int] = {}
        self.bad_streak = 0
        self.unw = FeetechUnwrapper(self.servo_ids)
        self.last_ok_t = time.time()
        self.last_error_print_t = 0.0

    def reset(self) -> None:
        self.prev_raw = {}
        self.bad_streak = 0
        self.unw.reset()

    def read_update(self, ctrl: ServoController) -> bool:
        try:
            ticks = ctrl.read_positions(self.servo_ids)
        except Exception as exc:
            now = time.time()
            if now - self.last_error_print_t > 1.0:
                print(f"[gello] read failed: {exc}")
                self.last_error_print_t = now
            return False

        missing = [sid for sid in self.servo_ids if sid not in ticks]
        if missing:
            self.bad_streak += 1
            return False

        if self.prev_raw:
            for sid in self.servo_ids:
                dt = wrap_ticks(int(ticks[sid]) - int(self.prev_raw[sid]))
                if abs(dt) > FEETECH_TICK_JUMP_TH:
                    self.bad_streak += 1
                    if self.bad_streak >= FEETECH_BAD_FRAME_HOLD:
                        print(f"[gello] resync after {self.bad_streak} bad frames")
                        self.reset()
                    return False

        self.bad_streak = 0
        self.prev_raw = dict(ticks)
        self.last_ok_t = time.time()
        if not self.unw._inited:
            self.unw.init_from_frame(ticks)
        else:
            self.unw.update(ticks)
        return True


class RobotDynamics:
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
        tau_dyn = pin.rnea(self.model, self.data, q_pin, v_pin, np.zeros(self.model.nv, dtype=np.float64))
        tau_g = pin.computeGeneralizedGravity(self.model, self.data, q_pin)
        return np.asarray((tau_dyn - tau_g)[self.arm_vidx], dtype=np.float64)

    def inertia_terms(self, q_arm: np.ndarray, ddq_arm: np.ndarray) -> np.ndarray:
        q_pin, _ = self.qv(q_arm, np.zeros(JOINT_COUNT, dtype=np.float64))
        mass = np.asarray(pin.crba(self.model, self.data, q_pin), dtype=np.float64)
        arm_mass = mass[np.ix_(self.arm_vidx, self.arm_vidx)]
        return arm_mass @ np.asarray(ddq_arm, dtype=np.float64)


def detect_default_gello_port() -> str:
    candidates = [
        "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14028931-if00",
        "/dev/ttyACM1",
        "/dev/feetech_servos",
        "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        "/dev/ttyUSB0",
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return DEFAULT_GELLO_PORT


def ticks_unwrapped_to_rad_calibrated(sid: int, tick_unwrapped: int) -> float:
    cfg = ARM_MAP[sid]
    sign = int(cfg["sign"])
    gear = float(cfg["gear"])
    zero_tick = CALIBRATED_ARM_OFFSET.get(sid, cfg["offset"])
    return float(sign) * gear * float(int(tick_unwrapped) - zero_tick) * RAD_PER_TICK


def capture_gello_zero(tick_snapshot: dict[int, int]) -> None:
    global CALIBRATED_ARM_OFFSET, CALIBRATED_GRIPPER_ZERO
    CALIBRATED_ARM_OFFSET = {sid: int(tick_snapshot[sid]) for sid in SERVO_IDS}
    CALIBRATED_GRIPPER_ZERO = int(tick_snapshot[GRIPPER_ID])
    print("\n[gello] zero calibrated:")
    for sid in ARM_IDS:
        print(f"  sid={sid} zero_tick={CALIBRATED_ARM_OFFSET[sid]}")
    print(f"  gripper_zero_tick={CALIBRATED_GRIPPER_ZERO}")


def map_gello_targets(unw: FeetechUnwrapper) -> tuple[np.ndarray, float]:
    q = np.zeros(JOINT_COUNT, dtype=np.float64)
    for idx, sid in enumerate(ARM_IDS):
        q[idx] = ticks_unwrapped_to_rad_calibrated(sid, unw.tick_unwrapped[sid])
    gripper_tick = int(unw.raw_prev.get(GRIPPER_ID, CALIBRATED_GRIPPER_ZERO))
    gripper_q = float(gripper_tick - CALIBRATED_GRIPPER_ZERO) * RAD_PER_TICK
    return q, gripper_q


def wait_for_gello_zero(ctrl: ServoController, reader: FeetechRobustReader) -> None:
    print("=" * 50)
    print("place GELLO at zero pose, then press Enter")
    print("=" * 50)
    while not STOP:
        updated = reader.read_update(ctrl)
        if updated and reader.unw._inited:
            q, _ = map_gello_targets(reader.unw)
            print(
                "\r[gello_zero] q_deg=["
                + ", ".join(f"{math.degrees(v):6.1f}" for v in q)
                + "]",
                end="",
                flush=True,
            )
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                capture_gello_zero(reader.unw.tick_unwrapped)
                print()
                return
        else:
            print("\r[gello_zero] waiting for stable servo data...", end="", flush=True)
        time.sleep(0.01)


def move_mit(
    arm: florid_usb.Arm,
    dynamics: RobotDynamics,
    q_target: np.ndarray,
    duration: float,
    control_rate: float,
) -> None:
    status = arm.get_arm_status()
    q_start = np.asarray(status["q"], dtype=np.float64)
    dq_zero = np.zeros(JOINT_COUNT, dtype=np.float64)
    steps = max(2, int(duration * control_rate))
    period = 1.0 / control_rate
    kp = np.array([20.0, 21.0, 21.0, 6.0, 5.0, 1.0], dtype=np.float64)
    kd = np.array([2.0, 2.0, 2.0, 0.9, 0.7, 0.1], dtype=np.float64)
    next_tick = time.perf_counter()
    for step in range(steps):
        if STOP:
            return
        frac = (step + 1) / steps
        smooth = frac * frac * (3.0 - 2.0 * frac)
        q_des = q_start + smooth * (q_target - q_start)
        tau_ff = np.clip(dynamics.gravity(q_des), -TAU_LIMIT, TAU_LIMIT)
        arm.send_mit_command(as_f32(q_des), as_f32(dq_zero), as_f32(tau_ff), as_f32(kp), as_f32(kd))
        next_tick += period
        while not STOP and time.perf_counter() < next_tick:
            pass


def settle_hold(arm: florid_usb.Arm, dynamics: RobotDynamics, seconds: float) -> None:
    status = arm.get_arm_status()
    hold_q = np.asarray(status["q"], dtype=np.float64)
    hold_tau = np.clip(dynamics.gravity(hold_q), -TAU_LIMIT, TAU_LIMIT)
    dq_zero = np.zeros(JOINT_COUNT, dtype=np.float64)
    kp = np.zeros(JOINT_COUNT, dtype=np.float64)
    kd = np.full(JOINT_COUNT, 0.4, dtype=np.float64)
    end_time = time.time() + seconds
    while not STOP and time.time() < end_time:
        arm.send_mit_command(as_f32(hold_q), as_f32(dq_zero), as_f32(hold_tau), as_f32(kp), as_f32(kd))
        time.sleep(0.01)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="USB GELLO teleop with computed torque")
    parser.add_argument("--arm-device", default=DEFAULT_ARM_DEVICE)
    parser.add_argument("--gello-port", default=None)
    parser.add_argument("--control-rate", type=float, default=CONTROL_RATE)
    parser.add_argument("--print-interval", type=float, default=PRINT_INTERVAL)
    parser.add_argument("--start-duration", type=float, default=3.0)
    parser.add_argument(
        "--start-pos",
        type=float,
        nargs=6,
        default=[0.0] * JOINT_COUNT,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
    )
    parser.add_argument("--skip-move", action="store_true")
    parser.add_argument("--no-coriolis", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    dynamics = RobotDynamics(URDF_PATH)
    gello_port = args.gello_port or detect_default_gello_port()
    gello_ctrl = ServoController(SERVO_IDS, gello_port, FEETECH_BAUD)
    gello_reader = FeetechRobustReader(SERVO_IDS)

    cfg = florid_usb.Config()
    cfg.device = args.arm_device
    arm = florid_usb.Arm(cfg)

    print(f"[gello] opening GELLO on {gello_port}")
    gello_ctrl.connect()
    print("[gello] GELLO connected")

    print(f"[gello] opening arm on {args.arm_device}")
    if not arm.connect():
        print("[gello] failed to open arm serial")
        gello_ctrl.disconnect()
        return 1

    session_started = False
    try:
        print("[gello] starting USB session...")
        if not arm.start_session(timeout=1.0):
            print("[gello] start_session failed")
            return 1
        session_started = True
        print("[gello] USB session active")

        wait_for_gello_zero(gello_ctrl, gello_reader)
        if STOP:
            return 0

        start_pos = np.clip(np.asarray(args.start_pos, dtype=np.float64), JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        if not args.skip_move:
            print(f"[gello] moving to start pose {np.round(start_pos, 3)}")
            move_mit(arm, dynamics, start_pos, args.start_duration, args.control_rate)
            settle_hold(arm, dynamics, 0.5)
            if STOP:
                return 0

        dt = 1.0 / args.control_rate
        next_tick = time.perf_counter()
        last_print = 0.0
        q_cmd_filt: Optional[np.ndarray] = None
        dq_cmd = np.zeros(JOINT_COUNT, dtype=np.float64)
        ddq_cmd = np.zeros(JOINT_COUNT, dtype=np.float64)

        print("[gello] teleop running, Ctrl+C to stop")
        print(f"[gello] CT kp={CT_KP.tolist()}, kd={CT_KD.tolist()}")
        print(f"[gello] MIT kp={MIT_KP.tolist()}, kd={MIT_KD.tolist()}")
        print("[gello] gripper is read-only in this USB version")

        while not STOP:
            if not gello_reader.read_update(gello_ctrl):
                time.sleep(0.001)
                continue

            q_raw, gripper_q = map_gello_targets(gello_reader.unw)
            q_raw = np.clip(q_raw, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
            if q_cmd_filt is None:
                q_cmd_filt = q_raw.copy()
            else:
                q_cmd_filt = (1.0 - TELEOP_POS_ALPHA) * q_cmd_filt + TELEOP_POS_ALPHA * q_raw
                small_motion = np.abs(q_cmd_filt - q_raw) < TELEOP_POS_DEADBAND
                q_cmd_filt[small_motion] = q_raw[small_motion]
            q_des = q_cmd_filt

            status = arm.get_arm_status()
            actual_q = np.asarray(status["q"], dtype=np.float64)
            actual_dq = np.asarray(status["dq"], dtype=np.float64)

            pos_err = q_des - actual_q
            vel_err = dq_cmd - actual_dq
            gravity_torque = dynamics.gravity(actual_q)
            coriolis_torque = np.zeros(JOINT_COUNT, dtype=np.float64)
            if not args.no_coriolis:
                coriolis_torque = dynamics.coriolis(actual_q, actual_dq)
            inertia_ff = dynamics.inertia_terms(actual_q, ddq_cmd)
            feedback_torque = dynamics.inertia_terms(actual_q, CT_KP * pos_err + CT_KD * vel_err)
            total_torque = gravity_torque + coriolis_torque + inertia_ff + feedback_torque
            tau_cmd = np.clip(total_torque, -TAU_LIMIT, TAU_LIMIT)

            arm.send_mit_command(
                as_f32(q_des),
                as_f32(dq_cmd),
                as_f32(tau_cmd),
                as_f32(MIT_KP),
                as_f32(MIT_KD),
            )

            now = time.perf_counter()
            if now - last_print >= args.print_interval:
                print(
                    "[gello] "
                    f"q_des={np.array2string(q_des, precision=3, suppress_small=True)} "
                    f"q_act={np.array2string(actual_q, precision=3, suppress_small=True)} "
                    f"tau={np.array2string(tau_cmd, precision=3, suppress_small=True)} "
                    f"gripper={gripper_q:.3f}"
                )
                last_print = now

            next_tick += dt
            while not STOP and time.perf_counter() < next_tick:
                pass
            if time.perf_counter() - next_tick > 0.2:
                next_tick = time.perf_counter()

        settle_hold(arm, dynamics, 0.5)
    finally:
        if session_started:
            arm.stop_session(1.0)
        arm.disconnect()
        gello_ctrl.disconnect()
        print("[gello] done")

    return 0


if __name__ == "__main__":
    sys.exit(main())
