#!/usr/bin/env python3
"""
Test all florid_usb Python API interfaces with a single arm.

Usage:
    python python/test_all_api.py [/dev/ttyACM0]

Each test prints PASS or FAIL. Control commands ramp to zero position with
conservative gains (50% of teleop parameters).
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

import florid_usb


PASS = "\033[92mPASS\033[0m"
FAIL = "\033[91mFAIL\033[0m"
SKIP = "\033[93mSKIP\033[0m"
BOLD = "\033[1m"

JOINT_COUNT = 6
CONTROL_PERIOD_S = 0.01  # 100 Hz
RAMP_STEPS = 200         # 2 seconds per ramp

# 50% of teleop MIT follower gains
ZERO_KP = np.array([15.0, 40.0, 35.0, 35.0, 30.0, 10.0], dtype=np.float32)
ZERO_KD = np.array([0.35, 0.6, 0.55, 0.5, 0.5, 0.25], dtype=np.float32)

# Gripper (50% of teleop)
GRIPPER_KP = 5.0
GRIPPER_KD = 0.5

ZERO_Q = np.zeros(JOINT_COUNT, dtype=np.float32)
ZERO_DQ = np.zeros(JOINT_COUNT, dtype=np.float32)
ZERO_TAU = np.zeros(JOINT_COUNT, dtype=np.float32)


def ramp_to_target(
    arm: florid_usb.Arm,
    target_q: np.ndarray,
    kp: np.ndarray,
    kd: np.ndarray,
    send_fn: callable,
    label: str = "",
) -> None:
    """Ramp from current position to target_q over RAMP_STEPS steps."""
    cur = arm.get_arm_status()
    start_q = np.asarray(cur["q"], dtype=np.float32)
    print(f"  {label}ramp: {start_q[0]:.3f}..{start_q[5]:.3f} -> {target_q[0]:.3f}..{target_q[5]:.3f}")
    next_tick = time.perf_counter()
    for step in range(RAMP_STEPS):
        frac = step / RAMP_STEPS
        q = start_q + frac * (target_q - start_q)
        send_fn(q)
        next_tick += CONTROL_PERIOD_S
        while time.perf_counter() < next_tick:
            pass
    final = arm.get_arm_status()
    final_q = np.asarray(final["q"], dtype=np.float32)
    err = np.abs(final_q - target_q).max()
    print(f"  {label}done: max_err={err:.4f} rad")
    return err


# ── Constructor & Connection ──

def test_connect(cfg: florid_usb.Config) -> florid_usb.Arm:
    print(f"\n{BOLD}[test] Arm(config){BOLD}")
    arm = florid_usb.Arm(cfg)
    print(f"  Arm created: device={cfg.device}, baud_rate={cfg.baud_rate}")
    return arm


def test_is_connected_false(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] is_connected() before connect{BOLD}")
    c = arm.is_connected()
    print(f"  returned: {c}")
    print(f"  {PASS if not c else FAIL}")


def test_connect_device(arm: florid_usb.Arm) -> bool:
    print(f"\n{BOLD}[test] connect(){BOLD}")
    ok = arm.connect()
    print(f"  returned: {ok}")
    print(f"  {PASS if ok else FAIL}")
    return ok


def test_is_connected_true(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] is_connected() after connect{BOLD}")
    time.sleep(0.5)
    c = arm.is_connected()
    print(f"  returned: {c}")
    print(f"  {PASS if c else FAIL}")


# ── Session ──

def test_start_session(arm: florid_usb.Arm) -> bool:
    print(f"\n{BOLD}[test] start_session(timeout=2.0){BOLD}")
    ok = arm.start_session(timeout=2.0)
    print(f"  returned: {ok}")
    print(f"  {PASS if ok else FAIL}")
    return ok


def test_stop_session(arm: florid_usb.Arm) -> bool:
    print(f"\n{BOLD}[test] stop_session(timeout=2.0){BOLD}")
    ok = arm.stop_session(timeout=2.0)
    print(f"  returned: {ok}")
    print(f"  {PASS if ok else FAIL}")
    return ok


def test_disconnect(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] disconnect(){BOLD}")
    try:
        arm.disconnect()
        print(f"  {PASS}")
    except Exception as e:
        print(f"  exception: {e}")
        print(f"  {FAIL}")


def test_reconnect(cfg: florid_usb.Config) -> None:
    print(f"\n{BOLD}[test] reconnect — connect() after disconnect(){BOLD}")
    arm = florid_usb.Arm(cfg)
    ok = arm.connect()
    print(f"  connect returned: {ok}")
    ok = ok and arm.start_session(timeout=1.0)
    print(f"  start_session: {ok}")
    arm.stop_session(timeout=1.0)
    arm.disconnect()
    print(f"  {PASS if ok else FAIL}")


# ── Status Reads ──

def test_get_arm_status(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] get_arm_status(){BOLD}")
    s = arm.get_arm_status()
    required_keys = {"mode", "seq", "timestamp_us", "q", "dq", "tau", "gripper"}
    has_keys = required_keys.issubset(s.keys())
    q = s.get("q")
    q_ok = isinstance(q, np.ndarray) and q.shape == (6,) and q.dtype == np.float32
    gripper_ok = isinstance(s.get("gripper"), dict)
    print(f"  mode={s.get('mode')} seq={s.get('seq')}")
    print(f"  q[:3]={q[:3] if q is not None else 'N/A'} dtype={q.dtype if isinstance(q, np.ndarray) else 'N/A'}")
    print(f"  keys ok: {has_keys}, q ndarray float32(6): {q_ok}, gripper ok: {gripper_ok}")
    print(f"  {PASS if (has_keys and q_ok and gripper_ok) else FAIL}")


def test_get_gripper_status(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] get_gripper_status(){BOLD}")
    g = arm.get_gripper_status()
    required_keys = {"q", "dq", "tau", "temp_c", "enabled"}
    has_keys = required_keys.issubset(g.keys())
    print(f"  q={g.get('q'):.4f} enabled={g.get('enabled')}")
    print(f"  keys ok: {has_keys}")
    print(f"  {PASS if has_keys else FAIL}")


def test_get_motor_feedback(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] get_motor_feedback(timeout=2.0){BOLD}")
    fb = arm.get_motor_feedback(timeout=2.0)
    motors = fb.get("motors", [])
    count_ok = len(motors) == 7
    motor_keys = {"joint_id", "position_rad", "speed_rad_s", "torque_nm", "temp_c", "device_status", "enabled"}
    keys_ok = all(motor_keys.issuperset(m.keys()) for m in motors)
    print(f"  motors count: {len(motors)}")
    if motors:
        m0 = motors[0]
        print(f"  motor[0]: joint_id={m0.get('joint_id')} pos={m0.get('position_rad'):.4f}")
    print(f"  count ok: {count_ok}, keys ok: {keys_ok}")
    print(f"  {PASS if (count_ok and keys_ok) else FAIL}")


# ── Control Commands ──

def test_send_mit_command(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] send_mit_command() — ramp to zero{BOLD}")
    try:
        err = ramp_to_target(
            arm, ZERO_Q, ZERO_KP, ZERO_KD,
            lambda q: arm.send_mit_command(q, ZERO_DQ, ZERO_TAU, ZERO_KP, ZERO_KD, control_mode=1),
            label="MIT ",
        )
        ok = err < 0.15  # within 0.15 rad after 2s ramp
        print(f"  {PASS if ok else FAIL} (max_err={err:.4f})")
    except Exception as e:
        print(f"  exception: {e}")
        print(f"  {FAIL}")


def test_set_motor_control_mode(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] set_motor_control_mode(joint_id=0, mode='mit'){BOLD}")
    try:
        ok = arm.set_motor_control_mode(joint_id=0, mode="mit", timeout=1.0)
        print(f"  returned: {ok}")
        print(f"  {PASS if ok else FAIL}")
    except Exception as e:
        print(f"  exception: {e}")
        print(f"  {FAIL}")


def test_send_posvel_command(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] send_posvel_command() — hold zero with enabled_mask=0x3f{BOLD}")
    try:
        # Brief ramp to zero with posvel (motors must already be in posvel mode)
        for _ in range(20):
            arm.send_posvel_command(ZERO_Q, ZERO_DQ, enabled_mask=0x3f)
            time.sleep(CONTROL_PERIOD_S)
        print(f"  sent 20 frames @ 100 Hz")
        print(f"  {PASS}")
    except Exception as e:
        print(f"  exception: {e}")
        print(f"  {FAIL}")


def test_send_velocity_command(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] send_velocity_command() — zero velocity{BOLD}")
    try:
        for _ in range(10):
            arm.send_velocity_command(ZERO_DQ, enabled_mask=0x3f)
            time.sleep(CONTROL_PERIOD_S)
        print(f"  sent 10 frames")
        print(f"  {PASS}")
    except Exception as e:
        print(f"  exception: {e}")
        print(f"  {FAIL}")


def test_send_hybrid_command(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] send_hybrid_command() — hold zero{BOLD}")
    try:
        dq_limit = np.full(JOINT_COUNT, 10.0, dtype=np.float32)
        current_limit = np.full(JOINT_COUNT, 0.3, dtype=np.float32)
        for _ in range(20):
            arm.send_hybrid_command(ZERO_Q, dq_limit, current_limit, enabled_mask=0x3f)
            time.sleep(CONTROL_PERIOD_S)
        print(f"  sent 20 frames")
        print(f"  {PASS}")
    except Exception as e:
        print(f"  exception: {e}")
        print(f"  {FAIL}")


def test_send_gripper_command(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] send_gripper_command() — close/open cycle{BOLD}")
    try:
        # Read current gripper status
        gs = arm.get_gripper_status()
        start_q = float(gs["q"])
        print(f"  start q={start_q:.4f}")

        # Close 0.3 rad
        next_tick = time.perf_counter()
        for step in range(80):
            frac = step / 80
            target = start_q - 0.15 * frac
            arm.send_gripper_command(q=target, dq=0.0, tau=0.0,
                                     kp=GRIPPER_KP, kd=GRIPPER_KD, control_mode=1)
            next_tick += CONTROL_PERIOD_S
            while time.perf_counter() < next_tick:
                pass
        print(f"  closed -> q={arm.get_gripper_status()['q']:.4f}")

        # Open back
        next_tick = time.perf_counter()
        for step in range(80):
            frac = step / 80
            target = (start_q - 0.15) + 0.15 * frac
            arm.send_gripper_command(q=target, dq=0.0, tau=0.0,
                                     kp=GRIPPER_KP, kd=GRIPPER_KD, control_mode=1)
            next_tick += CONTROL_PERIOD_S
            while time.perf_counter() < next_tick:
                pass
        print(f"  opened -> q={arm.get_gripper_status()['q']:.4f}")
        print(f"  {PASS}")
    except Exception as e:
        print(f"  exception: {e}")
        print(f"  {FAIL}")


# ── Safety ──

def test_emergency_stop(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] emergency_stop(){BOLD}")
    try:
        arm.emergency_stop()
        print(f"  sent")
        time.sleep(0.3)
        print(f"  {PASS}")
    except Exception as e:
        print(f"  exception: {e}")
        print(f"  {FAIL}")


# ── Blocking Commands ──

def test_home_all(arm: florid_usb.Arm) -> bool:
    print(f"\n{BOLD}[test] home_all(timeout=5.0){BOLD}")
    ok = arm.home_all(timeout=5.0)
    print(f"  returned: {ok}")
    print(f"  {PASS if ok else FAIL}")
    return ok


def test_clear_faults(arm: florid_usb.Arm) -> bool:
    print(f"\n{BOLD}[test] clear_faults(timeout=2.0){BOLD}")
    ok = arm.clear_faults(timeout=2.0)
    print(f"  returned: {ok}")
    print(f"  {PASS if ok else FAIL}")
    return ok


# ── Main ──

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test all florid_usb Python API interfaces",
    )
    parser.add_argument("device", nargs="?", default="/dev/ttyACM0")
    parser.add_argument("--skip-home", action="store_true",
                        help="Skip home_all and clear_faults (safe mode)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print(f"florid_usb module: {florid_usb.__file__}")
    print(f"device: {args.device}")

    cfg = florid_usb.Config()
    cfg.device = args.device
    cfg.baud_rate = 115200

    passed = 0
    failed = 0
    skipped = 0

    def check(ok: bool, name: str) -> None:
        nonlocal passed, failed
        if ok:
            passed += 1
        else:
            failed += 1
            print(f"  >>> {name} FAILED <<<")

    # ── 1. Constructor + Connect ──
    arm = test_connect(cfg)
    test_is_connected_false(arm)
    ok = test_connect_device(arm)
    check(ok, "connect")
    if not ok:
        print(f"\n  Cannot proceed without connection. Exiting.")
        return 1
    test_is_connected_true(arm)

    # ── 2. Session ──
    ok = test_start_session(arm)
    check(ok, "start_session")

    # ── 3. Status Reads ──
    test_get_arm_status(arm)
    test_get_gripper_status(arm)
    test_get_motor_feedback(arm)

    # ── 4. Control Commands ──
    test_send_mit_command(arm)
    test_set_motor_control_mode(arm)
    test_send_posvel_command(arm)
    test_send_velocity_command(arm)
    test_send_hybrid_command(arm)
    test_send_gripper_command(arm)

    # ── 5. Safety ──
    test_emergency_stop(arm)

    # ── 6. Blocking Commands ──
    if not args.skip_home:
        ok = test_home_all(arm)
        check(ok, "home_all")
        time.sleep(0.5)
        ok = test_clear_faults(arm)
        check(ok, "clear_faults")
    else:
        skipped += 2
        print(f"\n  {SKIP} home_all (--skip-home)")
        print(f"  {SKIP} clear_faults (--skip-home)")

    # ── 7. Stop Session ──
    ok = test_stop_session(arm)
    check(ok, "stop_session")

    # ── 8. Disconnect & Reconnect ──
    test_disconnect(arm)
    test_reconnect(cfg)

    # ── Summary ──
    total = passed + failed + skipped
    print(f"\n{'=' * 40}")
    print(f"  {BOLD}Results: {PASS if failed == 0 else FAIL}{BOLD}")
    print(f"  Total:  {total}")
    print(f"  Passed: {passed}")
    print(f"  Failed: {failed}")
    print(f"  Skipped:{skipped}")
    print(f"{'=' * 40}")

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
