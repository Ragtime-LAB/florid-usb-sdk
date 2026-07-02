#!/usr/bin/env python3
"""
Test all florid_usb Python API interfaces with a single arm.

Usage:
    python python/test_all_api.py [/dev/ttyACM0]

Each test prints PASS or FAIL. Safe to run — no large movements.
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


def test_connect(cfg: florid_usb.Config) -> florid_usb.Arm:
    print(f"\n{BOLD}[test] Arm(config){BOLD}")
    arm = florid_usb.Arm(cfg)
    print(f"  Arm created: device={cfg.device}, baud_rate={cfg.baud_rate}")
    return arm


def test_is_connected(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] is_connected(){BOLD}")
    c = arm.is_connected()
    print(f"  returned: {c}")
    print(f"  {PASS if not c else FAIL}")


def test_connect_device(arm: florid_usb.Arm) -> None:
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


def test_start_session(arm: florid_usb.Arm) -> bool:
    print(f"\n{BOLD}[test] start_session(timeout=2.0){BOLD}")
    ok = arm.start_session(timeout=2.0)
    print(f"  returned: {ok}")
    print(f"  {PASS if ok else FAIL}")
    return ok


def test_get_arm_status(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] get_arm_status(){BOLD}")
    s = arm.get_arm_status()
    required_keys = {"mode", "seq", "timestamp_us", "q", "dq", "tau", "gripper"}
    has_keys = required_keys.issubset(s.keys())
    q = np.asarray(s.get("q", []))
    q_ok = q.shape == (6,) and q.dtype == np.float64
    gripper_ok = isinstance(s.get("gripper"), dict)
    print(f"  mode={s.get('mode')} seq={s.get('seq')}")
    print(f"  q[:3]={q[:3]} ...")
    print(f"  keys ok: {has_keys}, q shape ok: {q_ok}, gripper ok: {gripper_ok}")
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


def test_send_mit_command(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] send_mit_command() — hold current position{BOLD}")
    q = np.zeros(6, dtype=np.float32)
    dq = np.zeros(6, dtype=np.float32)
    tau = np.zeros(6, dtype=np.float32)
    kp = np.full(6, 4.0, dtype=np.float32)
    kd = np.full(6, 0.3, dtype=np.float32)

    try:
        for i in range(5):
            arm.send_mit_command(q, dq, tau, kp, kd, control_mode=1)
            time.sleep(0.002)
        print(f"  sent 5 frames @ ~500 Hz")
        print(f"  {PASS}")
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
    print(f"\n{BOLD}[test] send_posvel_command() — zero velocity hold{BOLD}")
    try:
        q = np.zeros(6, dtype=np.float32)
        dq = np.zeros(6, dtype=np.float32)
        arm.send_posvel_command(q, dq, enabled_mask=0x3f)
        print(f"  sent")
        print(f"  {PASS}")
    except Exception as e:
        print(f"  exception: {e}")
        print(f"  {FAIL}")


def test_send_velocity_command(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] send_velocity_command() — zero velocity{BOLD}")
    try:
        dq = np.zeros(6, dtype=np.float32)
        arm.send_velocity_command(dq, enabled_mask=0x3f)
        print(f"  sent")
        print(f"  {PASS}")
    except Exception as e:
        print(f"  exception: {e}")
        print(f"  {FAIL}")


def test_send_hybrid_command(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] send_hybrid_command() — hold position{BOLD}")
    try:
        q = np.zeros(6, dtype=np.float32)
        dq_limit = np.full(6, 10.0, dtype=np.float32)
        current_limit = np.full(6, 0.3, dtype=np.float32)
        arm.send_hybrid_command(q, dq_limit, current_limit, enabled_mask=0x3f)
        print(f"  sent")
        print(f"  {PASS}")
    except Exception as e:
        print(f"  exception: {e}")
        print(f"  {FAIL}")


def test_send_gripper_command(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] send_gripper_command() — hold{BOLD}")
    try:
        arm.send_gripper_command(q=0.0, dq=0.0, tau=0.0, kp=10.0, kd=1.0, control_mode=1)
        print(f"  sent")
        print(f"  {PASS}")
    except Exception as e:
        print(f"  exception: {e}")
        print(f"  {FAIL}")


def test_emergency_stop(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] emergency_stop(){BOLD}")
    try:
        arm.emergency_stop()
        print(f"  sent")
        print(f"  {PASS}")
    except Exception as e:
        print(f"  exception: {e}")
        print(f"  {FAIL}")


def test_home_all(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] home_all(timeout=5.0){BOLD}")
    ok = arm.home_all(timeout=5.0)
    print(f"  returned: {ok}")
    print(f"  {PASS if ok else FAIL}")
    return ok


def test_clear_faults(arm: florid_usb.Arm) -> None:
    print(f"\n{BOLD}[test] clear_faults(timeout=2.0){BOLD}")
    ok = arm.clear_faults(timeout=2.0)
    print(f"  returned: {ok}")
    print(f"  {PASS if ok else FAIL}")
    return ok


def test_stop_session(arm: florid_usb.Arm) -> None:
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


def test_reconnect(arm: florid_usb.Arm, cfg: florid_usb.Config) -> None:
    """Verify reconnection after disconnect."""
    print(f"\n{BOLD}[test] reconnect — connect() after disconnect(){BOLD}")
    # Re-create arm to test fresh state
    arm2 = florid_usb.Arm(cfg)
    ok = arm2.connect()
    print(f"  connect returned: {ok}")
    print(f"  {PASS if ok else FAIL}")
    if ok:
        time.sleep(0.3)
        arm2.disconnect()


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
    print(f"skip home/clear: {args.skip_home}")

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

    # ── 1. Constructor + connect ──
    arm = test_connect(cfg)
    test_is_connected(arm)
    ok = test_connect_device(arm)
    check(ok, "connect")

    if not ok:
        print(f"\n  Cannot proceed without connection. Exiting.")
        return 1

    test_is_connected_true(arm)

    # ── 2. Session ──
    ok = test_start_session(arm)
    check(ok, "start_session")

    # ── 3. Status reads (work with or without session) ──
    test_get_arm_status(arm)
    test_get_gripper_status(arm)
    test_get_motor_feedback(arm)

    # ── 4. Control commands ──
    test_send_mit_command(arm)
    test_set_motor_control_mode(arm)
    test_send_posvel_command(arm)
    test_send_velocity_command(arm)
    test_send_hybrid_command(arm)
    test_send_gripper_command(arm)

    # ── 5. Safety ──
    test_emergency_stop(arm)

    # ── 6. Blocking commands (optional) ──
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

    # ── 7. Stop session ──
    ok = test_stop_session(arm)
    check(ok, "stop_session")

    # ── 8. Disconnect ──
    test_disconnect(arm)
    test_reconnect(arm, cfg)

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
