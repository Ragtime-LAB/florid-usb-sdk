#!/usr/bin/env python3
"""
4-arm teleoperation — MIT leaders + Hybrid followers.

Usage:
    PYTHONPATH=python python3 python/teleop_hybrid.py
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

import florid_usb
import teleop_common as tc

LEADER_KD = np.zeros(tc.JOINT_COUNT, dtype=np.float64)

GRIPPER_KP = 10.0
GRIPPER_KD = 1.0
GRIPPER_TAU = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="4-arm teleoperation — MIT leaders + Hybrid followers"
    )
    parser.add_argument("--leader1-device", default="/dev/ttyACM0")
    parser.add_argument("--leader2-device", default="/dev/ttyACM1")
    parser.add_argument("--follower1-device", default="/dev/ttyACM2")
    parser.add_argument("--follower2-device", default="/dev/ttyACM3")
    parser.add_argument("--period", type=float, default=tc.DEFAULT_PERIOD_S)
    parser.add_argument("--print-interval", type=float, default=tc.DEFAULT_PRINT_INTERVAL_S)
    parser.add_argument("--sync-duration", type=float, default=tc.DEFAULT_SYNC_DURATION_S)
    parser.add_argument("--stale-timeout", type=float, default=tc.DEFAULT_STALE_TIMEOUT_S)
    parser.add_argument("--nonzero-eps", type=float, default=tc.DEFAULT_NONZERO_EPS)
    parser.add_argument("--leader-kd", type=float, nargs=6, default=LEADER_KD.tolist(), metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    parser.add_argument("--teleop-alpha", type=float, default=0.2)
    parser.add_argument("--leader-gravity-scale", type=float, nargs=6, default=[1.0] * tc.JOINT_COUNT, metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    parser.add_argument("--follower-dq-limit", type=float, nargs=6, default=[0.5, 0.5, 0.5, 0.5, 0.4, 0.3], metavar=("J1", "J2", "J3", "J4", "J5", "J6"), help="max velocity limit per joint in hybrid mode")
    parser.add_argument("--follower-current-limit", type=float, nargs=6, default=[0.1, 0.1, 0.1, 0.1, 0.08, 0.05], metavar=("J1", "J2", "J3", "J4", "J5", "J6"), help="current/force limit 0..1 per joint in hybrid mode")
    parser.add_argument("--gripper-kp", type=float, default=GRIPPER_KP)
    parser.add_argument("--gripper-kd", type=float, default=GRIPPER_KD)
    parser.add_argument("--gripper-tau", type=float, default=GRIPPER_TAU)
    return parser.parse_args()


def sync_followers_to_leaders(
    leader1: florid_usb.Arm, leader2: florid_usb.Arm,
    follower1: florid_usb.Arm, follower2: florid_usb.Arm,
    gravity: tc.HostGravityEstimator,
    freshness: tc.FreshnessMonitor,
    leader_kd: np.ndarray,
    leader_gravity_scale: np.ndarray,
    follower_dq_limit: np.ndarray,
    follower_current_limit: np.ndarray,
    duration_s: float, period_s: float,
) -> None:
    l1_q, _, _ = tc.get_state(leader1, "leader1", freshness)
    l2_q, _, _ = tc.get_state(leader2, "leader2", freshness)
    f1_start_q, _, _ = tc.get_state(follower1, "follower1", freshness)
    f2_start_q, _, _ = tc.get_state(follower2, "follower2", freshness)

    print("[teleop] syncing followers to current leader poses")
    print(f"[teleop] follower1 target={np.array2string(l1_q, precision=3, suppress_small=True)}")
    print(f"[teleop] follower2 target={np.array2string(l2_q, precision=3, suppress_small=True)}")

    dq_zero = np.zeros(tc.JOINT_COUNT, dtype=np.float64)
    steps = max(2, int(duration_s / period_s))
    next_tick = time.perf_counter()

    for step in range(steps):
        if tc.STOP:
            return
        frac = (step + 1) / steps
        smooth = frac * frac * (3.0 - 2.0 * frac)
        f1_q_des = f1_start_q + smooth * (l1_q - f1_start_q)
        f2_q_des = f2_start_q + smooth * (l2_q - f2_start_q)

        l1_now_q, _, _ = tc.get_state(leader1, "leader1", freshness)
        l2_now_q, _, _ = tc.get_state(leader2, "leader2", freshness)

        l1_tau_ff = tc.scaled_gravity(gravity, l1_now_q, leader_gravity_scale)
        l2_tau_ff = tc.scaled_gravity(gravity, l2_now_q, leader_gravity_scale)

        tc.send_leader_damping(leader1, l1_now_q, dq_zero, l1_tau_ff, leader_kd)
        tc.send_leader_damping(leader2, l2_now_q, dq_zero, l2_tau_ff, leader_kd)
        follower1.send_hybrid_command(tc.as_f32(f1_q_des), tc.as_f32(follower_dq_limit), tc.as_f32(follower_current_limit))
        follower2.send_hybrid_command(tc.as_f32(f2_q_des), tc.as_f32(follower_dq_limit), tc.as_f32(follower_current_limit))

        next_tick += period_s
        while not tc.STOP and time.perf_counter() < next_tick:
            pass
        if time.perf_counter() - next_tick > 0.2:
            next_tick = time.perf_counter()


def main() -> int:
    import signal
    args = parse_args()
    signal.signal(signal.SIGINT, tc.signal_handler)
    signal.signal(signal.SIGTERM, tc.signal_handler)

    if not (0.0 < args.teleop_alpha <= 1.0):
        raise SystemExit("--teleop-alpha must be in (0, 1]")
    if args.sync_duration <= 0.0:
        raise SystemExit("--sync-duration must be > 0")
    if args.stale_timeout <= 0.0:
        raise SystemExit("--stale-timeout must be > 0")

    gravity = tc.HostGravityEstimator(tc.URDF_PATH)
    freshness = tc.FreshnessMonitor(args.stale_timeout)

    leader1 = tc.make_arm(args.leader1_device)
    leader2 = tc.make_arm(args.leader2_device)
    follower1 = tc.make_arm(args.follower1_device)
    follower2 = tc.make_arm(args.follower2_device)

    leader1_started = False
    leader2_started = False
    follower1_started = False
    follower2_started = False

    leader_kd = np.asarray(args.leader_kd, dtype=np.float64)
    leader_gravity_scale = np.asarray(args.leader_gravity_scale, dtype=np.float64)
    follower_dq_limit = np.asarray(args.follower_dq_limit, dtype=np.float64)
    follower_current_limit = np.asarray(args.follower_current_limit, dtype=np.float64)

    try:
        tc.connect_and_start(leader1, "leader1", args.leader1_device)
        leader1_started = True
        tc.connect_and_start(leader2, "leader2", args.leader2_device)
        leader2_started = True
        tc.connect_and_start(follower1, "follower1", args.follower1_device)
        follower1_started = True
        tc.connect_and_start(follower2, "follower2", args.follower2_device)
        follower2_started = True

        tc.switch_joints_to_mode(follower1, "follower1", "hybrid")
        tc.switch_joints_to_mode(follower2, "follower2", "hybrid")

        tc.wait_for_fresh_status(
            [("leader1", leader1, args.leader1_device),
             ("leader2", leader2, args.leader2_device),
             ("follower1", follower1, args.follower1_device),
             ("follower2", follower2, args.follower2_device)],
            freshness,
            timeout_s=max(1.0, args.stale_timeout * 3.0),
            nonzero_eps=args.nonzero_eps,
        )

        dq_zero = np.zeros(tc.JOINT_COUNT, dtype=np.float64)

        sync_followers_to_leaders(
            leader1, leader2, follower1, follower2,
            gravity, freshness, leader_kd,
            leader_gravity_scale,
            follower_dq_limit, follower_current_limit,
            args.sync_duration, args.period,
        )
        if tc.STOP:
            return 0

        q1_filt: np.ndarray | None = None
        q2_filt: np.ndarray | None = None
        last_print = 0.0
        next_tick = time.perf_counter()

        print("[teleop] 4-arm teleop running (MIT leaders + Hybrid followers), Ctrl+C to stop")
        print(f"[teleop] follower dq_limit={follower_dq_limit.tolist()}")
        print(f"[teleop] follower current_limit={follower_current_limit.tolist()}")
        print(f"[teleop] gripper: leader kp=0 kd=0 (free), follower kp={args.gripper_kp} kd={args.gripper_kd}")

        gripper_kp = np.float64(args.gripper_kp)
        gripper_kd = np.float64(args.gripper_kd)
        gripper_tau = np.float64(args.gripper_tau)

        while not tc.STOP:
            l1_status = tc.get_status(leader1, "leader1", freshness)
            l2_status = tc.get_status(leader2, "leader2", freshness)
            f1_status = tc.get_status(follower1, "follower1", freshness)
            f2_status = tc.get_status(follower2, "follower2", freshness)
            l1_q, _, _ = tc.state_from_status(l1_status)
            l2_q, _, _ = tc.state_from_status(l2_status)
            f1_q, _, _ = tc.state_from_status(f1_status)
            f2_q, _, _ = tc.state_from_status(f2_status)

            if q1_filt is None:
                q1_filt = l1_q.copy()
            else:
                q1_filt = (1.0 - args.teleop_alpha) * q1_filt + args.teleop_alpha * l1_q
            if q2_filt is None:
                q2_filt = l2_q.copy()
            else:
                q2_filt = (1.0 - args.teleop_alpha) * q2_filt + args.teleop_alpha * l2_q

            l1_tau_ff = tc.scaled_gravity(gravity, l1_q, leader_gravity_scale)
            l2_tau_ff = tc.scaled_gravity(gravity, l2_q, leader_gravity_scale)

            tc.send_leader_damping(leader1, l1_q, dq_zero, l1_tau_ff, leader_kd)
            tc.send_leader_damping(leader2, l2_q, dq_zero, l2_tau_ff, leader_kd)
            follower1.send_hybrid_command(tc.as_f32(q1_filt), tc.as_f32(follower_dq_limit), tc.as_f32(follower_current_limit))
            follower2.send_hybrid_command(tc.as_f32(q2_filt), tc.as_f32(follower_dq_limit), tc.as_f32(follower_current_limit))

            l1_gs = leader1.get_gripper_status()
            l2_gs = leader2.get_gripper_status()
            leader1.send_gripper_command(q=l1_gs["q"], dq=0.0, tau=0.0, kp=0.0, kd=0.0, control_mode=1)
            leader2.send_gripper_command(q=l2_gs["q"], dq=0.0, tau=0.0, kp=0.0, kd=0.0, control_mode=1)
            follower1.send_gripper_command(q=l1_gs["q"], dq=0.0, tau=gripper_tau, kp=gripper_kp, kd=gripper_kd, control_mode=1)
            follower2.send_gripper_command(q=l2_gs["q"], dq=0.0, tau=gripper_tau, kp=gripper_kp, kd=gripper_kd, control_mode=1)

            now = time.perf_counter()
            if now - last_print >= args.print_interval:
                print(f"[teleop] {tc.format_status('L1', l1_status)} | {tc.format_status('F1', f1_status)}")
                print(f"[teleop] {tc.format_status('L2', l2_status)} | {tc.format_status('F2', f2_status)}")
                print(f"[teleop] grip L1={l1_gs['q']:.4f} F1={follower1.get_gripper_status()['q']:.4f} | L2={l2_gs['q']:.4f} F2={follower2.get_gripper_status()['q']:.4f}")
                last_print = now

            next_tick += args.period
            while not tc.STOP and time.perf_counter() < next_tick:
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
            tc.stop_and_disconnect(follower2, "follower2", mode="hybrid")
        if follower1_started:
            tc.stop_and_disconnect(follower1, "follower1", mode="hybrid")
        if leader2_started:
            tc.stop_and_disconnect(leader2, "leader2")
        if leader1_started:
            tc.stop_and_disconnect(leader1, "leader1")


if __name__ == "__main__":
    sys.exit(main())
