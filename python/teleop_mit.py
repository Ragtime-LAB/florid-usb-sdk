#!/usr/bin/env python3
"""
4-arm teleoperation — MIT protocol for all arms, with live tuning GUI.

Usage:
    PYTHONPATH=python python3 python/teleop_mit.py
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

import numpy as np

import florid_usb
import teleop_common as tc

FOLLOWER_KP = np.array([30.0, 80.0, 70.0, 70.0, 60.0, 20.0], dtype=np.float64)
FOLLOWER_KD = np.array([0.7, 1.2, 1.1, 1.0, 1.0, 0.5], dtype=np.float64)
LEADER_KD = np.zeros(tc.JOINT_COUNT, dtype=np.float64)

GRIPPER_KP = 10.0
GRIPPER_KD = 1.0
GRIPPER_TAU = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="4-arm teleoperation — MIT protocol"
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
    parser.add_argument("--follower-kp", type=float, nargs=6, default=FOLLOWER_KP.tolist(), metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    parser.add_argument("--follower-kd", type=float, nargs=6, default=FOLLOWER_KD.tolist(), metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    parser.add_argument("--teleop-alpha", type=float, default=0.2)
    parser.add_argument("--leader-gravity-scale", type=float, nargs=6, default=[1.0] * tc.JOINT_COUNT, metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    parser.add_argument("--follower-gravity-scale", type=float, nargs=6, default=[1.0] * tc.JOINT_COUNT, metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    parser.add_argument("--gripper-kp", type=float, default=GRIPPER_KP)
    parser.add_argument("--gripper-kd", type=float, default=GRIPPER_KD)
    parser.add_argument("--gripper-tau", type=float, default=GRIPPER_TAU)
    parser.add_argument("--no-gui", action="store_true", help="run without tuning GUI")
    return parser.parse_args()


def run_control(
    leader1: florid_usb.Arm, leader2: florid_usb.Arm,
    follower1: florid_usb.Arm, follower2: florid_usb.Arm,
    gravity: tc.HostGravityEstimator,
    freshness: tc.FreshnessMonitor,
    params: tc.TunableParams,
    args: argparse.Namespace,
) -> None:
    dq_zero = np.zeros(tc.JOINT_COUNT, dtype=np.float64)
    q1_filt: np.ndarray | None = None
    q2_filt: np.ndarray | None = None
    last_print = 0.0
    next_tick = time.perf_counter()

    while not tc.STOP:
        s = params.snapshot()

        l1_status = tc.get_status(leader1, "leader1", freshness)
        l2_status = tc.get_status(leader2, "leader2", freshness)
        f1_status = tc.get_status(follower1, "follower1", freshness)
        f2_status = tc.get_status(follower2, "follower2", freshness)
        l1_q, _, _ = tc.state_from_status(l1_status)
        l2_q, _, _ = tc.state_from_status(l2_status)
        f1_q, _, _ = tc.state_from_status(f1_status)
        f2_q, _, _ = tc.state_from_status(f2_status)

        a = s["teleop_alpha"]
        if q1_filt is None:
            q1_filt = l1_q.copy()
        else:
            q1_filt = (1.0 - a) * q1_filt + a * l1_q
        if q2_filt is None:
            q2_filt = l2_q.copy()
        else:
            q2_filt = (1.0 - a) * q2_filt + a * l2_q

        l1_tau_ff = tc.scaled_gravity(gravity, l1_q, s["leader_gravity_scale"])
        l2_tau_ff = tc.scaled_gravity(gravity, l2_q, s["leader_gravity_scale"])
        f1_tau_ff = tc.scaled_gravity(gravity, f1_q, s["follower_gravity_scale"])
        f2_tau_ff = tc.scaled_gravity(gravity, f2_q, s["follower_gravity_scale"])

        tc.send_leader_damping(leader1, l1_q, dq_zero, l1_tau_ff, s["leader_kd"])
        tc.send_leader_damping(leader2, l2_q, dq_zero, l2_tau_ff, s["leader_kd"])
        follower1.send_mit_command(tc.as_f32(q1_filt), tc.as_f32(dq_zero), tc.as_f32(f1_tau_ff), tc.as_f32(s["follower_kp"]), tc.as_f32(s["follower_kd"]))
        follower2.send_mit_command(tc.as_f32(q2_filt), tc.as_f32(dq_zero), tc.as_f32(f2_tau_ff), tc.as_f32(s["follower_kp"]), tc.as_f32(s["follower_kd"]))

        l1_gs = leader1.get_gripper_status()
        l2_gs = leader2.get_gripper_status()
        leader1.send_gripper_command(q=l1_gs["q"], dq=0.0, tau=0.0, kp=0.0, kd=0.0, control_mode=1)
        leader2.send_gripper_command(q=l2_gs["q"], dq=0.0, tau=0.0, kp=0.0, kd=0.0, control_mode=1)
        follower1.send_gripper_command(q=l1_gs["q"], dq=0.0, tau=s["gripper_tau"], kp=s["gripper_kp"], kd=s["gripper_kd"], control_mode=1)
        follower2.send_gripper_command(q=l2_gs["q"], dq=0.0, tau=s["gripper_tau"], kp=s["gripper_kp"], kd=s["gripper_kd"], control_mode=1)

        params.increment_loop()

        now = time.perf_counter()
        if now - last_print >= args.print_interval:
            t_f1 = follower1.get_gripper_status()["q"]
            t_f2 = follower2.get_gripper_status()["q"]
            print(f"[teleop] {tc.format_status('L1', l1_status)} | {tc.format_status('F1', f1_status)}")
            print(f"[teleop] {tc.format_status('L2', l2_status)} | {tc.format_status('F2', f2_status)}")
            print(f"[teleop] grip L1={l1_gs['q']:.4f} F1={t_f1:.4f} | L2={l2_gs['q']:.4f} F2={t_f2:.4f}")
            last_print = now

        next_tick += args.period
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

    params = tc.TunableParams(
        follower_kp=np.asarray(args.follower_kp, dtype=np.float64),
        follower_kd=np.asarray(args.follower_kd, dtype=np.float64),
        leader_kd=np.asarray(args.leader_kd, dtype=np.float64),
        teleop_alpha=args.teleop_alpha,
        leader_gravity_scale=np.asarray(args.leader_gravity_scale, dtype=np.float64),
        follower_gravity_scale=np.asarray(args.follower_gravity_scale, dtype=np.float64),
        gripper_kp=args.gripper_kp,
        gripper_kd=args.gripper_kd,
        gripper_tau=args.gripper_tau,
    )

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
    control_thread: threading.Thread | None = None

    try:
        tc.connect_and_start(leader1, "leader1", args.leader1_device)
        leader1_started = True
        tc.connect_and_start(leader2, "leader2", args.leader2_device)
        leader2_started = True
        tc.connect_and_start(follower1, "follower1", args.follower1_device)
        follower1_started = True
        tc.connect_and_start(follower2, "follower2", args.follower2_device)
        follower2_started = True

        tc.wait_for_fresh_status(
            [("leader1", leader1, args.leader1_device),
             ("leader2", leader2, args.leader2_device),
             ("follower1", follower1, args.follower1_device),
             ("follower2", follower2, args.follower2_device)],
            freshness,
            timeout_s=max(1.0, args.stale_timeout * 3.0),
            nonzero_eps=args.nonzero_eps,
        )

        if tc.STOP:
            return 0

        l1_q, _, _ = tc.get_state(leader1, "leader1", freshness)
        l2_q, _, _ = tc.get_state(leader2, "leader2", freshness)
        f1_start_q, _, _ = tc.get_state(follower1, "follower1", freshness)
        f2_start_q, _, _ = tc.get_state(follower2, "follower2", freshness)

        print("[teleop] syncing followers to current leader poses")
        print(f"[teleop] follower1 target={np.array2string(l1_q, precision=3, suppress_small=True)}")
        print(f"[teleop] follower2 target={np.array2string(l2_q, precision=3, suppress_small=True)}")

        dq_zero = np.zeros(tc.JOINT_COUNT, dtype=np.float64)
        sync_steps = max(2, int(args.sync_duration / args.period))
        sync_tick = time.perf_counter()
        for step in range(sync_steps):
            if tc.STOP:
                return 0
            frac = (step + 1) / sync_steps
            smooth = frac * frac * (3.0 - 2.0 * frac)
            f1_q_des = f1_start_q + smooth * (l1_q - f1_start_q)
            f2_q_des = f2_start_q + smooth * (l2_q - f2_start_q)

            l1_now_q, _, _ = tc.get_state(leader1, "leader1", freshness)
            l2_now_q, _, _ = tc.get_state(leader2, "leader2", freshness)

            l1_tau_ff = tc.scaled_gravity(gravity, l1_now_q, params.snapshot()["leader_gravity_scale"])
            l2_tau_ff = tc.scaled_gravity(gravity, l2_now_q, params.snapshot()["leader_gravity_scale"])

            tc.send_leader_damping(leader1, l1_now_q, dq_zero, l1_tau_ff, np.zeros(tc.JOINT_COUNT, dtype=np.float64))
            tc.send_leader_damping(leader2, l2_now_q, dq_zero, l2_tau_ff, np.zeros(tc.JOINT_COUNT, dtype=np.float64))
            follower1.send_mit_command(
                tc.as_f32(f1_q_des), tc.as_f32(dq_zero), tc.as_f32(tc.scaled_gravity(gravity, f1_q_des, params.snapshot()["follower_gravity_scale"])),
                tc.as_f32(params.snapshot()["follower_kp"]), tc.as_f32(params.snapshot()["follower_kd"]),
            )
            follower2.send_mit_command(
                tc.as_f32(f2_q_des), tc.as_f32(dq_zero), tc.as_f32(tc.scaled_gravity(gravity, f2_q_des, params.snapshot()["follower_gravity_scale"])),
                tc.as_f32(params.snapshot()["follower_kp"]), tc.as_f32(params.snapshot()["follower_kd"]),
            )

            sync_tick += args.period
            while not tc.STOP and time.perf_counter() < sync_tick:
                pass
            if time.perf_counter() - sync_tick > 0.2:
                sync_tick = time.perf_counter()

        if tc.STOP:
            return 0

        print("[teleop] 4-arm MIT teleop running, Ctrl+C to stop")
        print(f"[teleop] follower kp={args.follower_kp} kd={args.follower_kd}")

        control_thread = threading.Thread(
            target=run_control,
            args=(leader1, leader2, follower1, follower2, gravity, freshness, params, args),
            daemon=True,
        )
        control_thread.start()

        if args.no_gui:
            while not tc.STOP:
                time.sleep(0.1)
        else:
            from tuning_ui import TuningUI
            ui = TuningUI(params)
            ui.run()

        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"[teleop] error: {exc}")
        return 1
    finally:
        tc.STOP = True
        if control_thread is not None:
            control_thread.join(timeout=2.0)
        if follower2_started:
            tc.stop_and_disconnect(follower2, "follower2")
        if follower1_started:
            tc.stop_and_disconnect(follower1, "follower1")
        if leader2_started:
            tc.stop_and_disconnect(leader2, "leader2")
        if leader1_started:
            tc.stop_and_disconnect(leader1, "leader1")


if __name__ == "__main__":
    sys.exit(main())
