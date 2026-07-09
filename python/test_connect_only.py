#!/usr/bin/env python3
"""
Minimal connect-only test — connects and starts session but sends NO control
commands.  Lets you observe what the firmware does during the initial
SessionStart → pcMode transition in isolation.

Usage:
    PYTHONPATH=python python3 python/test_connect_only.py
    PYTHONPATH=python python3 python/test_connect_only.py /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

import florid_usb


STOP = False
DEFAULT_DEVICE = "/dev/ttyACM0"


def _signal_handler(signum, _frame) -> None:
    global STOP
    STOP = True
    print(f"\n[connect_test] signal {signum}, stopping...")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Minimal connect test — no control commands"
    )
    parser.add_argument("device", nargs="?", default=DEFAULT_DEVICE)
    parser.add_argument(
        "--duration",
        type=float,
        default=15.0,
        help="how long to stay connected before stopping (seconds)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.05,
        help="status poll interval (seconds)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    cfg = florid_usb.Config()
    cfg.device = args.device
    arm = florid_usb.Arm(cfg)

    print(f"[connect_test] opening {args.device}")
    t0 = time.perf_counter()
    if not arm.connect():
        print("[connect_test] connect failed")
        return 1
    print(f"[connect_test] connected in {time.perf_counter() - t0:.3f}s")

    # ── Poll status before session starts ──
    print("[connect_test] polling status before session...")
    for i in range(5):
        if STOP:
            break
        try:
            s = arm.get_arm_status()
            print(
                f"  [{i}] q={[f'{v:.3f}' for v in s['q']]} "
                f"mode={s.get('arm_mode', '?')}"
            )
        except Exception as e:
            print(f"  [{i}] get_arm_status failed: {e}")
        time.sleep(0.05)

    # ── Start session ──
    session_started = False
    print("[connect_test] starting session...")
    t0 = time.perf_counter()
    if not arm.start_session(timeout=3.0):
        print("[connect_test] start_session failed")
        arm.disconnect()
        return 1
    print(f"[connect_test] session active in {time.perf_counter() - t0:.3f}s")
    session_started = True

    # ── Poll status — NO control commands sent ──
    print("[connect_test] observing — no MIT commands will be sent")
    last_print = 0.0
    start = time.perf_counter()
    first_q = None

    while not STOP and (time.perf_counter() - start) < args.duration:
        try:
            s = arm.get_arm_status()
            q = s["q"]
            dq = s["dq"]
            tau = s["tau"]
            mode = s.get("arm_mode", "?")

            if first_q is None:
                first_q = list(q)

            t_elapsed = time.perf_counter() - start
            now = time.perf_counter()
            if now - last_print >= args.interval:
                q_str = ",".join(f"{v:+.3f}" for v in q)
                dq_str = ",".join(f"{v:+.3f}" for v in dq)
                tau_str = ",".join(f"{v:+.3f}" for v in tau)
                print(
                    f"[{t_elapsed:6.2f}s] "
                    f"mode={mode} "
                    f"q=[{q_str}] "
                    f"dq=[{dq_str}] "
                    f"tau=[{tau_str}]"
                )
                last_print = now

                # Print a warning if joints are moving significantly
                for i in range(6):
                    if abs(dq[i]) > 0.5:
                        print(
                            f"  ⚠ joint {i+1} moving fast: "
                            f"q={q[i]:+.3f} dq={dq[i]:+.3f} tau={tau[i]:+.3f}"
                        )

        except Exception as e:
            print(f"  poll error: {e}")

        time.sleep(0.002)  # 500Hz-ish, don't busy-spin

    # ── Summary ──
    duration = time.perf_counter() - start
    print(f"\n[connect_test] observed for {duration:.1f}s")
    if first_q:
        s = arm.get_arm_status()
        final_q = s["q"]
        diff = [final_q[i] - first_q[i] for i in range(6)]
        max_move = max(abs(d) for d in diff)
        print(f"  initial q: {[f'{v:.3f}' for v in first_q]}")
        print(f"  final   q: {[f'{v:.3f}' for v in final_q]}")
        print(f"  max joint drift: {max_move:.3f} rad")
        if max_move > 0.1:
            print("  ⚠ significant drift detected during connect-only phase")

    # ── Cleanup ──
    if session_started:
        print("[connect_test] stopping session...")
        arm.stop_session(1.0)
    arm.disconnect()
    print("[connect_test] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
