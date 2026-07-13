#!/usr/bin/env python3
"""
Tkinter gripper controller.

Connects to a Florid arm over a USB CDC serial port, displays the live
gripper (joint 6) status, and provides a "回零" (set zero point) button that
calibrates the gripper's current position as its zero reference via
``Arm.set_zero(6)``.

Usage:
    PYTHONPATH=build/python python python/gripper_controller.py [/dev/ttyACM0]
"""

import sys
import threading
import time

import tkinter as tk
from tkinter import ttk, messagebox

import florid_usb

GRIPPER_JOINT_ID = 6


class GripperController:
    def __init__(self, root: tk.Tk, default_device: str):
        self.root = root
        self.arm = None
        self.running = False
        self.lock = threading.Lock()
        self.last_status = {}

        self.device_var = tk.StringVar(value=default_device)
        self.baud_var = tk.StringVar(value="115200")

        self.root.title("Gripper Controller")
        self.root.geometry("420x360")
        self._build_ui()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ──────────────────────────────────────────────
    #  UI
    # ──────────────────────────────────────────────

    def _build_ui(self) -> None:
        frm = ttk.Frame(self.root, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)

        # Connection row
        ttk.Label(frm, text="Serial port:").grid(row=0, column=0, sticky="w")
        ttk.Entry(frm, textvariable=self.device_var).grid(row=0, column=1, sticky="ew")

        ttk.Label(frm, text="Baud:").grid(row=1, column=0, sticky="w")
        ttk.Entry(frm, textvariable=self.baud_var, width=10).grid(
            row=1, column=1, sticky="w")

        self.connect_btn = ttk.Button(
            frm, text="Connect", command=self._connect)
        self.connect_btn.grid(row=2, column=0, pady=4)
        self.disconnect_btn = ttk.Button(
            frm, text="Disconnect", command=self._disconnect, state="disabled")
        self.disconnect_btn.grid(row=2, column=1, pady=4, sticky="w")

        # Status frame
        status_frm = ttk.LabelFrame(frm, text="Gripper status (joint 6)", padding=8)
        status_frm.grid(row=3, column=0, columnspan=2, sticky="ew", pady=8)

        self.fields = {}
        for i, key in enumerate(
                ("q (rad)", "dq (rad/s)", "tau (N·m)", "temp (°C)", "enabled")):
            ttk.Label(status_frm, text=key).grid(row=i, column=0, sticky="w")
            var = tk.StringVar(value="—")
            ttk.Label(status_frm, textvariable=var, anchor="e").grid(
                row=i, column=1, sticky="ew")
            self.fields[key] = var

        # Zero button
        self.zero_btn = ttk.Button(
            frm, text="回零 (设零点)", command=self._set_zero, state="disabled")
        self.zero_btn.grid(row=4, column=0, columnspan=2, sticky="ew", pady=6)

        # Log
        log_frm = ttk.LabelFrame(frm, text="Log", padding=4)
        log_frm.grid(row=5, column=0, columnspan=2, sticky="nsew", pady=4)
        frm.rowconfigure(5, weight=1)
        log_frm.rowconfigure(0, weight=1)
        log_frm.columnconfigure(0, weight=1)

        self.log_text = tk.Text(log_frm, height=6, state="disabled")
        self.log_text.grid(row=0, column=0, sticky="nsew")

        frm.columnconfigure(1, weight=1)

    # ──────────────────────────────────────────────
    #  Connection
    # ──────────────────────────────────────────────

    def _connect(self) -> None:
        device = self.device_var.get().strip() or "/dev/ttyACM0"
        try:
            baud = int(self.baud_var.get() or "115200")
        except ValueError:
            baud = 115200

        def worker() -> None:
            try:
                cfg = florid_usb.Config()
                cfg.device = device
                cfg.baud_rate = baud
                arm = florid_usb.Arm(cfg)
                if not arm.connect():
                    self._log(f"failed to open {device}")
                    return
                if not arm.start_session(timeout=1.0):
                    self._log("start_session failed")
                    arm.disconnect()
                    return
                with self.lock:
                    self.arm = arm
                    self.running = True
                self._log(f"connected to {device}")
                self.root.after(0, self._on_connected)
                self._poll_loop()
            except Exception as exc:  # noqa: BLE001
                self._log(f"connect error: {exc}")

        threading.Thread(target=worker, daemon=True).start()

    def _on_connected(self) -> None:
        self.connect_btn.configure(state="disabled")
        self.disconnect_btn.configure(state="normal")
        self.zero_btn.configure(state="normal")

    def _disconnect(self) -> None:
        with self.lock:
            self.running = False
        time.sleep(0.15)  # let poll loop exit
        with self.lock:
            arm = self.arm
            self.arm = None
        if arm is not None:
            try:
                arm.stop_session(timeout=1.0)
            except Exception:  # noqa: BLE001
                pass
            arm.disconnect()
        self._log("disconnected")
        self.connect_btn.configure(state="normal")
        self.disconnect_btn.configure(state="disabled")
        self.zero_btn.configure(state="disabled")
        for var in self.fields.values():
            var.set("—")

    # ──────────────────────────────────────────────
    #  Polling
    # ──────────────────────────────────────────────

    def _poll_loop(self) -> None:
        while True:
            with self.lock:
                arm = self.arm
                running = self.running
            if not running or arm is None:
                break
            try:
                gs = arm.get_gripper_status()
                with self.lock:
                    self.last_status = gs
                self.root.after(0, self._update_status, gs)
            except Exception as exc:  # noqa: BLE001
                self._log(f"status error: {exc}")
            time.sleep(0.1)

    def _update_status(self, gs: dict) -> None:
        self.fields["q (rad)"].set(f"{float(gs.get('q', 0.0)):.4f}")
        self.fields["dq (rad/s)"].set(f"{float(gs.get('dq', 0.0)):.4f}")
        self.fields["tau (N·m)"].set(f"{float(gs.get('tau', 0.0)):.4f}")
        self.fields["temp (°C)"].set(f"{float(gs.get('temp_c', 0.0)):.1f}")
        self.fields["enabled"].set("yes" if int(gs.get("enabled", 0)) else "no")

    # ──────────────────────────────────────────────
    #  Zero / set-zero-point
    # ──────────────────────────────────────────────

    def _set_zero(self) -> None:
        def worker() -> None:
            with self.lock:
                arm = self.arm
            if arm is None:
                self._log("not connected")
                return
            try:
                ok = arm.set_zero(GRIPPER_JOINT_ID, timeout=1.0)
                self._log("回零: gripper zero point set"
                          if ok else "回零 failed (firmware rejected)")
            except Exception as exc:  # noqa: BLE001
                self._log(f"set_zero error: {exc}")

        threading.Thread(target=worker, daemon=True).start()

    # ──────────────────────────────────────────────
    #  Helpers
    # ──────────────────────────────────────────────

    def _log(self, msg: str) -> None:
        self.root.after(0, self._append_log, msg)

    def _append_log(self, msg: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", msg + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _on_close(self) -> None:
        if self.arm is not None:
            self._disconnect()
        self.root.destroy()


def main() -> int:
    default_device = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    root = tk.Tk()
    GripperController(root, default_device)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
