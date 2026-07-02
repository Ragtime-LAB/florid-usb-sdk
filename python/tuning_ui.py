#!/usr/bin/env python3
"""Tkinter GUI for runtime tuning of MIT teleoperation parameters."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

import numpy as np

import teleop_common as tc


class TuningUI:
    """Scrollable slider panel for live parameter adjustment."""

    def __init__(
        self,
        params: tc.TunableParams,
        on_close: callable | None = None,
    ) -> None:
        self.params = params
        self.on_close = on_close

        self.root = tk.Tk()
        self.root.title("MIT Tuning Panel")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        sw = self.root.winfo_screenwidth()
        self.root.geometry(f"520x800+{sw - 540}+40")

        self._build_ui()
        self.root.after(500, self._poll_freq)

    def _build_ui(self) -> None:
        canvas = tk.Canvas(self.root, borderwidth=0, highlightthickness=0)
        scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)

        inner = ttk.Frame(canvas, padding=6)
        inner.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")),
        )
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.bind_all("<MouseWheel>", lambda e: canvas.yview_scroll(int(-e.delta // 30), "units"))

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # frequency
        self.freq_var = tk.StringVar(value="Control Frequency: --- Hz")
        ttk.Label(
            inner,
            textvariable=self.freq_var,
            font=("Consolas", 12, "bold"),
            foreground="#0066cc",
        ).pack(fill="x", padx=4, pady=(0, 6))

        sn = self.params.snapshot()

        self._sliders: dict[str, list[ttk.Scale]] = {}
        self._svars: dict[str, list[tk.StringVar]] = {}

        self._build_section(inner, "Follower KP", 0.0, 200.0, sn["follower_kp"],
                            lambda a: self.params.update_follower_kp(a), "kp")
        self._build_section(inner, "Follower KD", 0.0, 5.0, sn["follower_kd"],
                            lambda a: self.params.update_follower_kd(a), "kd")
        self._build_section(inner, "Leader KD", 0.0, 2.0, sn["leader_kd"],
                            lambda a: self.params.update_leader_kd(a), "leader_kd")
        self._build_single(inner, "Teleop Alpha", 0.01, 1.0, sn["teleop_alpha"],
                           lambda v: self.params.update_teleop_alpha(v), "alpha")
        self._build_section(inner, "Leader Gravity Scale", 0.0, 2.0, sn["leader_gravity_scale"],
                            lambda a: self.params.update_leader_gravity_scale(a), "lgs")
        self._build_section(inner, "Follower Gravity Scale", 0.0, 2.0, sn["follower_gravity_scale"],
                            lambda a: self.params.update_follower_gravity_scale(a), "fgs")
        self._build_single(inner, "Gripper KP", 0.0, 50.0, sn["gripper_kp"],
                           lambda v: self.params.update_gripper_kp(v), "gkp")
        self._build_single(inner, "Gripper KD", 0.0, 5.0, sn["gripper_kd"],
                           lambda v: self.params.update_gripper_kd(v), "gkd")
        self._build_single(inner, "Gripper Tau", -5.0, 5.0, sn["gripper_tau"],
                           lambda v: self.params.update_gripper_tau(v), "gtau")

    def _build_section(
        self,
        parent: ttk.Frame,
        title: str,
        vmin: float, vmax: float,
        init: np.ndarray,
        update_fn: callable,
        key: str,
    ) -> None:
        frame = ttk.LabelFrame(parent, text=title, padding=4)
        frame.pack(fill="x", padx=4, pady=2)
        scales: list[ttk.Scale] = []
        svars: list[tk.StringVar] = []

        for i in range(6):
            row = ttk.Frame(frame)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text=f"J{i+1}:", width=3).pack(side="left")
            var = tk.StringVar(value=f"{init[i]:.3f}")
            ttk.Label(row, textvariable=var, width=8, anchor="e").pack(side="right")
            scale = ttk.Scale(row, from_=vmin, to=vmax, value=init[i])
            scale.pack(side="left", fill="x", expand=True, padx=(2, 2))
            scales.append(scale)
            svars.append(var)

        for i, s in enumerate(scales):
            def _cb(val: str, idx: int = i, sc: list[ttk.Scale] = scales,
                    vr: list[tk.StringVar] = svars, fn: callable = update_fn) -> None:
                vr[idx].set(f"{float(val):.3f}")
                arr = np.array([x.get() for x in sc], dtype=np.float64)
                fn(arr)
            s.configure(command=_cb)

        self._sliders[key] = scales
        self._svars[key] = svars

    def _build_single(
        self,
        parent: ttk.Frame,
        title: str,
        vmin: float, vmax: float,
        init: float,
        update_fn: callable,
        key: str,
    ) -> None:
        frame = ttk.LabelFrame(parent, text=title, padding=4)
        frame.pack(fill="x", padx=4, pady=2)
        var = tk.StringVar(value=f"{init:.3f}")
        row = ttk.Frame(frame)
        row.pack(fill="x", pady=1)
        ttk.Label(row, textvariable=var, width=8, anchor="e").pack(side="right")
        scale = ttk.Scale(row, from_=vmin, to=vmax, value=init)
        scale.pack(side="left", fill="x", expand=True, padx=(2, 2))

        def _cb(val: str) -> None:
            var.set(f"{float(val):.3f}")
            update_fn(float(val))
        scale.configure(command=_cb)

        self._sliders[key] = [scale]
        self._svars[key] = [var]

    def _poll_freq(self) -> None:
        freq = self.params.poll_freq()
        self.freq_var.set(f"Control Frequency: {freq:.1f} Hz")
        if not tc.STOP:
            self.root.after(500, self._poll_freq)

    def _on_close(self) -> None:
        tc.STOP = True
        if self.on_close:
            self.on_close()
        self.root.quit()

    def run(self) -> None:
        self.root.mainloop()
