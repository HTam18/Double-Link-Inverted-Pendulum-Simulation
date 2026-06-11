from __future__ import annotations

import customtkinter as ctk
from python.gui.ui.widgets import figma


class ReplayPanel(ctk.CTkFrame):
    def __init__(self, parent, state, replay, load_command=None):
        super().__init__(parent, fg_color="transparent")
        self.state = state
        self.replay = replay
        self.load_command = load_command
        figma.label(self, "Replay", size=15, weight="bold").pack(anchor="w", pady=(0, 6))
        row = ctk.CTkFrame(self, fg_color="transparent")
        row.pack(fill="x")
        figma.button(row, "Load", self._load, accent=True).pack(side="left", fill="x", expand=True, padx=(0, 4))
        figma.button(row, "Play", self.replay.play, accent=True).pack(side="left", fill="x", expand=True, padx=4)
        figma.button(row, "Pause", self.replay.pause).pack(side="left", fill="x", expand=True, padx=4)
        figma.button(row, "Stop", self.replay.stop).pack(side="left", fill="x", expand=True, padx=(4, 0))
        self.slider = ctk.CTkSlider(self, from_=0, to=1, command=self._on_slider)
        self.slider.pack(fill="x", pady=(10, 0))
        self.label = figma.label(self, "0 / 0", size=11)
        self.label.pack(anchor="w", pady=(4, 0))
        self.speed_var = ctk.StringVar(value="1.0")
        figma.label(self, "Speed", size=12).pack(anchor="w", pady=(8, 2))
        figma.combo(self, ["0.5", "1.0", "2.0", "4.0", "8.0"], variable=self.speed_var, command=lambda _: self.replay.set_speed(float(self.speed_var.get()))).pack(fill="x")

    def _load(self):
        if self.load_command is not None:
            self.load_command()

    def _on_slider(self, value):
        if self.state.frame_count > 0:
            self.state.current_frame = int(float(value))

    def refresh(self):
        max_frame = max(0, self.state.frame_count - 1)
        self.slider.configure(to=max(1, max_frame))
        self.slider.set(min(self.state.current_frame, max_frame))
        self.label.configure(text=f"{self.state.current_frame} / {max_frame}")
