from __future__ import annotations

import customtkinter as ctk
from python.gui.ui.widgets import figma


class LogPanel(ctk.CTkFrame):
    def __init__(self, parent, state):
        super().__init__(parent, fg_color="transparent")
        self.state = state
        figma.label(self, "Log", size=15, weight="bold").pack(anchor="w", pady=(0, 6))
        self.text = ctk.CTkTextbox(self, height=90, corner_radius=12)
        self.text.pack(fill="both", expand=True)
        self.text.insert("end", "GUI initialized.\n")
        self.text.configure(state="disabled")

    def refresh(self):
        changed = False
        while not self.state.log_queue.empty():
            msg = self.state.log_queue.get_nowait()
            self.text.configure(state="normal")
            self.text.insert("end", f"{msg}\n")
            self.text.see("end")
            changed = True
        if changed:
            self.text.configure(state="disabled")
