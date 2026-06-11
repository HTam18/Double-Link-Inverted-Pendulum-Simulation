from __future__ import annotations

import customtkinter as ctk
from python.gui.ui import theme
from python.gui.ui.widgets import figma


class HeaderPanel(ctk.CTkFrame):
    def __init__(self, parent, state):
        super().__init__(parent, fg_color=theme.BG)
        self.state = state
        self.title = figma.label(self, "Double Link Pendulum Dashboard", size=20, weight="bold")
        self.title.grid(row=0, column=0, sticky="w")
        self.status = figma.label(self, "Ready", size=12, color=theme.ACCENT)
        self.status.grid(row=0, column=1, sticky="e")
        self.grid_columnconfigure(0, weight=1)

    def refresh(self):
        self.status.configure(text=self.state.status_text)
