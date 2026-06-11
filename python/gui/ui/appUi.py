from __future__ import annotations

import customtkinter as ctk

from python.gui.ui import theme
from python.gui.ui.widgets import figma
from python.gui.ui.panels.headerPanel import HeaderPanel
from python.gui.ui.panels.resultPanel import ResultPanel
from python.gui.ui.panels.replayPanel import ReplayPanel
from python.gui.ui.panels.forceMetricsPanel import ForceMetricsPanel
from python.gui.ui.panels.pendulumPanel import PendulumPanel
from python.gui.ui.panels.graphPanel import GraphPanel
from python.gui.ui.panels.logPanel import LogPanel


class AppUI:
    def __init__(self, root, state, replay):
        self.root = root
        self.state = state
        self.replay = replay
        self.panels = []
        self._build_skeleton()
        self._build_panels()
        self.root.after(16, self._tick)

    def _build_skeleton(self):
        self.root.title("Double Link Pendulum Dashboard")
        self.root.geometry("1380x820")
        self.root.minsize(1180, 740)
        self.root.configure(fg_color=theme.BG)

        self.root.grid_columnconfigure(0, weight=0, minsize=455)
        self.root.grid_columnconfigure(1, weight=1, uniform="content")
        self.root.grid_columnconfigure(2, weight=1, uniform="content")
        self.root.grid_rowconfigure(2, weight=1)

        self.header_host = ctk.CTkFrame(self.root, fg_color=theme.BG)
        self.header_host.grid(row=0, column=0, columnspan=3, sticky="ew", padx=18, pady=(12, 6))

        self.left = figma.frame(self.root, width=455)
        self.left.grid(row=1, column=0, rowspan=2, sticky="nsw", padx=(18, 8), pady=(0, 18))
        self.left.grid_propagate(False)
        self.left.grid_columnconfigure(0, weight=1)

        self.log_host = figma.frame(self.root)
        self.log_host.grid(row=1, column=1, columnspan=2, sticky="nsew", padx=(8, 18), pady=(0, 8))
        self.log_host.grid_columnconfigure(0, weight=1)
        self.log_host.grid_rowconfigure(0, weight=1)

        self.replay_host = figma.frame(self.root)
        self.replay_host.grid(row=2, column=1, sticky="nsew", padx=(8, 8), pady=(8, 18))
        self.replay_host.grid_columnconfigure(0, weight=1)
        self.replay_host.grid_rowconfigure(0, weight=1)

        self.graph_host = figma.frame(self.root)
        self.graph_host.grid(row=2, column=2, sticky="nsew", padx=(8, 18), pady=(8, 18))
        self.graph_host.grid_columnconfigure(0, weight=1)
        self.graph_host.grid_rowconfigure(0, weight=1)

    def _build_panels(self):
        self.header = HeaderPanel(self.header_host, self.state)
        self.header.pack(fill="x")

        self.resultPanel = ResultPanel(self.left, self.state, self._on_loaded, self._on_result_selection_changed)
        self.resultPanel.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 6))
        self.replayPanel = ReplayPanel(self.left, self.state, self.replay, load_command=self.resultPanel.load_case)
        self.replayPanel.grid(row=1, column=0, sticky="ew", padx=10, pady=6)
        self.forceMetricsPanel = ForceMetricsPanel(
            self.left, self.state, self.resultPanel.workflowLoader, on_metric_changed=self.resultPanel.set_force_metric_row
        )
        self.forceMetricsPanel.grid(row=2, column=0, sticky="ew", padx=10, pady=(6, 10))
        self.logPanel = LogPanel(self.log_host, self.state)
        self.logPanel.grid(row=0, column=0, sticky="nsew", padx=12, pady=12)

        self.pendulumPanel = PendulumPanel(self.replay_host, self.state)
        self.pendulumPanel.grid(row=0, column=0, sticky="nsew", padx=12, pady=12)

        self.graphPanel = GraphPanel(self.graph_host, self.state)
        self.graphPanel.grid(row=0, column=0, sticky="nsew", padx=12, pady=12)

        self._on_result_selection_changed(self.resultPanel.workflow_var.get(), self.resultPanel.selected_case_key())

        self.panels = [
            self.header,
            self.replayPanel,
            self.pendulumPanel,
            self.graphPanel,
            self.logPanel,
        ]

    def _on_result_selection_changed(self, workflow: str, case_key: str):
        is_force = workflow == "Force recovery replay"
        if hasattr(self, "forceMetricsPanel"):
            if is_force:
                self.forceMetricsPanel.grid()
                self.forceMetricsPanel.set_defaults_for_replay_case(case_key)
            else:
                self.forceMetricsPanel.grid_remove()

    def _on_loaded(self):
        self.graphPanel.refresh(force=True)
        self.pendulumPanel.refresh(force=True)

    def _tick(self):
        if self.state.is_playing:
            self.replay.step()
        for panel in self.panels:
            if hasattr(panel, "refresh"):
                panel.refresh()
        self.root.after(16, self._tick)
