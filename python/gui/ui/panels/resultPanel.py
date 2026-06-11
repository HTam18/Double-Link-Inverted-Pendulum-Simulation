from __future__ import annotations

import customtkinter as ctk

from python.gui.core.displayNames import display_name, display_route
from python.gui.services.workflowLoader import WorkflowCase, WorkflowResultLoader
from python.gui.ui import theme
from python.gui.ui.widgets import figma


WORKFLOW_TARGET = "Target transition replay"
WORKFLOW_FORCE = "Force recovery replay"


class ResultPanel(ctk.CTkFrame):
    """Project-aware result selector.

    This panel only selects an existing MATLAB/Simulink workflow result.
    It does not run plant dynamics, controller, TVLQR, recovery, or numerical
    integration in Python.
    """

    def __init__(self, parent, state, on_loaded, on_selection_changed=None):
        super().__init__(parent, fg_color="transparent")
        self.state = state
        self.on_loaded = on_loaded
        self.on_selection_changed = on_selection_changed
        self.workflowLoader = WorkflowResultLoader(state.project_root)
        self.workflow_var = ctk.StringVar(value=WORKFLOW_TARGET)
        self.case_var = ctk.StringVar(value="")
        self._case_by_label: dict[str, WorkflowCase] = {}
        self._selected_force_metric_row: dict[str, str] | None = None

        self.grid_columnconfigure(0, weight=1)
        figma.label(self, "Results", size=15, weight="bold").grid(row=0, column=0, sticky="w", pady=(0, 8))

        self.workflow_row = self._field_row(self, "Workflow")
        self.workflow_row.grid(row=1, column=0, sticky="ew", pady=(0, 6))
        self.workflow_combo = figma.combo(
            self.workflow_row.input_host,
            [WORKFLOW_TARGET, WORKFLOW_FORCE],
            variable=self.workflow_var,
            command=lambda _: self._refresh_cases(),
        )
        self.workflow_combo.pack(fill="x")

        self.case_row = self._field_row(self, "Replay case")
        self.case_row.grid(row=2, column=0, sticky="ew", pady=(0, 2))
        self.case_combo = figma.combo(self.case_row.input_host, [], variable=self.case_var, command=lambda _: self._on_case_changed())
        self.case_combo.pack(fill="x")

        self._refresh_cases()

    def _field_row(self, parent, title: str):
        row = ctk.CTkFrame(parent, fg_color="transparent")
        row.grid_columnconfigure(0, weight=0, minsize=76)
        row.grid_columnconfigure(1, weight=1)
        figma.label(row, title, size=11, color=theme.MUTED).grid(row=0, column=0, sticky="w", padx=(0, 8))
        row.input_host = ctk.CTkFrame(row, fg_color="transparent")
        row.input_host.grid(row=0, column=1, sticky="ew")
        row.input_host.grid_columnconfigure(0, weight=1)
        return row

    def _cases_for_workflow(self) -> list[WorkflowCase]:
        if self.workflow_var.get() == WORKFLOW_FORCE:
            return self.workflowLoader.force_cases()
        return self.workflowLoader.target_cases()

    def _refresh_cases(self):
        cases = self._cases_for_workflow()
        self._case_by_label = {case.label: case for case in cases}
        labels = list(self._case_by_label.keys())
        self.case_combo.configure(values=labels)
        self.case_var.set(labels[0] if labels else "")
        self._notify_selection_changed()

    def _on_case_changed(self):
        self._notify_selection_changed()

    def _notify_selection_changed(self):
        if self.on_selection_changed:
            self.on_selection_changed(self.workflow_var.get(), self.selected_case_key())

    def selected_case_key(self) -> str:
        case = self._case_by_label.get(self.case_var.get())
        return "" if case is None else case.key

    def set_force_metric_row(self, row: dict[str, str] | None) -> None:
        self._selected_force_metric_row = row

    def load_case(self):
        case = self._case_by_label.get(self.case_var.get())
        if case is None:
            self.state.log("No replay case selected.")
            return
        try:
            if self.workflow_var.get() == WORKFLOW_FORCE:
                bundle = self.workflowLoader.load_force_recovery(case.key, self._selected_force_metric_row)
                if self._selected_force_metric_row:
                    for key, value in self._selected_force_metric_row.items():
                        if key in {
                            "case_index", "case_type", "target", "event_name", "link_id", "contact_ratio",
                            "direction", "force_N", "duration_s", "success", "recovery_time_s", "saturation_fraction",
                            "fail_reason", "final_angle_error_rad", "final_velocity_norm", "max_abs_x_m",
                            "max_abs_u_cmd_N", "planner_selected_variant",
                        }:
                            bundle.summary.metrics[f"metric.{key}"] = str(value)
            else:
                bundle = self.workflowLoader.load_target_transition(case.key)
            self.state.bundle = bundle
            self.state.selected_result_dir = bundle.summary.source_path.parent if bundle.summary.source_path else None
            self.state.reset_playback()
            self.state.status_text = case.label
            self.state.log(f"Loaded workflow: {self.workflow_var.get()} | {case.label}")
            route = bundle.summary.metrics.get("route")
            if route:
                self.state.log(f"Route: {route}")
            event = bundle.summary.metrics.get("event")
            if event:
                self.state.log(f"Replay event: {display_name(event)}")
            metric_event = bundle.summary.metrics.get("metric.event_name")
            if metric_event:
                self.state.log(f"Metric case: {display_name(metric_event)}")
            note = bundle.summary.metrics.get("animation_note")
            if note:
                self.state.log(f"Animation note: {note}")
            self.on_loaded()
        except Exception as exc:
            self.state.status_text = "Load failed"
            self.state.log(f"ERROR loading workflow result: {exc}")
