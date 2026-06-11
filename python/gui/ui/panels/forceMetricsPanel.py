from __future__ import annotations

import customtkinter as ctk

from python.gui.core.displayNames import display_name
from python.gui.services.workflowLoader import WorkflowResultLoader
from python.gui.ui import theme
from python.gui.ui.widgets import figma


class ForceMetricsPanel(ctk.CTkFrame):
    """Force recovery case metrics browser.

    This panel reads CSV metrics exported by MATLAB/Simulink. It does not create
    new recovery simulations or trajectories in Python.
    """

    FIELD_LABELS = {
        "case_type": "Case type",
        "target": "Target",
        "force_N": "Force N",
        "link_id": "Link",
        "contact_ratio": "Contact ratio",
        "direction": "Direction",
    }

    def __init__(self, parent, state, workflowLoader: WorkflowResultLoader, on_metric_changed=None):
        super().__init__(parent, fg_color="transparent")
        self.state = state
        self.workflowLoader = workflowLoader
        self.on_metric_changed = on_metric_changed
        self.force_vars: dict[str, ctk.StringVar] = {}
        self.force_combos: dict[str, ctk.CTkComboBox] = {}
        self.display_to_raw: dict[str, dict[str, str]] = {}
        self.raw_to_display: dict[str, dict[str, str]] = {}
        self._build()
        self.set_defaults_for_replay_case("target_recovery")
        self._update_force_metrics()

    def _display_values(self, key: str, values: list[str]) -> list[str]:
        displays = [display_name(v) for v in values]
        self.display_to_raw[key] = dict(zip(displays, values))
        self.raw_to_display[key] = dict(zip(values, displays))
        return displays

    def _set_raw_value(self, key: str, raw_value: str) -> None:
        if key in self.force_vars:
            self.force_vars[key].set(self.raw_to_display.get(key, {}).get(raw_value, display_name(raw_value)))

    def _raw_value(self, key: str) -> str:
        value = self.force_vars[key].get()
        return self.display_to_raw.get(key, {}).get(value, value)

    def _compact_field(self, parent, key: str, title: str, values: list[str], row: int, col: int):
        host = ctk.CTkFrame(parent, fg_color="transparent")
        host.grid(row=row, column=col, sticky="ew", padx=3, pady=(0, 6))
        host.grid_columnconfigure(0, weight=1)
        figma.label(host, title, size=10, color=theme.MUTED).grid(row=0, column=0, sticky="w", pady=(0, 2))
        display_values = self._display_values(key, values or [""])
        var = ctk.StringVar(value=display_values[0])
        combo = figma.combo(host, display_values, variable=var, command=lambda _=None: self._update_force_metrics())
        combo.grid(row=1, column=0, sticky="ew")
        self.force_vars[key] = var
        self.force_combos[key] = combo

    def _build(self):
        self.grid_columnconfigure((0, 1, 2), weight=1, uniform="force")
        figma.label(self, "Force case metrics", size=15, weight="bold").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(2, 8)
        )
        values = self.workflowLoader.force_metric_filter_values()
        self._compact_field(self, "case_type", self.FIELD_LABELS["case_type"], values.get("case_type", [""]), 1, 0)
        self._compact_field(self, "target", self.FIELD_LABELS["target"], values.get("target", [""]), 1, 1)
        self._compact_field(self, "force_N", self.FIELD_LABELS["force_N"], values.get("force_N", ["1"]), 1, 2)
        self._compact_field(self, "link_id", self.FIELD_LABELS["link_id"], values.get("link_id", [""]), 2, 0)
        self._compact_field(self, "contact_ratio", self.FIELD_LABELS["contact_ratio"], values.get("contact_ratio", [""]), 2, 1)
        self._compact_field(self, "direction", self.FIELD_LABELS["direction"], values.get("direction", [""]), 2, 2)

        self.metric_text = ctk.CTkTextbox(
            self,
            height=150,
            corner_radius=10,
            fg_color="#FFFFFF",
            border_width=1,
            border_color=theme.BORDER,
            font=ctk.CTkFont(family=theme.MONO_FONT, size=10),
            text_color=theme.TEXT,
        )
        self.metric_text.grid(row=3, column=0, columnspan=3, sticky="ew", padx=3, pady=(2, 0))
        self.metric_text.insert("end", "Select filters to view metrics.")
        self.metric_text.configure(state="disabled")

    def set_defaults_for_replay_case(self, case_key: str) -> None:
        if case_key == "after_switching":
            defaults = {
                "case_type": "after_full_switching",
                "target": "up_up",
                "link_id": "2",
                "contact_ratio": "1.00",
                "direction": "left",
                "force_N": "1",
            }
        else:
            defaults = {
                "case_type": "target_recovery",
                "target": "up_up",
                "link_id": "1",
                "contact_ratio": "0.25",
                "direction": "right",
                "force_N": "1",
            }
        for key, value in defaults.items():
            self._set_raw_value(key, value)
        self._update_force_metrics()

    def selected_force_filters(self) -> dict[str, str]:
        return {key: self._raw_value(key) for key in self.force_vars}

    def _update_force_metrics(self):
        f = self.selected_force_filters()
        row = self.workflowLoader.find_force_metric_case(
            f.get("case_type", ""),
            f.get("target", ""),
            f.get("link_id", ""),
            f.get("contact_ratio", ""),
            f.get("direction", ""),
            f.get("force_N", ""),
        )
        text = self.workflowLoader.format_force_metric_case(row)
        self.metric_text.configure(state="normal")
        self.metric_text.delete("1.0", "end")
        self.metric_text.insert("end", text)
        self.metric_text.configure(state="disabled")
        if self.on_metric_changed:
            self.on_metric_changed(row)
