from __future__ import annotations

from pathlib import Path

from python.gui.services.workflows.models import (
    FORCE_RECOVERY_CASES,
    RECOVERY_METRICS_CSV,
    RECOVERY_RESULT,
    SWITCHING_RESULT,
    TARGET_TRANSITION_CASES,
    WorkflowCase,
)
from python.gui.services.workflows.recoveryLoader import (
    find_force_metric_case,
    force_metric_filter_values,
    force_metric_rows,
    format_force_metric_case,
    load_force_recovery,
)
from python.gui.services.workflows.switchingLoader import load_target_transition


class WorkflowResultLoader:
    def __init__(self, project_root: Path):
        self.project_root = Path(project_root)

    @property
    def switching_path(self) -> Path:
        return self.project_root / SWITCHING_RESULT

    @property
    def recovery_path(self) -> Path:
        return self.project_root / RECOVERY_RESULT

    @property
    def recovery_metrics_path(self) -> Path:
        return self.project_root / RECOVERY_METRICS_CSV

    def target_cases(self) -> list[WorkflowCase]:
        return list(TARGET_TRANSITION_CASES)

    def force_cases(self) -> list[WorkflowCase]:
        return list(FORCE_RECOVERY_CASES)

    def force_metric_filter_values(self) -> dict[str, list[str]]:
        return force_metric_filter_values()

    def force_metric_rows(self) -> list[dict[str, str]]:
        return force_metric_rows(self.recovery_metrics_path)

    def find_force_metric_case(self, case_type: str, target: str, link_id: str, contact_ratio: str, direction: str, force_N: str) -> dict[str, str] | None:
        return find_force_metric_case(self.recovery_metrics_path, case_type, target, link_id, contact_ratio, direction, force_N)

    def format_force_metric_case(self, row: dict[str, str] | None) -> str:
        return format_force_metric_case(row)

    def load_target_transition(self, case_key: str):
        return load_target_transition(self.project_root, SWITCHING_RESULT, case_key)

    def load_force_recovery(self, case_key: str, selected_metric_row: dict[str, str] | None = None):
        return load_force_recovery(self.project_root, RECOVERY_RESULT, case_key, selected_metric_row)
