from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class TelemetrySeries:
    time_s: np.ndarray = field(default_factory=lambda: np.zeros(0))
    state: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))
    u_cmd: np.ndarray | None = None
    u_actual: np.ndarray | None = None
    mode: list[str] | None = None
    active_target: list[str] | None = None
    requested_target: list[str] | None = None
    force_x_N: np.ndarray | None = None
    force_active: np.ndarray | None = None
    force_event: dict[str, Any] | None = None
    source_file: Path | None = None
    source_kind: str = "unknown"

    @property
    def has_state(self) -> bool:
        return self.time_s.size > 0 and self.state.ndim == 2 and self.state.shape[1] >= 6

    @property
    def frame_count(self) -> int:
        if not self.has_state:
            return 0
        return int(min(len(self.time_s), self.state.shape[0]))

    def frame(self, index: int) -> dict[str, Any]:
        if not self.has_state:
            return {}
        idx = int(np.clip(index, 0, self.frame_count - 1))
        return {
            "time_s": float(self.time_s[idx]),
            "x": float(self.state[idx, 0]),
            "x_dot": float(self.state[idx, 1]),
            "theta1": float(self.state[idx, 2]),
            "theta1_dot": float(self.state[idx, 3]),
            "theta2": float(self.state[idx, 4]),
            "theta2_dot": float(self.state[idx, 5]),
            "u_cmd": None if self.u_cmd is None or idx >= len(self.u_cmd) else float(self.u_cmd[idx]),
            "u_actual": None if self.u_actual is None or idx >= len(self.u_actual) else float(self.u_actual[idx]),
            "mode": None if self.mode is None or idx >= len(self.mode) else self.mode[idx],
            "active_target": None if self.active_target is None or idx >= len(self.active_target) else self.active_target[idx],
            "requested_target": None if self.requested_target is None or idx >= len(self.requested_target) else self.requested_target[idx],
        }


@dataclass
class ResultSummary:
    workflow: str = "unknown"
    overall_pass: str = "unknown"
    success_rate: str = "unknown"
    backend: str = "MATLAB/Simulink"
    source_path: Path | None = None
    metrics: dict[str, str] = field(default_factory=dict)


@dataclass
class ResultBundle:
    telemetry: TelemetrySeries = field(default_factory=TelemetrySeries)
    summary: ResultSummary = field(default_factory=ResultSummary)
    tables: dict[str, list[dict[str, str]]] = field(default_factory=dict)
