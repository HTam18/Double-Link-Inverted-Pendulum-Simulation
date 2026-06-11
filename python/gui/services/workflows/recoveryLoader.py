from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
from scipy.io import loadmat

from python.gui.core.telemetry import ResultBundle, ResultSummary, TelemetrySeries
from python.gui.core.displayNames import display_name
from python.gui.services.workflows.common import float_vector, format_metric, load_csv_rows, same_decimal, string_array


FORCE_CASE_TYPES = ["target_recovery", "after_full_switching"]
FORCE_TARGETS = ["up_up", "up_down", "down_up"]
FORCE_LINK_IDS = ["1", "2"]
FORCE_CONTACT_RATIOS = ["0.25", "0.50", "0.75", "1.00"]
FORCE_DIRECTIONS = ["left", "right"]
FORCE_LEVELS_N = ["1"]
FORCE_METRIC_DISPLAY_FIELDS = [
    "success",
    "recovery_time_s",
    "saturation_fraction",
    "fail_reason",
    "final_angle_error_rad",
    "final_velocity_norm",
    "max_abs_x_m",
    "max_abs_u_cmd_N",
    "planner_selected_variant",
]


def force_metric_filter_values() -> dict[str, list[str]]:
    return {
        "case_type": list(FORCE_CASE_TYPES),
        "target": list(FORCE_TARGETS),
        "link_id": list(FORCE_LINK_IDS),
        "contact_ratio": list(FORCE_CONTACT_RATIOS),
        "direction": list(FORCE_DIRECTIONS),
        "force_N": list(FORCE_LEVELS_N),
    }


def force_metric_rows(path: Path) -> list[dict[str, str]]:
    return load_csv_rows(path)


def find_force_metric_case(path: Path, case_type: str, target: str, link_id: str, contact_ratio: str, direction: str, force_N: str) -> dict[str, str] | None:
    for row in force_metric_rows(path):
        if str(row.get("case_type", "")) != str(case_type):
            continue
        if str(row.get("target", "")) != str(target):
            continue
        if str(row.get("link_id", "")) != str(link_id):
            continue
        if not same_decimal(row.get("contact_ratio", ""), contact_ratio, 2):
            continue
        if str(row.get("direction", "")) != str(direction):
            continue
        if not same_decimal(row.get("force_N", ""), force_N, 0):
            continue
        return row
    return None


def format_force_metric_case(row: dict[str, str] | None) -> str:
    if not row:
        return "No metrics row found for selected filters."
    lines = [
        f"{display_name('case_index')}: {row.get('case_index', '--')}",
        f"{display_name('event_name')}: {row.get('event_name', '--')}",
    ]
    for field in FORCE_METRIC_DISPLAY_FIELDS:
        lines.append(f"{display_name(field)}: {format_metric(row.get(field, '--'))}")
    return "\n".join(lines)


def expected_case_type(case_key: str) -> str:
    return "after_full_switching" if case_key == "after_switching" else "target_recovery"


def metric_matches_rep(case_key: str, row: dict[str, str] | None, rep: dict[str, Any]) -> bool:
    if not row:
        return False
    event = rep.get("event", {}) if isinstance(rep, dict) else {}
    if not isinstance(event, dict):
        event = {}
    target = str(rep.get("target_name", ""))
    checks = [
        str(row.get("case_type", "")) == expected_case_type(case_key),
        str(row.get("target", "")) == target,
        str(row.get("link_id", "")) == str(event.get("link_id", "")),
        same_decimal(row.get("contact_ratio", ""), event.get("contact_ratio", ""), 2),
        str(row.get("direction", "")) == str(event.get("direction_name", "")),
        same_decimal(row.get("force_N", ""), event.get("force_N", ""), 0),
    ]
    return all(checks)


def event_dict_from_metric(row: dict[str, str] | None, fallback: dict[str, Any] | None) -> dict[str, Any] | None:
    base: dict[str, Any] = dict(fallback or {})
    if row:
        base.update({
            "name": row.get("event_name", base.get("name", "")),
            "link_id": row.get("link_id", base.get("link_id", "")),
            "contact_ratio": row.get("contact_ratio", base.get("contact_ratio", "")),
            "direction_name": row.get("direction", base.get("direction_name", "")),
            "force_N": row.get("force_N", base.get("force_N", "")),
            "duration_s": row.get("duration_s", base.get("duration_s", "0.08")),
            "case_index": row.get("case_index", ""),
            "case_type": row.get("case_type", ""),
            "target": row.get("target", ""),
        })
    if "start_time_s" not in base or base.get("start_time_s") in {None, ""}:
        base["start_time_s"] = 0.7
    try:
        base["end_time_s"] = float(base.get("start_time_s", 0.7)) + float(base.get("duration_s", 0.08))
    except Exception:
        base["end_time_s"] = base.get("end_time_s", 0.78)
    return base or None


def force_series_from_rep(rep: dict[str, Any], path: Path, kind: str, selected_metric_row: dict[str, str] | None = None) -> TelemetrySeries:
    time = np.asarray(rep["time_s"], dtype=float).reshape(-1)
    state = np.asarray(rep["states"], dtype=float)
    if state.shape[0] != len(time) and state.shape[1] == len(time):
        state = state.T
    n = min(len(time), state.shape[0])
    event = event_dict_from_metric(selected_metric_row, rep.get("event", {}))
    target = str((event or {}).get("target") or rep.get("target_name", ""))
    mode = string_array(rep.get("recovery_state"), n)
    return TelemetrySeries(
        time_s=time[:n],
        state=state[:n, :6],
        u_cmd=float_vector(rep.get("u_cmd"), n),
        u_actual=None,
        mode=mode,
        active_target=[target] * n,
        requested_target=[target] * n,
        force_x_N=float_vector(rep.get("force_x_N"), n),
        force_active=float_vector(rep.get("active"), n),
        force_event=event,
        source_file=path,
        source_kind=f"recovery-{kind}",
    )


def load_force_recovery(project_root: Path, recovery_result: Path, case_key: str, selected_metric_row: dict[str, str] | None = None) -> ResultBundle:
    path = project_root / recovery_result
    if not path.exists():
        raise FileNotFoundError(path)
    data = loadmat(path, simplify_cells=True)
    reps = data.get("representative_sims", {})
    if not isinstance(reps, dict):
        raise ValueError("Force recovery result does not contain representative simulations.")
    if case_key not in {"target_recovery", "after_switching"}:
        raise ValueError(f"Unknown force recovery case: {case_key}")
    rep = reps.get(case_key)
    if not isinstance(rep, dict):
        raise ValueError(f"Force recovery result does not contain representative simulation: {case_key}.")
    selected_has_trajectory = metric_matches_rep(case_key, selected_metric_row, rep)
    animation_metric_row = selected_metric_row if selected_has_trajectory else None
    series = force_series_from_rep(rep, path, case_key, animation_metric_row)
    event = series.force_event or rep.get("event", reps.get(f"{case_key}_event", {}))
    event_name = event.get("name", "") if isinstance(event, dict) else ""
    target_name = str(rep.get("target_name", reps.get(f"{case_key}_target", "")))
    summary_obj = data.get("summary", {}) if isinstance(data.get("summary", {}), dict) else {}
    summary = ResultSummary(
        workflow="recovery",
        overall_pass=str(summary_obj.get("overall_pass", "unknown")),
        success_rate=str(summary_obj.get("overall_success_rate", "unknown")),
        backend="MATLAB/Simulink",
        source_path=path,
        metrics={
            "workflow": "Force recovery replay",
            "case": display_name(case_key),
            "event": event_name,
            "target": str((event or {}).get("target", target_name)) if isinstance(event, dict) else target_name,
            "selected_metric_has_trajectory": "1" if selected_has_trajectory else "0",
            "animation_note": "Selected metrics row is the exported representative trajectory." if selected_has_trajectory else "Selected metrics row is metrics only; replay remains the selected representative trajectory.",
        },
    )
    return ResultBundle(telemetry=series, summary=summary)
