from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
from scipy.io import loadmat

from python.gui.core.telemetry import ResultBundle, ResultSummary, TelemetrySeries
from python.gui.core.displayNames import display_route
from python.gui.services.workflows.common import as_list, float_vector, string_array
from python.gui.services.workflows.models import TARGET_TRANSITION_CASES


ROUTES: dict[str, list[str]] = {
    "down_down_to_up_up": ["down_down_to_up_up"],
    "up_up_to_up_down": ["up_up_to_up_down"],
    "up_down_to_up_up": ["up_down_to_up_up"],
    "up_up_to_down_up": ["up_up_to_down_up"],
    "down_up_to_up_up": ["down_up_to_up_up"],
    "down_down_to_up_down": ["down_down_to_up_up", "up_up_to_up_down"],
    "down_down_to_down_up": ["down_down_to_up_up", "up_up_to_down_up"],
    "up_down_to_down_up": ["up_down_to_up_up", "up_up_to_down_up"],
    "down_up_to_up_down": ["down_up_to_up_up", "up_up_to_up_down"],
}


def switching_log_to_series(data: dict[str, Any], path: Path) -> TelemetrySeries:
    log = data["log"]
    time = np.asarray(log["t"], dtype=float).reshape(-1)
    state = np.asarray(log["state"], dtype=float)
    if state.shape[0] != len(time) and state.shape[1] == len(time):
        state = state.T
    n = min(len(time), state.shape[0])
    return TelemetrySeries(
        time_s=time[:n],
        state=state[:n, :6],
        u_cmd=float_vector(log.get("u_cmd"), n),
        u_actual=float_vector(log.get("u_actual"), n),
        mode=string_array(log.get("mode"), n),
        active_target=string_array(log.get("active_target"), n),
        requested_target=string_array(log.get("requested_target"), n),
        source_file=path,
        source_kind="switching-full-log",
    )


def command_index(data: dict[str, Any]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for item in as_list(data.get("command_results")):
        if not isinstance(item, dict):
            continue
        edge = str(item.get("edge_names", ""))
        if edge:
            out[edge] = item
    return out


def slice_series(series: TelemetrySeries, start_s: float, end_s: float, edge_name: str) -> TelemetrySeries:
    mask = (series.time_s >= float(start_s)) & (series.time_s <= float(end_s))
    idx = np.where(mask)[0]
    if idx.size == 0:
        raise ValueError(f"No telemetry samples for edge {edge_name} [{start_s}, {end_s}].")
    start, end = int(idx[0]), int(idx[-1]) + 1
    return TelemetrySeries(
        time_s=series.time_s[start:end] - series.time_s[start],
        state=series.state[start:end].copy(),
        u_cmd=None if series.u_cmd is None else series.u_cmd[start:end].copy(),
        u_actual=None if series.u_actual is None else series.u_actual[start:end].copy(),
        mode=None if series.mode is None else list(series.mode[start:end]),
        active_target=None if series.active_target is None else list(series.active_target[start:end]),
        requested_target=None if series.requested_target is None else list(series.requested_target[start:end]),
        force_x_N=None,
        force_active=None,
        source_file=series.source_file,
        source_kind=f"switching-edge:{edge_name}",
    )


def concat_series(parts: list[TelemetrySeries], source_path: Path, source_kind: str) -> TelemetrySeries:
    if not parts:
        return TelemetrySeries(source_file=source_path, source_kind=source_kind)
    times: list[np.ndarray] = []
    states: list[np.ndarray] = []
    u_cmds: list[np.ndarray] = []
    u_actuals: list[np.ndarray] = []
    modes: list[str] = []
    active_targets: list[str] = []
    requested_targets: list[str] = []
    offset = 0.0
    dt = 0.02
    have_u_actual = all(p.u_actual is not None for p in parts)
    for i, part in enumerate(parts):
        t = np.asarray(part.time_s, dtype=float).reshape(-1)
        if t.size == 0:
            continue
        if t.size > 1:
            diffs = np.diff(t)
            valid = diffs[np.isfinite(diffs) & (diffs > 1e-8)]
            if valid.size:
                dt = float(np.median(valid))
        if i > 0:
            t = t[1:]
            state = part.state[1:, :]
            u_cmd = None if part.u_cmd is None else part.u_cmd[1:]
            u_actual = None if part.u_actual is None else part.u_actual[1:]
            mode = [] if part.mode is None else list(part.mode[1:])
            active_target = [] if part.active_target is None else list(part.active_target[1:])
            requested_target = [] if part.requested_target is None else list(part.requested_target[1:])
        else:
            state = part.state
            u_cmd = part.u_cmd
            u_actual = part.u_actual
            mode = [] if part.mode is None else list(part.mode)
            active_target = [] if part.active_target is None else list(part.active_target)
            requested_target = [] if part.requested_target is None else list(part.requested_target)
        if t.size == 0:
            continue
        t0 = t[0]
        new_t = (t - t0) + offset
        offset = float(new_t[-1] + dt)
        times.append(new_t)
        states.append(np.asarray(state, dtype=float))
        if u_cmd is not None:
            u_cmds.append(np.asarray(u_cmd, dtype=float).reshape(-1))
        if have_u_actual and u_actual is not None:
            u_actuals.append(np.asarray(u_actual, dtype=float).reshape(-1))
        modes.extend(mode)
        active_targets.extend(active_target)
        requested_targets.extend(requested_target)
    time_s = np.concatenate(times) if times else np.zeros(0)
    state = np.vstack(states) if states else np.zeros((0, 6))
    n = min(len(time_s), state.shape[0])
    return TelemetrySeries(
        time_s=time_s[:n],
        state=state[:n, :6],
        u_cmd=np.concatenate(u_cmds)[:n] if u_cmds else None,
        u_actual=np.concatenate(u_actuals)[:n] if u_actuals else None,
        mode=modes[:n] if modes else None,
        active_target=active_targets[:n] if active_targets else None,
        requested_target=requested_targets[:n] if requested_targets else None,
        source_file=source_path,
        source_kind=source_kind,
    )


def load_target_transition(project_root: Path, switching_result: Path, case_key: str) -> ResultBundle:
    path = project_root / switching_result
    if not path.exists():
        raise FileNotFoundError(path)
    data = loadmat(path, simplify_cells=True)
    full = switching_log_to_series(data, path)
    summary = ResultSummary(workflow="switching", overall_pass="1", backend="MATLAB/Simulink", source_path=path)
    case = next((c for c in TARGET_TRANSITION_CASES if c.key == case_key), None)
    if case is None:
        raise ValueError(f"Unknown target transition case: {case_key}")
    if case_key == "full_sequence":
        full.source_kind = "switching-target-full-sequence"
        summary.metrics.update({"workflow": "Target transition replay", "case": case.label, "route": "Complete target sequence"})
        return ResultBundle(telemetry=full, summary=summary)
    commands = command_index(data)
    route = ROUTES[case_key]
    parts: list[TelemetrySeries] = []
    for edge in route:
        cmd = commands.get(edge)
        if cmd is None:
            raise ValueError(f"Target transition result does not contain edge: {display_route([edge])}")
        parts.append(slice_series(full, float(cmd["start_time_s"]), float(cmd["end_time_s"]), edge))
    combined = concat_series(parts, path, f"switching-target-route:{case_key}")
    summary.metrics.update({"workflow": "Target transition replay", "case": case.label, "route": display_route(route), "composed": "1" if len(route) > 1 else "0"})
    return ResultBundle(telemetry=combined, summary=summary)
