from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

import numpy as np
from scipy.io import loadmat

from python.gui.core.telemetry import ResultBundle, ResultSummary, TelemetrySeries


def _as_array(value: Any) -> np.ndarray | None:
    try:
        arr = np.asarray(value)
    except Exception:
        return None
    if arr.size == 0:
        return None
    return np.squeeze(arr)


def _as_float_array(value: Any) -> np.ndarray | None:
    arr = _as_array(value)
    if arr is None:
        return None
    try:
        out = np.asarray(arr, dtype=float)
    except Exception:
        return None
    if out.size == 0:
        return None
    return np.squeeze(out)


def _as_string_list(value: Any, n: int | None = None) -> list[str] | None:
    arr = _as_array(value)
    if arr is None:
        return None
    if isinstance(arr, np.ndarray):
        values = arr.reshape(-1).tolist()
    elif isinstance(arr, (list, tuple)):
        values = list(arr)
    else:
        values = [arr]
    out = [str(v) for v in values]
    return out[:n] if n is not None else out


def _walk(obj: Any, depth: int = 0, max_depth: int = 8):
    if depth > max_depth:
        return
    yield obj
    if isinstance(obj, dict):
        for k, v in obj.items():
            if not str(k).startswith("__"):
                yield from _walk(v, depth + 1, max_depth)
    elif isinstance(obj, (list, tuple)):
        for v in obj:
            yield from _walk(v, depth + 1, max_depth)
    elif isinstance(obj, np.ndarray) and obj.dtype == object:
        for v in obj.reshape(-1):
            yield from _walk(v, depth + 1, max_depth)


def _get_path(obj: Any, path: str) -> Any | None:
    cur = obj
    for part in path.split("."):
        if isinstance(cur, dict) and part in cur:
            cur = cur[part]
        else:
            return None
    return cur


def _find_by_names(data: dict[str, Any], names: list[str], numeric: bool = True, min_ndim: int = 1):
    for name in names:
        val = _get_path(data, name)
        arr = _as_float_array(val) if numeric else _as_array(val)
        if arr is not None and getattr(arr, "ndim", 0) >= min_ndim:
            return arr
    keys = {name.split(".")[-1].lower() for name in names}
    for obj in _walk(data):
        if isinstance(obj, dict):
            for k, v in obj.items():
                if str(k).lower() in keys:
                    arr = _as_float_array(v) if numeric else _as_array(v)
                    if arr is not None and getattr(arr, "ndim", 0) >= min_ndim:
                        return arr
    return None


def _normalize_state(state: np.ndarray) -> np.ndarray:
    state = np.asarray(state, dtype=float)
    state = np.squeeze(state)
    if state.ndim != 2:
        raise ValueError(f"Expected 2D state matrix, got {state.shape}")
    if state.shape[1] >= 6:
        return state[:, :6]
    if state.shape[0] >= 6:
        return state.T[:, :6]
    raise ValueError(f"Cannot normalize state matrix shape {state.shape}")


def _summary_from_mat(data: dict[str, Any], path: Path) -> ResultSummary:
    summary_obj = data.get("summary") or data.get("summary44") or data.get("summary45") or {}
    metrics: dict[str, str] = {}
    if isinstance(summary_obj, dict):
        for key, value in summary_obj.items():
            if isinstance(value, dict):
                for k2, v2 in value.items():
                    if np.asarray(v2).size <= 3:
                        metrics[f"{key}.{k2}"] = str(np.squeeze(v2))
            elif np.asarray(value).size <= 3:
                metrics[str(key)] = str(np.squeeze(value))
    workflow = metrics.get("workflow", "unknown")
    if str(workflow) == "unknown":
        lowered_path = str(path).lower()
        if "switching" in lowered_path:
            workflow = "switching"
        elif "recovery" in lowered_path:
            workflow = "recovery"
    if str(workflow) == "40":
        workflow = "switching"
    elif str(workflow) == "42":
        workflow = "recovery"
    overall_pass = metrics.get("overall_pass", "unknown")
    success_rate = (
        metrics.get("overall_success_rate")
        or metrics.get("controller_backend_success_rate")
        or metrics.get("metrics.available_transition_success_count")
        or "unknown"
    )
    backend = metrics.get("backend_controller", "MATLAB/Simulink")
    return ResultSummary(workflow=str(workflow), overall_pass=str(overall_pass), success_rate=str(success_rate), backend=str(backend), source_path=path, metrics=metrics)


def _series_from_mat(data: dict[str, Any], path: Path) -> TelemetrySeries:
    time = _find_by_names(data, ["log.t", "t", "time", "time_s", "representative_sims.target_recovery.time_s"])
    state = _find_by_names(data, ["log.state", "state", "states", "x", "representative_sims.target_recovery.states"], min_ndim=2)

    if time is None or state is None:
        time = _find_by_names(data, ["representative_sims.after_switching.time_s"])
        state = _find_by_names(data, ["representative_sims.after_switching.states"], min_ndim=2)

    if time is None or state is None:
        return TelemetrySeries(source_file=path, source_kind="mat-no-telemetry")

    time = np.asarray(time, dtype=float).reshape(-1)
    state = _normalize_state(state)
    n = min(len(time), state.shape[0])
    time = time[:n]
    state = state[:n]

    def opt(names: list[str]) -> np.ndarray | None:
        arr = _find_by_names(data, names)
        if arr is None:
            return None
        arr = np.asarray(arr, dtype=float).reshape(-1)
        return arr[:n]

    def opt_str(names: list[str]) -> list[str] | None:
        arr = _find_by_names(data, names, numeric=False)
        if arr is None:
            return None
        return _as_string_list(arr, n)

    return TelemetrySeries(
        time_s=time,
        state=state,
        u_cmd=opt(["log.u_cmd", "u_cmd", "representative_sims.target_recovery.u_cmd", "representative_sims.after_switching.u_cmd"]),
        u_actual=opt(["log.u_actual", "u_actual"]),
        mode=opt_str(["log.mode", "mode", "representative_sims.target_recovery.recovery_state", "representative_sims.after_switching.recovery_state"]),
        active_target=opt_str(["log.active_target", "active_target"]),
        requested_target=opt_str(["log.requested_target", "requested_target"]),
        force_x_N=opt(["force_x_N", "representative_sims.target_recovery.force_x_N", "representative_sims.after_switching.force_x_N"]),
        force_active=opt(["active", "force_event_active", "representative_sims.target_recovery.active", "representative_sims.after_switching.active"]),
        source_file=path,
        source_kind="mat",
    )


def _load_csv_table(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8", errors="ignore") as f:
        return list(csv.DictReader(f))


def _series_from_csv(path: Path) -> TelemetrySeries:
    rows = _load_csv_table(path)
    if not rows:
        return TelemetrySeries(source_file=path, source_kind="csv-empty")
    def col(name: str, default: float = 0.0):
        values = []
        for row in rows:
            try: values.append(float(row.get(name, default)))
            except Exception: values.append(default)
        return np.asarray(values, dtype=float)
    if "sim_time_s" in rows[0]:
        time = col("sim_time_s")
    elif "time_s" in rows[0]:
        time = col("time_s")
    else:
        time = np.arange(len(rows), dtype=float)
    state = np.column_stack([
        col("x", 0.0), col("x_dot", 0.0), col("theta1", np.pi),
        col("theta1_dot", 0.0), col("theta2", np.pi), col("theta2_dot", 0.0),
    ])
    return TelemetrySeries(
        time_s=time,
        state=state,
        mode=[r.get("mode", "csv_log") for r in rows],
        active_target=[r.get("active_target", "") for r in rows],
        requested_target=[r.get("requested_target", "") for r in rows],
        force_active=col("force_event_active") if "force_event_active" in rows[0] else None,
        source_file=path,
        source_kind="csv",
    )


class MatlabResultLoader:
    def load(self, path: str | Path) -> ResultBundle:
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(path)
        bundle = ResultBundle()
        if path.suffix.lower() == ".mat":
            data = loadmat(path, simplify_cells=True)
            bundle.summary = _summary_from_mat(data, path)
            bundle.telemetry = _series_from_mat(data, path)
            self._attach_neighbor_tables(bundle, path.parent)
            return bundle
        if path.suffix.lower() == ".csv":
            bundle.telemetry = _series_from_csv(path)
            bundle.summary = ResultSummary(source_path=path, workflow="csv", backend="MATLAB/Simulink")
            self._attach_neighbor_tables(bundle, path.parent)
            return bundle
        raise ValueError(f"Unsupported result file: {path}")

    def _attach_neighbor_tables(self, bundle: ResultBundle, folder: Path) -> None:
        for csv_file in sorted(folder.glob("*.csv")):
            try:
                bundle.tables[csv_file.name] = _load_csv_table(csv_file)
            except Exception:
                continue
