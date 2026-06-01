"""Utility functions for Phase 35 Python extension.

This module only reads MATLAB/Simulink result files. It does not implement
plant dynamics, control laws, TVLQR, hybrid logic, or simulation integration.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from scipy.io import loadmat


PROJECT_MARKERS = ("shared", "matlab", "results")


@dataclass
class LoggedSeries:
    time: np.ndarray
    state: np.ndarray
    u_cmd: np.ndarray | None = None
    u_actual: np.ndarray | None = None
    mode: np.ndarray | None = None
    source_file: Path | None = None


def find_project_root(start: Path | None = None) -> Path:
    """Find project root by walking upward from *start* or current directory."""
    current = (start or Path.cwd()).resolve()
    candidates = [current, *current.parents]
    for candidate in candidates:
        if all((candidate / marker).exists() for marker in PROJECT_MARKERS):
            return candidate
    raise FileNotFoundError(
        "Cannot find project root. Run this script from inside the project folder."
    )


def load_mat(path: str | Path) -> dict[str, Any]:
    """Load a MATLAB .mat file using scipy and simplify MATLAB structs."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"MAT file not found: {path}")
    return loadmat(path, simplify_cells=True)


def _walk_values(obj: Any, depth: int = 0, max_depth: int = 8) -> Iterable[Any]:
    if depth > max_depth:
        return
    yield obj
    if isinstance(obj, dict):
        for key, value in obj.items():
            if str(key).startswith("__"):
                continue
            yield from _walk_values(value, depth + 1, max_depth)
    elif isinstance(obj, (list, tuple)):
        for value in obj:
            yield from _walk_values(value, depth + 1, max_depth)


def _to_numeric_array(value: Any) -> np.ndarray | None:
    try:
        arr = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    if arr.size == 0 or not np.all(np.isfinite(arr)):
        return None
    return np.squeeze(arr)


def get_by_path(obj: Any, path: str) -> Any | None:
    """Read nested dict/object field path such as result.t or phase30.state."""
    current = obj
    for part in path.split("."):
        if isinstance(current, dict) and part in current:
            current = current[part]
        else:
            return None
    return current


def find_first_array(data: dict[str, Any], names: list[str], min_ndim: int = 1) -> np.ndarray | None:
    """Find a numeric array by exact or nested field name."""
    for name in names:
        direct = get_by_path(data, name)
        arr = _to_numeric_array(direct)
        if arr is not None and arr.ndim >= min_ndim:
            return arr

    name_set = {name.split(".")[-1].lower() for name in names}
    for obj in _walk_values(data):
        if isinstance(obj, dict):
            for key, value in obj.items():
                if str(key).lower() in name_set:
                    arr = _to_numeric_array(value)
                    if arr is not None and arr.ndim >= min_ndim:
                        return arr
    return None


def normalize_state_matrix(state: np.ndarray) -> np.ndarray:
    """Return state as N x 6 matrix when possible."""
    state = np.asarray(state, dtype=float)
    state = np.squeeze(state)
    if state.ndim != 2:
        raise ValueError(f"Expected 2D state array, got shape {state.shape}")
    if state.shape[1] == 6:
        return state
    if state.shape[0] == 6:
        return state.T
    raise ValueError(f"Cannot identify state order in array with shape {state.shape}")


def load_logged_series(path: str | Path) -> LoggedSeries:
    """Load common logged signals from Phase 29/30/32 MATLAB result files."""
    path = Path(path)
    data = load_mat(path)

    time = find_first_array(data, ["t", "time", "result.t", "log.t", "logs.t"])
    state = find_first_array(
        data,
        [
            "state",
            "x",
            "result.state",
            "result.x",
            "log.state",
            "logs.state",
            "true_state",
        ],
        min_ndim=2,
    )
    if time is None or state is None:
        raise ValueError(
            f"Could not find time/state in {path}. Expected fields like t/time and state/x."
        )

    time = np.asarray(time, dtype=float).reshape(-1)
    state = normalize_state_matrix(state)

    n = min(len(time), state.shape[0])
    time = time[:n]
    state = state[:n, :]

    def optional(names: list[str]) -> np.ndarray | None:
        arr = find_first_array(data, names)
        if arr is None:
            return None
        arr = np.asarray(arr, dtype=float).reshape(-1)
        return arr[:n]

    return LoggedSeries(
        time=time,
        state=state,
        u_cmd=optional(["u_cmd", "result.u_cmd", "log.u_cmd", "logs.u_cmd"]),
        u_actual=optional(["u_actual", "result.u_actual", "log.u_actual", "logs.u_actual"]),
        mode=optional(["mode", "mode_id", "result.mode", "result.mode_id", "log.mode", "logs.mode"]),
        source_file=path,
    )


def read_key_value_summary(path: str | Path) -> dict[str, str]:
    """Read text summary lines formatted as key = value."""
    path = Path(path)
    result: dict[str, str] = {}
    if not path.exists():
        return result
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key.strip()] = value.strip()
    return result


def read_csv_rows(path: str | Path) -> list[dict[str, str]]:
    path = Path(path)
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8", errors="ignore") as f:
        return list(csv.DictReader(f))


def write_csv_rows(path: str | Path, rows: list[dict[str, Any]]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
