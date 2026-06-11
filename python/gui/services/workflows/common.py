from __future__ import annotations

from pathlib import Path
import csv
from typing import Any

import numpy as np


def load_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8", errors="ignore") as f:
        return list(csv.DictReader(f))


def canonical_decimal(value: object, digits: int = 2) -> str:
    try:
        return f"{float(str(value)):.{digits}f}"
    except Exception:
        return str(value)


def same_decimal(left: object, right: object, digits: int = 2) -> bool:
    return canonical_decimal(left, digits) == canonical_decimal(right, digits)


def format_metric(value: str) -> str:
    if value is None or value == "":
        return "--"
    try:
        f = float(value)
        if not np.isfinite(f):
            return str(value)
        if abs(f) >= 1000 or (0 < abs(f) < 1e-3):
            return f"{f:.3e}"
        return f"{f:.4g}"
    except Exception:
        return str(value)


def as_list(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    if isinstance(value, tuple):
        return list(value)
    arr = np.asarray(value)
    if arr.dtype == object:
        return arr.reshape(-1).tolist()
    return [value]


def string_array(value: Any, n: int) -> list[str]:
    if value is None:
        return [""] * n
    arr = np.asarray(value)
    if arr.dtype == object:
        values = [str(v) for v in arr.reshape(-1).tolist()]
    else:
        values = [str(v) for v in np.ravel(arr).tolist()]
    if len(values) < n:
        values += [values[-1] if values else ""] * (n - len(values))
    return values[:n]


def float_vector(value: Any, n: int) -> np.ndarray | None:
    if value is None:
        return None
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size < n:
        return np.pad(arr, (0, n - arr.size), mode="edge") if arr.size else None
    return arr[:n]
