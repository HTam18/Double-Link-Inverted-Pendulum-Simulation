from __future__ import annotations

from pathlib import Path

PROJECT_MARKERS = ("matlab", "python", "results")


def find_project_root(start: Path | None = None) -> Path:
    current = (start or Path.cwd()).resolve()
    for candidate in [current, *current.parents]:
        if all((candidate / marker).exists() for marker in PROJECT_MARKERS):
            return candidate
    raise FileNotFoundError("Cannot find Double_Link_Pendulum project root.")
