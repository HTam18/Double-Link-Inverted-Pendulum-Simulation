from __future__ import annotations

from pathlib import Path
import numpy as np
from scipy.io import savemat


def generate(output: Path) -> Path:
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    t = np.linspace(0, 12, 1201)
    state = np.column_stack([
        0.35 * np.sin(0.35 * t),
        0.35 * 0.35 * np.cos(0.35 * t),
        np.pi + 0.15 * np.sin(1.2 * t),
        0.18 * np.cos(1.2 * t),
        np.pi + 0.10 * np.sin(1.4 * t + 0.4),
        0.14 * np.cos(1.4 * t + 0.4),
    ])
    log = {"t": t, "state": state, "u_cmd": 2 * np.sin(t), "u_actual": 1.8 * np.sin(t - 0.05)}
    summary = {"workflow": "fixture", "overall_pass": 1, "overall_success_rate": 1.0, "backend_controller": "fixture_controller"}
    savemat(output, {"log": log, "summary": summary})
    return output


if __name__ == "__main__":
    out = generate(Path("results/python_gui_fake/fake_logged_result.mat"))
    print(f"fake_result = {out}")
