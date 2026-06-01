"""Replay MATLAB logged data in a lightweight Python viewer.

This is not a realtime controller. It only visualizes states already exported by
MATLAB/Simulink.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import matplotlib.pyplot as plt
import numpy as np

ROOT_HINT = Path(__file__).resolve().parents[2]
if str(ROOT_HINT) not in sys.path:
    sys.path.insert(0, str(ROOT_HINT))

from python_extension.common.matlab_results import find_project_root, load_logged_series  # noqa: E402


def replay(result_file: Path, speed: float = 1.0, link_length: float = 0.5) -> None:
    series = load_logged_series(result_file)
    t = series.time
    state = series.state
    x = state[:, 0]
    theta1 = state[:, 2]
    theta2 = state[:, 4]

    l1 = link_length
    l2 = link_length

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_title("Phase 35 logged-data replay viewer")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(np.nanmin(x) - 0.7, np.nanmax(x) + 0.7)
    ax.set_ylim(-1.2, 1.2)

    cart, = ax.plot([], [], linewidth=6)
    link, = ax.plot([], [], marker="o")
    info = ax.text(0.02, 0.95, "", transform=ax.transAxes)

    plt.ion()
    fig.show()
    previous_t = t[0]
    for i in range(len(t)):
        p0 = np.array([x[i], 0.0])
        p1 = p0 + np.array([l1 * np.sin(theta1[i]), -l1 * np.cos(theta1[i])])
        p2 = p1 + np.array([l2 * np.sin(theta2[i]), -l2 * np.cos(theta2[i])])
        cart.set_data([p0[0] - 0.09, p0[0] + 0.09], [0, 0])
        link.set_data([p0[0], p1[0], p2[0]], [p0[1], p1[1], p2[1]])
        info.set_text(f"sample {i+1}/{len(t)} | t={t[i]:.2f}s")
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        dt = max(0.0, float(t[i] - previous_t)) / max(speed, 1e-6)
        time.sleep(min(dt, 0.2))
        previous_t = t[i]
    plt.ioff()
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=None)
    parser.add_argument("--result-file", type=Path, default=None)
    parser.add_argument("--speed", type=float, default=1.0)
    args = parser.parse_args()

    root = find_project_root(args.project_root)
    result_file = args.result_file or root / "results" / "phase30_full_hybrid" / "full_hybrid_result.mat"
    replay(result_file, speed=args.speed)


if __name__ == "__main__":
    main()
