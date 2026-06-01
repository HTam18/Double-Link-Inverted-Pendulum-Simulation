"""Create a simple animation from MATLAB logged states.

This is an optional visualization extension. It only reads logged states from a
MATLAB result file; it does not compute dynamics or control commands.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

ROOT_HINT = Path(__file__).resolve().parents[2]
if str(ROOT_HINT) not in sys.path:
    sys.path.insert(0, str(ROOT_HINT))

from python_extension.common.matlab_results import find_project_root, load_logged_series  # noqa: E402


def make_animation(result_file: Path, output_file: Path, stride: int = 1, link_length: float = 0.5) -> Path:
    series = load_logged_series(result_file)
    t = series.time[::stride]
    state = series.state[::stride]

    x = state[:, 0]
    theta1 = state[:, 2]
    theta2 = state[:, 4]
    l1 = link_length
    l2 = link_length

    # Angle convention: theta=0 downward, theta=pi upright. Coordinates use y up.
    p0x = x
    p0y = np.zeros_like(x)
    p1x = p0x + l1 * np.sin(theta1)
    p1y = p0y - l1 * np.cos(theta1)
    p2x = p1x + l2 * np.sin(theta2)
    p2y = p1y - l2 * np.cos(theta2)

    output_file.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_title("Double link pendulum animation from MATLAB log")
    ax.set_xlabel("cart position x [m]")
    ax.set_ylabel("vertical position [m]")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="box")
    margin = 0.6
    ax.set_xlim(np.nanmin(p2x) - margin, np.nanmax(p2x) + margin)
    ax.set_ylim(np.nanmin(p2y) - margin, np.nanmax(p1y) + margin)

    rail_line, = ax.plot([], [], linewidth=1)
    cart_line, = ax.plot([], [], linewidth=6)
    link_line, = ax.plot([], [], marker="o")
    time_text = ax.text(0.02, 0.95, "", transform=ax.transAxes)

    rail_y = -0.05
    rail_line.set_data([np.nanmin(x) - margin, np.nanmax(x) + margin], [rail_y, rail_y])

    def update(i: int):
        cart_width = 0.18
        cart_line.set_data([p0x[i] - cart_width / 2, p0x[i] + cart_width / 2], [0, 0])
        link_line.set_data([p0x[i], p1x[i], p2x[i]], [p0y[i], p1y[i], p2y[i]])
        time_text.set_text(f"t = {t[i]:.2f} s")
        return cart_line, link_line, time_text

    ani = FuncAnimation(fig, update, frames=len(t), interval=80, blit=True)
    writer = PillowWriter(fps=12)
    ani.save(output_file, writer=writer)
    plt.close(fig)
    return output_file


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=None)
    parser.add_argument(
        "--result-file",
        type=Path,
        default=None,
        help="MATLAB result file. Default: results/phase30_full_hybrid/full_hybrid_result.mat",
    )
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--stride", type=int, default=1)
    args = parser.parse_args()

    root = find_project_root(args.project_root)
    result_file = args.result_file or root / "results" / "phase30_full_hybrid" / "full_hybrid_result.mat"
    output = args.output or root / "results" / "phase35_python_extension" / "phase35_animation.gif"
    out = make_animation(result_file, output, stride=max(1, args.stride))
    print(f"phase35_animation_file = {out}")
    print("Phase 35 animate_from_matlab_results: PASS")


if __name__ == "__main__":
    main()
