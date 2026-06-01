"""Analyze Phase 31 Monte Carlo metrics exported by MATLAB.

Phase 35 rule: this script reads MATLAB-generated CSV/MAT outputs only. It does
not re-run a simulation and does not implement any controller.
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

ROOT_HINT = Path(__file__).resolve().parents[2]
if str(ROOT_HINT) not in sys.path:
    sys.path.insert(0, str(ROOT_HINT))

from python_extension.common.matlab_results import find_project_root, read_csv_rows, write_csv_rows  # noqa: E402


def _float(row: dict[str, str], names: list[str], default: float = np.nan) -> float:
    for name in names:
        if name in row and row[name] not in (None, ""):
            try:
                return float(row[name])
            except ValueError:
                continue
    return default


def _text(row: dict[str, str], names: list[str], default: str = "unknown") -> str:
    for name in names:
        value = row.get(name)
        if value:
            return str(value)
    return default


def analyze(root: Path, output_dir: Path) -> tuple[Path, Path, Path]:
    csv_path = root / "results" / "phase31_monte_carlo" / "phase31_monte_carlo_metrics.csv"
    rows = read_csv_rows(csv_path)
    if not rows:
        raise FileNotFoundError(
            f"No Monte Carlo CSV found at {csv_path}. Run MATLAB Phase 31 first."
        )

    output_dir.mkdir(parents=True, exist_ok=True)

    fail_reasons = [_text(row, ["fail_reason", "fail", "reason"]) for row in rows]
    counts = Counter(fail_reasons)
    count_rows = [
        {"failure_reason": reason, "count": count, "fraction": count / len(rows)}
        for reason, count in counts.most_common()
    ]
    counts_csv = output_dir / "phase35_monte_carlo_failure_counts.csv"
    write_csv_rows(counts_csv, count_rows)

    success_values = [_float(row, ["success", "is_success"], 0.0) for row in rows]
    final_angle = [_float(row, ["final_angle_error_rad", "final_angle_error", "angle_error_rad"]) for row in rows]
    final_vel = [_float(row, ["final_velocity_norm", "velocity_norm"]) for row in rows]
    max_x = [_float(row, ["max_abs_x_m", "max_x", "max_abs_x"]) for row in rows]
    sat = [_float(row, ["saturation_fraction", "sat", "sat_fraction"]) for row in rows]

    summary = {
        "num_runs": len(rows),
        "success_count": int(np.nansum(success_values)),
        "success_rate": float(np.nanmean(success_values)),
        "worst_final_angle_error_rad": float(np.nanmax(final_angle)),
        "worst_final_velocity_norm": float(np.nanmax(final_vel)),
        "worst_max_abs_x_m": float(np.nanmax(max_x)),
        "mean_saturation_fraction": float(np.nanmean(sat)),
        "dominant_failure_reason": count_rows[0]["failure_reason"] if count_rows else "none",
    }

    summary_txt = output_dir / "phase35_monte_carlo_analysis.txt"
    summary_txt.write_text(
        "Phase 35 Monte Carlo analysis from MATLAB Phase 31 outputs\n"
        "========================================================\n"
        "This file is produced by Python extension scripts that only read MATLAB results.\n\n"
        + "\n".join(f"{key} = {value}" for key, value in summary.items())
        + "\n\nFailure reason counts:\n"
        + "\n".join(
            f"- {row['failure_reason']}: {row['count']} ({row['fraction']:.3f})"
            for row in count_rows
        )
        + "\n",
        encoding="utf-8",
    )

    fig = plt.figure(figsize=(12, 7))
    fig.suptitle("Phase 31 Monte Carlo analysis read by Python extension")

    ax1 = fig.add_subplot(2, 2, 1)
    ax1.bar(range(1, len(success_values) + 1), success_values)
    ax1.set_title("Success by run")
    ax1.set_xlabel("run")
    ax1.set_ylim(0, 1.05)
    ax1.grid(True, axis="y", alpha=0.3)

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.bar([row["failure_reason"] for row in count_rows], [row["count"] for row in count_rows])
    ax2.set_title("Failure reason counts")
    ax2.tick_params(axis="x", rotation=35)
    ax2.grid(True, axis="y", alpha=0.3)

    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(range(1, len(final_angle) + 1), final_angle, marker="o")
    ax3.set_title("Final angle error")
    ax3.set_xlabel("run")
    ax3.set_ylabel("rad")
    ax3.grid(True, alpha=0.3)

    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(range(1, len(sat) + 1), sat, marker="o")
    ax4.set_title("Saturation fraction")
    ax4.set_xlabel("run")
    ax4.grid(True, alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    plot_file = output_dir / "phase35_monte_carlo_analysis.png"
    fig.savefig(plot_file, dpi=160)
    plt.close(fig)

    return summary_txt, counts_csv, plot_file


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    args = parser.parse_args()

    root = find_project_root(args.project_root)
    output_dir = args.output_dir or root / "results" / "phase35_python_extension"
    summary, counts, plot = analyze(root, output_dir)
    print(f"phase35_monte_carlo_summary = {summary}")
    print(f"phase35_monte_carlo_failure_counts = {counts}")
    print(f"phase35_monte_carlo_plot = {plot}")
    print("Phase 35 analyze_monte_carlo_results: PASS")


if __name__ == "__main__":
    main()
