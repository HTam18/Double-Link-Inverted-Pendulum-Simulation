"""Create a report dashboard from MATLAB/Simulink result files.

Phase 35 rule: this script only reads existing MATLAB outputs and creates
figures. It does not simulate the plant or execute any controller.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

ROOT_HINT = Path(__file__).resolve().parents[2]
if str(ROOT_HINT) not in sys.path:
    sys.path.insert(0, str(ROOT_HINT))

from python_extension.common.matlab_results import (  # noqa: E402
    find_project_root,
    read_csv_rows,
    read_key_value_summary,
)


def _float_value(text: str | None, default: float = np.nan) -> float:
    if text is None:
        return default
    try:
        return float(str(text).strip())
    except ValueError:
        return default


def _scenario_rows(root: Path) -> list[dict[str, str]]:
    csv_path = root / "results" / "phase33_evaluation" / "phase33_summary_metrics.csv"
    rows = read_csv_rows(csv_path)
    if rows:
        return rows

    # Fallback summary rows if Phase 33 CSV is not present in a packaged ZIP.
    p29 = read_key_value_summary(root / "results" / "phase29_tvlqr_tracking" / "phase29_clean_summary.txt")
    p30 = read_key_value_summary(root / "results" / "phase30_full_hybrid" / "phase30_full_hybrid_summary.txt")
    p31 = read_key_value_summary(root / "results" / "phase31_monte_carlo" / "phase31_monte_carlo_summary.txt")
    p32 = read_key_value_summary(root / "results" / "phase32_simulink" / "phase32_simulink_summary.txt")

    rows = []
    if p29:
        rows.extend(
            [
                {
                    "scenario_id": "S1",
                    "scenario_name": "Phase 29 T1 open-loop feedforward",
                    "success": p29.get("T1_open_loop_tracking_gate_pass", "0"),
                    "final_angle_error_rad": p29.get("T1_open_loop_final_max_angle_error_rad", "nan"),
                    "final_velocity_norm": p29.get("T1_open_loop_final_velocity_norm", "nan"),
                    "max_abs_x_m": p29.get("T1_open_loop_max_abs_x_m", "nan"),
                    "saturation_fraction": p29.get("T1_open_loop_saturation_fraction", "nan"),
                },
                {
                    "scenario_id": "S4",
                    "scenario_name": "Phase 29 T4 TVLQR actuator delay + sensor noise",
                    "success": p29.get("T4_noise_tvlqr_tracking_gate_pass", "0"),
                    "final_angle_error_rad": p29.get("T4_noise_tvlqr_final_max_angle_error_rad", "nan"),
                    "final_velocity_norm": p29.get("T4_noise_tvlqr_final_velocity_norm", "nan"),
                    "max_abs_x_m": p29.get("T4_noise_tvlqr_max_abs_x_m", "nan"),
                    "saturation_fraction": p29.get("T4_noise_tvlqr_saturation_fraction", "nan"),
                },
            ]
        )
    if p30:
        rows.append(
            {
                "scenario_id": "S5",
                "scenario_name": "Phase 30 full hybrid MATLAB",
                "success": p30.get("phase30_pass", "0"),
                "final_angle_error_rad": p30.get("phase30_final_angle_error_rad", "nan"),
                "final_velocity_norm": p30.get("phase30_final_velocity_norm", "nan"),
                "max_abs_x_m": p30.get("phase30_max_abs_x_m", "nan"),
                "saturation_fraction": p30.get("phase30_saturation_fraction", "nan"),
            }
        )
    if p31:
        rows.append(
            {
                "scenario_id": "S6",
                "scenario_name": "Phase 31 standard Monte Carlo",
                "success": p31.get("phase31_success_rate", "nan"),
                "final_angle_error_rad": p31.get("phase31_worst_final_angle_error_rad", "nan"),
                "final_velocity_norm": p31.get("phase31_worst_final_velocity_norm", "nan"),
                "max_abs_x_m": "nan",
                "saturation_fraction": p31.get("phase31_mean_saturation_fraction", "nan"),
            }
        )
    if p32:
        rows.append(
            {
                "scenario_id": "S7",
                "scenario_name": "Phase 32 Simulink equivalence",
                "success": p32.get("phase32_pass", "0"),
                "final_angle_error_rad": p32.get("phase32_final_angle_error_rad", "nan"),
                "final_velocity_norm": p32.get("phase32_final_velocity_norm", "nan"),
                "max_abs_x_m": p32.get("phase32_max_abs_x_m", "nan"),
                "saturation_fraction": p32.get("phase32_saturation_fraction", "nan"),
            }
        )
    return rows


def create_dashboard(root: Path, output_dir: Path) -> Path:
    rows = _scenario_rows(root)
    if not rows:
        raise FileNotFoundError(
            "No Phase 33 CSV or fallback summary files found. Run MATLAB Phase 33 first."
        )

    labels = [row.get("scenario_id", f"S{i+1}") for i, row in enumerate(rows)]
    names = [row.get("scenario_name", label) for row, label in zip(rows, labels)]
    angle = [_float_value(row.get("final_angle_error_rad")) for row in rows]
    velocity = [_float_value(row.get("final_velocity_norm")) for row in rows]
    saturation = [_float_value(row.get("saturation_fraction")) for row in rows]
    success = [_float_value(row.get("success")) for row in rows]

    output_dir.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(13, 9))
    fig.suptitle("Phase 35 Python dashboard from MATLAB/Simulink results", fontsize=14)

    ax1 = fig.add_subplot(2, 2, 1)
    ax1.bar(labels, success)
    ax1.set_ylim(0, 1.05)
    ax1.set_title("Scenario success / success rate")
    ax1.set_ylabel("value")
    ax1.grid(True, axis="y", alpha=0.3)

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.bar(labels, angle)
    ax2.set_title("Final angle error")
    ax2.set_ylabel("rad")
    ax2.grid(True, axis="y", alpha=0.3)

    ax3 = fig.add_subplot(2, 2, 3)
    ax3.bar(labels, velocity)
    ax3.set_title("Final velocity norm")
    ax3.set_ylabel("state velocity norm")
    ax3.grid(True, axis="y", alpha=0.3)

    ax4 = fig.add_subplot(2, 2, 4)
    ax4.bar(labels, saturation)
    ax4.set_title("Saturation fraction")
    ax4.set_ylabel("fraction")
    ax4.grid(True, axis="y", alpha=0.3)

    fig.text(
        0.02,
        0.01,
        "Scenarios: " + "; ".join(f"{label}: {name}" for label, name in zip(labels, names)),
        fontsize=8,
        wrap=True,
    )
    fig.tight_layout(rect=[0, 0.05, 1, 0.96])

    out = output_dir / "phase35_dashboard.png"
    fig.savefig(out, dpi=160)
    plt.close(fig)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    args = parser.parse_args()

    root = find_project_root(args.project_root)
    output_dir = args.output_dir or root / "results" / "phase35_python_extension"
    out = create_dashboard(root, output_dir)
    print(f"phase35_dashboard_file = {out}")
    print("Phase 35 plot_dashboard: PASS")


if __name__ == "__main__":
    main()
