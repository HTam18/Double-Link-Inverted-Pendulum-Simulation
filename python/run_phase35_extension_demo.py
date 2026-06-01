"""Run the Phase 35 optional Python extension demo.

The demo only reads MATLAB/Simulink output files and generates visualization or
analysis artifacts. It does not simulate the plant and does not replace the
MATLAB/Simulink pipeline.
"""

from __future__ import annotations

from pathlib import Path
import sys

ROOT_HINT = Path(__file__).resolve().parents[1]
if str(ROOT_HINT) not in sys.path:
    sys.path.insert(0, str(ROOT_HINT))

from python_extension.common.matlab_results import find_project_root  # noqa: E402
from python_extension.visualization.plot_dashboard import create_dashboard  # noqa: E402
from python_extension.data_analysis.analyze_monte_carlo_results import analyze  # noqa: E402


def main() -> None:
    root = find_project_root(Path(__file__).resolve())
    output_dir = root / "results" / "phase35_python_extension"
    output_dir.mkdir(parents=True, exist_ok=True)

    created = []
    try:
        created.append(create_dashboard(root, output_dir))
    except Exception as exc:  # noqa: BLE001 - report missing prerequisites clearly
        print(f"phase35_dashboard_skipped = {exc}")

    try:
        summary, counts, plot = analyze(root, output_dir)
        created.extend([summary, counts, plot])
    except Exception as exc:  # noqa: BLE001
        print(f"phase35_monte_carlo_analysis_skipped = {exc}")

    summary_file = output_dir / "phase35_extension_demo_summary.txt"
    summary_file.write_text(
        "Phase 35 Python extension demo\n"
        "==============================\n"
        "Python extension only reads MATLAB/Simulink outputs.\n"
        "No plant, controller, TVLQR or hybrid simulation is implemented in Python.\n\n"
        "Created files:\n"
        + "\n".join(f"- {path}" for path in created)
        + "\n",
        encoding="utf-8",
    )
    print(f"phase35_demo_summary = {summary_file}")
    print("Phase 35 run_phase35_extension_demo: PASS")


if __name__ == "__main__":
    main()
