from __future__ import annotations

from pathlib import Path
import sys

ROOT_HINT = Path(__file__).resolve().parents[2]
if str(ROOT_HINT) not in sys.path:
    sys.path.insert(0, str(ROOT_HINT))

try:
    import customtkinter as ctk
except ImportError as exc:
    raise SystemExit("Missing dependency: customtkinter. Install with: pip install customtkinter") from exc

from python.gui.core.appState import AppState
from python.gui.services.projectPaths import find_project_root
from python.gui.services.replayEngine import ReplayEngine
from python.gui.ui.appUi import AppUI
from python.gui.ui.theme import apply_theme


def main() -> int:
    project_root = find_project_root(Path(__file__).resolve())
    apply_theme()
    state = AppState(project_root=project_root)
    replay = ReplayEngine(state)
    root = ctk.CTk()
    AppUI(root, state, replay)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
