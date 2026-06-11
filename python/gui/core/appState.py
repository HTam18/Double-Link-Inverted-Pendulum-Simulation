from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import queue

from .telemetry import ResultBundle


@dataclass
class AppState:
    project_root: Path
    bundle: ResultBundle = field(default_factory=ResultBundle)
    current_frame: int = 0
    is_playing: bool = False
    playback_speed: float = 1.0
    selected_result_dir: Path | None = None
    command_outbox: Path | None = None
    status_text: str = "Ready"
    log_queue: "queue.Queue[str]" = field(default_factory=queue.Queue)

    def log(self, message: str) -> None:
        self.log_queue.put(str(message))

    def reset_playback(self) -> None:
        self.current_frame = 0
        self.is_playing = False

    @property
    def frame_count(self) -> int:
        return self.bundle.telemetry.frame_count
