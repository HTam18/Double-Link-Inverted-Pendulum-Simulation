from __future__ import annotations

import time
import numpy as np

from python.gui.core.appState import AppState


class ReplayEngine:
    def __init__(self, state: AppState) -> None:
        self.state = state
        self._last_wall_time: float | None = None
        self._sim_time_carry: float = 0.0

    def play(self) -> None:
        if self.state.frame_count > 0:
            if self.state.current_frame >= self.state.frame_count - 1:
                self.state.current_frame = 0
            self.state.is_playing = True
            self._last_wall_time = time.perf_counter()
            self._sim_time_carry = 0.0
            self.state.log("Replay started")

    def pause(self) -> None:
        self.state.is_playing = False
        self._last_wall_time = None
        self._sim_time_carry = 0.0
        self.state.log("Replay paused")

    def stop(self) -> None:
        self.state.reset_playback()
        self._last_wall_time = None
        self._sim_time_carry = 0.0
        self.state.log("Replay stopped")

    def set_speed(self, speed: float) -> None:
        self.state.playback_speed = max(0.1, min(float(speed), 10.0))

    def _sample_dt(self) -> float:
        tel = self.state.bundle.telemetry
        if tel.frame_count < 2:
            return 0.02
        dt = np.diff(np.asarray(tel.time_s[: min(tel.frame_count, 5000)], dtype=float))
        dt = dt[np.isfinite(dt) & (dt > 1e-6)]
        if dt.size == 0:
            return 0.02
        return float(np.median(dt))

    def step(self) -> None:
        if self.state.frame_count <= 0 or not self.state.is_playing:
            return
        now = time.perf_counter()
        if self._last_wall_time is None:
            self._last_wall_time = now
            return
        elapsed = max(0.0, now - self._last_wall_time)
        self._last_wall_time = now

        sample_dt = self._sample_dt()
        sim_advance = elapsed * float(self.state.playback_speed) + self._sim_time_carry
        frame_step = int(sim_advance / sample_dt)
        self._sim_time_carry = sim_advance - frame_step * sample_dt
        if frame_step <= 0:
            return

        self.state.current_frame = min(self.state.frame_count - 1, self.state.current_frame + frame_step)
        if self.state.current_frame >= self.state.frame_count - 1:
            self.state.is_playing = False
            self._last_wall_time = None
            self._sim_time_carry = 0.0
