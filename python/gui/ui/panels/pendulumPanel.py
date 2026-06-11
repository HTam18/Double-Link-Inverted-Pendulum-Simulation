from __future__ import annotations

import numpy as np
import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle, FancyArrowPatch

from python.gui.ui import theme
from python.gui.ui.widgets import figma


class PendulumPanel(ctk.CTkFrame):
    def __init__(self, parent, state):
        super().__init__(parent, fg_color="transparent")
        self.state = state
        figma.label(self, "Replay", size=15, weight="bold").pack(anchor="w", pady=(0, 6))
        self.fig = Figure(figsize=(6.2, 5.2), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.fig.subplots_adjust(left=0.12, right=0.98, top=0.96, bottom=0.12)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.widget = self.canvas.get_tk_widget()
        self.widget.pack(fill="both", expand=True)
        self._last_key = None
        self._last_frame = None
        self._cached_limits = None
        self._cached_points = None
        self._background = None
        self._canvas_size = None
        self._init_plot()

    def _init_plot(self):
        self.ax.clear()
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, color=theme.GRID, alpha=0.75, linewidth=0.8)
        self.ax.tick_params(labelsize=9)
        self.ax.set_xlabel("x [m]", fontsize=10)
        self.ax.set_ylabel("y [m]", fontsize=10)
        self.rail, = self.ax.plot([], [], color=theme.RAIL, linewidth=1.4, zorder=1)
        self.cart = Rectangle((0, -0.06), 0.30, 0.12, facecolor=theme.PLOT_X, edgecolor="#0F355D", linewidth=1.2, zorder=5)
        self.cart.set_animated(True)
        self.ax.add_patch(self.cart)
        self.link1_line, = self.ax.plot([], [], color=theme.PLOT_THETA1, linewidth=3.4, marker="o", markersize=5.8, zorder=6, animated=True)
        self.link2_line, = self.ax.plot([], [], color=theme.PLOT_THETA2, linewidth=3.4, marker="o", markersize=5.8, zorder=6, animated=True)
        self.trail1, = self.ax.plot([], [], color=theme.PLOT_THETA1, linewidth=1.8, alpha=0.22, zorder=2, animated=True)
        self.trail2, = self.ax.plot([], [], color=theme.PLOT_THETA2, linewidth=1.8, alpha=0.22, zorder=2, animated=True)
        self.force_arrow = FancyArrowPatch(
            (0, 0), (0, 0), arrowstyle="-|>", mutation_scale=18,
            linewidth=2.4, color=theme.DANGER, zorder=8, animated=True
        )
        self.force_arrow.set_visible(False)
        self.ax.add_patch(self.force_arrow)
        self.force_label = self.ax.text(0, 0, "", color=theme.DANGER, fontsize=9, weight="bold", animated=True, zorder=9)
        self.force_label.set_visible(False)
        self.status_text = self.ax.text(0.02, 0.98, "No result loaded", transform=self.ax.transAxes, va="top", ha="left", fontsize=10, animated=True)
        self.canvas.draw_idle()

    def _reset_cache_if_needed(self, key):
        if key != self._last_key:
            self._cached_limits = None
            self._cached_points = None
            self._background = None

    def _trajectory_points(self):
        if self._cached_points is not None:
            return self._cached_points
        tel = self.state.bundle.telemetry
        st = tel.state[: tel.frame_count, :]
        x = st[:, 0]
        th1 = st[:, 2]
        th2 = st[:, 4]
        l1 = l2 = 0.5
        p1x = x + l1 * np.sin(th1)
        p1y = -l1 * np.cos(th1)
        p2x = p1x + l2 * np.sin(th2)
        p2y = p1y - l2 * np.cos(th2)
        self._cached_points = (x, p1x, p1y, p2x, p2y)
        return self._cached_points

    def _compute_limits(self):
        tel = self.state.bundle.telemetry
        if not tel.has_state:
            return (-1.2, 1.2), (-1.15, 1.15)
        x, p1x, p1y, p2x, p2y = self._trajectory_points()
        xs = np.concatenate([x, p1x, p2x])
        ys = np.concatenate([np.zeros_like(x), p1y, p2y])
        x_min = float(np.nanmin(xs)) - 0.15
        x_max = float(np.nanmax(xs)) + 0.15
        y_min = float(np.nanmin(ys)) - 0.15
        y_max = float(np.nanmax(ys)) + 0.15
        if (x_max - x_min) < 1.55:
            cx = 0.5 * (x_min + x_max)
            x_min, x_max = cx - 0.775, cx + 0.775
        if (x_max - x_min) > 2.8:
            cx = 0.5 * (x_min + x_max)
            x_min, x_max = cx - 1.4, cx + 1.4
        if (y_max - y_min) < 1.55:
            cy = 0.5 * (y_min + y_max)
            y_min, y_max = cy - 0.775, cy + 0.775
        return (x_min, x_max), (y_min, y_max)

    def _ensure_background(self):
        current_size = (self.widget.winfo_width(), self.widget.winfo_height())
        if self._background is not None and self._canvas_size == current_size:
            return
        self._canvas_size = current_size
        self.canvas.draw()
        self._background = self.canvas.copy_from_bbox(self.ax.bbox)

    def _draw_dynamic(self):
        if self._background is None:
            self.canvas.draw_idle()
            return
        self.canvas.restore_region(self._background)
        for artist in [self.trail1, self.trail2, self.cart, self.link1_line, self.link2_line, self.force_arrow, self.force_label, self.status_text]:
            self.ax.draw_artist(artist)
        self.canvas.blit(self.ax.bbox)
        self.canvas.flush_events()

    def _event_float(self, event: dict, key: str, default: float) -> float:
        try:
            return float(event.get(key, default))
        except Exception:
            return default

    def _update_force_arrow(self, frame: int, f: dict, p0, p1, p2) -> None:
        event = self.state.bundle.telemetry.force_event
        if not event:
            self.force_arrow.set_visible(False)
            self.force_label.set_visible(False)
            return
        try:
            link_id = int(float(event.get("link_id", 1)))
        except Exception:
            link_id = 1
        ratio = min(1.0, max(0.0, self._event_float(event, "contact_ratio", 0.5)))
        force_n = self._event_float(event, "force_N", 1.0)
        direction = str(event.get("direction_name", event.get("direction", "right"))).lower()
        sign = -1.0 if direction == "left" else 1.0

        if link_id == 2:
            cx = p1[0] + ratio * (p2[0] - p1[0])
            cy = p1[1] + ratio * (p2[1] - p1[1])
        else:
            cx = p0[0] + ratio * (p1[0] - p0[0])
            cy = p0[1] + ratio * (p1[1] - p0[1])

        length = 0.20 + 0.035 * min(abs(force_n), 5.0)
        start_xy = (cx - sign * length * 0.55, cy)
        end_xy = (cx + sign * length * 0.55, cy)
        self.force_arrow.set_positions(start_xy, end_xy)
        self.force_arrow.set_visible(True)

        active = False
        tel = self.state.bundle.telemetry
        if tel.force_active is not None and frame < len(tel.force_active):
            try:
                active = float(tel.force_active[frame]) > 0
            except Exception:
                active = False
        else:
            start_t = self._event_float(event, "start_time_s", 0.7)
            duration = self._event_float(event, "duration_s", 0.08)
            active = start_t <= float(f.get("time_s", 0.0)) <= start_t + duration
        self.force_arrow.set_alpha(1.0 if active else 0.45)
        label = f"F={force_n:g}N {direction}"
        self.force_label.set_text(label)
        self.force_label.set_position((end_xy[0] + sign * 0.03, end_xy[1] + 0.03))
        self.force_label.set_visible(True)
        self.force_label.set_alpha(1.0 if active else 0.55)

    def refresh(self, force: bool = False):
        tel = self.state.bundle.telemetry
        key = (tel.source_file, tel.frame_count)
        frame = int(min(max(self.state.current_frame, 0), max(tel.frame_count - 1, 0)))
        if not force and key == self._last_key and frame == self._last_frame:
            return
        self._reset_cache_if_needed(key)
        self._last_key = key
        self._last_frame = frame

        if not tel.has_state:
            self.status_text.set_text("No telemetry")
            self.link1_line.set_data([], [])
            self.link2_line.set_data([], [])
            self.trail1.set_data([], [])
            self.trail2.set_data([], [])
            if hasattr(self, "force_arrow"):
                self.force_arrow.set_visible(False)
                self.force_label.set_visible(False)
            self._background = None
            self.canvas.draw_idle()
            return

        if force or self._cached_limits is None:
            self._cached_limits = self._compute_limits()
            self.ax.set_xlim(*self._cached_limits[0])
            self.ax.set_ylim(*self._cached_limits[1])
            self.rail.set_data([self._cached_limits[0][0], self._cached_limits[0][1]], [-0.06, -0.06])
            self._background = None

        f = tel.frame(frame)
        x = f["x"]
        th1 = f["theta1"]
        th2 = f["theta2"]
        l1 = l2 = 0.5
        p0 = (x, 0.0)
        p1 = (x + l1 * np.sin(th1), -l1 * np.cos(th1))
        p2 = (p1[0] + l2 * np.sin(th2), p1[1] - l2 * np.cos(th2))

        self.cart.set_xy((x - 0.15, -0.06))
        self.link1_line.set_data([p0[0], p1[0]], [p0[1], p1[1]])
        self.link2_line.set_data([p1[0], p2[0]], [p1[1], p2[1]])

        tail = min(90, frame + 1)
        start = max(0, frame + 1 - tail)
        _, p1x, p1y, p2x, p2y = self._trajectory_points()
        self.trail1.set_data(p1x[start:frame + 1], p1y[start:frame + 1])
        self.trail2.set_data(p2x[start:frame + 1], p2y[start:frame + 1])

        self._update_force_arrow(frame, f, p0, p1, p2)

        mode = f.get("mode") or "--"
        target = f.get("active_target") or "--"
        self.status_text.set_text(f"t={f['time_s']:.2f}s   mode={mode}   target={target}")
        self._ensure_background()
        self._draw_dynamic()
