from __future__ import annotations

import numpy as np
import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from python.gui.core.displayNames import display_name
from python.gui.ui import theme
from python.gui.ui.widgets import figma


class GraphPanel(ctk.CTkFrame):
    """Scientific signal panel.

    Layout rule:
    - Matplotlib canvas and value badges are two separate Tk columns.
    - Badges never overlay the canvas, so they cannot crop the plots.
    - The four subplots use equal-height axes; the small title text is drawn
      inside each axis instead of using Matplotlib title space. This keeps the
      badge rows visually aligned with x, theta1, theta2 and control.
    """

    BADGE_W = 108
    BADGE_H = 72

    def __init__(self, parent, state):
        super().__init__(parent, fg_color="transparent")
        self.state = state
        figma.label(self, "Graphs", size=15, weight="bold").grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 6)
        )

        self.content = ctk.CTkFrame(self, fg_color="transparent")
        self.content.grid(row=1, column=0, columnspan=2, sticky="nsew")
        self.content.grid_columnconfigure(0, weight=1)
        self.content.grid_columnconfigure(1, weight=0, minsize=118)
        self.content.grid_rowconfigure(0, weight=1)

        self.fig = Figure(figsize=(5.65, 5.2), dpi=100)
        self.axes = self.fig.subplots(4, 1, sharex=True)
        self.fig.subplots_adjust(left=0.15, right=0.985, top=0.985, bottom=0.075, hspace=0.26)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.content)
        self.widget = self.canvas.get_tk_widget()
        self.widget.grid(row=0, column=0, sticky="nsew")

        self.badge_host = ctk.CTkFrame(self.content, fg_color="transparent", width=118)
        self.badge_host.grid(row=0, column=1, sticky="nsew", padx=(10, 0))
        self.badge_host.grid_propagate(False)
        for row in range(4):
            self.badge_host.grid_rowconfigure(row, weight=1, uniform="badge_row")
        self.badge_host.grid_columnconfigure(0, weight=1)

        self.badges = {
            "x": figma.value_badge(self.badge_host, "x", theme.PLOT_X, width=self.BADGE_W, height=self.BADGE_H),
            "theta1": figma.value_badge(self.badge_host, display_name("theta1"), theme.PLOT_THETA1, width=self.BADGE_W, height=self.BADGE_H),
            "theta2": figma.value_badge(self.badge_host, display_name("theta2"), theme.PLOT_THETA2, width=self.BADGE_W, height=self.BADGE_H),
            "u_cmd": figma.value_badge(self.badge_host, display_name("u_cmd"), theme.PLOT_U, width=self.BADGE_W, height=self.BADGE_H),
        }
        for row, key in enumerate(["x", "theta1", "theta2", "u_cmd"]):
            self.badges[key].grid(row=row, column=0, sticky="n", pady=(4, 0))

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)

        self._last_source = None
        self._last_frame = None
        self._cursor_lines = []
        self._background = None
        self._canvas_size = None
        self.refresh(force=True)

    def _style_axis(self, ax, title: str, y_label: str):
        ax.grid(True, color=theme.GRID, alpha=0.8, linewidth=0.8)
        ax.tick_params(labelsize=8, pad=2)
        ax.set_ylabel(y_label, fontsize=8, labelpad=7)
        ax.text(
            0.01, 0.93, title,
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=9,
            color=theme.TEXT,
            bbox=dict(facecolor="white", edgecolor="none", alpha=0.72, pad=1.5),
        )

    def _format_value(self, value, unit: str) -> str:
        if value is None:
            return "--"
        try:
            val = float(value)
        except Exception:
            return "--"
        if not np.isfinite(val):
            return "--"
        return f"{val:+.3f} {unit}"

    def _update_badges(self):
        tel = self.state.bundle.telemetry
        if not tel.has_state:
            for card in self.badges.values():
                card.value_label.configure(text="--")
            return
        idx = int(np.clip(self.state.current_frame, 0, tel.frame_count - 1))
        frame = tel.frame(idx)
        self.badges["x"].value_label.configure(text=self._format_value(frame.get("x"), "m"))
        self.badges["theta1"].value_label.configure(text=self._format_value(frame.get("theta1"), "rad"))
        self.badges["theta2"].value_label.configure(text=self._format_value(frame.get("theta2"), "rad"))
        self.badges["u_cmd"].value_label.configure(text=self._format_value(frame.get("u_cmd"), "N"))

    def _full_redraw(self):
        tel = self.state.bundle.telemetry
        for ax in self.axes:
            ax.clear()
        self._cursor_lines.clear()
        self._background = None
        self._canvas_size = None

        if not tel.has_state:
            self.axes[0].text(0.5, 0.5, "No telemetry", ha="center", va="center")
            self.canvas.draw_idle()
            self._update_badges()
            return

        t = np.asarray(tel.time_s)
        ax_x, ax_th1, ax_th2, ax_u = self.axes
        ax_x.plot(t, tel.state[:, 0], color=theme.PLOT_X, linewidth=2.0)
        ax_th1.plot(t, tel.state[:, 2], color=theme.PLOT_THETA1, linewidth=2.0)
        ax_th2.plot(t, tel.state[:, 4], color=theme.PLOT_THETA2, linewidth=2.0)
        if tel.u_cmd is not None:
            ax_u.plot(t[: len(tel.u_cmd)], tel.u_cmd, color=theme.PLOT_U, linewidth=1.8, label=display_name("u_cmd"))
        if tel.u_actual is not None:
            ax_u.plot(t[: len(tel.u_actual)], tel.u_actual, color=theme.PLOT_U_ACTUAL, linewidth=1.6, label=display_name("u_actual"))

        titles = ["x", display_name("theta1"), display_name("theta2"), "Control"]
        ylabels = ["m", "rad", "rad", "N"]
        for ax, title, ylab in zip(self.axes, titles, ylabels):
            self._style_axis(ax, title, ylab)
            cursor = ax.axvline(
                t[0] if len(t) else 0.0,
                color="#111827",
                linewidth=1.8,
                alpha=0.95,
                linestyle="--",
                animated=True,
            )
            self._cursor_lines.append(cursor)
        self.axes[-1].set_xlabel("time [s]", fontsize=9, labelpad=4)
        if tel.u_cmd is not None or tel.u_actual is not None:
            ax_u.legend(loc="upper right", fontsize=8, frameon=False)

        self.canvas.draw()
        self._background = self.canvas.copy_from_bbox(self.fig.bbox)
        self._canvas_size = (self.widget.winfo_width(), self.widget.winfo_height())
        self._update_badges()
        self._draw_cursor_only()

    def _draw_cursor_only(self):
        tel = self.state.bundle.telemetry
        if not tel.has_state or not self._cursor_lines:
            return
        current_size = (self.widget.winfo_width(), self.widget.winfo_height())
        if self._background is None or current_size != self._canvas_size:
            self.canvas.draw()
            self._background = self.canvas.copy_from_bbox(self.fig.bbox)
            self._canvas_size = current_size
        idx = int(np.clip(self.state.current_frame, 0, tel.frame_count - 1))
        current_t = float(tel.time_s[idx])
        for cursor in self._cursor_lines:
            cursor.set_xdata([current_t, current_t])
        self.canvas.restore_region(self._background)
        for cursor in self._cursor_lines:
            cursor.axes.draw_artist(cursor)
        self.canvas.blit(self.fig.bbox)
        self.canvas.flush_events()

    def refresh(self, force: bool = False):
        tel = self.state.bundle.telemetry
        source = (tel.source_file, tel.frame_count)
        frame = int(self.state.current_frame)
        if force or source != self._last_source:
            self._last_source = source
            self._last_frame = frame
            self._full_redraw()
            return
        if frame == self._last_frame:
            return
        self._last_frame = frame
        self._update_badges()
        self._draw_cursor_only()
