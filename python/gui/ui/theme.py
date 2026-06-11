from __future__ import annotations

import customtkinter as ctk

BG = "#F4F7FB"
CARD = "#FFFFFF"
BORDER = "#D9E2EC"
TEXT = "#14213D"
MUTED = "#6B7280"
ACCENT = "#2A9D8F"
ACCENT_DARK = "#1F7A70"
WARN = "#F4A261"
DANGER = "#E76F51"
OK = "#2A9D8F"
PLOT_X = "#1f77b4"
PLOT_THETA1 = "#ff7f0e"
PLOT_THETA2 = "#2ca02c"
PLOT_U = "#d62728"
PLOT_U_ACTUAL = "#9467bd"
GRID = "#DCE4ED"
RAIL = "#9AA5B1"
TRAIL_ALPHA = 0.22
FONT_FAMILY = "Segoe UI"
MONO_FONT = "Cascadia Mono"


def apply_theme() -> None:
    ctk.set_appearance_mode("light")
    ctk.set_default_color_theme("blue")
    try:
        import matplotlib as mpl
        mpl.rcParams.update({
            "font.family": FONT_FAMILY,
            "font.size": 9,
            "axes.titlesize": 10,
            "axes.labelsize": 9,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "legend.fontsize": 8,
            "axes.edgecolor": "#1F2937",
            "axes.linewidth": 0.8,
            "figure.facecolor": "#FFFFFF",
            "axes.facecolor": "#FFFFFF",
        })
    except Exception:
        pass
