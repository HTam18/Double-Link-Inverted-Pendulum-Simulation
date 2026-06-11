from __future__ import annotations

import customtkinter as ctk

from python.gui.ui import theme


def frame(parent, **kwargs):
    return ctk.CTkFrame(
        parent,
        fg_color=kwargs.pop("fg_color", theme.CARD),
        corner_radius=kwargs.pop("corner_radius", 14),
        border_width=kwargs.pop("border_width", 1),
        border_color=kwargs.pop("border_color", theme.BORDER),
        **kwargs,
    )


def label(parent, text: str, size: int = 13, weight: str = "normal", color: str | None = None, **kwargs):
    return ctk.CTkLabel(
        parent,
        text=text,
        font=ctk.CTkFont(family=theme.FONT_FAMILY, size=size, weight=weight),
        text_color=color or theme.TEXT,
        **kwargs,
    )


def button(parent, text: str, command=None, accent: bool = False, **kwargs):
    fg = theme.ACCENT if accent else "#E8EEF5"
    hover = theme.ACCENT_DARK if accent else "#DDE7F0"
    text_color = "#FFFFFF" if accent else theme.TEXT
    return ctk.CTkButton(
        parent,
        text=text,
        command=command,
        fg_color=fg,
        hover_color=hover,
        text_color=text_color,
        corner_radius=10,
        height=34,
        font=ctk.CTkFont(family=theme.FONT_FAMILY, size=12),
        **kwargs,
    )


def combo(parent, values: list[str], variable=None, **kwargs):
    return ctk.CTkComboBox(
        parent,
        values=values,
        variable=variable,
        fg_color="#FFFFFF",
        border_color=theme.BORDER,
        button_color=theme.ACCENT,
        button_hover_color=theme.ACCENT_DARK,
        corner_radius=10,
        font=ctk.CTkFont(family=theme.FONT_FAMILY, size=12),
        dropdown_font=ctk.CTkFont(family=theme.FONT_FAMILY, size=12),
        **kwargs,
    )


def entry(parent, textvariable=None, **kwargs):
    return ctk.CTkEntry(
        parent,
        textvariable=textvariable,
        fg_color="#FFFFFF",
        border_color=theme.BORDER,
        corner_radius=10,
        font=ctk.CTkFont(family=theme.FONT_FAMILY, size=12),
        **kwargs,
    )


def metric_card(parent, title: str, value: str = "--", subtitle: str = ""):
    card = frame(parent)
    card.grid_columnconfigure(0, weight=1)
    title_label = label(card, title, size=11, color=theme.MUTED)
    title_label.grid(row=0, column=0, sticky="w", padx=10, pady=(8, 0))
    value_label = label(card, value, size=18, weight="bold")
    value_label.grid(row=1, column=0, sticky="w", padx=10, pady=(1, 0))
    sub_label = label(card, subtitle, size=10, color=theme.MUTED)
    sub_label.grid(row=2, column=0, sticky="w", padx=10, pady=(0, 8))
    card.value_label = value_label
    card.sub_label = sub_label
    return card


def value_badge(parent, name: str, color: str, width: int = 108, height: int = 72):
    card = frame(parent, corner_radius=12, width=width, height=height)
    card.grid_propagate(False)
    card.pack_propagate(False)
    card.configure(width=width, height=height)
    card.grid_columnconfigure(0, weight=1)
    label(card, name, size=11, color=theme.MUTED).grid(row=0, column=0, sticky="w", padx=10, pady=(8, 0))
    value_label = label(card, "--", size=16, weight="bold", color=color)
    value_label.grid(row=1, column=0, sticky="w", padx=10, pady=(2, 8))
    card.value_label = value_label
    return card
