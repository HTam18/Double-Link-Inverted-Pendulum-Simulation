from __future__ import annotations

DISPLAY_TEXT = {
    "switching": "Target transition results",
    "recovery": "Recovery results",
    "robustness": "Robustness results",
    "final_demo": "Final demo results",
    "full_sequence": "Complete target sequence",
    "target_recovery": "Target recovery",
    "after_switching": "After switching recovery",
    "after_full_switching": "After full switching",
    "down_down": "Down Down",
    "up_up": "Up Up",
    "up_down": "Up Down",
    "down_up": "Down Up",
    "left": "Left",
    "right": "Right",
    "case_index": "Case index",
    "case_type": "Case type",
    "event_name": "Event",
    "target": "Target",
    "link_id": "Link",
    "contact_ratio": "Contact ratio",
    "direction": "Direction",
    "force_N": "Force N",
    "duration_s": "Duration s",
    "success": "Success",
    "recovery_time_s": "Recovery time s",
    "saturation_fraction": "Saturation fraction",
    "fail_reason": "Fail reason",
    "final_angle_error_rad": "Final angle error rad",
    "final_velocity_norm": "Final velocity norm",
    "max_abs_x_m": "Max abs x m",
    "max_abs_u_cmd_N": "Max abs command N",
    "planner_selected_variant": "Planner variant",
    "u_cmd": "Command",
    "u_actual": "Actual command",
    "theta1": "Theta 1",
    "theta2": "Theta 2",
    "x_dot": "Cart velocity",
    "theta1_dot": "Theta 1 velocity",
    "theta2_dot": "Theta 2 velocity",
}


def display_name(value: object) -> str:
    """Convert an internal token into a user-facing label."""
    text = str(value)
    if text in DISPLAY_TEXT:
        return DISPLAY_TEXT[text]
    return text.replace("_", " ").strip().title()


def display_route(route: str | list[str] | tuple[str, ...]) -> str:
    """Format target route tokens for display."""
    if isinstance(route, str):
        parts = [p.strip() for p in route.split("→")]
    else:
        parts = list(route)
    return " → ".join(display_transition(part) for part in parts if str(part).strip())


def display_transition(edge: object) -> str:
    text = str(edge)
    if "_to_" in text:
        left, right = text.split("_to_", 1)
        return f"{display_name(left)} → {display_name(right)}"
    return display_name(text)
