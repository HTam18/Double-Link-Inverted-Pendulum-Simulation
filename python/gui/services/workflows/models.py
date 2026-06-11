from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


SWITCHING_RESULT = Path("results/switching/switchingResults.mat")
RECOVERY_RESULT = Path("results/recovery/recoveryResults.mat")
RECOVERY_METRICS_CSV = Path("results/recovery/recoveryAllMetrics.csv")


@dataclass(frozen=True)
class WorkflowCase:
    key: str
    label: str
    description: str


TARGET_TRANSITION_CASES: tuple[WorkflowCase, ...] = (
    WorkflowCase("full_sequence", "Complete target sequence", "Replay the complete MATLAB multi-target sequence."),
    WorkflowCase("down_down_to_up_up", "Down Down → Up Up", "Direct target transition."),
    WorkflowCase("up_up_to_up_down", "Up Up → Up Down", "Direct target transition."),
    WorkflowCase("up_down_to_up_up", "Up Down → Up Up", "Direct target transition."),
    WorkflowCase("up_up_to_down_up", "Up Up → Down Up", "Direct target transition."),
    WorkflowCase("down_up_to_up_up", "Down Up → Up Up", "Direct target transition."),
    WorkflowCase("down_down_to_up_down", "Down Down → Up Down", "Composed from Down Down → Up Up → Up Down."),
    WorkflowCase("down_down_to_down_up", "Down Down → Down Up", "Composed from Down Down → Up Up → Down Up."),
    WorkflowCase("up_down_to_down_up", "Up Down → Down Up", "Composed from Up Down → Up Up → Down Up."),
    WorkflowCase("down_up_to_up_down", "Down Up → Up Down", "Composed from Down Up → Up Up → Down Up."),
)

FORCE_RECOVERY_CASES: tuple[WorkflowCase, ...] = (
    WorkflowCase("target_recovery", "Target recovery", "Force event while the system is stabilized around a target."),
    WorkflowCase("after_switching", "After switching recovery", "Force event after the target switching sequence."),
)
