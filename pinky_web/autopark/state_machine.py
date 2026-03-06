from dataclasses import dataclass, field
from typing import List, Optional, Tuple

GridCell = Tuple[int, int]


class States:
    IDLE = "IDLE"
    PLAN_PATH = "PLAN_PATH"
    NAVIGATE = "NAVIGATE"
    DEGRADED_NAV = "DEGRADED_NAV"
    STOP_ALIGN = "STOP_ALIGN"
    REVERSE_DOCK = "REVERSE_DOCK"
    WAIT = "WAIT"
    UNDOCK = "UNDOCK"
    LOST_RECOVERY = "LOST_RECOVERY"
    SAFE_STOP = "SAFE_STOP"
    DONE = "DONE"


@dataclass
class StateMachine:
    state: str = States.IDLE
    target_slot: Optional[int] = None
    current_axis: str = "x"
    pending_axis: Optional[str] = None
    pending_axis_count: int = 0
    path: List[GridCell] = field(default_factory=list)
    waypoint_idx: int = 0

    def transition(self, new_state: str) -> None:
        self.state = new_state

    def set_path(self, path: List[GridCell]) -> None:
        self.path = path
        self.waypoint_idx = 1 if len(path) > 1 else 0

    def current_waypoint(self) -> Optional[GridCell]:
        if 0 <= self.waypoint_idx < len(self.path):
            return self.path[self.waypoint_idx]
        return None

    def advance_waypoint(self) -> None:
        if self.waypoint_idx < len(self.path) - 1:
            self.waypoint_idx += 1

    def is_last_waypoint(self) -> bool:
        return self.waypoint_idx >= len(self.path) - 1

    def update_axis_hysteresis(self, desired_axis: str, confidence_ok: bool, frames_required: int) -> None:
        if desired_axis == self.current_axis:
            self.pending_axis = None
            self.pending_axis_count = 0
            return

        if not confidence_ok:
            self.pending_axis = None
            self.pending_axis_count = 0
            return

        if self.pending_axis != desired_axis:
            self.pending_axis = desired_axis
            self.pending_axis_count = 1
            return

        self.pending_axis_count += 1
        if self.pending_axis_count >= frames_required:
            self.current_axis = desired_axis
            self.pending_axis = None
            self.pending_axis_count = 0
