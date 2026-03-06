from dataclasses import dataclass

from .config import AutoParkConfig


class ObstacleAction:
    NONE = "NONE"
    SLOW = "SLOW"
    RIGHT_DETOUR = "RIGHT_DETOUR"
    LEFT_DETOUR = "LEFT_DETOUR"
    BACKWARD_RECOVERY = "BACKWARD_RECOVERY"
    SAFE_STOP = "SAFE_STOP"


@dataclass
class ObstacleHandler:
    cfg: AutoParkConfig
    persist_count: int = 0
    attempts: int = 0

    def reset(self) -> None:
        self.persist_count = 0
        self.attempts = 0

    def update(self, obstacle_detected: bool) -> str:
        if not obstacle_detected:
            self.persist_count = 0
            return ObstacleAction.NONE

        self.persist_count += 1
        if self.persist_count <= self.cfg.obstacle_persist_frames:
            return ObstacleAction.SLOW

        self.attempts += 1
        if self.attempts == 1:
            return ObstacleAction.RIGHT_DETOUR
        if self.attempts == 2:
            return ObstacleAction.LEFT_DETOUR
        if self.attempts <= self.cfg.obstacle_max_attempts:
            return ObstacleAction.BACKWARD_RECOVERY
        return ObstacleAction.SAFE_STOP
