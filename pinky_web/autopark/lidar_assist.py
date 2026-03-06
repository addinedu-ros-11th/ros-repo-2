import math
from dataclasses import dataclass
from typing import List

from sensor_msgs.msg import LaserScan

from .config import AutoParkConfig


@dataclass
class LidarSnapshot:
    front_min_m: float = math.inf
    right_min_m: float = math.inf
    left_min_m: float = math.inf
    rear_min_m: float = math.inf
    front_blocked: bool = False
    side_blocked: bool = False
    rear_blocked: bool = False


class LidarAssist:
    def __init__(self, config: AutoParkConfig) -> None:
        self.cfg = config

    @staticmethod
    def _wrap(rad: float) -> float:
        while rad > math.pi:
            rad -= 2.0 * math.pi
        while rad < -math.pi:
            rad += 2.0 * math.pi
        return rad

    def _sector_min(
        self,
        angles: List[float],
        ranges: List[float],
        center_rad: float,
        half_width_rad: float,
    ) -> float:
        candidates = []
        for ang, rng in zip(angles, ranges):
            if not math.isfinite(rng):
                continue
            if rng < self.cfg.lidar_range_min_valid_m or rng > self.cfg.lidar_range_max_valid_m:
                continue
            if abs(self._wrap(ang - center_rad)) <= half_width_rad:
                candidates.append(rng)

        if not candidates:
            return math.inf
        return min(candidates)

    def process_scan(self, msg: LaserScan) -> LidarSnapshot:
        n = len(msg.ranges)
        if n == 0:
            return LidarSnapshot()

        angles = [msg.angle_min + i * msg.angle_increment for i in range(n)]
        ranges = list(msg.ranges)

        front = self._sector_min(
            angles,
            ranges,
            center_rad=0.0,
            half_width_rad=math.radians(self.cfg.lidar_front_sector_deg),
        )
        right = self._sector_min(
            angles,
            ranges,
            center_rad=-math.pi / 2.0,
            half_width_rad=math.radians(self.cfg.lidar_side_sector_deg),
        )
        left = self._sector_min(
            angles,
            ranges,
            center_rad=math.pi / 2.0,
            half_width_rad=math.radians(self.cfg.lidar_side_sector_deg),
        )
        rear = self._sector_min(
            angles,
            ranges,
            center_rad=math.pi,
            half_width_rad=math.radians(self.cfg.lidar_rear_sector_deg),
        )

        return LidarSnapshot(
            front_min_m=front,
            right_min_m=right,
            left_min_m=left,
            rear_min_m=rear,
            front_blocked=front < self.cfg.lidar_obstacle_front_m,
            side_blocked=(right < self.cfg.lidar_obstacle_side_m) or (left < self.cfg.lidar_obstacle_side_m),
            rear_blocked=rear < self.cfg.lidar_reverse_clearance_m,
        )
