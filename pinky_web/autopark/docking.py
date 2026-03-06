import math
import statistics
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

from .config import AutoParkConfig
from .controller import wrap_angle
from .vision import VisionMeasurement


@dataclass
class ReverseStatus:
    done: bool
    abort: bool
    v_cmd: float
    w_cmd: float


class Docking:
    def __init__(self, config: AutoParkConfig) -> None:
        self.cfg = config
        self.dist_hist: Deque[float] = deque(maxlen=config.stop_align_median_frames)
        self.theta_hist: Deque[float] = deque(maxlen=config.stop_align_median_frames)
        self.backoff_until = 0.0

        self.reverse_start_xy: Optional[Tuple[float, float]] = None
        self.reverse_start_yaw: Optional[float] = None

    def reset_alignment(self) -> None:
        self.dist_hist.clear()
        self.theta_hist.clear()
        self.backoff_until = 0.0

    def update_stop_alignment(
        self,
        now_s: float,
        meas: VisionMeasurement,
    ) -> Tuple[float, bool]:
        if self.backoff_until > now_s:
            return self.cfg.stop_backoff_speed_mps, False

        if meas.distance_to_blue_line is None or meas.theta_blue is None:
            return self.cfg.v_precision_mps * 0.6, False

        self.dist_hist.append(meas.distance_to_blue_line)
        self.theta_hist.append(wrap_angle(meas.theta_blue))

        if (
            meas.blue_y_mean is not None
            and meas.blue_y_mean > ((self.cfg.image_height * 2.0 / 3.0) - self.cfg.bottom_exclude_px) * 0.92
        ):
            self.backoff_until = now_s + self.cfg.stop_backoff_duration_s
            return self.cfg.stop_backoff_speed_mps, False

        if len(self.dist_hist) < self.cfg.stop_align_median_frames:
            return self.cfg.v_precision_mps, False

        dist_med = statistics.median(self.dist_hist)
        done = abs(dist_med - self.cfg.stop_target_distance_m) <= self.cfg.stop_distance_tolerance_m
        return self.cfg.v_precision_mps, done

    def start_reverse(self, x: float, y: float, yaw: float) -> None:
        self.reverse_start_xy = (x, y)
        self.reverse_start_yaw = yaw

    def update_reverse(self, x: float, y: float, yaw: float) -> ReverseStatus:
        if self.reverse_start_xy is None or self.reverse_start_yaw is None:
            return ReverseStatus(done=False, abort=True, v_cmd=0.0, w_cmd=0.0)

        dx = x - self.reverse_start_xy[0]
        dy = y - self.reverse_start_xy[1]
        traveled = math.hypot(dx, dy)

        drift = abs(math.degrees(wrap_angle(yaw - self.reverse_start_yaw)))
        if drift > self.cfg.reverse_drift_abort_deg:
            return ReverseStatus(done=False, abort=True, v_cmd=0.0, w_cmd=0.0)

        if traveled >= self.cfg.reverse_distance_m:
            return ReverseStatus(done=True, abort=False, v_cmd=0.0, w_cmd=0.0)

        return ReverseStatus(done=False, abort=False, v_cmd=-self.cfg.v_reverse_mps, w_cmd=0.0)
