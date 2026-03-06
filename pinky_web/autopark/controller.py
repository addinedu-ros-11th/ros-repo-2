import math
from dataclasses import dataclass
from typing import Optional, Tuple

from .config import AutoParkConfig
from .vision import VisionMeasurement


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def wrap_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


class Controller:
    def __init__(self, config: AutoParkConfig) -> None:
        self.cfg = config
        self.last_omega = 0.0

    def reset_rate_limiter(self) -> None:
        self.last_omega = 0.0

    def gain_schedule(self, v: float) -> Tuple[float, float]:
        speed_ratio = min(1.0, max(0.0, v / self.cfg.gain_schedule_speed_ref))
        scale = 1.0 - 0.45 * speed_ratio
        return self.cfg.k_theta * scale, self.cfg.k_d * scale

    def pure_pursuit_omega(self, pose: Pose2D, target_xy: Tuple[float, float], v: float) -> float:
        dx = target_xy[0] - pose.x
        dy = target_xy[1] - pose.y
        alpha = wrap_angle(math.atan2(dy, dx) - pose.yaw)
        return 2.0 * v * math.sin(alpha) / self.cfg.lookahead_m

    def _vision_errors(
        self,
        meas: VisionMeasurement,
        use_blue_only: bool,
        img_center_x: float,
    ) -> Tuple[Optional[float], float]:
        theta = None
        if use_blue_only:
            theta = meas.theta_blue
        else:
            theta = meas.theta_green if meas.theta_green is not None else meas.theta_blue

        e_theta = None if theta is None else wrap_angle(theta)

        e_perp = 0.0
        if meas.x_line_mean is not None:
            e_perp = (img_center_x - meas.x_line_mean) / img_center_x
        return e_theta, e_perp

    def compute_omega(
        self,
        pose: Pose2D,
        target_xy: Tuple[float, float],
        v: float,
        meas: VisionMeasurement,
        dt: float,
        use_blue_only: bool = False,
    ) -> float:
        omega_pp = self.pure_pursuit_omega(pose, target_xy, v)
        k_theta, k_d = self.gain_schedule(v)

        e_theta, e_perp = self._vision_errors(meas, use_blue_only, img_center_x=self.cfg.image_width * 0.5)

        correction = 0.0
        if e_theta is not None:
            if meas.confidence >= self.cfg.conf_high:
                correction = k_theta * e_theta + k_d * e_perp
            elif meas.confidence >= self.cfg.conf_low:
                correction = k_theta * e_theta

        omega = omega_pp + correction

        omega = max(-self.cfg.w_max, min(self.cfg.w_max, omega))
        max_delta = self.cfg.omega_rate_limit * max(dt, 1e-3)
        delta = omega - self.last_omega
        if delta > max_delta:
            omega = self.last_omega + max_delta
        elif delta < -max_delta:
            omega = self.last_omega - max_delta

        self.last_omega = omega
        return omega
