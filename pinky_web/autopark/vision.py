import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import cv2
import numpy as np

from .config import AutoParkConfig


@dataclass
class VisionMeasurement:
    theta_green: Optional[float]
    theta_blue: Optional[float]
    x_line_mean: Optional[float]
    distance_to_blue_line: Optional[float]
    confidence: float
    blue_y_mean: Optional[float]


class Vision:
    def __init__(self, config: AutoParkConfig) -> None:
        self.cfg = config
        self.theta_hist: Deque[float] = deque(maxlen=config.theta_history_len)

    def _fit_line(self, mask: np.ndarray) -> Tuple[Optional[float], Optional[float], float, float, int]:
        ys, xs = np.where(mask > 0)
        count = int(xs.size)
        if count < self.cfg.min_line_pixels:
            return None, None, 1.0, 0.0, count

        pts = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))
        vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        vx = float(vx)
        vy = float(vy)
        x0 = float(x0)
        y0 = float(y0)
        theta = math.atan2(vy, vx)

        nx = -vy
        ny = vx
        residuals = np.abs((pts[:, 0] - x0) * nx + (pts[:, 1] - y0) * ny)
        fit_score = 1.0 - min(1.0, float(np.mean(residuals)) / self.cfg.max_fit_residual_px)
        x_mean = float(np.mean(xs))
        y_mean = float(np.mean(ys))
        return theta, x_mean, fit_score, y_mean, count

    def process(self, image_bgr: np.ndarray) -> VisionMeasurement:
        if image_bgr.shape[1] != self.cfg.image_width or image_bgr.shape[0] != self.cfg.image_height:
            image_bgr = cv2.resize(image_bgr, (self.cfg.image_width, self.cfg.image_height))

        h = self.cfg.image_height
        roi_top = h // 3
        roi_bottom = h - self.cfg.bottom_exclude_px
        roi = image_bgr[roi_top:roi_bottom, :]
        roi_h, roi_w = roi.shape[:2]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        green_mask = cv2.inRange(
            hsv,
            np.array(self.cfg.hsv_green_low, dtype=np.uint8),
            np.array(self.cfg.hsv_green_high, dtype=np.uint8),
        )
        blue_mask = cv2.inRange(
            hsv,
            np.array(self.cfg.hsv_blue_low, dtype=np.uint8),
            np.array(self.cfg.hsv_blue_high, dtype=np.uint8),
        )

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)

        theta_green, xg, fit_g, _yg, px_g = self._fit_line(green_mask)
        theta_blue, xb, fit_b, yb, px_b = self._fit_line(blue_mask)

        line_theta = theta_blue if theta_blue is not None else theta_green
        if line_theta is not None:
            self.theta_hist.append(line_theta)

        pixels_norm = min(1.0, float(max(px_g, px_b)) / float(roi_w * roi_h * 0.20))
        fit_score = max(fit_g, fit_b)
        if len(self.theta_hist) > 1:
            var = float(np.var(np.array(self.theta_hist, dtype=np.float32)))
            stability = 1.0 - min(1.0, var / 0.2)
        else:
            stability = 0.0

        conf = 0.4 * pixels_norm + 0.4 * fit_score + 0.2 * stability
        conf = max(0.0, min(1.0, conf))

        x_line_mean = xb if xb is not None else xg
        distance_to_blue = None
        blue_y_mean = None
        if yb is not None:
            blue_y_mean = yb
            distance_to_blue = (
                self.cfg.blue_distance_scale_m_per_px * (roi_h - yb)
                + self.cfg.blue_distance_offset_m
            )

        return VisionMeasurement(
            theta_green=theta_green,
            theta_blue=theta_blue,
            x_line_mean=x_line_mean,
            distance_to_blue_line=distance_to_blue,
            confidence=conf,
            blue_y_mean=blue_y_mean,
        )
