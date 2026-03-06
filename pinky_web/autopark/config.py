from dataclasses import dataclass, field
from typing import Dict, Tuple


@dataclass
class AutoParkConfig:
    wheel_radius_m: float = 0.027
    wheel_track_m: float = 0.098
    # Camera lateral offset from wheel-center axis (+x right, -x left), y is forward axis.
    camera_lateral_offset_m: float = 0.030

    grid_size: int = 6
    cell_size_m: float = 0.25
    world_size_m: float = 1.5
    block_cells: Tuple[Tuple[int, int], ...] = (
        (1, 2),
        (1, 3),
        (3, 2),
        (3, 3),
        (5, 2),
        (5, 3),
    )

    park_map: Dict[int, Dict[str, Tuple[int, int]]] = field(
        default_factory=lambda: {
            1: {"park": (5, 0), "stop": (4, 0)},
            2: {"park": (5, 1), "stop": (4, 1)},
            3: {"park": (5, 2), "stop": (4, 2)},
            4: {"park": (5, 3), "stop": (4, 3)},
            5: {"park": (5, 4), "stop": (4, 4)},
            6: {"park": (0, 4), "stop": (1, 4)},
            7: {"park": (0, 3), "stop": (1, 3)},
            8: {"park": (0, 2), "stop": (1, 2)},
            9: {"park": (0, 1), "stop": (1, 1)},
            10: {"park": (0, 0), "stop": (1, 0)},
        }
    )

    image_width: int = 320
    image_height: int = 240
    bottom_exclude_px: int = 15

    hsv_green_low: Tuple[int, int, int] = (35, 60, 60)
    hsv_green_high: Tuple[int, int, int] = (90, 255, 255)
    hsv_blue_low: Tuple[int, int, int] = (90, 80, 60)
    hsv_blue_high: Tuple[int, int, int] = (130, 255, 255)

    conf_high: float = 0.7
    conf_low: float = 0.4
    theta_history_len: int = 12
    min_line_pixels: int = 80
    max_fit_residual_px: float = 15.0

    lookahead_m: float = 0.15
    v_nominal_mps: float = 0.15
    v_degraded_mps: float = 0.09
    v_creep_mps: float = 0.05
    v_precision_mps: float = 0.06
    v_reverse_mps: float = 0.06

    k_theta: float = 1.4
    k_d: float = 0.6
    w_max: float = 1.2
    omega_rate_limit: float = 2.5
    gain_schedule_speed_ref: float = 0.22

    axis_switch_conf_threshold: float = 0.65
    axis_switch_frames: int = 5

    lost_short_s: float = 0.8
    lost_long_s: float = 2.0
    lost_recovery_retries: int = 4
    lost_scan_angle_deg: float = 15.0
    lost_scan_omega: float = 0.5

    stop_target_distance_m: float = 0.27
    stop_distance_tolerance_m: float = 0.02
    stop_align_median_frames: int = 7
    stop_backoff_duration_s: float = 0.3
    stop_backoff_speed_mps: float = -0.03

    reverse_distance_m: float = 0.25
    reverse_drift_abort_deg: float = 6.0

    control_hz: float = 20.0
    waypoint_reached_m: float = 0.05
    heading_cell_update_tol_deg: float = 20.0

    obstacle_persist_frames: int = 3
    obstacle_max_attempts: int = 3
    lidar_enabled: bool = True
    lidar_topic: str = "/scan"
    lidar_range_min_valid_m: float = 0.05
    lidar_range_max_valid_m: float = 2.5
    lidar_front_sector_deg: float = 20.0
    lidar_side_sector_deg: float = 35.0
    lidar_rear_sector_deg: float = 25.0
    lidar_obstacle_front_m: float = 0.30
    lidar_obstacle_side_m: float = 0.22
    lidar_reverse_clearance_m: float = 0.18

    start_cell: Tuple[int, int] = (5, 5)
    exit_start_cell: Tuple[int, int] = (2, 0)
    exit_heading_axis: str = "-y"

    # Pixel-to-distance conversion for STOP alignment (camera-specific calibration value).
    blue_distance_scale_m_per_px: float = 0.0035
    blue_distance_offset_m: float = 0.05
