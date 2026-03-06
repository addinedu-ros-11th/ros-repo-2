#!/usr/bin/env python3
import math
import re
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover
    CvBridge = None

from autopark import (
    AutoParkConfig,
    Controller,
    Docking,
    LidarAssist,
    LidarSnapshot,
    ObstacleAction,
    ObstacleHandler,
    Planner,
    Pose2D,
    StateMachine,
    States,
    Vision,
    VisionMeasurement,
)
from autopark.controller import wrap_angle


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PinkyAutoParkNode(Node):
    def __init__(self) -> None:
        super().__init__("pinky_autopark_node")
        self.cfg = AutoParkConfig()
        self.planner = Planner(self.cfg)
        self.vision = Vision(self.cfg)
        self.controller = Controller(self.cfg)
        self.docking = Docking(self.cfg)
        self.sm = StateMachine()
        self.obstacle_handler = ObstacleHandler(self.cfg)
        self.lidar = LidarAssist(self.cfg)

        self.bridge = CvBridge() if CvBridge is not None else None

        self.voice_sub = self.create_subscription(String, "/voice_command", self.on_voice, 10)
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.on_image, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.scan_sub = None
        if self.cfg.lidar_enabled:
            self.scan_sub = self.create_subscription(LaserScan, self.cfg.lidar_topic, self.on_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(1.0 / self.cfg.control_hz, self.on_control)

        self.pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
        self.have_odom = False
        self.prev_odom_xy: Optional[Tuple[float, float]] = None

        self.current_cell = self.cfg.start_cell
        self.target_stop_cell: Optional[Tuple[int, int]] = None
        self.target_park_cell: Optional[Tuple[int, int]] = None
        self.mission_mode: str = "PARK"

        self.accumulated_distance = 0.0

        self.latest_vision = VisionMeasurement(
            theta_green=None,
            theta_blue=None,
            x_line_mean=None,
            distance_to_blue_line=None,
            confidence=0.0,
            blue_y_mean=None,
        )
        self.latest_lidar = LidarSnapshot()

        now = self.get_clock().now().nanoseconds / 1e9
        self.last_control_time = now
        self.last_vision_good_time = now

        self.recovery_base_yaw = 0.0
        self.recovery_scan_dir = 1.0
        self.recovery_retries = 0

        self.manual_obstacle_flag = False

        self.get_logger().info(
            "PINKY AutoPark node initialized "
            f"(wheel_radius={self.cfg.wheel_radius_m:.3f}m, "
            f"wheel_track={self.cfg.wheel_track_m:.3f}m, "
            f"camera_lateral_offset={self.cfg.camera_lateral_offset_m:.3f}m)"
        )

    def on_voice(self, msg: String) -> None:
        text = msg.data.strip().lower()

        if text in ("cancel", "stop", "safe_stop"):
            self.sm.transition(States.SAFE_STOP)
            return
        if text in ("exit", "출구", "go_exit"):
            self.mission_mode = "EXIT"
            self.sm.target_slot = None
            self.sm.transition(States.PLAN_PATH)
            self.get_logger().info("Received exit command")
            return
        if text == "obstacle_on":
            self.manual_obstacle_flag = True
            return
        if text == "obstacle_off":
            self.manual_obstacle_flag = False
            return

        slot = self._parse_slot(text)
        if slot is None:
            return

        self.sm.target_slot = slot
        self.mission_mode = "PARK"
        self.sm.transition(States.PLAN_PATH)
        self.get_logger().info(f"Received slot command: {slot}")

    def _parse_slot(self, text: str) -> Optional[int]:
        m = re.search(r"(10|[1-9])", text)
        if not m:
            return None
        slot = int(m.group(1))
        return slot if slot in self.cfg.park_map else None

    def on_image(self, msg: Image) -> None:
        if self.bridge is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return

        if frame is None or frame.size == 0:
            return

        self.latest_vision = self.vision.process(frame)
        if self.latest_vision.confidence >= self.cfg.conf_low:
            self.last_vision_good_time = self.get_clock().now().nanoseconds / 1e9

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        self.pose = Pose2D(x=float(p.x), y=float(p.y), yaw=yaw)
        self.have_odom = True

        xy = (self.pose.x, self.pose.y)
        if self.prev_odom_xy is not None:
            self.accumulated_distance += math.hypot(xy[0] - self.prev_odom_xy[0], xy[1] - self.prev_odom_xy[1])
        self.prev_odom_xy = xy

    def on_scan(self, msg: LaserScan) -> None:
        self.latest_lidar = self.lidar.process_scan(msg)

    def publish_cmd(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def on_control(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        dt = max(1e-3, now - self.last_control_time)
        self.last_control_time = now

        if not self.have_odom:
            self.publish_cmd(0.0, 0.0)
            return

        if self.sm.state == States.IDLE:
            self.publish_cmd(0.0, 0.0)
            return

        if self.sm.state == States.PLAN_PATH:
            self._handle_plan_path()
            self.publish_cmd(0.0, 0.0)
            return

        if self.sm.state in (States.NAVIGATE, States.DEGRADED_NAV):
            self._handle_navigation(now, dt)
            return

        if self.sm.state == States.LOST_RECOVERY:
            self._handle_lost_recovery(now)
            return

        if self.sm.state == States.STOP_ALIGN:
            self._handle_stop_align(now, dt)
            return

        if self.sm.state == States.REVERSE_DOCK:
            self._handle_reverse_dock()
            return

        if self.sm.state == States.WAIT:
            self.publish_cmd(0.0, 0.0)
            self.sm.transition(States.DONE)
            return

        if self.sm.state == States.UNDOCK:
            self.publish_cmd(self.cfg.v_creep_mps, 0.0)
            self.sm.transition(States.IDLE)
            return

        if self.sm.state in (States.SAFE_STOP, States.DONE):
            self.publish_cmd(0.0, 0.0)
            return

    def _handle_plan_path(self) -> None:
        if self.mission_mode == "EXIT":
            self.target_stop_cell = self.cfg.exit_start_cell
            self.target_park_cell = None
        else:
            if self.sm.target_slot is None:
                self.sm.transition(States.IDLE)
                return
            mapping = self.cfg.park_map[self.sm.target_slot]
            self.target_stop_cell = tuple(mapping["stop"])
            self.target_park_cell = tuple(mapping["park"])

        self.current_cell = self.planner.world_to_cell(self.pose.x, self.pose.y)
        path = self.planner.plan_path(self.current_cell, self.target_stop_cell)

        if not path:
            self.get_logger().error("Path planning failed")
            self.sm.transition(States.SAFE_STOP)
            return

        self.sm.set_path(path)
        self.docking.reset_alignment()
        self.obstacle_handler.reset()
        self.controller.reset_rate_limiter()
        self.sm.transition(States.NAVIGATE)

    def _desired_axis_from_segment(self, a: Tuple[int, int], b: Tuple[int, int]) -> str:
        dx = abs(b[0] - a[0])
        dy = abs(b[1] - a[1])
        return "x" if dx >= dy else "y"

    def _maybe_event_cell_update(self) -> None:
        if self.accumulated_distance < self.cfg.cell_size_m:
            return

        wp = self.sm.current_waypoint()
        heading_ok = False
        if wp is not None:
            desired = math.atan2(
                (wp[1] * self.cfg.cell_size_m + 0.5 * self.cfg.cell_size_m) - self.pose.y,
                (wp[0] * self.cfg.cell_size_m + 0.5 * self.cfg.cell_size_m) - self.pose.x,
            )
            heading_err = abs(math.degrees(wrap_angle(desired - self.pose.yaw)))
            heading_ok = heading_err <= self.cfg.heading_cell_update_tol_deg

        vision_ok = self.latest_vision.confidence >= self.cfg.conf_high
        if heading_ok or vision_ok:
            self.current_cell = self.planner.world_to_cell(self.pose.x, self.pose.y)
            self.accumulated_distance = 0.0

    def _waypoint_world(self) -> Optional[Tuple[float, float]]:
        wp = self.sm.current_waypoint()
        if wp is None:
            return None
        return self.planner.to_world(wp)

    def _handle_navigation(self, now: float, dt: float) -> None:
        wp_cell = self.sm.current_waypoint()
        wp_world = self._waypoint_world()
        if wp_cell is None or wp_world is None:
            self.sm.transition(States.STOP_ALIGN)
            self.publish_cmd(0.0, 0.0)
            return

        prev_cell = self.sm.path[max(0, self.sm.waypoint_idx - 1)]
        desired_axis = self._desired_axis_from_segment(prev_cell, wp_cell)
        self.sm.update_axis_hysteresis(
            desired_axis=desired_axis,
            confidence_ok=self.latest_vision.confidence >= self.cfg.axis_switch_conf_threshold,
            frames_required=self.cfg.axis_switch_frames,
        )

        dist_to_wp = math.hypot(wp_world[0] - self.pose.x, wp_world[1] - self.pose.y)
        if dist_to_wp <= self.cfg.waypoint_reached_m:
            if self.sm.is_last_waypoint():
                if self.mission_mode == "EXIT":
                    self.sm.transition(States.DONE)
                else:
                    self.sm.transition(States.STOP_ALIGN)
                self.publish_cmd(0.0, 0.0)
                return
            self.sm.advance_waypoint()
            wp_world = self._waypoint_world()
            if wp_world is None:
                self.sm.transition(States.STOP_ALIGN)
                self.publish_cmd(0.0, 0.0)
                return

        lost_for = now - self.last_vision_good_time
        if self.latest_vision.confidence < self.cfg.conf_low:
            if lost_for > self.cfg.lost_long_s:
                self._enter_lost_recovery()
                self.publish_cmd(0.0, 0.0)
                return
            if lost_for > self.cfg.lost_short_s:
                self.sm.transition(States.DEGRADED_NAV)

        if self.latest_vision.confidence >= self.cfg.conf_high and self.sm.state == States.DEGRADED_NAV:
            self.sm.transition(States.NAVIGATE)

        obstacle_detected = self.manual_obstacle_flag or self.latest_lidar.front_blocked
        obstacle_action = self.obstacle_handler.update(obstacle_detected)
        if obstacle_action == ObstacleAction.SAFE_STOP:
            self.sm.transition(States.SAFE_STOP)
            self.publish_cmd(0.0, 0.0)
            return
        if obstacle_action in (
            ObstacleAction.RIGHT_DETOUR,
            ObstacleAction.LEFT_DETOUR,
            ObstacleAction.BACKWARD_RECOVERY,
        ):
            if not self._replan_for_obstacle(obstacle_action):
                self.sm.transition(States.SAFE_STOP)
                self.publish_cmd(0.0, 0.0)
                return

        v = self.cfg.v_nominal_mps if self.sm.state == States.NAVIGATE else self.cfg.v_degraded_mps
        if obstacle_action == ObstacleAction.SLOW:
            v *= 0.6

        omega = self.controller.compute_omega(
            pose=self.pose,
            target_xy=wp_world,
            v=v,
            meas=self.latest_vision,
            dt=dt,
            use_blue_only=False,
        )

        self._maybe_event_cell_update()
        self.publish_cmd(v, omega)

    def _enter_lost_recovery(self) -> None:
        self.sm.transition(States.LOST_RECOVERY)
        self.recovery_base_yaw = self.pose.yaw
        self.recovery_scan_dir = 1.0
        self.recovery_retries = 0

    def _handle_lost_recovery(self, now: float) -> None:
        if self.latest_vision.confidence >= self.cfg.conf_high:
            self.sm.transition(States.DEGRADED_NAV)
            return

        max_offset = math.radians(self.cfg.lost_scan_angle_deg)
        yaw_offset = wrap_angle(self.pose.yaw - self.recovery_base_yaw)

        if abs(yaw_offset) > max_offset:
            self.recovery_scan_dir *= -1.0
            self.recovery_retries += 1
            self.recovery_base_yaw = self.pose.yaw

        if self.recovery_retries > self.cfg.lost_recovery_retries:
            self.sm.transition(States.SAFE_STOP)
            self.publish_cmd(0.0, 0.0)
            return

        v_cmd = 0.0 if self.latest_lidar.front_blocked else self.cfg.v_creep_mps
        self.publish_cmd(v_cmd, self.recovery_scan_dir * self.cfg.lost_scan_omega)

    def _handle_stop_align(self, now: float, dt: float) -> None:
        if self.latest_vision.confidence < self.cfg.conf_low:
            self.publish_cmd(0.0, 0.0)
            return

        target_xy = (self.pose.x + math.cos(self.pose.yaw) * self.cfg.lookahead_m, self.pose.y + math.sin(self.pose.yaw) * self.cfg.lookahead_m)
        v_cmd, done = self.docking.update_stop_alignment(now, self.latest_vision)
        omega = self.controller.compute_omega(
            pose=self.pose,
            target_xy=target_xy,
            v=abs(v_cmd),
            meas=self.latest_vision,
            dt=dt,
            use_blue_only=True,
        )

        self.publish_cmd(v_cmd, omega)
        if done:
            self.docking.start_reverse(self.pose.x, self.pose.y, self.pose.yaw)
            self.sm.transition(States.REVERSE_DOCK)

    def _handle_reverse_dock(self) -> None:
        if self.latest_lidar.rear_blocked:
            self.sm.transition(States.SAFE_STOP)
            self.publish_cmd(0.0, 0.0)
            return

        status = self.docking.update_reverse(self.pose.x, self.pose.y, self.pose.yaw)
        if status.abort:
            self.sm.transition(States.STOP_ALIGN)
            self.publish_cmd(0.0, 0.0)
            return

        self.publish_cmd(status.v_cmd, status.w_cmd)
        if status.done:
            self.sm.transition(States.DONE)
            self.publish_cmd(0.0, 0.0)

    def _neighbor_order_by_action(self, action: str) -> List[Tuple[int, int]]:
        yaw = self.pose.yaw
        heading_idx = int(round(((yaw + math.pi) / (math.pi / 2.0)))) % 4
        dirs = [(1, 0), (0, 1), (-1, 0), (0, -1)]

        forward = dirs[heading_idx]
        right = dirs[(heading_idx - 1) % 4]
        left = dirs[(heading_idx + 1) % 4]
        back = dirs[(heading_idx + 2) % 4]

        if action == ObstacleAction.RIGHT_DETOUR:
            return [right, forward, left]
        if action == ObstacleAction.LEFT_DETOUR:
            return [left, forward, right]
        return [back, left, right]

    def _replan_for_obstacle(self, action: str) -> bool:
        if self.target_stop_cell is None:
            return False

        self.current_cell = self.planner.world_to_cell(self.pose.x, self.pose.y)
        for dxy in self._neighbor_order_by_action(action):
            inter = (self.current_cell[0] + dxy[0], self.current_cell[1] + dxy[1])
            if not self.planner.is_free(inter):
                continue
            p1 = self.planner.plan_path(self.current_cell, inter)
            p2 = self.planner.plan_path(inter, self.target_stop_cell)
            if p1 and p2:
                merged = p1[:-1] + p2
                self.sm.set_path(merged)
                self.sm.transition(States.NAVIGATE)
                return True

        fallback = self.planner.plan_path(self.current_cell, self.target_stop_cell)
        if fallback:
            self.sm.set_path(fallback)
            self.sm.transition(States.NAVIGATE)
            return True
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PinkyAutoParkNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
