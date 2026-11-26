from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Sequence


def clamp(value: float, low: float, high: float) -> float:
    return low if value < low else (high if value > high else value)


@dataclass(slots=True)
class RobotPose:
    robot_id: int
    x_m: float
    y_m: float
    heading_rad: float
    speed_mps: float = 0.0
    battery_pct: float = 100.0
    source: str = "vision"


@dataclass(slots=True)
class MotionCommand:
    robot_id: int
    linear_mps: float
    angular_radps: float
    left_wheel_radps: float
    right_wheel_radps: float
    heading_target_rad: float
    cross_track_m: float
    lookahead_m: float
    brake_distance_m: float
    arrival: bool


@dataclass(slots=True)
class DriveConfig:
    wheel_base_m: float = 0.38
    wheel_radius_m: float = 0.065
    max_wheel_radps: float = 24.0
    max_linear_mps: float = 1.4
    max_angular_radps: float = 2.4
    accel_mps2: float = 0.9
    decel_mps2: float = 1.3
    lookahead_m: float = 0.55
    arrival_radius_m: float = 0.18
    heading_gain: float = 2.2
    max_lateral_accel_mps2: float = 1.1


class DifferentialDrive:
    def __init__(self, cfg: DriveConfig | None = None) -> None:
        self.cfg = cfg or DriveConfig()

    def wheels(self, linear_mps: float, angular_radps: float) -> tuple[float, float]:
        cfg = self.cfg
        half = 0.5 * cfg.wheel_base_m
        left = (linear_mps - angular_radps * half) / cfg.wheel_radius_m
        right = (linear_mps + angular_radps * half) / cfg.wheel_radius_m
        limit = cfg.max_wheel_radps
        return clamp(left, -limit, limit), clamp(right, -limit, limit)

    def rate_limit(
        self,
        target_linear: float,
        target_angular: float,
        current_linear: float,
        current_angular: float,
        dt_sec: float,
    ) -> tuple[float, float]:
        cfg = self.cfg
        dt = max(1e-4, float(dt_sec))
        dv = target_linear - current_linear
        dw = target_angular - current_angular
        max_dv = (cfg.accel_mps2 if dv > 0.0 else cfg.decel_mps2) * dt
        max_dw = (cfg.max_angular_radps * 2.0) * dt
        linear = current_linear + clamp(dv, -max_dv, max_dv)
        angular = current_angular + clamp(dw, -max_dw, max_dw)
        return clamp(linear, -cfg.max_linear_mps, cfg.max_linear_mps), clamp(
            angular, -cfg.max_angular_radps, cfg.max_angular_radps
        )


class PurePursuit:
    def __init__(self, cfg: DriveConfig | None = None) -> None:
        self.cfg = cfg or DriveConfig()
        self.drive = DifferentialDrive(self.cfg)
        self.linear = 0.0
        self.angular = 0.0

    def reset(self) -> None:
        self.linear = 0.0
        self.angular = 0.0

    def command(
        self,
        pose: RobotPose,
        path: Sequence[Sequence[int]],
        cell_size_m: float,
        dt_sec: float = 0.05,
        speed_limit_mps: float | None = None,
    ) -> MotionCommand:
        cfg = self.cfg
        if not path:
            self.reset()
            return MotionCommand(
                pose.robot_id, 0.0, 0.0, 0.0, 0.0, pose.heading_rad, 0.0, 0.0, 0.0, True
            )

        pts = [((float(p[0]) + 0.5) * cell_size_m, (float(p[1]) + 0.5) * cell_size_m) for p in path]
        dx = pts[-1][0] - pose.x_m
        dy = pts[-1][1] - pose.y_m
        final_dist = math.hypot(dx, dy)
        if final_dist <= cfg.arrival_radius_m:
            self.reset()
            return MotionCommand(
                pose.robot_id, 0.0, 0.0, 0.0, 0.0, pose.heading_rad, 0.0, 0.0, 0.0, True
            )

        lookahead = cfg.lookahead_m
        target = pts[-1]
        for px, py in pts:
            if math.hypot(px - pose.x_m, py - pose.y_m) >= lookahead:
                target = (px, py)
                break

        tx = target[0] - pose.x_m
        ty = target[1] - pose.y_m
        dist = max(1e-6, math.hypot(tx, ty))
        target_heading = math.atan2(ty, tx)
        angle_error = math.atan2(
            math.sin(target_heading - pose.heading_rad), math.cos(target_heading - pose.heading_rad)
        )
        cross_track = abs(math.sin(angle_error)) * dist

        curvature = min(4.0, 2.0 * cross_track / max(lookahead * lookahead, 1e-6))
        speed = cfg.max_linear_mps
        if curvature > 1e-4:
            speed = min(speed, math.sqrt(cfg.max_lateral_accel_mps2 / curvature))
        speed *= max(0.15, 1.0 - abs(angle_error) / math.pi)
        if speed_limit_mps is not None:
            speed = min(speed, max(0.0, float(speed_limit_mps)))

        angular = clamp(cfg.heading_gain * angle_error, -cfg.max_angular_radps, cfg.max_angular_radps)
        self.linear, self.angular = self.drive.rate_limit(
            speed, angular, self.linear, self.angular, dt_sec
        )
        left, right = self.drive.wheels(self.linear, self.angular)
        brake = self.linear * self.linear / (2.0 * max(0.1, cfg.decel_mps2))
        return MotionCommand(
            pose.robot_id,
            round(self.linear, 4),
            round(self.angular, 4),
            round(left, 4),
            round(right, 4),
            round(target_heading, 4),
            round(cross_track, 4),
            round(dist, 4),
            round(brake, 4),
            False,
        )


def motion_dict(command: MotionCommand) -> dict[str, Any]:
    return {
        "robot_id": command.robot_id,
        "linear_mps": command.linear_mps,
        "angular_radps": command.angular_radps,
        "left_wheel_radps": command.left_wheel_radps,
        "right_wheel_radps": command.right_wheel_radps,
        "heading_target_rad": command.heading_target_rad,
        "cross_track_m": command.cross_track_m,
        "lookahead_m": command.lookahead_m,
        "brake_distance_m": command.brake_distance_m,
        "arrival": command.arrival,
    }


def battery_drain(distance_m: float, turns: int, payload_kg: float) -> float:
    base = 0.42 * max(0.0, float(distance_m))
    turn_cost = 0.08 * max(0, int(turns))
    load_cost = 0.015 * max(0.0, float(payload_kg)) * max(0.0, float(distance_m))
    return round(base + turn_cost + load_cost, 3)


def pose_from_marker(
    robot_id: int,
    cell: Sequence[float],
    heading_deg: float,
    speed_mps: float = 0.0,
    battery_pct: float = 100.0,
) -> RobotPose:
    return RobotPose(
        int(robot_id),
        float(cell[0]) * 1.0,
        float(cell[1]) * 1.0,
        math.radians(float(heading_deg)),
        float(speed_mps),
        float(battery_pct),
        "aruco",
    )


def fleet_utilization(robot_speeds: list[float], max_speed_mps: float = 1.4) -> float:
    if not robot_speeds or max_speed_mps <= 0.0:
        return 0.0
    total = sum(clamp(v / max_speed_mps, 0.0, 1.0) for v in robot_speeds)
    return round(total / len(robot_speeds), 4)
