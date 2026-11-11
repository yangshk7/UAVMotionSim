"""Shared command and capture conditions for the external baseline comparison."""

import json
from dataclasses import dataclass
from math import hypot, isfinite, radians


@dataclass(frozen=True)
class AlignedRoute:
    altitude_m: float = 300.0
    reposition_x_m: float = -150.0
    start_x_m: float = -102.4
    duration_s: float = 20.48
    speed_mps: float = 10.0
    position_tolerance_m: float = 0.5
    capture_x_tolerance_m: float = 2.0
    velocity_tolerance_mps: float = 0.5
    settle_s: float = 1.0
    command_period_s: float = 0.1
    cross_track_gain_per_s: float = 0.5
    correction_limit_mps: float = 1.0
    heading_tolerance_deg: float = 3.0
    yaw_rate_limit_deg_s: float = 10.0
    center_y_m: float = 0.0

    @classmethod
    def from_file(cls, path):
        route = cls(**json.loads(path.read_text(encoding="utf-8")))
        if any(not isinstance(v, (int, float)) or isinstance(v, bool) or not isfinite(v)
               for v in vars(route).values()):
            raise ValueError("Route values must be finite numbers")
        if not (5 <= route.altitude_m <= 1000 and 0.5 <= route.speed_mps <= 12
                and 2 <= route.duration_s <= 3600 and route.reposition_x_m < route.start_x_m):
            raise ValueError("External route exceeds supported altitude/speed/duration limits")
        for key, value in vars(route).items():
            if key not in ("reposition_x_m", "start_x_m", "center_y_m") and value <= 0:
                raise ValueError(f"Route {key} must be positive")
        return route

    def settle_start(self, previous, clock, state, yaw):
        if not self.reposition_ready(*state) or not abs(yaw) <= radians(self.heading_tolerance_deg):
            return None
        return clock if previous is None else previous

    def reposition_ready(self, x, y, z, vx, vy, vz):
        return (abs(x - self.reposition_x_m) <= self.position_tolerance_m
                and abs(y - self.center_y_m) <= self.position_tolerance_m
                and abs(z + self.altitude_m) <= self.position_tolerance_m
                and hypot(vx, vy, vz) <= self.velocity_tolerance_mps)

    def capture_ready(self, x, y, z, vx, vy, vz, yaw):
        return (self.start_x_m <= x <= self.start_x_m + self.capture_x_tolerance_m
                and abs(y - self.center_y_m) <= self.position_tolerance_m
                and abs(z + self.altitude_m) <= self.position_tolerance_m
                and abs(vx - self.speed_mps) <= self.velocity_tolerance_mps
                and hypot(vy, vz) <= self.velocity_tolerance_mps
                and abs(yaw) <= radians(self.heading_tolerance_deg))

    def yaw_rate(self, yaw):
        limit = radians(self.yaw_rate_limit_deg_s)
        return max(-limit, min(limit, -yaw))

    def velocity_ned(self, y, z):
        def correction(error):
            limit = self.correction_limit_mps
            return max(-limit, min(limit, self.cross_track_gain_per_s * error))

        return self.speed_mps, correction(self.center_y_m-y), correction(-self.altitude_m - z)
