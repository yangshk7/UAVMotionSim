import numpy as np

from uavsim.frames.quaternion import to_matrix
from uavsim.frames.rotation import euler_to_matrix
from uavsim.vehicle.control_allocator import ControlAllocator

from .attitude_controller import AttitudeController
from .position_controller import PositionController
from .rate_controller import RateController
from .velocity_controller import VelocityController, desired_attitude


class CascadeController:
    """通过整数积分步调度；所有输出零阶保持。只读取真实状态。"""

    def __init__(self, config, vehicle):
        c = config.controller
        self.config = config
        self.vehicle = vehicle
        rate = config.simulation.dynamics_rate_hz
        self.strides = [
            rate // getattr(c, name + "_rate_hz") for name in ["position", "velocity", "attitude", "angular"]
        ]
        self.position = PositionController(c)
        self.velocity = VelocityController(c, vehicle.config.gravity_mps2)
        self.attitude = AttitudeController(c)
        self.rate = RateController(c, vehicle.inertia)
        self.allocator = ControlAllocator(vehicle, c.allocation_weights)
        self.velocity_command = np.zeros(3)
        self.acceleration_command = np.zeros(3)
        self.rate_command = np.zeros(3)
        self.motor_command = np.full(6, vehicle.hover_speed_radps)
        self.residual = np.zeros(4)
        self.desired_rotation = np.eye(3)
        self.total_thrust = vehicle.config.mass_kg * vehicle.config.gravity_mps2
        self.update_counts = np.zeros(4, dtype=int)

    def update(self, step, state, reference, attitude_offset_rad=None, rate_offset_radps=None):
        cfg = self.config
        c = cfg.controller
        rotation = to_matrix(state[6:10])
        saturated = np.linalg.norm(self.residual) > 1e-7
        if step % self.strides[0] == 0:
            self.velocity_command = self.position.update(
                reference["position_N_m"], reference["velocity_N_mps"], state[:3]
            )
            self.update_counts[0] += 1
        if step % self.strides[1] == 0:
            self.acceleration_command = self.velocity.update(
                self.velocity_command,
                state[3:6],
                reference["acceleration_N_mps2"],
                1 / c.velocity_rate_hz,
                saturated,
            )
            self.update_counts[1] += 1
        if step % self.strides[2] == 0:
            desired, force = desired_attitude(
                self.acceleration_command, reference["yaw_rad"], cfg.vehicle.mass_kg, cfg.vehicle.gravity_mps2
            )
            offset_rotation = np.eye(3)
            if attitude_offset_rad is not None:
                offset_rotation = euler_to_matrix(attitude_offset_rad)
                desired = desired @ offset_rotation
            self.desired_rotation = desired
            self.total_thrust = max(0.0, -float(force @ rotation[:, 2]))
            desired_rate = offset_rotation.T @ np.array([0.0, 0.0, reference["yaw_rate_radps"]])
            if rate_offset_radps is not None:
                desired_rate += rate_offset_radps
            self.rate_command = self.attitude.update(rotation, desired, desired_rate)
            self.update_counts[2] += 1
        if step % self.strides[3] == 0:
            torque = self.rate.update(self.rate_command, state[10:13], 1 / c.angular_rate_hz, saturated)
            thrust, self.residual = self.allocator.allocate(np.r_[self.total_thrust, torque])
            self.motor_command = np.sqrt(thrust / cfg.vehicle.kf)
            self.update_counts[3] += 1
        return self.motor_command
