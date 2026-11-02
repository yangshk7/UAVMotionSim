import numpy as np
import pytest
from numpy.testing import assert_allclose

from uavsim.config.validator import Config, VehicleConfig
from uavsim.dynamics.equations import RigidBodyDynamics
from uavsim.dynamics.integrator import rk4_step
from uavsim.dynamics.state import initial_state
from uavsim.vehicle.control_allocator import ControlAllocator
from uavsim.vehicle.hex_x12 import HexX12
from uavsim.vehicle.motor import motor_derivative


def test_hex_geometry_and_allocation():
    vehicle = HexX12(VehicleConfig())
    forces = np.full(6, 19.62)
    assert_allclose(vehicle.allocation_matrix @ forces, [117.72, 0, 0, 0], atol=1e-13)
    for i in range(6):
        physical = np.cross(vehicle.rotor_positions_b_m[i], [0, 0, -1])
        assert_allclose(vehicle.allocation_matrix[1:3, i], physical[:2])
    allocator = ControlAllocator(vehicle)
    for wrench in [[117.72, 1, -1, 0.3], [1000, 100, -100, 50]]:
        f, residual = allocator.allocate(wrench)
        assert np.all((f >= 0) & (f <= 42))
        if wrench[0] < 200:
            assert_allclose(residual, 0, atol=1e-12)


def test_hover_and_free_fall():
    vehicle = HexX12(VehicleConfig())
    model = RigidBodyDynamics(vehicle)
    zeros = np.zeros(3)
    hover = initial_state(np.zeros(3), np.zeros(3), [1, 0, 0, 0], vehicle.hover_speed_radps)
    assert_allclose(model.derivative(hover, hover[13:], zeros, zeros), 0, atol=1e-13)
    state = initial_state(np.zeros(3), np.zeros(3), [1, 0, 0, 0], 0.0)
    for _ in range(100):
        state = rk4_step(model, state, np.zeros(6), 0.001, (zeros,) * 3, (zeros,) * 3)
    assert_allclose(state[:6], [0, 0, 0.5 * 9.81 * 0.1**2, 0, 0, 9.81 * 0.1], atol=1e-13)


def test_motor_analytic_and_torque():
    vehicle = HexX12(VehicleConfig())
    model = RigidBodyDynamics(vehicle)
    state = initial_state(np.zeros(3), np.zeros(3), [1, 0, 0, 0], 0.0)
    z = np.zeros(3)
    torque = np.array([0, 0, 0.1])
    for _ in range(100):
        state = rk4_step(model, state, np.full(6, 100.0), 0.001, (z,) * 3, (torque,) * 3)
    assert_allclose(state[13:], 100 * (1 - np.exp(-1)), rtol=1e-9)
    assert_allclose(state[12], 0.01, atol=1e-13)
    assert_allclose(motor_derivative(np.zeros(6), np.full(6, 1e6), 0.1, 700), 7000.0)


@pytest.mark.parametrize(
    "data",
    [
        {"simulation": {"dynamics_rate_hz": 3100}},
        {"vehicle": {"mass_kg": -1}},
        {"vehicle": {"inertia_kg_m2": [[1, 1, 0], [0, 1, 0], [0, 0, 1]]}},
        {"trajectory": {"speed_mps": 20}},
        {"pass_seeds": [3, 3]},
        {"disturbances": {"translation": {"x": {"injection_mode": "dynamics"}}}},
    ],
)
def test_invalid_config(data):
    with pytest.raises(ValueError):
        Config.model_validate(data)
