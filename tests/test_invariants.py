import numpy as np
from numpy.testing import assert_allclose

from uavsim.config.validator import VehicleConfig
from uavsim.dynamics.equations import RigidBodyDynamics
from uavsim.dynamics.integrator import rk4_step
from uavsim.dynamics.state import initial_state
from uavsim.frames.quaternion import to_matrix
from uavsim.vehicle.hex_x12 import HexX12


def test_torque_free_energy_and_inertial_momentum():
    vehicle = HexX12(VehicleConfig())
    dynamics = RigidBodyDynamics(vehicle)
    state = initial_state(np.zeros(3), np.zeros(3), [1.0, 0.0, 0.0, 0.0], 0.0)
    state[10:13] = [1.0, 2.0, 3.0]
    momentum = vehicle.inertia @ state[10:13]
    energy = 0.5 * state[10:13] @ momentum
    z = np.zeros(3)
    for _ in range(200):
        state = rk4_step(dynamics, state, np.zeros(6), 0.001, (z,) * 3, (z,) * 3)
    final_momentum = to_matrix(state[6:10]) @ vehicle.inertia @ state[10:13]
    final_energy = 0.5 * state[10:13] @ vehicle.inertia @ state[10:13]
    assert_allclose(final_momentum, momentum, rtol=1e-10, atol=1e-10)
    assert_allclose(final_energy, energy, rtol=1e-10)
