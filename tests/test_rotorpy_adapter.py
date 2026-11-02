"""用独立六旋翼方程交叉核对 RotorPy 坐标与六旋翼旋翼力矩。"""

import numpy as np
from numpy.testing import assert_allclose
from rotorpy.vehicles.multirotor import Multirotor

from uavsim.config.loader import load_config
from uavsim.dynamics.equations import RigidBodyDynamics
from uavsim.dynamics.rotorpy_hex import RotorPyHexDynamics
from uavsim.vehicle.hex_x12 import HexX12


def test_rotorpy_hex_derivative_matches_independent_equations():
    cfg = load_config("configs/rotorpy_hex_x12.yaml")
    vehicle = HexX12(cfg.vehicle)
    expected = RigidBodyDynamics(vehicle)
    adapter = RotorPyHexDynamics(vehicle)
    assert isinstance(adapter.model, Multirotor)
    assert adapter.model.num_rotors == 6
    assert adapter.model.f_to_TM.shape == (4, 6)
    state = np.zeros(19)
    state[3:6] = [9.0, 0.4, -0.1]
    state[6:10] = [0.996, 0.04, -0.06, 0.05]
    state[6:10] /= np.linalg.norm(state[6:10])
    state[10:13] = [0.02, -0.03, 0.04]
    state[13:] = [400, 600, 700, 550, 650, 450]
    command = np.array([500, 520, 540, 560, 580, 600], dtype=float)
    force = np.array([1.0, -2.0, 3.0])
    torque = np.array([0.1, -0.2, 0.3])
    assert_allclose(
        adapter.derivative(state, command, force, torque),
        expected.derivative(state, command, force, torque),
        atol=1e-10,
        rtol=1e-12,
    )
