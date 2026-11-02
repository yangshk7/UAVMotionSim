import numpy as np
from numpy.testing import assert_allclose

from uavsim.config.validator import Config
from uavsim.controllers.cascade import CascadeController
from uavsim.dynamics.equations import RigidBodyDynamics
from uavsim.dynamics.integrator import rk4_step
from uavsim.dynamics.state import initial_state
from uavsim.frames.quaternion import from_matrix
from uavsim.trajectory.reference_manager import build_reference
from uavsim.vehicle.hex_x12 import HexX12


def fly(config, initial_velocity=None, position_offset=None, duration=8.0):
    vehicle = HexX12(config.vehicle)
    dynamics = RigidBodyDynamics(vehicle)
    controller = CascadeController(config, vehicle)
    rate = config.simulation.dynamics_rate_hz
    time = np.arange(round(duration * rate) + 1) / rate
    ref = build_reference(config, time)
    state = initial_state(
        ref["position_N_m"][0],
        ref["velocity_N_mps"][0] if initial_velocity is None else initial_velocity,
        from_matrix(ref["rotation_NB"][0]),
        vehicle.hover_speed_radps,
    )
    if position_offset is not None:
        state[:3] += position_offset
    z = np.zeros(3)
    history = np.empty((len(time), 19))
    for i in range(len(time)):
        history[i] = state
        reference = {key: value[i] for key, value in ref.items()}
        command = controller.update(i, state, reference)
        if i < len(time) - 1:
            state = rk4_step(dynamics, state, command, 1 / rate, (z,) * 3, (z,) * 3)
    return history, ref, controller


def test_cruise_and_schedule():
    cfg = Config()
    states, ref, controller = fly(cfg, duration=0.2)
    assert_allclose(states[:, :3], ref["position_N_m"], atol=1e-9)
    assert_allclose(states[:, 3:6], ref["velocity_N_mps"], atol=1e-9)
    assert_allclose(controller.update_counts, [5, 11, 41, 41])


def test_speed_step_and_height_recovery():
    cfg = Config.model_validate({"simulation": {"dynamics_rate_hz": 1600}})
    states, ref, _ = fly(
        cfg, initial_velocity=np.array([9.0, 0.0, 0.0]), position_offset=[0.0, 0.0, 0.2], duration=10.0
    )
    assert np.linalg.norm(states[-1, 3:6] - ref["velocity_N_mps"][-1]) < 0.1
    assert abs(states[-1, 2] - ref["position_N_m"][-1, 2]) < 0.05
    assert np.max(np.abs(np.linalg.norm(states[:, 6:10], axis=1) - 1)) < 1e-14
    assert np.all(np.isfinite(states))
