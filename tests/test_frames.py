import numpy as np
from numpy.testing import assert_allclose

from uavsim.frames.coordinate import C, axial_to_sar, polar_to_sar, rotation_to_sar
from uavsim.frames.quaternion import derivative, from_matrix, normalize, to_matrix
from uavsim.frames.rotation import euler_to_matrix, skew
from uavsim.sar_adapter.antenna_geometry import antenna_geometry


def test_reflection_and_axial_rate():
    r = euler_to_matrix([0.3, -0.1, 0.5])
    w = np.array([0.2, 0.4, -0.3])
    rs = rotation_to_sar(r)
    assert_allclose(rs @ rs.T, np.eye(3), atol=1e-14)
    assert_allclose(np.linalg.det(rs), 1.0)
    assert_allclose(C @ (r @ skew(w)) @ C, rs @ skew(axial_to_sar(w)))
    assert_allclose(polar_to_sar([1, 2, 3]), [1, 2, -3])


def test_quaternion_kinematics():
    r = euler_to_matrix([0.4, 0.2, -0.7])
    q = from_matrix(r)
    assert_allclose(to_matrix(q), r, atol=1e-14)
    w = np.array([0.1, 0.2, 0.3])
    h = 1e-6
    numeric = (to_matrix(normalize(q + h * derivative(q, w))) - r) / h
    assert_allclose(numeric, r @ skew(w), atol=1e-7)


def test_antenna_geometry():
    lever = np.array([0.0, 0.1, 0.2])
    p = np.array([0.0, -0.1, -300.2])
    g = antenna_geometry(p, np.zeros(3), np.eye(3), [0, 0, 1.0], lever, 50.0)
    assert_allclose(g["apc_position_N_m"], [0, 0, -300])
    assert_allclose(g["apc_velocity_N_mps"], [-0.1, 0, 0])
    assert_allclose(g["beam_N_unit"], [0, np.cos(np.deg2rad(50)), np.sin(np.deg2rad(50))])
    r = euler_to_matrix([0.3, -0.2, 0.5])
    g = antenna_geometry(p, np.zeros(3), r, [0, 0, 0], lever, 50.0)
    assert_allclose(g["apc_position_N_m"] - p, r @ lever)
