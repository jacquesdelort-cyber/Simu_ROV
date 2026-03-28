import numpy as np
import pytest

from src.utils.utils import create_point_with_target_length


def _dist(P, Q):
    P = np.asarray(P, dtype=float)
    Q = np.asarray(Q, dtype=float)
    return float(np.linalg.norm(Q - P))


def test_create_point_returns_false_when_ab_too_long():
    P = np.array([[0.0, 0.0], [5.0, -1.0]])
    ok, C = create_point_with_target_length(P, l_seg_target=2.0)
    H = 0.5 * (P[0, :2] + P[-1, :2])
    assert ok is False
    assert np.allclose(C, H)


def test_create_point_returns_midpoint_when_ab_equal_2l():
    P = np.array([[0.0, 0.0], [4.0, 0.0]])
    ok, C = create_point_with_target_length(P, l_seg_target=2.0)
    assert ok is True
    assert np.allclose(C, np.array([2.0, 0.0]))


def test_create_point_when_a_equals_b():
    P = np.array([[1.0, -2.0], [1.0, -2.0]])
    ok, C = create_point_with_target_length(P, l_seg_target=3.0)
    assert ok is True
    assert np.allclose(C, np.array([4.0, -2.0]))


def test_create_point_equal_target_distances_general_case():
    P = np.array(
        [
            [0.0, 0.0],
            [1.0, -1.0],
            [2.0, -2.0],
            [3.0, -1.5],
        ]
    )
    l = 2.5
    ok, C = create_point_with_target_length(P, l)
    A = np.array([0.0, 0.0])
    B = np.array([3.0, -1.5])
    assert ok is True
    assert _dist(A, C) == pytest.approx(l, rel=1e-8, abs=1e-8)
    assert _dist(B, C) == pytest.approx(l, rel=1e-8, abs=1e-8)


def test_create_point_clips_input_points_and_keeps_c_under_surface():
    # Les points > 0 sont clippés en interne; on vérifie surtout C_y <= 0.
    P = np.array(
        [
            [0.0, 1.0],
            [1.0, 2.0],
            [2.0, 1.5],
            [3.0, 0.2],
        ]
    )
    ok, C = create_point_with_target_length(P, 2.0)
    assert ok is True
    assert C[1] <= 1e-12

