import numpy as np
import pytest

from src.utils.utils import scale_slack


def _length_sum(A, B, C):
    """Somme des longueurs AB + BC."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    C = np.asarray(C, dtype=float)
    return np.linalg.norm(B - A) + np.linalg.norm(C - B)


def test_scale_slack_identity_sc_1():
    A = np.array([0.0, 0.0])
    B = np.array([1.0, 0.0])
    C = np.array([0.0, 1.0])
    Bp = scale_slack(A, B, C, sc=1.0)
    assert np.allclose(Bp, B)


def test_scale_slack_aligned_points_return_B():
    # A, B, C alignés sur l'axe x
    A = np.array([0.0, 0.0])
    B = np.array([1.0, 0.0])
    C = np.array([2.0, 0.0])
    Bp = scale_slack(A, B, C, sc=2.0)
    assert np.allclose(Bp, B)


def test_scale_slack_sc_greater_than_1_increases_sum():
    A = np.array([0.0, 0.0])
    B = np.array([1.0, 0.5])
    C = np.array([0.0, 1.0])
    sc = 1.5
    initial = _length_sum(A, B, C)
    Bp = scale_slack(A, B, C, sc=sc)
    new_sum = _length_sum(A, Bp, C)

    # Somme correctement mise à l'échelle
    assert new_sum == pytest.approx(sc * initial, rel=1e-6, abs=1e-8)

    # Colinéarité et même sens : vecteurs MB et MB' parallèles et produit scalaire >= 0
    M = 0.5 * (A + B)
    v_MB = B - M
    v_MBp = Bp - M
    cross = v_MB[0] * v_MBp[1] - v_MB[1] * v_MBp[0]
    dot = np.dot(v_MB, v_MBp)
    assert abs(cross) <= 1e-8
    assert dot >= -1e-8


def test_scale_slack_sc_less_than_1_decreases_sum():
    A = np.array([0.0, 0.0])
    B = np.array([1.0, 0.5])
    C = np.array([0.0, 1.0])
    sc = 0.7
    initial = _length_sum(A, B, C)
    Bp = scale_slack(A, B, C, sc=sc)
    new_sum = _length_sum(A, Bp, C)

    assert new_sum == pytest.approx(sc * initial, rel=1e-6, abs=1e-8)


def test_scale_slack_degenerate_AB_or_BC_returns_B():
    A = np.array([0.0, 0.0])
    B = np.array([1.0, 1.0])
    # Cas A == B
    Bp1 = scale_slack(A, A, B, sc=2.0)
    assert np.allclose(Bp1, A)

    # Cas B == C
    Bp2 = scale_slack(A, B, B, sc=2.0)
    assert np.allclose(Bp2, B)


def test_scale_slack_invalid_sc_raises():
    A = np.array([0.0, 0.0])
    B = np.array([1.0, 0.0])
    C = np.array([0.0, 1.0])
    with pytest.raises(ValueError):
        scale_slack(A, B, C, sc=0.0)

