import numpy as np
import pytest

from src.utils.utils import deplacer_point


def _dist(P, Q):
    P = np.asarray(P, dtype=float)
    Q = np.asarray(Q, dtype=float)
    return float(np.linalg.norm(Q - P))


def test_deplacer_point_aligned_returns_midpoint():
    # A, B, C alignés sur l'axe x
    A = np.array([0.0, 0.0])
    B = np.array([1.0, 0.0])
    C = np.array([3.0, 0.0])
    D = deplacer_point(A, B, C)
    expected = 0.5 * (A + C)
    assert np.allclose(D, expected)


def test_deplacer_point_degenerate_A_equals_B():
    # Cas dégénéré A == B : on prend le milieu de A et C
    A = np.array([1.0, -1.0])
    B = A.copy()
    C = np.array([3.0, -5.0])
    D = deplacer_point(A, B, C)
    expected = 0.5 * (A + C)
    assert np.allclose(D, expected)


def test_deplacer_point_non_aligned_mediatrice_and_sum_constraint():
    # Triangle quelconque non aligné
    A = np.array([0.0, 0.0])
    B = np.array([2.0, 0.0])
    C = np.array([1.0, 2.0])

    D = deplacer_point(A, B, C)

    # D doit être sur la médiatrice de AC : AD == CD
    AD = _dist(A, D)
    CD = _dist(C, D)
    assert AD == pytest.approx(CD, rel=1e-6, abs=1e-8)

    # Et AD + DC = AB + BC
    AB = _dist(A, B)
    BC = _dist(B, C)
    lhs = AD + CD
    rhs = AB + BC
    assert lhs == pytest.approx(rhs, rel=1e-6, abs=1e-8)


def test_deplacer_point_side_towards_C():
    # Vérifie que D est bien du côté de C par rapport à la médiatrice.
    A = np.array([0.0, 0.0])
    B = np.array([2.0, 0.0])
    C = np.array([1.0, 3.0])

    D = deplacer_point(A, B, C)

    # Médiatrice de AC : on utilise AC comme direction de base
    M = 0.5 * (A + C)
    AC = C - A
    # vecteur normal à AC
    n = np.array([-AC[1], AC[0]], dtype=float)
    n /= np.linalg.norm(n)

    side_B = np.dot(B - M, n)
    side_D = np.dot(D - M, n)

    # B et D doivent être du même côté de la médiatrice (produit > 0)
    assert side_B * side_D > 0

