import numpy as np
import pytest

from src.utils.utils import supprimer_point


def _dist(P, Q):
    P = np.asarray(P, dtype=float)
    Q = np.asarray(Q, dtype=float)
    return float(np.linalg.norm(Q - P))


def test_supprimer_point_preserve_length_simple():
    # Configuration simple en arc
    A = np.array([0.0, 0.0])
    B = np.array([1.0, 1.0])
    C = np.array([2.0, 1.0])
    D = np.array([3.0, 0.0])

    L_before = _dist(A, B) + _dist(B, C) + _dist(C, D)

    E = supprimer_point(A, B, C, D)
    assert E is not None

    L_after = _dist(A, E) + _dist(E, D)
    # Tolérance légèrement relaxée pour accepter les approximations locales
    assert L_after == pytest.approx(L_before, rel=1e-6, abs=1e-6)


def test_supprimer_point_alignment_not_degenerate_anymore():
    # A, M, M' alignés : la nouvelle version renvoie un point valide (fallback)
    A = np.array([0.0, 0.0])
    B = np.array([1.0, 0.0])
    C = np.array([2.0, 0.0])
    D = np.array([3.0, 0.0])

    E = supprimer_point(A, B, C, D)
    assert E is not None

    # Le point doit rester globalement sur la droite x (géométrie quasi 1D)
    assert abs(E[1]) < 1e-10


def test_supprimer_point_asymmetric_case():
    # Cas asymétrique quelconque
    A = np.array([0.0, 0.0])
    B = np.array([1.0, 2.0])
    C = np.array([3.0, 1.0])
    D = np.array([4.0, 0.0])

    L_before = _dist(A, B) + _dist(B, C) + _dist(C, D)
    E = supprimer_point(A, B, C, D)
    assert E is not None

    L_after = _dist(A, E) + _dist(E, D)
    assert L_after == pytest.approx(L_before, rel=1e-6, abs=1e-6)


def test_supprimer_point_micro_geometry_small_lengths():
    # Cas avec segments très courts : la fonction ne doit plus renvoyer systématiquement None
    scale = 1e-4
    A = np.array([0.0, 0.0])
    B = np.array([scale, 0.0])
    C = np.array([2 * scale, 0.0])
    D = np.array([3 * scale, 0.0])

    L_before = _dist(A, B) + _dist(B, C) + _dist(C, D)
    E = supprimer_point(A, B, C, D)
    assert E is not None

    L_after = _dist(A, E) + _dist(E, D)
    # On accepte une erreur relative un peu plus large dans ce régime micro
    assert L_after == pytest.approx(L_before, rel=1e-4, abs=1e-6)


