import numpy as np
import pytest

from src.utils.utils import enforce_cable_segments_nb


def _length(P: np.ndarray) -> float:
    P = np.asarray(P, dtype=float)
    return float(np.sum(np.linalg.norm(P[1:] - P[:-1], axis=1)))


def test_enforce_cable_segments_nb_simple_reduction():
    # Câble en arc avec 5 points -> 4 segments, cible 2 segments
    P = np.array(
        [
            [0.0, 0.0],
            [1.0, 1.0],
            [2.0, 1.0],
            [3.0, 1.0],
            [4.0, 0.0],
        ]
    )
    L_before = _length(P)

    P_new, ok = enforce_cable_segments_nb(P, N_target_seg=2)

    assert ok
    assert P_new.shape[0] - 1 == 2
    L_after = _length(P_new)
    assert L_after == pytest.approx(L_before, rel=1e-6, abs=1e-8)


def test_enforce_cable_segments_nb_no_change_when_already_target():
    P = np.array(
        [
            [0.0, 0.0],
            [2.0, 1.0],
            [4.0, 0.0],
        ]
    )
    L_before = _length(P)

    P_new, ok = enforce_cable_segments_nb(P, N_target_seg=2)

    assert ok
    assert P_new.shape == P.shape
    assert np.allclose(P_new, P)
    L_after = _length(P_new)
    assert L_after == pytest.approx(L_before, rel=1e-12, abs=1e-12)


def test_enforce_cable_segments_nb_cannot_reach_target_returns_false():
    # Trivial cas à 2 segments, cible 5 segments : impossible
    P = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])

    P_new, ok = enforce_cable_segments_nb(P, N_target_seg=5)

    assert not ok
    # ne doit pas modifier le câble de façon absurde
    assert P_new.shape == P.shape
    assert np.allclose(P_new, P)

