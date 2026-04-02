"""Tests unitaires pour CableSolver._normalize_cable_length (sans rapport graphique)."""

from __future__ import annotations

import numpy as np

from src.solvers.cable_solver import CableSolver, NCL_SOURCE_HISTORICAL_FALLBACK

from tests.cable_shared_cases import CASE_FALLBACK_LT_SHORT


class _DummyEnv:
    pass


def _make_solver(n_segments: int = 50) -> CableSolver:
    params = {"d": 0.01, "rho_cable": 1500.0}
    return CableSolver(N_segments=n_segments, params=params, environment=_DummyEnv())


def test_normalize_cable_length_fallback_straight_when_chord_exceeds_target():
    """Corde bateau–ROV > L_target : branche segments puis cohérence (souvent mode straight)."""
    solver = _make_solver()
    case = CASE_FALLBACK_LT_SHORT
    bateau = case.boat
    rov = case.rov
    x = np.asarray(case.x_cable, dtype=float)
    y = np.asarray(case.y_cable, dtype=float)

    x_norm, y_norm, _straight_mode, ncl_src = solver._normalize_cable_length(
        x,
        y,
        case.l_target,
        x_boat=bateau[0],
        y_boat=bateau[1],
        x_rov=rov[0],
        y_rov=rov[1],
        mode_test=True,
    )
    assert ncl_src == NCL_SOURCE_HISTORICAL_FALLBACK
    x_norm = np.asarray(x_norm, dtype=float)
    y_norm = np.asarray(y_norm, dtype=float)
    assert x_norm.shape == y_norm.shape
    assert len(x_norm) >= 2
    assert np.all(np.isfinite(x_norm)) and np.all(np.isfinite(y_norm))
    assert np.all(y_norm <= 1e-9)
    assert np.isclose(x_norm[0], float(bateau[0]))
    assert np.isclose(y_norm[0], min(float(bateau[1]), 0.0))


def test_normalize_cable_length_preserves_length_order_of_magnitude():
    """Cas déjà proche d'une ligne : sortie de longueur finie et extrémités cohérentes."""
    solver = _make_solver()
    x = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0], dtype=float)
    y = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
    L_target = 5.0

    x_norm, y_norm, _straight_mode, ncl_src = solver._normalize_cable_length(
        x,
        y,
        L_target,
        x_boat=0.0,
        y_boat=0.0,
        x_rov=5.0,
        y_rov=0.0,
        mode_test=True,
    )
    assert ncl_src == NCL_SOURCE_HISTORICAL_FALLBACK
    x_norm = np.asarray(x_norm, dtype=float)
    y_norm = np.asarray(y_norm, dtype=float)
    seg = np.sqrt(np.diff(x_norm) ** 2 + np.diff(y_norm) ** 2)
    L_out = float(np.sum(seg))
    assert np.isclose(L_out, L_target, rtol=1e-3, atol=0.1)
    assert np.isclose(x_norm[0], 0.0)
    assert np.isclose(y_norm[0], 0.0)
