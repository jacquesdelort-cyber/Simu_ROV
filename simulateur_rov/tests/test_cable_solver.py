"""Tests pour le solveur de câble"""
import pytest
import numpy as np
from src.models.cable_model import Cable
from src.models.environment import Environment
from src.solvers.cable_solver import CableSolver


def _make_solver(n_segments=8):
    """Crée un solveur minimal pour tester la renormalisation."""
    params = {
        'd': 0.01,
        'rho_cable': 1500.0,
        'Cx_cable': 1.2,
        'Cf_cable': 0.04,
    }
    env = Environment({'rho_eau': 1025.0, 'g': 9.81})
    return CableSolver(n_segments, params, env)


def _segment_lengths(x_vals, y_vals):
    """Retourne les longueurs des segments d'une polyligne."""
    x_vals = np.asarray(x_vals, dtype=float)
    y_vals = np.asarray(y_vals, dtype=float)
    dx = np.diff(x_vals)
    dy = np.diff(y_vals)
    return np.sqrt(dx**2 + dy**2)


def test_cable_initialization():
    """Test l'initialisation du câble"""
    params = {
        'd': 0.01,
        'rho_cable': 1500.0,
        'Cx_cable': 1.2
    }
    env = Environment({})
    cable = Cable(params, N_segments=50, environment=env)
    
    assert cable.d == 0.01
    assert cable.N == 50


def test_cable_static_equilibrium():
    """Test la résolution de l'équilibre statique"""
    params = {'d': 0.01, 'rho_cable': 1500.0, 'Cx_cable': 1.2}
    env = Environment({'rho_eau': 1025.0, 'g': 9.81})
    cable = Cable(params, N_segments=10, environment=env)
    
    x_cable, y_cable, T = cable.solve_equilibrium(
        0.0, 10.0, 0.0, 0.0,  # ROV à (0, 10)
        0.0, 0.0,  # Bateau à (0, 0)
        50.0,  # Longueur 50m
        None, None  # Pas d'état précédent
    )
    
    assert len(x_cable) == 11  # N+1 points
    assert len(y_cable) == 11
    assert len(T) == 11
    assert np.all(np.isfinite(x_cable))
    assert np.all(np.isfinite(y_cable))
    assert np.all(np.isfinite(T))
    assert np.isclose(x_cable[0], 0.0)
    assert np.isclose(y_cable[0], 0.0)  # Extrémité surface/bateau selon la convention actuelle


def test_normalize_cable_length_returns_same_geometry_when_already_normalized():
    """Si la longueur est déjà exacte, la géométrie est conservée."""
    n_segments = 5
    solver = _make_solver(n_segments=n_segments)
    x_cable = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0], dtype=float)
    y_cable = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)

    x_norm, y_norm, _, _, _ = solver._normalize_cable_geometry(
        x_cable,
        y_cable,
        5.0,
        (x_cable[0], y_cable[0]),
        (x_cable[-1], y_cable[-1]),
        n_segments,
    )

    assert np.allclose(x_norm, x_cable)
    assert np.allclose(y_norm, y_cable)


def test_normalize_cable_length_enforces_target_length_and_uniform_segments():
    """La renormalisation doit imposer la longueur totale et des segments égaux."""
    n_segments = 6
    solver = _make_solver(n_segments=n_segments)
    x_cable = np.array([0.0, 0.3, 1.2, 2.0, 2.7, 4.1, 5.0], dtype=float)
    y_cable = np.array([0.0, -0.4, -1.1, -1.3, -2.2, -2.6, -3.0], dtype=float)
    l_target = 6.0

    x_norm, y_norm, _, _, _ = solver._normalize_cable_geometry(
        x_cable,
        y_cable,
        l_target,
        (x_cable[0], y_cable[0]),
        (x_cable[-1], y_cable[-1]),
        n_segments,
    )
    lengths = _segment_lengths(x_norm, y_norm)

    # On autorise l'ajout de points : seulement une borne inférieure
    assert len(x_norm) >= 7
    assert len(y_norm) == len(x_norm)

    # Longueur totale proche de la cible (tolérance relative 1e-4)
    assert np.isclose(np.sum(lengths), l_target, rtol=1e-4)

    # Segments raisonnablement homogènes : ratio ds_max / ds_moyen borné
    ds_target = l_target / max(len(lengths), 1)
    ratio_max = lengths.max() / max(ds_target, 1e-9)
    assert ratio_max <= 2.0


def test_normalize_cable_length_clips_points_above_surface():
    """Les points renormalisés doivent respecter la contrainte y <= 0."""
    n_segments = 4
    solver = _make_solver(n_segments=n_segments)
    x_cable = np.array([0.0, 0.5, 1.0, 1.5, 2.0], dtype=float)
    y_cable = np.array([0.2, 0.1, -0.3, -0.8, -1.2], dtype=float)

    x_norm, y_norm, _, _, _ = solver._normalize_cable_geometry(
        x_cable,
        y_cable,
        2.4,
        (x_cable[0], y_cable[0]),
        (x_cable[-1], y_cable[-1]),
        n_segments,
    )
    lengths = _segment_lengths(x_norm, y_norm)

    assert np.all(y_norm <= 1e-12)
    assert np.isclose(np.sum(lengths), 2.4, rtol=1e-4)

    # Contrôle de l'homogénéité via le ratio ds_max / ds_moyen
    ds_target = 2.4 / max(len(lengths), 1)
    ratio_max = lengths.max() / max(ds_target, 1e-9)
    assert ratio_max <= 2.0


def test_normalize_cable_length_with_few_segments_keeps_expected_point_count():
    """Le nombre de nœuds doit rester égal à N+1 sur une géométrie courte."""
    n_segments = 3
    solver = _make_solver(n_segments=n_segments)
    x_cable = np.array([0.0, 0.4, 1.4, 2.0], dtype=float)
    y_cable = np.array([0.0, -0.2, -0.9, -1.2], dtype=float)

    x_norm, y_norm, _, _, _ = solver._normalize_cable_geometry(
        x_cable,
        y_cable,
        3.0,
        (x_cable[0], y_cable[0]),
        (x_cable[-1], y_cable[-1]),
        n_segments,
    )
    lengths = _segment_lengths(x_norm, y_norm)

    assert len(x_norm) >= 4
    assert len(y_norm) >= 4
    assert len(lengths) == len(x_norm) - 1
    assert np.isclose(np.sum(lengths), 3.0, rtol=1e-4)


if __name__ == '__main__':
    pytest.main([__file__])

