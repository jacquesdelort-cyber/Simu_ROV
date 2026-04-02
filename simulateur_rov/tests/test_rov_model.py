"""Tests pour le modèle ROV"""
import warnings

import pytest
import numpy as np
from src.models.rov_model import ROV
from src.models.environment import Environment


def test_rov_initialization():
    """Test l'initialisation du ROV"""
    params = {
        'm': 100.0,
        'a': 0.5,
        'b': 1.0,
        'h': 0.5,
        'Cx': 0.8,
        'Cy': 1.0
    }
    rov = ROV(params)
    
    assert rov.m == 100.0
    assert rov.V == 0.5 * 1.0 * 0.5
    assert rov.Sx == 0.5 * 0.5


def test_rov_drag_force():
    """Test le calcul de la force de traînée"""
    rov = ROV({'m': 100.0, 'a': 0.5, 'b': 1.0, 'h': 0.5, 'Cx': 0.8, 'Cy': 1.0})
    env = Environment({'rho_eau': 1025.0, 'g': 9.81})
    
    # Vitesse nulle -> pas de traînée
    Fx, Fy = rov.compute_drag_force(0.0, 0.0, 10.0, env)
    assert abs(Fx) < 1e-6
    assert abs(Fy) < 1e-6


def test_rov_drag_force_finite_at_extreme_velocity():
    """Vitesses énormes : pas d'overflow, forces finies ; traînée saturée à v_max_drag."""
    rov = ROV({'m': 100.0, 'a': 0.5, 'b': 1.0, 'h': 0.5, 'Cx': 0.8, 'Cy': 1.0})
    env = Environment({'rho_eau': 1025.0, 'g': 9.81})
    with warnings.catch_warnings(record=True) as recorded:
        warnings.simplefilter("always")
        Fx, Fy = rov.compute_drag_force(1e200, 1e200, -10.0, env)
    assert np.isfinite(Fx) and np.isfinite(Fy)
    overflow_msgs = [
        w for w in recorded if issubclass(w.category, RuntimeWarning) and "overflow" in str(w.message).lower()
    ]
    assert not overflow_msgs

    Fx_cap, Fy_cap = rov.compute_drag_force(rov.v_max_drag, rov.v_max_drag, -10.0, env)
    assert Fx == pytest.approx(Fx_cap)
    assert Fy == pytest.approx(Fy_cap)


def test_rov_buoyancy():
    """Test le calcul de la poussée d'Archimède"""
    rov = ROV({'m': 100.0, 'a': 0.5, 'b': 1.0, 'h': 0.5})
    env = Environment({'rho_eau': 1025.0, 'g': 9.81})
    
    F_buoyancy = rov.compute_buoyancy_force(env)
    assert F_buoyancy > 0  # Force vers le haut


if __name__ == '__main__':
    pytest.main([__file__])

