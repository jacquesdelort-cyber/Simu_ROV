"""Tests pour le solveur de câble"""
import pytest
import numpy as np
from src.models.cable_model import Cable
from src.models.environment import Environment


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
    assert x_cable[0] == 0.0  # Au niveau du ROV
    assert y_cable[0] == 10.0
    assert y_cable[-1] == 0.0  # À la surface


if __name__ == '__main__':
    pytest.main([__file__])

