"""
Tests d'intégration temporelle (``ROVSystem.integrate`` : RK4 à géométrie figée).

Complètent les scripts racine ``test_horizontal_movement.py`` et
``test_vertical_movement.py``. Convention eau : ``y <= 0`` sous la surface.

Les pas de simulation sont **courts** et le maillage câble **réduit** (``N`` petit).
Chaque pas appelle le solveur de câble une fois, en fin de pas.
"""
from __future__ import annotations

import numpy as np
import pytest

import src.solvers.cable_solver as cable_solver_module
from src.models.system_model import ROVSystem
from src.utils.initial_conditions import get_initial_state
from src.utils.parameters import get_default_parameters


@pytest.fixture(autouse=True)
def _silence_trace_print(monkeypatch):
    monkeypatch.setattr(cable_solver_module, "trace_print", lambda *args, **kwargs: None)


def _make_system(n_segments: int = 4) -> ROVSystem:
    params = get_default_parameters()
    system = ROVSystem(params, N_segments=n_segments)
    # Moins d'étapes RK → moins d'appels au solveur câble (coûteux en dynamique)
    system.integrator.rtol = 1e-3
    system.integrator.atol = 1e-5
    return system


def _u_constant(**kwargs):
    base = {"Fx_rov": 0.0, "Fy_rov": 0.0, "vx_boat_cmd": 0.0, "dL_dt": 0.0}

    def u_func(_t):
        u = base.copy()
        u.update(kwargs)
        return u

    return u_func


def test_compute_derivatives_first_step_finite():
    """Un seul appel : branche statique (``x_cable_prev`` nul au premier pas)."""
    system = _make_system()
    y0 = get_initial_state(
        system,
        x_rov=0.0,
        y_rov=-10.0,
        x_boat=0.0,
        L=50.0,
        use_current_geometry=False,
    )
    system.x_cable_prev = None
    system.y_cable_prev = None
    dydt = system.compute_derivatives(0.0, y0, _u_constant()(0.0))
    assert np.all(np.isfinite(dydt))


def test_integrate_short_span_success_and_finite_state():
    system = _make_system()
    y0 = get_initial_state(
        system,
        x_rov=0.0,
        y_rov=-10.0,
        x_boat=0.0,
        L=50.0,
        use_current_geometry=False,
    )
    sol = system.integrate([0.0, 0.1], y0, _u_constant(), dt_max=0.1)
    assert sol.success, sol.message
    y_end = sol.sol(sol.t[-1])
    unpacked = system.unpack_state(y_end)
    for arr in (unpacked[6], unpacked[7], unpacked[8]):
        assert np.all(np.isfinite(arr))
    assert float(np.max(unpacked[7])) <= 1e-2


def test_constant_positive_Fx_moves_rov_toward_positive_x():
    system = _make_system()
    y0 = get_initial_state(
        system,
        x_rov=0.0,
        y_rov=-10.0,
        x_boat=0.0,
        L=50.0,
        use_current_geometry=False,
    )
    x0, _, _, _, _, _, _, _, _, _ = system.unpack_state(y0)
    sol = system.integrate([0.0, 0.28], y0, _u_constant(Fx_rov=220.0), dt_max=0.1)
    assert sol.success, sol.message
    xf, _, _, _, _, _, _, _, _, _ = system.unpack_state(sol.sol(sol.t[-1]))
    assert xf - float(x0) > 0.005


def test_constant_downward_Fy_increases_depth():
    """Fy négatif = vers le bas : le ROV s'enfonce (y diminue)."""
    system = _make_system()
    y0 = get_initial_state(
        system,
        x_rov=0.0,
        y_rov=-10.0,
        x_boat=0.0,
        L=50.0,
        use_current_geometry=False,
    )
    _, y0r, _, _, _, _, _, _, _, _ = system.unpack_state(y0)
    sol = system.integrate([0.0, 0.22], y0, _u_constant(Fy_rov=-2500.0), dt_max=0.1)
    assert sol.success, sol.message
    _, y1r, _, _, _, _, _, _, _, _ = system.unpack_state(sol.sol(sol.t[-1]))
    assert y1r - float(y0r) < -0.005


def test_constant_positive_dL_dt_increases_cable_length():
    system = _make_system()
    y0 = get_initial_state(
        system,
        x_rov=0.0,
        y_rov=-10.0,
        x_boat=0.0,
        L=48.0,
        use_current_geometry=False,
    )
    *_, L0 = system.unpack_state(y0)
    sol = system.integrate([0.0, 0.2], y0, _u_constant(dL_dt=0.22), dt_max=0.1)
    assert sol.success, sol.message
    *_, L1 = system.unpack_state(sol.sol(sol.t[-1]))
    assert float(L1) > float(L0) + 0.02
