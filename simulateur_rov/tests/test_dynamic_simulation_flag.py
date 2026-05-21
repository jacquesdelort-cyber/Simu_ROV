"""Tests du paramètre dynamic_simulation (mode quasi-statique câble)."""
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
    return ROVSystem(params, N_segments=n_segments)


def test_quasi_static_mode_skips_dynamic_solver(monkeypatch):
    system = _make_system()
    system.dynamic_simulation = False
    system.use_current_geometry_for_static = False
    y0 = get_initial_state(
        system,
        x_rov=0.0,
        y_rov=-10.0,
        x_boat=0.0,
        L=50.0,
        use_current_geometry=False,
    )
    x_cable, y_cable = system.unpack_state(y0)[6:8]
    system.x_cable_prev = np.array(x_cable, copy=True)
    system.y_cable_prev = np.array(y_cable, copy=True)

    dynamic_called = {"value": False}
    original_dynamic = system.cable.solver.solve_equilibrium_dynamic

    def _spy_dynamic(*args, **kwargs):
        dynamic_called["value"] = True
        return original_dynamic(*args, **kwargs)

    monkeypatch.setattr(
        system.cable.solver,
        "solve_equilibrium_dynamic",
        _spy_dynamic,
    )

    u = {"Fx_rov": 0.0, "Fy_rov": 0.0, "vx_boat_cmd": 0.0, "dL_dt": 0.0}
    dydt = system.compute_derivatives(0.1, y0, u)
    assert np.all(np.isfinite(dydt))
    assert dynamic_called["value"] is False


def test_dynamic_mode_uses_dynamic_solver_when_prev_set(monkeypatch):
    system = _make_system()
    system.dynamic_simulation = True
    y0 = get_initial_state(
        system,
        x_rov=0.0,
        y_rov=-10.0,
        x_boat=0.0,
        L=50.0,
        use_current_geometry=False,
    )
    x_cable, y_cable = system.unpack_state(y0)[6:8]
    system.x_cable_prev = np.array(x_cable, copy=True)
    system.y_cable_prev = np.array(y_cable, copy=True)

    dynamic_called = {"value": False}
    original_dynamic = system.cable.solver.solve_equilibrium_dynamic

    def _spy_dynamic(*args, **kwargs):
        dynamic_called["value"] = True
        return original_dynamic(*args, **kwargs)

    monkeypatch.setattr(
        system.cable.solver,
        "solve_equilibrium_dynamic",
        _spy_dynamic,
    )

    u = {"Fx_rov": 0.0, "Fy_rov": 0.0, "vx_boat_cmd": 0.0, "dL_dt": 0.0}
    system.compute_derivatives(0.1, y0, u)
    assert dynamic_called["value"] is True
