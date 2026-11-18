"""Tests intégration contrainte (half-explicit) et projection d'état câble."""
import numpy as np
import pytest

import src.solvers.cable_solver as cable_solver
from src.models.state_projection import (
    compute_cable_constraint_metrics,
    project_cable_state_inplace,
)
from src.models.system_model import ROVSystem
from src.solvers.projected_integrator import integrate_span_with_projection


@pytest.fixture(autouse=True)
def _silence_trace_print_for_windows_encoding(monkeypatch):
    """Évite UnicodeEncodeError sur trace_print (émojis) avec stdout cp1252."""

    def _quiet(_level, *_args, **_kwargs):
        return None

    monkeypatch.setattr(cable_solver, "trace_print", _quiet)


def _minimal_params():
    return {
        "rov": {"m": 100.0, "a": 0.5, "b": 1.0, "h": 0.5, "Cx": 0.8, "Cy": 1.0},
        "cable": {
            "d": 0.01,
            "rho_cable": 1500.0,
            "Cx_cable": 1.2,
            "Cf_cable": 0.04,
            "tension_rupture": 50.0,
        },
        "boat": {"m": 10000.0, "drag_coefficient": 0.5},
        "environment": {"rho_eau": 1025.0, "g": 9.81, "mu": 1e-3},
    }


def test_compute_cable_constraint_metrics_basic():
    N = 5
    system = ROVSystem(_minimal_params(), N_segments=N)
    L = 30.0
    xb, xr = 0.0, 10.0
    yb, yr = 0.0, -15.0
    n = N + 1
    tparam = np.linspace(0.0, 1.0, n)
    x_cable = xb + tparam * (xr - xb)
    y_cable = yb + tparam * (yr - yb)
    scale = L / sum(
        float(np.hypot(x_cable[i + 1] - x_cable[i], y_cable[i + 1] - y_cable[i]))
        for i in range(n - 1)
    )
    x_cable = xb + (x_cable - xb) * scale
    y_cable = yb + (y_cable - yb) * scale
    T = np.ones(n) * 5.0
    y = system.pack_state(xr, yr, 0.0, 0.0, xb, 0.0, x_cable, y_cable, T, L)
    m = compute_cable_constraint_metrics(system, y)
    assert m["rel_L_mismatch"] < 0.02
    assert m["max_y_cable"] <= 1e-6


def test_project_cable_state_inplace_improves_mismatch():
    N = 5
    system = ROVSystem(_minimal_params(), N_segments=N)
    L = 25.0
    xb, xr = 0.0, 8.0
    yr = -12.0
    xc, yc, Tn = system.cable.solver.solve_equilibrium_static(
        xr, yr, xb, L, rov_m=system.rov.m, rov_vol=system.rov.V
    )
    y = system.pack_state(xr, yr, 0.0, 0.0, xb, 0.0, xc, yc, Tn, L)
    # Incohérence état : L scalaire ≠ somme des segments (sans déformer la polyline).
    y[-1] = L * 1.08
    before = compute_cable_constraint_metrics(system, y)
    assert before["rel_L_mismatch"] > 0.04
    ok = project_cable_state_inplace(system, y, t_sim=0.0, k_tail=10)
    assert ok
    after = compute_cable_constraint_metrics(system, y)
    assert after["rel_L_mismatch"] < 0.08
    assert after["max_y_cable"] <= 1e-4


def test_projection_keeps_rov_hold_on_a_straight_cable():
    """Câble droit : la projection ne doit pas réécrire la tension à la caténaire du fil."""
    rho = 1025.0
    mass = 30.0
    params = _minimal_params()
    params["rov"] = {
        "m": mass,
        "vol": mass / rho,
        "a": 0.4,
        "b": 0.8,
        "h": 0.3,
        "Cx": 0.0,
        "Cy": 0.0,
    }
    params["cable"] = {
        "d": 0.002,
        "rho_cable": rho,
        "Cx_cable": 1.0,
        "Cf_cable": 0.0,
        "tension_rupture": 500.0,
    }
    params["environment"]["rho_eau"] = rho
    system = ROVSystem(params, N_segments=4)
    depth = 50.0
    n = system.N + 1
    x_cable = np.zeros(n)
    y_cable = np.linspace(0.0, -depth, n)
    y = system.pack_state(
        0.0, -depth, 0.0, -0.50, 0.0, 0.0, x_cable, y_cable, np.zeros(n), depth
    )
    system._hold_fx = 0.0
    system._hold_fy = -40.0
    system._hold_dL_dt = 0.50
    assert project_cable_state_inplace(system, y, t_sim=64.1, k_tail=4)
    _xr, _yr, _vx, _vy, _xb, _vb, _xc, _yc, tension, _L = system.unpack_state(y)
    assert float(tension[0]) == pytest.approx(40.0, abs=1.0)
    assert float(tension[-1]) == pytest.approx(40.0, abs=1.0)

    # Mou large : le fil ne retient pas le ROV, la tension du fil seul reste.
    y_slack = system.pack_state(
        0.0, -depth, 0.0, -0.50, 0.0, 0.0, x_cable, y_cable, np.zeros(n), depth + 20.0
    )
    assert project_cable_state_inplace(system, y_slack, t_sim=64.1, k_tail=4)
    _xr, _yr, _vx, _vy, _xb, _vb, _xc, _yc, tension_slack, _L = system.unpack_state(y_slack)
    assert float(np.max(tension_slack)) < 5.0


def test_hold_uses_payout_speed_not_the_free_fall_speed():
    """À câble tendu, la tension équilibre les forces à v = dL/dt, pas à la vitesse libre."""
    rho = 1025.0
    mass = 30.0
    params = _minimal_params()
    params["rov"] = {
        "m": mass,
        "vol": mass / rho,
        "a": 0.4,
        "b": 0.8,
        "h": 0.3,
        "Cx": 0.0,
        "Cy": 1.0,
    }
    params["environment"]["rho_eau"] = rho
    system = ROVSystem(params, N_segments=4)
    system._hold_fx = 0.0
    system._hold_fy = -80.0
    system._hold_dL_dt = 0.5
    depth = 50.0
    tension = system.cable_hold_tension(0.0, -depth, 0.0, -2.0, 0.0, depth)
    # Traînée à -0,5 m/s : Cy * 0,5 * rho * (a*b) * v^2
    drag_up = 1.0 * 0.5 * rho * (0.4 * 0.8) * 0.5 * 0.5
    assert tension == pytest.approx(80.0 - drag_up, abs=1.0)
    vx, vy = system.taut_rov_velocity(0.0, -depth, 0.0, -2.0, 0.0, depth)
    assert vy == pytest.approx(-0.5)


def test_integrate_span_with_projection_runs():
    N = 4
    system = ROVSystem(_minimal_params(), N_segments=N)
    L = 40.0
    xb, xr = 0.0, 5.0
    yr = -20.0
    xc, yc, Tn = system.cable.solver.solve_equilibrium_static(
        xr, yr, xb, L, rov_m=system.rov.m, rov_vol=system.rov.V
    )
    system.x_cable_prev = np.asarray(xc, dtype=float).copy()
    system.y_cable_prev = np.asarray(yc, dtype=float).copy()
    y0 = system.pack_state(xr, yr, 0.0, 0.0, xb, 0.0, xc, yc, Tn, L)

    def u_func(_t):
        return {
            "Fx_rov": 0.0,
            "Fy_rov": 0.0,
            "vx_boat_cmd": 0.0,
            "dL_dt": 0.0,
        }

    y1 = integrate_span_with_projection(
        system, (0.0, 0.05), y0, u_func, n_substeps=2, project=True
    )
    assert np.all(np.isfinite(y1))
    assert y1.shape == y0.shape
    m = compute_cable_constraint_metrics(system, y1)
    assert m["rel_L_mismatch"] < 0.15


def test_frozen_derivatives_do_not_call_the_nonlinear_solver(monkeypatch):
    """Pendant un pas RK, le second membre ne relance pas le solveur de câble."""
    N = 4
    system = ROVSystem(_minimal_params(), N_segments=N)
    L = 40.0
    xb, xr, yr = 0.0, 5.0, -20.0
    xc, yc, tn = system.cable.solver.solve_equilibrium_static(
        xr, yr, xb, L, rov_m=system.rov.m, rov_vol=system.rov.V
    )
    y0 = system.pack_state(xr, yr, 0.0, 0.0, xb, 0.0, xc, yc, tn, L)
    system.x_cable_prev = np.asarray(xc, dtype=float).copy()
    system.y_cable_prev = np.asarray(yc, dtype=float).copy()

    def _boom(*_args, **_kwargs):
        raise AssertionError("solveur non linéaire appelé à géométrie figée")

    monkeypatch.setattr(system.cable, "solve_equilibrium", _boom)
    system._freeze_cable_geometry = True

    def u_func(_t):
        return {"Fx_rov": 0.0, "Fy_rov": 0.0, "vx_boat_cmd": 0.0, "dL_dt": 0.0}

    dydt = system.compute_derivatives(0.0, y0, u_func(0.0))
    assert np.all(np.isfinite(dydt))
    system._freeze_cable_geometry = False


def test_system_integrate_span_with_projection_api():
    N = 3
    system = ROVSystem(_minimal_params(), N_segments=N)
    L = 35.0
    xb, xr = 0.0, 4.0
    yr = -18.0
    xc, yc, Tn = system.cable.solver.solve_equilibrium_static(
        xr, yr, xb, L, rov_m=system.rov.m, rov_vol=system.rov.V
    )
    system.x_cable_prev = np.asarray(xc, dtype=float).copy()
    system.y_cable_prev = np.asarray(yc, dtype=float).copy()
    y0 = system.pack_state(xr, yr, 0.0, 0.0, xb, 0.0, xc, yc, Tn, L)

    def u_func(_t):
        return {"Fx_rov": 0.0, "Fy_rov": 0.0, "vx_boat_cmd": 0.0, "dL_dt": 0.0}

    y1 = system.integrate_span_with_projection(
        [0.0, 0.02], y0, u_func, dt_max=0.1, n_substeps=3
    )
    assert y1.shape == y0.shape
    r = system.compute_constraint_residuals(y1)
    assert "rel_L_mismatch" in r
