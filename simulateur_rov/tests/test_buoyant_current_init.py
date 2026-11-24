import numpy as np
import pytest

from src.models.environment import Environment
import src.solvers.cable_solver as cable_solver_module
from src.solvers.cable_solver import CableSolver, BUOYANT_MIN_Y_MARGIN_M
from src.solvers.forces import compute_cable_forces


@pytest.fixture(autouse=True)
def _silence_trace_print(monkeypatch):
    monkeypatch.setattr(cable_solver_module, "trace_print", lambda *args, **kwargs: None)


def _make_buoyant_solver(v_courant_raw: str, n_segments: int = 300) -> CableSolver:
    params = {
        "d": 0.001,
        "rho_cable": 950.0,
        "Cx_cable": 0.9,
        "Cf_cable": 0.04,
    }
    env = Environment({"rho_eau": 1030.0, "g": 9.81, "v_courant": v_courant_raw})
    env.v_courant_raw = v_courant_raw
    return CableSolver(n_segments, params, env)


def _linear_reference_x(y_vals, x_boat, y_boat, x_rov, y_rov):
    y_vals = np.asarray(y_vals, dtype=float)
    denom = float(y_rov - y_boat)
    if abs(denom) <= 1e-9:
        return np.full_like(y_vals, float(x_boat))
    t = (y_vals - float(y_boat)) / denom
    return float(x_boat) + t * (float(x_rov) - float(x_boat))


def test_positive_current_drags_vertical_cable_toward_positive_x():
    """Un courant mission positif pousse le câble vers les x positifs."""
    env = Environment({"rho_eau": 1030.0, "g": 9.81, "v_courant": "0.0:0.5"})
    env.v_courant_raw = "0.0:0.5"
    y = np.linspace(0.0, -20.0, 11)
    x = np.zeros_like(y)
    zeros = np.zeros_like(y)
    params = {"d": 0.001, "rho_cable": 950.0, "Cx_cable": 0.9, "Cf_cable": 0.04}
    fx, *_ = compute_cable_forces(x, y, zeros, zeros, env, params, 20.0)
    assert float(np.sum(fx)) > 0.5


def test_buoyant_zero_current_cable_stays_shallower_than_chord():
    """Câble moins dense que l'eau : courbure vers la surface (y plus grand que la corde)."""
    solver = _make_buoyant_solver("0.0", n_segments=180)
    x_rov, y_rov, x_boat, L = 30.0, -400.0, 0.0, 430.0
    x_cable, y_cable, _ = solver.solve_equilibrium_static_with_current(
        x_rov, y_rov, x_boat, L
    )
    dx = float(x_rov - x_boat)
    assert abs(dx) > 1e-6
    for i in range(1, len(y_cable) - 1):
        t = (float(x_cable[i]) - float(x_boat)) / dx
        y_chord = float(t) * float(y_rov)
        assert float(y_cable[i]) >= y_chord - 1e-2


def test_static_with_current_keeps_no_current_geometry_for_buoyant():
    solver = _make_buoyant_solver("0.0", n_segments=150)
    x_rov, y_rov, x_boat, L = 30.0, -400.0, 0.0, 430.0

    x_current, y_current, _ = solver.solve_equilibrium_static_with_current(
        x_rov, y_rov, x_boat, L
    )
    x_pure, y_pure, _ = solver._pure_catenary_equilibrium_buoyant_cable(
        x_rov, y_rov, x_boat, L
    )

    assert np.allclose(x_current, x_pure, atol=1e-9, rtol=0.0)
    assert np.allclose(y_current, y_pure, atol=1e-9, rtol=0.0)


def test_skew_buoyant_profile_varies_with_depth_layers():
    solver = _make_buoyant_solver("0.0:0.0 100:1.0 200:0.0 300:-1.0 400:0.0", n_segments=260)
    x_rov, y_rov, x_boat, L = 30.0, -400.0, 0.0, 430.0

    x0, y0, _ = solver._pure_catenary_equilibrium_buoyant_cable(x_rov, y_rov, x_boat, L)
    x1, y1 = solver._skew_buoyant_polyline_for_static_current(
        x0, y0, L, x_boat, 0.0, x_rov, y_rov
    )
    x_ref = _linear_reference_x(y1, x_boat, 0.0, x_rov, y_rov)
    x_off = np.asarray(x1, dtype=float) - x_ref
    y = np.asarray(y1, dtype=float)

    m_pos = (y <= -60.0) & (y >= -160.0)
    m_neg = (y <= -260.0) & (y >= -340.0)
    assert np.any(m_pos) and np.any(m_neg)
    off_pos = float(np.mean(x_off[m_pos]))
    off_neg = float(np.mean(x_off[m_neg]))
    # La chaînette flottante de base est symétrisée par rapport à la corde ; le sens relatif
    # des déports moyens entre bandes peut s'inverser. On exige seulement une signature distincte.
    assert abs(off_pos - off_neg) > 1e-2


def _submerged_x(x_cable, y_cable, y_rov, y_top=-20.0):
    """Abscisses de la partie immergée, hors du mou posé à la surface et de l'extrémité ROV."""
    x = np.asarray(x_cable, dtype=float)
    y = np.asarray(y_cable, dtype=float)
    mask = (y < y_top) & (y > float(y_rov) + 20.0)
    return x[mask]


def test_dynamic_buoyant_above_manhattan_bulges_with_positive_current():
    """L > Manhattan : le courant positif à mi-profondeur déporte la verticale vers +x."""
    x_rov, y_rov, x_boat, L = 43.0, -365.0, 0.0, 462.0
    prev_x = np.linspace(x_boat, x_rov, 81)
    prev_y = np.linspace(0.0, y_rov, 81)

    straight = _make_buoyant_solver("0.0", n_segments=80)
    x0, y0, _ = straight.solve_equilibrium_dynamic(
        x_rov, y_rov, 0.0, 0.0, x_boat, 0.0, L, prev_x, prev_y
    )
    x_sub0 = _submerged_x(x0, y0, y_rov)
    assert x_sub0.size > 10
    assert float(np.std(x_sub0)) < 1.0
    assert float(np.max(x_sub0) - np.min(x_sub0)) < 2.0

    curved = _make_buoyant_solver("0.0:0.0  200:1.0  400:0.0", n_segments=80)
    x1, y1, _ = curved.solve_equilibrium_dynamic(
        x_rov, y_rov, 0.0, 0.0, x_boat, 0.0, L, prev_x, prev_y
    )
    band = (np.asarray(y1) <= -150.0) & (np.asarray(y1) >= -250.0)
    assert np.any(band)
    assert float(np.mean(np.asarray(x1)[band])) > x_rov + 1.0
    assert float(np.std(_submerged_x(x1, y1, y_rov))) > 1.0


def test_static_with_current_avoids_points_below_rov_for_buoyant():
    solver = _make_buoyant_solver("0.0:0.0 100:1.0 200:0.0 300:-1.0 400:0.0", n_segments=260)
    x_rov, y_rov, x_boat, L = 30.0, -400.0, 0.0, 430.0

    _, y_cable, _ = solver.solve_equilibrium_static_with_current(x_rov, y_rov, x_boat, L)
    assert float(np.min(y_cable)) >= float(y_rov) - BUOYANT_MIN_Y_MARGIN_M - 1e-6
