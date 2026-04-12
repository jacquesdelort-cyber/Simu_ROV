"""Refus du résultat segments + straight_mode si le saut ROV > 0,1 m (fallback)."""
import numpy as np
import pytest

import src.solvers.cable_solver as cable_solver
from src.models.system_model import ROVSystem


@pytest.fixture(autouse=True)
def _silence_trace_print(monkeypatch):
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


def test_normalize_cable_length_straight_large_snap_uses_fallback():
    """Corde bateau–ROV >> L : segments renverrait straight_mode avec un grand saut ROV → fallback."""
    N = 40
    system = ROVSystem(_minimal_params(), N_segments=N)
    sol = system.cable.solver
    xb, yb = 0.0, 0.0
    xr, yr = 80.0, -300.0
    L_t = 40.0
    n = N + 1
    t = np.linspace(0.0, 1.0, n)
    xc = xb + t * (xr - xb)
    yc = yb + t * (yr - yb)
    _xo, _yo, _sm, src = sol._normalize_cable_length(
        np.asarray(xc, dtype=float),
        np.asarray(yc, dtype=float),
        float(L_t),
        x_boat=float(xb),
        y_boat=float(yb),
        x_rov=float(xr),
        y_rov=float(yr),
        k_tail=10,
        t=1.0,
        mode_test=True,
    )
    assert src == cable_solver.NCL_SOURCE_HISTORICAL_FALLBACK
