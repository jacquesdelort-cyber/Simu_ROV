"""Les tensions d'extrémité ne doivent pas changer de signe avec la direction."""
import numpy as np
import pytest

import src.solvers.cable_solver as cable_solver_module
from src.models.environment import Environment
from src.solvers.cable_solver import CableSolver, _nonnegative_end_tensions


@pytest.fixture(autouse=True)
def _silence_trace_print(monkeypatch):
    monkeypatch.setattr(cable_solver_module, "trace_print", lambda *args, **kwargs: None)


def _ends(ub, ur, force):
    matrix = np.column_stack((-np.asarray(ub, dtype=float), np.asarray(ur, dtype=float)))
    load = -np.asarray(force, dtype=float)
    return _nonnegative_end_tensions(matrix, load)


def test_vertical_cable_weight_is_held_at_the_boat():
    tension = _ends([0.0, -1.0], [0.0, -1.0], [0.0, -8.8])
    assert tension[0] == pytest.approx(8.8, abs=0.3)
    assert tension[1] == pytest.approx(0.0, abs=0.3)
    assert np.all(tension >= 0.0)


def test_open_catenary_solution_is_kept():
    boat = np.array([0.50, -0.866])
    rov = np.array([-0.20, -0.980])
    matrix = np.column_stack((-boat, rov))
    expected = np.array([40.0, 25.0])
    tension = _nonnegative_end_tensions(matrix, matrix @ expected)
    assert tension == pytest.approx(expected, abs=1.0)


def test_nearly_parallel_tangents_do_not_flip_with_a_small_angle():
    """
    Le même chargement, avec une tangente de tête décalée de moins d'un degré,
    donnait une solution algébrique tantôt à -2000 N, tantôt à +1000 N.
    """
    rov = np.array([-0.18, -0.984])
    rov = rov / np.linalg.norm(rov)
    boat0 = np.array([-0.202, -0.979])
    norms = []
    for eps in np.linspace(-0.04, 0.04, 9):
        boat = boat0 + np.array([eps, 0.0])
        boat = boat / np.linalg.norm(boat)
        tension = _ends(boat, rov, [5.0, -10.0])
        assert np.all(tension >= -1e-9)
        norms.append(float(np.linalg.norm(tension)))
    assert max(norms) < 40.0
    assert max(norms) - min(norms) < 25.0


def test_slack_polyline_tension_stays_with_the_cable_weight():
    params = {
        "d": 0.01,
        "rho_cable": 1500.0,
        "Cx_cable": 1.2,
        "Cf_cable": 0.04,
    }
    env = Environment({"rho_eau": 1025.0, "g": 9.81, "current_profile": None})
    solver = CableSolver(8, params, env)
    weight = (1500.0 - 1025.0) * np.pi * (0.005 ** 2) * 9.81
    s = np.linspace(0.0, 1.0, 9)
    x = -3.3 * s + 0.6 * np.sin(np.pi * s)
    y = -19.5 * s
    previous = None
    for shift in (0.0, 0.02, -0.02, 0.05):
        x_try = x.copy()
        x_try[1] += shift
        tension = solver._compute_catenary_tensions(x_try, y, weight)
        assert np.all(tension >= -1e-8)
        assert float(np.max(tension)) < 80.0
        if previous is not None:
            assert abs(float(tension[0]) - previous) < 30.0
        previous = float(tension[0])
