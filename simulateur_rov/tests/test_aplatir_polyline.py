import numpy as np

from src.solvers.cable_solver import CableSolver
from src.models.environment import Environment


def _make_solver():
    params = {"d": 0.01, "rho_cable": 1500.0}
    env = Environment({"rho_eau": 1025.0, "g": 9.81})
    return CableSolver(N_segments=10, params=params, environment=env)


def _projection_orthogonale(Q: np.ndarray) -> np.ndarray:
    """Projection orthogonale de tous les points de Q sur la droite Q[0]Q[-1]."""
    Q = np.asarray(Q, dtype=float)
    q0 = Q[0]
    qn = Q[-1]
    d = qn - q0
    den = float(np.dot(d, d))
    if den <= 1e-18:
        return np.broadcast_to(q0, Q.shape)
    t_proj = np.dot(Q - q0, d) / den
    return q0 + t_proj[:, None] * d


def test_aplatir_polyline_k_zero_returns_projection():
    solver = _make_solver()
    Q = np.array([[0.0, 0.0], [1.0, 2.0], [2.0, 1.0]], dtype=float)
    k = 0.0
    T, expl = solver.aplatir_polyline(Q, k)
    assert expl == ""
    assert T is not None
    Q_proj = _projection_orthogonale(Q)
    assert np.allclose(T, Q_proj, rtol=1e-8, atol=1e-8)


def test_aplatir_polyline_k_one_returns_Q():
    solver = _make_solver()
    Q = np.array([[0.0, 0.0], [1.0, 2.0], [2.0, 1.0]], dtype=float)
    k = 1.0
    T, expl = solver.aplatir_polyline(Q, k)
    assert expl == ""
    assert T is not None
    assert np.allclose(T, Q, rtol=1e-12, atol=1e-12)


def test_aplatir_polyline_degenerate_line():
    solver = _make_solver()
    Q = np.array([[3.0, -1.0], [1.0, 0.0], [3.0, -1.0]], dtype=float)  # Q[0] == Q[-1]
    k = 1.5
    q0 = Q[0]
    T_expected = q0 + k * (Q - q0)
    T, expl = solver.aplatir_polyline(Q, k)
    assert expl == ""
    assert T is not None
    assert np.allclose(T, T_expected, rtol=1e-8, atol=1e-8)


def test_aplatir_polyline_does_not_mutate_Q():
    solver = _make_solver()
    Q = np.array([[0.0, 0.0], [1.0, 2.0], [2.0, 1.0]], dtype=float)
    Q_copy = Q.copy()
    T, expl = solver.aplatir_polyline(Q, 0.7)
    assert expl == ""
    assert T is not None
    assert np.array_equal(Q, Q_copy)


def test_aplatir_polyline_invalid_shape_returns_none():
    solver = _make_solver()
    Q_bad = np.array([[0.0, 0.0]], dtype=float)
    T, expl = solver.aplatir_polyline(Q_bad, 1.0)
    assert expl == ""
    assert T is None
