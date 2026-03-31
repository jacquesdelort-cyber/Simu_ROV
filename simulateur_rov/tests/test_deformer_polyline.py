import numpy as np

from src.solvers.cable_solver import CableSolver
from src.models.environment import Environment


def _make_solver():
    params = {"d": 0.01, "rho_cable": 1500.0}
    env = Environment({"rho_eau": 1025.0, "g": 9.81})
    return CableSolver(N_segments=10, params=params, environment=env)


def _polyline_length(points: np.ndarray) -> float:
    points = np.asarray(points, dtype=float)
    diffs = np.diff(points, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def test_deformer_polyline_returns_none_when_aligned():
    solver = _make_solver()
    Q = np.array([[0.0, 0.0], [0.5, 0.0], [1.0, 0.0]], dtype=float)
    R, expl = solver.deformer_polyline(Q, L_target=1.0, atol=1e-8, rtol=1e-8)
    assert expl == ""
    assert R is None


def test_deformer_polyline_finds_k_for_target_length():
    # Q : point du milieu à y=1, endpoints sur y=0.
    # Droite de référence : segment Q[0]Q[-1] (axe x).
    # Pour k>=0, le point du milieu va sur y=k, donc
    # L(k) = 2*sqrt(0.5^2 + k^2).
    solver = _make_solver()
    Q = np.array([[0.0, 0.0], [0.5, 1.0], [1.0, 0.0]], dtype=float)

    k_true = 2.0
    R_true, _ = solver.aplatir_polyline(Q, k_true)
    assert R_true is not None
    L_target = _polyline_length(R_true)

    R, expl = solver.deformer_polyline(Q, L_target=L_target, atol=1e-9, rtol=1e-9)
    assert expl == ""
    assert R is not None
    assert np.isclose(_polyline_length(R), L_target, rtol=1e-6, atol=1e-6)


def test_deformer_polyline_returns_none_when_impossible():
    solver = _make_solver()
    Q = np.array([[0.0, 0.0], [0.5, 1.0], [1.0, 0.0]], dtype=float)
    # k=0 donne une longueur minimale : 2*0.5 = 1.0
    R, expl = solver.deformer_polyline(Q, L_target=0.8, atol=1e-9, rtol=1e-9)
    assert expl == ""
    assert R is None


def test_deformer_polyline_allows_k_greater_than_1():
    # Même cas que test_deformer_polyline_finds_k_for_target_length :
    # - Q endpoints sur l'axe x (y=0)
    # - le point milieu est verticalement projeté sur l'axe x
    # Ainsi, R[1,1] = k et length(R) = 2*sqrt(0.5^2 + k^2).
    solver = _make_solver()
    Q = np.array([[0.0, 0.0], [0.5, 1.0], [1.0, 0.0]], dtype=float)

    k_true = np.sqrt(2.0)  # ~1.414 > 1
    R_true, _ = solver.aplatir_polyline(Q, k_true)
    assert R_true is not None
    L_target = _polyline_length(R_true)

    R, expl = solver.deformer_polyline(Q, L_target=L_target, atol=1e-9, rtol=1e-9)
    assert expl == ""
    assert R is not None
    assert R[1, 1] > 1.0  # vérifie qu'on a bien trouvé un k > 1 dans ce cas
    assert np.isclose(_polyline_length(R), L_target, rtol=1e-6, atol=1e-6)


def test_deformer_polyline_does_not_mutate_Q():
    solver = _make_solver()
    Q = np.array([[0.0, 0.0], [0.5, 1.0], [1.0, 0.0]], dtype=float)
    Q_copy = Q.copy()
    R_true, _ = solver.aplatir_polyline(Q, 1.3)
    assert R_true is not None
    L_target = _polyline_length(R_true)
    solver.deformer_polyline(Q, L_target=L_target, atol=1e-9, rtol=1e-9)
    assert np.array_equal(Q, Q_copy)
