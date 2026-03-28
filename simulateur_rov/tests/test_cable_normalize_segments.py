import numpy as np

from src.solvers.cable_solver import CableSolver


class _DummyEnv:
    pass


def _make_solver() -> CableSolver:
    params = {"d": 0.01, "rho_cable": 1500.0}
    return CableSolver(N_segments=10, params=params, environment=_DummyEnv())


def test_normalize_cable_segments_fallback_straight_line_when_too_short():
    solver = _make_solver()
    # Bateau et ROV très éloignés, L_target trop court
    bateau = (0.0, 0.0)
    rov = (10.0, -2.0)
    x = np.array([bateau[0], rov[0]])
    y = np.array([bateau[1], rov[1]])

    ok, x_new, y_new = solver._normalize_cable_segments(x, y, L_target=5.0, bateau=bateau, rov=rov, N_target=6)
    assert ok is False
    assert len(x_new) == 7
    assert len(y_new) == 7
    # extrémités recollées
    assert np.isclose(x_new[0], bateau[0])
    assert np.isclose(y_new[0], 0.0)
    assert np.isclose(x_new[-1], rov[0])
    assert np.isclose(y_new[-1], rov[1])


def test_normalize_cable_segments_invalid_target():
    solver = _make_solver()
    bateau = (0.0, 0.0)
    rov = (2.0, -1.0)
    x = np.array([0.0, 1.0, 2.0])
    y = np.array([0.0, -0.5, -1.0])

    ok, x_new, y_new = solver._normalize_cable_segments(x, y, L_target=3.0, bateau=bateau, rov=rov, N_target=1)
    assert ok is False
    assert np.allclose(x_new, x)
    assert np.allclose(y_new, y)


def test_normalize_cable_segments_general_shape_has_right_count_and_endpoints():
    solver = _make_solver()
    bateau = (0.0, 0.0)
    rov = (6.0, -2.0)
    # Polyline sous la surface (avec un peu de courbure)
    x = np.array([0.0, 1.5, 3.0, 4.2, 6.0])
    y = np.array([0.0, -0.4, -1.2, -1.7, -2.0])

    N_target = 6
    ok, x_new, y_new = solver._normalize_cable_segments(x, y, L_target=8.0, bateau=bateau, rov=rov, N_target=N_target)
    assert ok is True
    assert len(x_new) == N_target + 1
    assert len(y_new) == N_target + 1
    assert np.isclose(x_new[0], bateau[0])
    assert np.isclose(y_new[0], 0.0)
    assert np.isclose(x_new[-1], rov[0])
    assert np.isclose(y_new[-1], rov[1])
    assert np.all(y_new <= 1e-12)

    # Version pré-zigzag : on valide la structure, pas l'égalité stricte des longueurs.
