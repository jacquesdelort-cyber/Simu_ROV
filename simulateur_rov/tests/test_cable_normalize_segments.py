import numpy as np

from src.solvers.cable_solver import CableSolver

from tests.cable_shared_cases import CASE_FALLBACK_LT_SHORT


class _DummyEnv:
    pass


def _make_solver() -> CableSolver:
    params = {"d": 0.01, "rho_cable": 1500.0}
    return CableSolver(N_segments=10, params=params, environment=_DummyEnv())


def test_normalize_cable_segments_fallback_straight_line_when_too_short():
    solver = _make_solver()
    case = CASE_FALLBACK_LT_SHORT
    bateau = case.boat
    rov = case.rov
    x = np.asarray(case.x_cable, dtype=float)
    y = np.asarray(case.y_cable, dtype=float)

    ok, x_new, y_new, straight_mode, expl = solver._normalize_cable_segments(
        x, y, L_target=case.l_target, bateau=bateau, rov=rov, N_target=case.n_target
    )
    assert expl == ""
    assert straight_mode is True
    assert ok is True
    assert len(x_new) == case.n_target + 1
    assert len(y_new) == case.n_target + 1
    assert np.isclose(x_new[0], bateau[0])
    assert np.isclose(y_new[0], 0.0)
    # Corde bateau–ROV > L_target : extrémité câble = ROV ramené sur la droite à distance L_target
    bx, by = float(bateau[0]), float(bateau[1])
    chord = np.hypot(rov[0] - bx, rov[1] - by)
    u = np.array([(rov[0] - bx) / chord, (rov[1] - by) / chord])
    rov_adj = np.array([bx, by]) + case.l_target * u
    assert np.isclose(x_new[-1], rov_adj[0], rtol=1e-5)
    assert np.isclose(y_new[-1], rov_adj[1], rtol=1e-5)
    segs = np.sqrt(np.diff(x_new) ** 2 + np.diff(y_new) ** 2)
    assert np.isclose(float(segs.sum()), case.l_target, rtol=1e-5)


def test_normalize_cable_segments_invalid_target():
    solver = _make_solver()
    bateau = (0.0, 0.0)
    rov = (2.0, -1.0)
    x = np.array([0.0, 1.0, 2.0])
    y = np.array([0.0, -0.5, -1.0])

    ok, x_new, y_new, straight_mode, expl = solver._normalize_cable_segments(
        x, y, L_target=3.0, bateau=bateau, rov=rov, N_target=1
    )
    assert straight_mode is False
    assert "Un seul segment" in expl
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
    ok, x_new, y_new, straight_mode, expl = solver._normalize_cable_segments(
        x, y, L_target=8.0, bateau=bateau, rov=rov, N_target=N_target
    )
    assert straight_mode is False
    assert expl == ""
    assert ok is True
    assert len(x_new) == N_target + 1
    assert len(y_new) == N_target + 1
    assert np.isclose(x_new[0], bateau[0])
    assert np.isclose(y_new[0], 0.0)
    assert np.isclose(x_new[-1], rov[0])
    assert np.isclose(y_new[-1], rov[1])
    assert np.all(y_new <= 1e-12)

    # Version pré-zigzag : on valide la structure, pas l'égalité stricte des longueurs.
