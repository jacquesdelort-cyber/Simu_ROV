import json
from pathlib import Path

import numpy as np
import pytest

import src.solvers.cable_solver as cable_solver_module
from src.models.state_projection import reconcile_straight_mode_after_normalize
from src.models.system_model import ROVSystem
from src.utils.initial_conditions import get_initial_state


@pytest.fixture(autouse=True)
def _silence_trace_print(monkeypatch):
    monkeypatch.setattr(cable_solver_module, "trace_print", lambda *args, **kwargs: None)


def _load_mission_data(mission_name: str):
    mission_file = (
        Path(__file__).resolve().parents[1]
        / "Missions"
        / mission_name
        / "Param_mission.json"
    )
    with mission_file.open("r", encoding="utf-8") as f:
        return json.load(f)


def _build_system(parameters: dict, n_segments: int):
    return ROVSystem(parameters, N_segments=n_segments)


def _compute_length(x_cable, y_cable):
    return float(np.sum(np.hypot(np.diff(x_cable), np.diff(y_cable))))


def _linear_reference_x(y_vals, x_boat, y_boat, x_rov, y_rov):
    y_vals = np.asarray(y_vals, dtype=float)
    denom = float(y_rov - y_boat)
    if abs(denom) <= 1e-12:
        return np.full_like(y_vals, float(x_boat))
    t = (y_vals - float(y_boat)) / denom
    return float(x_boat) + t * (float(x_rov) - float(x_boat))


def _discrete_turning_values(x_vals, y_vals):
    p = np.column_stack((np.asarray(x_vals, dtype=float), np.asarray(y_vals, dtype=float)))
    v1 = p[1:-1] - p[:-2]
    v2 = p[2:] - p[1:-1]
    turn = v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0]
    return turn


def _finalize_cable_like_simulation_ui(system, x_cable, y_cable, L, x_boat, y_boat, x_rov, y_rov):
    """Même post-traitement câble que ``SimulationTab.initialize_system`` (strict + courant)."""
    xc = np.asarray(x_cable, dtype=float)
    yc = np.asarray(y_cable, dtype=float)
    Lf = float(L)
    xr, yr = float(x_rov), float(y_rov)
    xc, yc, sm, _ = system.cable.solver._normalize_cable_length(
        xc,
        yc,
        Lf,
        x_boat=float(x_boat),
        y_boat=float(y_boat),
        x_rov=xr,
        y_rov=yr,
        k_tail=10,
        t=0.0,
    )
    xc, yc, xr, yr = reconcile_straight_mode_after_normalize(
        xc, yc, sm, xr, yr, float(x_boat), float(y_boat)
    )
    xc, yc = system.cable.solver._enforce_uniform_current_concavity(
        xc, yc, float(x_boat), float(y_boat), xr, yr, strength=0.78
    )
    xc, yc, sm2, _ = system.cable.solver._normalize_cable_length(
        xc,
        yc,
        Lf,
        x_boat=float(x_boat),
        y_boat=float(y_boat),
        x_rov=xr,
        y_rov=yr,
        k_tail=10,
        t=0.0,
    )
    xc, yc, xr, yr = reconcile_straight_mode_after_normalize(
        xc, yc, sm2, xr, yr, float(x_boat), float(y_boat)
    )
    return xc, yc, xr, yr


def _run_mission_init_strict_static_checks(mission_name: str):
    mission = _load_mission_data(mission_name)
    parameters = mission["parameters"]
    init_params = mission["init_params"]

    # Use a reduced mesh for test runtime stability.
    n_segments = 220
    x_rov = float(init_params["x_rov_init"])
    y_rov = float(init_params["y_rov_init"])
    x_boat = float(init_params["x_boat_init"])
    L_init = float(init_params["L_init"])

    system_cur = _build_system(parameters, n_segments=n_segments)
    system_cur.environment.v_courant_raw = str(init_params["v_courant"])
    y0_cur = get_initial_state(
        system_cur,
        x_rov=x_rov,
        y_rov=y_rov,
        x_boat=x_boat,
        L=L_init,
        use_current_geometry=True,
        cable_init_mode="strict_static",
        legacy_init_geometry=False,
    )
    (_, _, _, _, _, _, x_cable_cur, y_cable_cur, t_cur, _) = system_cur.unpack_state(y0_cur)

    # Invariants at t=0 for strict static initialization.
    assert np.isclose(float(x_cable_cur[0]), x_boat, atol=1e-9)
    assert np.isclose(float(y_cable_cur[0]), 0.0, atol=1e-9)
    assert np.isclose(float(x_cable_cur[-1]), x_rov, atol=1e-9)
    assert np.isclose(float(y_cable_cur[-1]), y_rov, atol=1e-9)
    assert np.all(np.asarray(y_cable_cur, dtype=float) <= 1e-9)
    assert np.isclose(_compute_length(x_cable_cur, y_cable_cur), L_init, rtol=2e-3)
    assert np.all(np.isfinite(np.asarray(t_cur, dtype=float)))

    # Compare with no-current baseline using same mission geometry.
    system_no = _build_system(parameters, n_segments=n_segments)
    system_no.environment.v_courant_raw = "0.0"
    y0_no = get_initial_state(
        system_no,
        x_rov=x_rov,
        y_rov=y_rov,
        x_boat=x_boat,
        L=L_init,
        use_current_geometry=False,
        cable_init_mode="strict_static",
        legacy_init_geometry=False,
    )
    (_, _, _, _, _, _, x_cable_no, _y_cable_no, _, _) = system_no.unpack_state(y0_no)

    dx = np.asarray(x_cable_cur, dtype=float) - np.asarray(x_cable_no, dtype=float)
    assert float(np.max(np.abs(dx))) > 1e-3


def test_mission_init_courant_1_strict_static_invariants_and_current_effect():
    _run_mission_init_strict_static_checks("M_test_init_courant_1")


def test_mission_init_courant_1_smooth_bulge_same_side_as_courant_0():
    """
    Profil mission proche de l'uniforme (rampes minces) : pas de double arc ni bosse côté x<0.
    """
    mission = _load_mission_data("M_test_init_courant_1")
    parameters = mission["parameters"]
    init_params = mission["init_params"]
    n_segments = 220
    x_rov = float(init_params["x_rov_init"])
    y_rov = float(init_params["y_rov_init"])
    x_boat = float(init_params["x_boat_init"])
    y_boat = 0.0
    L_init = float(init_params["L_init"])

    system_cur = _build_system(parameters, n_segments=n_segments)
    system_cur.environment.v_courant_raw = str(init_params["v_courant"])
    y0_cur = get_initial_state(
        system_cur,
        x_rov=x_rov,
        y_rov=y_rov,
        x_boat=x_boat,
        L=L_init,
        use_current_geometry=True,
        cable_init_mode="strict_static",
        legacy_init_geometry=False,
    )
    (xr0, yr0, _, _, _, _, x_raw, y_raw, _, _) = system_cur.unpack_state(y0_cur)
    xc, yc, xr_f, yr_f = _finalize_cable_like_simulation_ui(
        system_cur,
        x_raw,
        y_raw,
        L_init,
        x_boat,
        y_boat,
        xr0,
        yr0,
    )
    x_ref = _linear_reference_x(yc, x_boat, y_boat, xr_f, yr_f)
    x_off = np.asarray(xc, dtype=float) - x_ref
    assert float(np.mean(x_off)) > 1e-3
    assert float(np.min(xc)) >= float(x_boat) - 0.15

    turn = _discrete_turning_values(xc, yc)
    scale = max(float(np.median(np.abs(turn))), 1e-12)
    active = np.abs(turn) > 0.02 * scale
    assert np.any(active)
    active_turn = turn[active]
    assert max(
        float(np.mean(active_turn > 0.0)),
        float(np.mean(active_turn < 0.0)),
    ) >= 0.4


def test_mission_init_courant_0_strict_static_invariants_and_current_effect():
    mission = _load_mission_data("M_test_init_courant_0")
    parameters = mission["parameters"]
    init_params = mission["init_params"]

    n_segments = 220
    x_rov = float(init_params["x_rov_init"])
    y_rov = float(init_params["y_rov_init"])
    x_boat = float(init_params["x_boat_init"])
    y_boat = 0.0
    L_init = float(init_params["L_init"])

    system_cur = _build_system(parameters, n_segments=n_segments)
    system_cur.environment.v_courant_raw = str(init_params["v_courant"])
    y0_cur = get_initial_state(
        system_cur,
        x_rov=x_rov,
        y_rov=y_rov,
        x_boat=x_boat,
        L=L_init,
        use_current_geometry=True,
        cable_init_mode="strict_static",
        legacy_init_geometry=False,
    )
    (xr0, yr0, _, _, _, _, x_raw, y_raw, t_cur, _) = system_cur.unpack_state(y0_cur)

    # Même finalisation que l’onglet Simulation (sinon ``get_initial_state`` seul ne reflète pas le tracé).
    x_cable_cur, y_cable_cur, x_rov_f, y_rov_f = _finalize_cable_like_simulation_ui(
        system_cur,
        x_raw,
        y_raw,
        L_init,
        x_boat,
        y_boat,
        xr0,
        yr0,
    )

    # Courbure locale (après finalisation UI ; la post-normalisation mélange parfois les signes discrets).
    turn = _discrete_turning_values(x_cable_cur, y_cable_cur)
    scale = max(float(np.median(np.abs(turn))), 1e-12)
    active = np.abs(turn) > 0.02 * scale
    assert np.any(active)
    active_turn = turn[active]
    frac_pos = float(np.mean(active_turn > 0.0))
    frac_neg = float(np.mean(active_turn < 0.0))
    assert max(frac_pos, frac_neg) >= 0.4

    # Invariants strict_static (après post-traitement affiché).
    assert np.isclose(float(x_cable_cur[0]), x_boat, atol=1e-9)
    assert np.isclose(float(y_cable_cur[0]), y_boat, atol=1e-9)
    assert np.isclose(float(x_cable_cur[-1]), x_rov_f, atol=1e-9)
    assert np.isclose(float(y_cable_cur[-1]), y_rov_f, atol=1e-9)
    assert np.isclose(float(x_rov_f), x_rov, atol=1e-6)
    assert np.isclose(float(y_rov_f), y_rov, atol=1e-6)
    assert np.all(np.asarray(y_cable_cur, dtype=float) <= 1e-9)
    assert np.isclose(_compute_length(x_cable_cur, y_cable_cur), L_init, rtol=2e-3)
    assert np.all(np.isfinite(np.asarray(t_cur, dtype=float)))

    # Courbure globale : v mission > 0 → décalage x > ligne bateau-ROV (bosse côté +x, comme le graphe courant).
    x_ref = _linear_reference_x(y_cable_cur, x_boat, y_boat, x_rov_f, y_rov_f)
    x_offset = np.asarray(x_cable_cur, dtype=float) - x_ref
    assert float(np.mean(x_offset)) > 1e-3
    assert float(np.max(x_offset)) > 1e-2

    # Pas de variations brutales: la courbure discrète ne saute pas fortement.
    if active_turn.size >= 5:
        kappa_jump = np.abs(np.diff(active_turn))
        jump_p95 = float(np.percentile(kappa_jump, 95))
        kappa_typ = max(float(np.median(np.abs(active_turn))), 1e-12)
        assert jump_p95 <= 6.0 * kappa_typ


def test_mission_init_courant_2_strict_static_s_curve_opposite_bands():
    """
    M_test_init_courant_2 : inversion de courant vers 200 m de profondeur.
    v mission > 0 (couches hautes) → déport x > corde ; v mission < 0 (couches basses) → déport x < corde.
    """
    mission = _load_mission_data("M_test_init_courant_2")
    parameters = mission["parameters"]
    init_params = mission["init_params"]
    n_segments = 220
    x_rov = float(init_params["x_rov_init"])
    y_rov = float(init_params["y_rov_init"])
    x_boat = float(init_params["x_boat_init"])
    y_boat = 0.0
    L_init = float(init_params["L_init"])
    v_raw = str(init_params["v_courant"])

    system_cur = _build_system(parameters, n_segments=n_segments)
    system_cur.environment.v_courant_raw = v_raw
    y0_cur = get_initial_state(
        system_cur,
        x_rov=x_rov,
        y_rov=y_rov,
        x_boat=x_boat,
        L=L_init,
        use_current_geometry=True,
        cable_init_mode="strict_static",
        legacy_init_geometry=False,
    )
    (xr0, yr0, _, _, _, _, x_raw, y_raw, _, _) = system_cur.unpack_state(y0_cur)
    xc, yc, xr_f, yr_f = _finalize_cable_like_simulation_ui(
        system_cur, x_raw, y_raw, L_init, x_boat, y_boat, xr0, yr0
    )
    y_arr = np.asarray(yc, dtype=float)
    v_node = np.asarray(
        [
            system_cur.environment.get_current_velocity(float(yy), v_raw)
            for yy in y_arr
        ],
        dtype=float,
    )
    x_ref = _linear_reference_x(y_arr, x_boat, y_boat, xr_f, yr_f)
    x_off = np.asarray(xc, dtype=float) - x_ref

    m_pos_v = (v_node > 0.25) & (y_arr < -30.0) & (y_arr > y_rov + 20.0)
    m_neg_v = (v_node < -0.25) & (y_arr < -210.0) & (y_arr > y_rov + 20.0)
    assert np.count_nonzero(m_pos_v) >= 8
    assert np.count_nonzero(m_neg_v) >= 8
    assert float(np.mean(x_off[m_pos_v])) > 5e-4
    assert float(np.mean(x_off[m_neg_v])) < -5e-4
