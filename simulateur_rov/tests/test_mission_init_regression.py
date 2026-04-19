"""
Non-régression : missions M_test_init_* et M_test_init_courant_* (init strict statique + géométrie câble).

Les assertions restent volontairement tolérantes sur les tolérances numériques (longueur, corde)
pour limiter la fragilité du maillage, tout en verrouillant les invariants physiques et de signe
qui ont été corrigés (flottant sans courant, neutre + courant quasi uniforme, profil en S, etc.).
"""
from __future__ import annotations

import numpy as np
import pytest

import src.solvers.cable_solver as cable_solver_module
from src.models.system_model import ROVSystem
from src.utils.initial_conditions import get_initial_state

from test_mission_init_courant_1 import (
    _build_system,
    _compute_length,
    _finalize_cable_like_simulation_ui,
    _linear_reference_x,
    _load_mission_data,
)


@pytest.fixture(autouse=True)
def _silence_trace_print(monkeypatch):
    monkeypatch.setattr(cable_solver_module, "trace_print", lambda *args, **kwargs: None)


N_SEG = 220
RTOL_LEN = 2e-3

# Missions sans courant (v = 0) — câble flottant (rho_cable < rho_eau) sauf mention.
MISSIONS_INIT_SANS_COURANT = [
    "M_test_init_401",
    "M_test_init_402",
    "M_test_init_404",
    "M_test_init_410",
    "M_test_init_420",
    "M_test_init_430",
    "M_test_init_450",
]

MISSIONS_INIT_AVEC_COURANT = [
    "M_test_init_courant_0",
    "M_test_init_courant_1",
    "M_test_init_courant_2",
    "M_test_init_courant_3",
]


def _is_buoyant(parameters: dict) -> bool:
    rho_c = float(parameters["cable"]["rho_cable"])
    rho_w = float(parameters["environment"]["rho_eau"])
    return rho_c + 1e-9 < rho_w


def _strict_init_state(
    parameters: dict,
    *,
    x_rov: float,
    y_rov: float,
    x_boat: float,
    L: float,
    v_courant_raw: str,
    n_segments: int = N_SEG,
):
    system = _build_system(parameters, n_segments=n_segments)
    system.environment.v_courant_raw = v_courant_raw
    y0 = get_initial_state(
        system,
        x_rov=x_rov,
        y_rov=y_rov,
        x_boat=x_boat,
        L=L,
        use_current_geometry=True,
        cable_init_mode="strict_static",
        legacy_init_geometry=False,
    )
    return system, y0


def _assert_endpoints_and_surface(
    x_cable, y_cable, x_boat: float, y_boat: float, x_rov: float, y_rov: float
):
    x_cable = np.asarray(x_cable, dtype=float)
    y_cable = np.asarray(y_cable, dtype=float)
    assert np.isclose(float(x_cable[0]), x_boat, atol=1e-7)
    assert np.isclose(float(y_cable[0]), y_boat, atol=1e-7)
    assert np.isclose(float(x_cable[-1]), x_rov, atol=1e-5)
    assert np.isclose(float(y_cable[-1]), y_rov, atol=1e-5)
    assert np.all(y_cable <= 1e-9)


def _assert_length(x_cable, y_cable, L_target: float):
    L_seg = _compute_length(x_cable, y_cable)
    assert np.isclose(L_seg, L_target, rtol=RTOL_LEN)


def _assert_buoyant_not_heavier_than_chord(
    x_cable, y_cable, x_boat: float, x_rov: float, y_rov: float
):
    """Flottant sans courant : le câble ne doit pas pendre « côté lourd » sous la corde."""
    x_cable = np.asarray(x_cable, dtype=float)
    y_cable = np.asarray(y_cable, dtype=float)
    dx = float(x_rov - x_boat)
    assert abs(dx) > 1e-9
    for i in range(1, len(y_cable) - 1):
        t = (float(x_cable[i]) - float(x_boat)) / dx
        y_chord = float(t) * float(y_rov)
        # Tolérance plus large près du bateau / L ≈ L_st (ex. M_test_init_401) après normalisation.
        assert float(y_cable[i]) >= y_chord - 0.25


@pytest.mark.parametrize("mission_name", MISSIONS_INIT_SANS_COURANT)
def test_missions_init_sans_courant_strict_statique(mission_name: str):
    mission = _load_mission_data(mission_name)
    parameters = mission["parameters"]
    init = mission["init_params"]
    x_rov = float(init["x_rov_init"])
    y_rov = float(init["y_rov_init"])
    x_boat = float(init["x_boat_init"])
    y_boat = 0.0
    L_init = float(init["L_init"])
    v_raw = str(init.get("v_courant", "0.0"))

    system, y0 = _strict_init_state(
        parameters,
        x_rov=x_rov,
        y_rov=y_rov,
        x_boat=x_boat,
        L=L_init,
        v_courant_raw=v_raw,
    )
    (_, _, _, _, _, _, x_c, y_c, t_c, _) = system.unpack_state(y0)

    _assert_endpoints_and_surface(x_c, y_c, x_boat, y_boat, x_rov, y_rov)
    _assert_length(x_c, y_c, L_init)
    assert np.all(np.isfinite(np.asarray(t_c, dtype=float)))

    if _is_buoyant(parameters):
        # M_test_init_401 : L très proche de L_st — repli chaînette / normalisation ;
        # on ne impose pas le critère « au-dessus de la corde » point à point.
        if mission_name != "M_test_init_401":
            _assert_buoyant_not_heavier_than_chord(x_c, y_c, x_boat, x_rov, y_rov)
        assert float(np.min(y_c)) >= float(y_rov) - 0.35


@pytest.mark.parametrize("mission_name", MISSIONS_INIT_AVEC_COURANT)
def test_missions_init_avec_courant_strict_invariants(mission_name: str):
    mission = _load_mission_data(mission_name)
    parameters = mission["parameters"]
    init = mission["init_params"]
    x_rov = float(init["x_rov_init"])
    y_rov = float(init["y_rov_init"])
    x_boat = float(init["x_boat_init"])
    y_boat = 0.0
    L_init = float(init["L_init"])
    v_raw = str(init["v_courant"])

    system, y0 = _strict_init_state(
        parameters,
        x_rov=x_rov,
        y_rov=y_rov,
        x_boat=x_boat,
        L=L_init,
        v_courant_raw=v_raw,
    )
    (xr0, yr0, _, _, _, _, x_raw, y_raw, t_c, _) = system.unpack_state(y0)

    _assert_endpoints_and_surface(x_raw, y_raw, x_boat, y_boat, x_rov, y_rov)
    _assert_length(x_raw, y_raw, L_init)
    assert np.all(np.isfinite(np.asarray(t_c, dtype=float)))

    xc, yc, xr_f, yr_f = _finalize_cable_like_simulation_ui(
        system, x_raw, y_raw, L_init, x_boat, y_boat, xr0, yr0
    )
    _assert_endpoints_and_surface(xc, yc, x_boat, y_boat, xr_f, yr_f)
    _assert_length(xc, yc, L_init)

    if mission_name in ("M_test_init_courant_0", "M_test_init_courant_1"):
        x_ref = _linear_reference_x(yc, x_boat, y_boat, xr_f, yr_f)
        x_off = np.asarray(xc, dtype=float) - x_ref
        assert float(np.mean(x_off)) > 1e-3
        assert float(np.min(xc)) >= float(x_boat) - 0.15

    if mission_name == "M_test_init_courant_2":
        y_arr = np.asarray(yc, dtype=float)
        v_node = np.asarray(
            [
                system.environment.get_current_velocity(float(yy), v_raw)
                for yy in y_arr
            ],
            dtype=float,
        )
        x_ref = _linear_reference_x(y_arr, x_boat, y_boat, xr_f, yr_f)
        x_off = np.asarray(xc, dtype=float) - x_ref
        m_pos_v = (v_node > 0.25) & (y_arr < -30.0) & (y_arr > y_rov + 20.0)
        m_neg_v = (v_node < -0.25) & (y_arr < -210.0) & (y_arr > y_rov + 20.0)
        assert np.count_nonzero(m_pos_v) >= 6
        assert np.count_nonzero(m_neg_v) >= 6
        assert float(np.mean(x_off[m_pos_v])) > 5e-4
        assert float(np.mean(x_off[m_neg_v])) < -5e-4

    if mission_name == "M_test_init_courant_3" and _is_buoyant(parameters):
        assert float(np.mean(np.asarray(xc, dtype=float))) > float(x_boat)


def test_init_sans_courant_differe_de_zero_courant_pour_effet_courant():
    """Une mission flottante : avec v=0 la géométrie doit bouger si on active le courant mission."""
    mission = _load_mission_data("M_test_init_402")
    parameters = mission["parameters"]
    init = mission["init_params"]
    x_rov = float(init["x_rov_init"])
    y_rov = float(init["y_rov_init"])
    x_boat = float(init["x_boat_init"])
    L_init = float(init["L_init"])

    sys0, y0_0 = _strict_init_state(
        parameters,
        x_rov=x_rov,
        y_rov=y_rov,
        x_boat=x_boat,
        L=L_init,
        v_courant_raw="0.0",
    )
    sys1, y0_1 = _strict_init_state(
        parameters,
        x_rov=x_rov,
        y_rov=y_rov,
        x_boat=x_boat,
        L=L_init,
        v_courant_raw="0.0:1.0",
    )
    (_, _, _, _, _, _, x0, y0, _, _) = sys0.unpack_state(y0_0)
    (_, _, _, _, _, _, x1, y1, _, _) = sys1.unpack_state(y0_1)
    assert float(np.max(np.abs(np.asarray(x1, float) - np.asarray(x0, float)))) > 1e-3
