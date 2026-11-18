"""Lois continues du câble : smoothstep, pénalité de mou, mélange quasi-tendu."""
from __future__ import annotations

import numpy as np
import pytest

from src.solvers.cable_continuity import (
    CONSTRAINT_OMEGA_RAD_S,
    CONSTRAINT_SLACK_BAND_M,
    CONSTRAINT_ZETA,
    NEAR_TAUT_R_MAX,
    TAU_SLACK_S,
    TAU_TAUT_S,
    blend_polyline_near_taut,
    continuous_tau_tension,
    rov_accel_scale,
    smooth_vertical_node_velocity,
    smoothstep,
    straight_blend_weight,
    payout_matched_velocity,
    taut_constraint_tension,
    taut_penalty_newton,
)


def test_smoothstep_is_flat_at_ends():
    assert smoothstep(0.0) == 0.0
    assert smoothstep(1.0) == 1.0
    # Dérivée 6z(1-z) nulle en 0 et en 1 : l'écart au bord est O(h^2).
    h = 1e-4
    assert smoothstep(h) < 1e-7
    assert 1.0 - smoothstep(1.0 - h) < 1e-7


def test_straight_blend_weight_ramps_over_five_percent():
    assert straight_blend_weight(0.99) == 1.0
    assert straight_blend_weight(1.0) == 1.0
    assert straight_blend_weight(NEAR_TAUT_R_MAX) == 0.0
    assert straight_blend_weight(1.2) == 0.0
    mid = straight_blend_weight(1.0 + 0.5 * (NEAR_TAUT_R_MAX - 1.0))
    assert 0.2 < mid < 0.8


def test_tau_and_penalty_match_at_zero_slack():
    assert continuous_tau_tension(-0.1) == TAU_TAUT_S
    assert continuous_tau_tension(0.0) == TAU_TAUT_S
    assert continuous_tau_tension(0.05) == pytest.approx(TAU_SLACK_S)
    assert taut_penalty_newton(100.0, 0.0, 50.0) == 0.0
    assert taut_penalty_newton(100.0, 0.01, 50.0) == 0.0
    assert taut_penalty_newton(100.0, -1.0, 50.0) > 0.0
    # Déficit doublé, encore sous le plafond : la pénalité quadruple.
    # bande = 0,02 * L = 0,8 m ; déficits 0,2 m et 0,4 m.
    p1 = taut_penalty_newton(80.0, -0.2, 40.0)
    p2 = taut_penalty_newton(80.0, -0.4, 40.0)
    assert p2 == pytest.approx(4.0 * p1)


def test_taut_constraint_holds_rov_when_slack_is_zero():
    # Câble vertical, mou nul, descente plus rapide que le dévidage.
    # La réaction équilibre la force vers le bas et freine l'excès de vitesse.
    mass = 30.0
    f_down = 25.0
    v_out = 0.65
    dL_dt = 0.48
    tension = taut_constraint_tension(
        0.0, -f_down, 0.0, 1.0, 0.0, -v_out, dL_dt, 0.0, mass
    )
    s_dot = v_out - dL_dt
    expected = f_down + mass * (2.0 * CONSTRAINT_ZETA * CONSTRAINT_OMEGA_RAD_S * s_dot)
    assert tension == pytest.approx(expected)
    # À la vitesse du dévidage, il ne reste que la force d'écartement.
    held = taut_constraint_tension(
        0.0, -f_down, 0.0, 1.0, 0.0, -dL_dt, dL_dt, 0.0, mass
    )
    assert held == pytest.approx(f_down)


def test_payout_matched_velocity_follows_winch_when_taut():
    # Câble vertical, descente libre plus rapide que le dévidage.
    vx, vy = payout_matched_velocity(0.0, -2.0, 0.0, 1.0, 0.5, 0.0)
    assert vx == pytest.approx(0.0)
    assert vy == pytest.approx(-0.5)
    # Mou large : la vitesse n'est pas imposée.
    vx_s, vy_s = payout_matched_velocity(0.1, -2.0, 0.0, 1.0, 0.5, 1.0)
    assert vx_s == pytest.approx(0.1)
    assert vy_s == pytest.approx(-2.0)


def test_taut_constraint_is_zero_when_the_cable_is_slack():
    assert taut_constraint_tension(
        0.0, -40.0, 0.0, 1.0, 0.0, -0.65, 0.48, 1.0, 30.0
    ) == 0.0
    # Les autres forces ramènent le ROV vers le bateau : le fil ne pousse pas.
    assert taut_constraint_tension(
        0.0, 40.0, 0.0, 1.0, 0.0, 0.2, 0.0, 0.0, 30.0
    ) == 0.0
    # Au bord de la bande le raccord est nul.
    assert taut_constraint_tension(
        0.0, -40.0, 0.0, 1.0, 0.0, -0.65, 0.48, CONSTRAINT_SLACK_BAND_M, 30.0
    ) == 0.0


def test_rov_accel_scale_is_one_when_slack_and_zero_when_very_taut():
    assert rov_accel_scale(0.1, 50.0) == 1.0
    assert rov_accel_scale(-0.02 * 50.0, 50.0) == 0.0
    assert 0.0 < rov_accel_scale(-0.01 * 50.0, 50.0) < 1.0


def test_surface_velocity_has_no_hard_cutoff_underwater():
    y = np.array([0.0, -0.025, -0.05, -1.0])
    vy = np.array([0.4, 0.4, 0.4, 0.4])
    out = smooth_vertical_node_velocity(y, vy)
    assert out[0] == 0.0
    assert 0.0 < out[1] < 0.4
    assert out[2] == pytest.approx(0.4)
    assert out[3] == pytest.approx(0.4)
    above = smooth_vertical_node_velocity(np.array([0.1]), np.array([1.0]))
    assert above[0] < 0.0


def test_blend_uses_same_weight_and_is_identity_when_slack_is_large():
    x = np.array([0.0, 1.0, 3.0])
    y = np.array([0.0, -2.0, -4.0])
    tension = np.array([10.0, 12.0, 14.0])
    x2, y2, t2 = blend_polyline_near_taut(x, y, tension, 0.0, 3.0, -4.0, 20.0)
    assert np.allclose(x2, x)
    assert np.allclose(y2, y)
    assert np.allclose(t2, tension)

    # Distance droite = 5 m, L = 5 m : corde, tension linéaire entre les bouts.
    x3, y3, t3 = blend_polyline_near_taut(x, y, tension, 0.0, 3.0, -4.0, 5.0)
    assert np.allclose(x3, np.linspace(0.0, 3.0, 3))
    assert np.allclose(y3, np.linspace(0.0, -4.0, 3))
    assert np.allclose(t3, np.linspace(tension[0], tension[-1], 3))
