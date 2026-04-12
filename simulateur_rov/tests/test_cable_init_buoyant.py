"""Tests init chaînette câble flottant (repère z = -y)."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from src.models.system_model import ROVSystem
from src.utils.cable_init_buoyant import (
    build_buoyant_cable_polyline,
    is_buoyant_cable,
    manhattan_length,
    solve_a_vertex_at_boat,
    straight_length,
)


def _load_mission_params(name: str) -> dict:
    root = Path(__file__).resolve().parent.parent / "Missions" / name / "Param_mission.json"
    data = json.loads(root.read_text(encoding="utf-8"))
    return data["parameters"]


def test_is_buoyant_m_test_init():
    params = _load_mission_params("M_test_init_420")
    system = ROVSystem(params, N_segments=100)
    assert is_buoyant_cable(system) is True


def test_L0_reference_positive():
    a, L0 = solve_a_vertex_at_boat(30.0, 400.0)
    assert a > 0
    assert L0 > straight_length(30.0, 400.0)
    assert L0 < manhattan_length(30.0, 400.0)


@pytest.mark.parametrize(
    "L",
    [403.0, 410.0, 420.0],
)
def test_build_polyline_M_test_init_geometry(L: float):
    params = _load_mission_params("M_test_init_420")
    system = ROVSystem(params, N_segments=200)
    xr = 30.0
    yr = -400.0
    xb, yb = 0.0, 0.0
    xs, ys = build_buoyant_cable_polyline(xb, yb, xr, yr, L, system.N)
    assert xs is not None and ys is not None
    assert len(xs) == system.N + 1
    assert abs(float(xs[0]) - xb) < 1e-6 and abs(float(ys[0]) - yb) < 1e-6
    assert abs(float(xs[-1]) - xr) < 1e-2 and abs(float(ys[-1]) - yr) < 1e-2
    L_seg = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
    assert abs(L_seg - L) < max(5.0, 0.02 * L)
    y_line = yb + (xs - xb) / (xr - xb) * (yr - yb)
    assert float(np.min(ys - y_line)) <= 1e-6


def test_case1_returns_none_so_polyline_elsewhere():
    """L >= Manhattan : le builder délègue (None)."""
    xb, yb, xr, yr = 0.0, 0.0, 30.0, -400.0
    L = 450.0
    xs, ys = build_buoyant_cable_polyline(xb, yb, xr, yr, L, 50)
    assert xs is None and ys is None


def test_slack_side_auto_prefers_toward_rov_for_420():
    """auto : priorité toward_rov -> pas de grand slack en x' négatif (L=420, 30,-400)."""
    xb, yb, xr, yr, L = 0.0, 0.0, 30.0, -400.0, 420.0
    xs_a, _ = build_buoyant_cable_polyline(
        xb, yb, xr, yr, L, 150, slack_side="auto"
    )
    xs_o, _ = build_buoyant_cable_polyline(
        xb, yb, xr, yr, L, 150, slack_side="opposite"
    )
    assert xs_a is not None and xs_o is not None
    assert float(np.min(xs_a)) > -1.0
    assert float(np.min(xs_o)) < -5.0


def test_slack_side_toward_rov_vertex_x_M_test_init_420():
    """Sommet H sur la surface entre bateau et ROV (x_H' ~ 18 m pour L=420, pas ~8 m qui serait L~412)."""
    xs, ys = build_buoyant_cable_polyline(
        0.0, 0.0, 30.0, -400.0, 420.0, 250, slack_side="toward_rov"
    )
    assert xs is not None and ys is not None
    horiz = np.where(np.abs(ys) < 0.08)[0]
    assert len(horiz) >= 2
    xh = float(np.max(xs[horiz]))
    assert 15.0 < xh < 22.0
