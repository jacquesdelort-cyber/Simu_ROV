"""Tests légers pour les commandes scenario dans simulation_thread (inspection source)."""
from __future__ import annotations

from pathlib import Path

SIMULATION_THREAD_PATH = (
    Path(__file__).resolve().parents[1] / "src" / "ui" / "pyqt_widgets" / "simulation_thread.py"
)


def _source() -> str:
    return SIMULATION_THREAD_PATH.read_text(encoding="utf-8")


def test_auto_mode_uses_fx_for_auto_without_overwriting_scenario_cmd():
    source = _source()
    assert "fx_for_auto" in source
    assert "fy_for_auto" in source
    assert "Fx_rov=fx_for_auto" in source
    assert "Fy_rov=fy_for_auto" in source
    assert "if fx_rov_cmd is None:" in source
    assert "else:\n                                    fx_for_auto = float(fx_rov_cmd)" in source
