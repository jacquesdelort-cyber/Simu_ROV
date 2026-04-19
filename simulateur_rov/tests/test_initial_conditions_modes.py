import numpy as np

from src.utils.initial_conditions import get_initial_state


class _DummySolver:
    def __init__(self):
        self.with_current_calls = 0
        self.static_calls = 0

    def solve_equilibrium_static_with_current(self, *_args, **_kwargs):
        self.with_current_calls += 1
        return (
            np.array([99.0, 50.0, 1.0], dtype=float),
            np.array([99.0, -2.0, -8.0], dtype=float),
            np.array([1.0, 2.0, 3.0], dtype=float),
        )

    def solve_equilibrium_static(self, *_args, **_kwargs):
        self.static_calls += 1
        return (
            np.array([77.0, 42.0, 2.0], dtype=float),
            np.array([77.0, -1.0, -7.0], dtype=float),
            np.array([4.0, 5.0, 6.0], dtype=float),
        )


class _DummySystem:
    def __init__(self):
        self.rov = type("ROV", (), {"m": 10.0, "V": 0.01})()
        self.cable = type("Cable", (), {"solver": _DummySolver()})()

    def pack_state(self, x_rov, y_rov, vx_rov, vy_rov, x_boat, vx_boat, x_cable, y_cable, t_cable, L):
        return {
            "x_rov": x_rov,
            "y_rov": y_rov,
            "vx_rov": vx_rov,
            "vy_rov": vy_rov,
            "x_boat": x_boat,
            "vx_boat": vx_boat,
            "x_cable": np.asarray(x_cable, dtype=float),
            "y_cable": np.asarray(y_cable, dtype=float),
            "t_cable": np.asarray(t_cable, dtype=float),
            "L": L,
        }


def test_get_initial_state_uses_strict_mode_by_default():
    system = _DummySystem()
    y0 = get_initial_state(system, x_rov=12.0, y_rov=-6.0, x_boat=3.0, L=40.0)

    assert system.cable.solver.with_current_calls == 1
    assert system.cable.solver.static_calls == 0
    assert y0["x_cable"][0] == 3.0
    assert y0["y_cable"][0] == 0.0
    assert y0["x_cable"][-1] == 12.0
    assert y0["y_cable"][-1] == -6.0


def test_get_initial_state_legacy_mode_keeps_solver_geometry():
    system = _DummySystem()
    y0 = get_initial_state(
        system,
        x_rov=12.0,
        y_rov=-6.0,
        x_boat=3.0,
        L=40.0,
        use_current_geometry=False,
        legacy_init_geometry=True,
    )

    assert system.cable.solver.with_current_calls == 0
    assert system.cable.solver.static_calls == 1
    assert y0["x_cable"][0] == 77.0
    assert y0["y_cable"][0] == 77.0
