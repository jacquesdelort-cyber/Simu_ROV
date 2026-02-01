import numpy as np

from src.models.environment import Environment


def test_get_current_velocity_constant_string():
    env = Environment({})
    assert env.get_current_velocity(0.0, "0.5") == 0.5
    assert env.get_current_velocity(42.0, "-1.25") == -1.25


def test_get_current_velocity_profile_string_interpolation():
    env = Environment({})
    v = env.get_current_velocity(25.0, "0:0;10:0.5;50:1.0")
    assert np.isclose(v, 0.6875)


def test_get_current_velocity_comma_decimal():
    env = Environment({})
    assert env.get_current_velocity(0.0, "0,75") == 0.75


def test_get_current_velocity_odd_numbers_fallback_to_constant():
    env = Environment({})
    assert env.get_current_velocity(0.0, "0:0;10") == 0.0


def test_get_current_velocity_none_uses_profile():
    env = Environment({"current_profile": lambda y: 2.0})
    assert env.get_current_velocity(0.0) == 2.0
