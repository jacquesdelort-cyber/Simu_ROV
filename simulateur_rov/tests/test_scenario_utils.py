import pytest

from src.utils.scenario_utils import (
    verifier_syntaxe_scenario,
    commande_scenario,
    auto_L_1,
    auto_L_8,
    auto_L_min,
    consigne_moulinet,
)


def test_verifier_syntaxe_scenario_valid_cases():
    valid_cases = [
        "",
        "   ",
        "(t>0: 6) (t>15:-2.68)",
        "(y<-15: 12)",
        "(x>10:5)",
        "(t>0:3)  (t>10.5:-1) (l<50:-17.1) (t > 199 : -1.1)",
        "(t>1e-3:2.5)",
    ]

    for scen in valid_cases:
        assert verifier_syntaxe_scenario(scen) is True, f"Devrait être valide: {scen!r}"


def test_verifier_syntaxe_scenario_invalid_cases():
    invalid_cases = [
        "(t=0; 6) (t>15:-2.68)",
        "(p<= -15: 12)",
        "(t>0:3)  (t>10.5:-1;7.0) (l<50:-17.1) (t > 199 : -1.1)",
        "(u<12:17)",
    ]

    for scen in invalid_cases:
        assert verifier_syntaxe_scenario(scen) is False, f"Devrait être invalide: {scen!r}"


def test_commande_scenario_progression_et_reset():
    scen = "(t>0:1) (t>5:2)"

    assert commande_scenario("Fx_rov", scen, t=0, L=0, y=0, x=0) is None
    assert commande_scenario("Fx_rov", scen, t=1, L=0, y=0, x=0) == 1.0
    assert commande_scenario("Fx_rov", scen, t=2, L=0, y=0, x=0) is None
    assert commande_scenario("Fx_rov", scen, t=6, L=0, y=0, x=0) == 2.0
    assert commande_scenario("Fx_rov", scen, t=7, L=0, y=0, x=0) is None

    assert commande_scenario("Fx_rov", scen, t=0, L=0, y=0, x=0) is None
    assert commande_scenario("Fx_rov", scen, t=1, L=0, y=0, x=0) == 1.0

    scen_long = "(l>10:3)"
    assert commande_scenario("dL_dt", scen_long, t=0, L=0, y=0, x=0) is None
    assert commande_scenario("dL_dt", scen_long, t=1, L=12, y=0, x=0) == 3.0
    assert commande_scenario("Fx_rov", scen_long, t=1, L=12, y=0, x=0) == 3.0

    scen_vx = "(t>0:4)"
    assert commande_scenario("Vx_bateau", scen_vx, t=0, L=0, y=0, x=0) is None
    assert commande_scenario("Vx_bateau", scen_vx, t=1, L=0, y=0, x=0) == 4.0


def test_commande_scenario_param_commande_invalide():
    with pytest.raises(SystemExit):
        commande_scenario("BadParam", "(t>0:1)", t=1, L=0, y=0, x=0)


def test_controle_auto_dL_dt_k3_n3():
    if hasattr(auto_L_1, "_history"):
        auto_L_1._history = []

    sc_calls = [
        (0.0, -10.0, 100.0, 100.0, 10.0, 0.0, 0.0, 0.0, 0.0),
        (1.0, -10.0, 101.0, 101.0, 11.0, 0.0, 0.0, 0.0, 0.0),
        (2.0, -10.0, 102.0, 102.0, 12.0, 0.0, 0.0, 0.0, 0.0),
        (3.0, -10.0, 103.0, 103.0, 13.0, 0.0, 0.0, 0.0, 0.0),
    ]

    result = None
    for args in sc_calls:
        result = auto_L_1(*args, Trupt=30.0, Tcible=15.0, K=3, N=3)
    assert result is not None
    ret, _ = result
    # Limitation d'accélération du moulinet (gamma_min=-0.5) peut plafonner à -0.5 m/s par pas.
    assert ret == pytest.approx(-0.5, rel=1e-6)

    if hasattr(auto_L_1, "_history"):
        auto_L_1._history = []

    result = None
    for i in range(4):
        result = auto_L_1(
            float(i), -10.0, 100.0 + i, 100.0 + i, 20.0 + i, 0.0, 0.0, 0.0, 0.0,
            Trupt=30.0, Tcible=15.0, K=3, N=3,
        )
    ret, _ = result
    assert ret == pytest.approx(1.0, rel=1e-6)


def test_auto_L_8_slack_floor_when_ascending_with_low_slack():
    """En remontée (Fy>=0), un slack critique doit imposer un débit positif, pas un hold à 0."""
    for attr in ("_history", "_slack_mode", "_tension_mode", "_tension_mode_age", "_minpay_active"):
        if hasattr(auto_L_8, attr):
            delattr(auto_L_8, attr)

    L_straight = 400.0
    slack = 0.05
    L = L_straight + slack
    t0, dt = 3.5, 0.1
    calls = [
        (t0 - dt, -399.0, L + 0.1, L_straight + 0.15, 0.06, 0.0, 0.2),
        (t0, -398.9, L, L_straight, 0.06, 0.0, 0.2),
    ]
    ret = None
    for args in calls:
        ret, exp = auto_L_8(
            *args,
            x_rov=30.0,
            x_boat=0.0,
            Trupt=100.0,
            Tcible=0.25,
            dl_dt_min=-1.0,
            dl_dt_max=1.0,
            K=3,
            N=1,
        )
    assert ret is not None
    assert ret > 0.0, f"dL/dt doit être positif avec slack critique, got {ret} ({exp})"
    assert "ADD_SLACK" in exp or "SLACK_FLOOR" in exp


def test_auto_L_8_controle_cable_with_data_dict():
    """Ne doit pas lever TypeError sur y_boat_curr (scalaire 0.0)."""
    for attr in ("_history", "_slack_mode", "_tension_mode", "_tension_mode_age", "_minpay_active"):
        if hasattr(auto_L_8, attr):
            delattr(auto_L_8, attr)

    n_pts = 5
    data = {
        "x_boat": [0.0],
        "x_cable_curr": [0.0, 25.0, 50.0, 75.0, 100.0],
        "y_cable_curr": [0.0, -90.0, -180.0, -270.0, -360.0],
    }
    ret, exp = auto_L_8(
        40.6,
        -367.23,
        420.0,
        L_straight=380.75,
        T_boat=89.95,
        T_rov=25.0,
        Fx_rov=0.0,
        Fy_rov=0.2,
        x_rov=100.54,
        x_boat=0.0,
        Trupt=100.0,
        Tcible=50.0,
        K=3,
        N=1,
        data=data,
    )
    assert isinstance(ret, float)
    assert isinstance(exp, str)


def test_consigne_moulinet_history_has_T_boat():
    if hasattr(consigne_moulinet, "_history"):
        consigne_moulinet._history = []

    consigne_moulinet(0.0, -400.0, 35.0, 420.0)
    ret, exp = consigne_moulinet(0.1, -399.9, 34.0, 420.0)

    assert "T_boat" in consigne_moulinet._history[0]
    assert consigne_moulinet._history[0]["T_boat"] == pytest.approx(34.0)
    assert isinstance(ret, float)
    assert isinstance(exp, str)


def test_auto_L_min_two_steps_no_keyerror():
    if hasattr(consigne_moulinet, "_history"):
        consigne_moulinet._history = []

    auto_L_min(0.0, -400.0, 420.0, T_boat=35.0)
    ret, exp = auto_L_min(0.1, -399.9, 420.0, T_boat=34.0)

    assert "T_boat" in consigne_moulinet._history[0]
    assert isinstance(ret, float)
    assert isinstance(exp, str)
