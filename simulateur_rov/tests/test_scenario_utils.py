import pytest

from src.utils.scenario_utils import (
    verifier_syntaxe_scenario,
    commande_scenario,
    auto_L_1,
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
        (0.0, -10.0, 100.0, 0.0, 0.0, 10.0),
        (1.0, -10.0, 101.0, 0.0, 0.0, 11.0),
        (2.0, -10.0, 102.0, 0.0, 0.0, 12.0),
        (3.0, -10.0, 103.0, 0.0, 0.0, 13.0),
    ]

    result = None
    for args in sc_calls:
        result = auto_L_1(*args, Trupt=30.0, Tcible=15.0, K=3, N=3)
    assert result is not None
    assert result == pytest.approx(-2.0 / 3.0, rel=1e-6)

    if hasattr(auto_L_1, "_history"):
        auto_L_1._history = []

    result = None
    for _ in range(4):
        result = auto_L_1(0.0, -10.0, 100.0, 0.0, 0.0, 20.0, Trupt=30.0, Tcible=15.0, K=3, N=3)
    assert result == pytest.approx(1.0 / 3.0, rel=1e-6)
