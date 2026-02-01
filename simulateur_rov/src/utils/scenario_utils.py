"""Validation de la syntaxe des scénarios."""
from __future__ import annotations

import re

from src.utils.logger import trace_print


_ALLOWED_PREFIXES = {"t", "l", "p"}
_NUMBER_REGEX = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
_COUPLE_REGEX = re.compile(
    rf"\(\s*(?P<prefix>[tLlPp])\s*(?P<op>[<>])\s*(?P<y>{_NUMBER_REGEX})\s*:"
    rf"\s*(?P<z>{_NUMBER_REGEX})\s*\)"
)


def verifier_syntaxe_scenario(scen: str) -> bool:
    """
    Valide une chaîne de scénario.

    Un scénario est une suite de couples de la forme (x>y:z) ou (x<y:z).
    - x ∈ {"t", "l", "p"}
    - y et z sont convertibles en float
    - une chaîne vide est valide
    """
    if scen is None:
        return False

    text = str(scen).strip()
    if text == "":
        return True

    pos = 0
    length = len(text)
    while True:
        while pos < length and text[pos].isspace():
            pos += 1
        if pos >= length:
            return True

        match = _COUPLE_REGEX.match(text, pos)
        if not match:
            return False

        if match.group("prefix").lower() not in _ALLOWED_PREFIXES:
            return False

        try:
            float(match.group("y"))
            float(match.group("z"))
        except ValueError:
            return False

        pos = match.end()

    return True


def _parse_scenario_criteria(scen: str) -> list[tuple[str, str, float, float]]:
    if scen is None:
        return []

    text = str(scen).strip()
    if text == "":
        return []

    criteria: list[tuple[str, str, float, float]] = []
    pos = 0
    length = len(text)
    while True:
        while pos < length and text[pos].isspace():
            pos += 1
        if pos >= length:
            return criteria

        match = _COUPLE_REGEX.match(text, pos)
        if not match:
            return []

        prefix = match.group("prefix").lower()
        if prefix not in _ALLOWED_PREFIXES:
            return []

        try:
            y_val = float(match.group("y"))
            z_val = float(match.group("z"))
        except ValueError:
            return []

        criteria.append((prefix, match.group("op"), y_val, z_val))
        pos = match.end()


def commande_scenario(param_commande: str, sc: str, t: float, L: float, p: float) -> float | None:
    """
    Calcule une consigne à partir d'un scénario et d'un paramètre de commande.
    """
    if not hasattr(commande_scenario, "cpt_Fx_rov"):
        commande_scenario.cpt_Fx_rov = 1
        commande_scenario.cpt_Fy_rov = 1
        commande_scenario.cpt_dL_dt = 1
        commande_scenario.cpt_Vx_bateau = 1

    if t == 0:
        commande_scenario.cpt_Fx_rov = 1
        commande_scenario.cpt_Fy_rov = 1
        commande_scenario.cpt_dL_dt = 1
        commande_scenario.cpt_Vx_bateau = 1

    counters = {
        "Fx_rov": "cpt_Fx_rov",
        "Fy_rov": "cpt_Fy_rov",
        "dL_dt": "cpt_dL_dt",
        "Vx_bateau": "cpt_Vx_bateau",
    }
    counter_name = counters.get(param_commande)
    if counter_name is None:
        trace_print(8, f"Erreur: param_commande invalide: {param_commande!r}")
        raise SystemExit(1)

    criteria = _parse_scenario_criteria(sc)
    k = getattr(commande_scenario, counter_name)
    if len(criteria) < k:
        return None

    prefix, op, y_val, z_val = criteria[k - 1]
    values = {"t": t, "l": L, "p": p}
    x_val = values[prefix]

    if (op == ">" and x_val > y_val) or (op == "<" and x_val < y_val):
        setattr(commande_scenario, counter_name, k + 1)
        return z_val

    return None


def auto_L_1(
    t: float,
    p: float,
    L: float,
    Fx_rov: float,
    Fy_rov: float,
    T: float | None,
    Trupt: float | None = None,
    Tcible: float | None = None,
    K: int = 10,
    N: int = 20,
) -> float:
    """
    Calcule une consigne automatique pour dL/dt à partir de l'historique.
    """
    if not hasattr(auto_L_1, "_history"):
        auto_L_1._history = []

    if Trupt is None:
        try:
            from src.utils.parameters import get_default_parameters

            params = get_default_parameters()
            Trupt = float(params.get("cable", {}).get("tension_rupture", 50.0))
        except Exception:
            Trupt = 50.0
    if Tcible is None:
        Tcible = Trupt / 2.0

    history = auto_L_1._history
    history.insert(
        0,
        {
            "t": float(t),
            "p": float(p),
            "L": float(L),
            "Fx_rov": float(Fx_rov),
            "Fy_rov": float(Fy_rov),
            "T": None if T is None else float(T),
            "dt": None,
            "dL": None,
            "dT": None,
            "dL_dt": None
        },
    )
    if len(history) > K + 1:
        del history[K + 1 :]

    if len(history) ==1:
        return 0.0


    curr = history[0]
    prev = history[1]
    curr["dL"] = curr["L"] - prev["L"]
    curr["dT"] = curr["T"] - prev["T"]
    curr["dt"] = curr["t"] - prev["t"]
    curr["dL_dt"] = curr["dL"] / curr["dt"] if curr["dt"] != 0 else None

    
    
    if abs(T - Tcible)  < 1:
        return 0.0
    if abs(Tcible - prev["T"]) < 1:
        return 0.0

    z = (T-Tcible)/abs(prev["T"]-Tcible)
    if z < -2:
        ret = -2 * curr["dL_dt"] -0.1
    elif z < -1:
        ret = -1 * abs(curr["dL_dt"]) -0.1
    elif z < -0.5:
        ret = -0.5 * abs(curr["dL_dt"]) -0.1
    elif z < 0:
        ret = -0.25
    elif z < 0.5:
        ret = 0.25 * abs(curr["dL_dt"]) +0.1
    elif z < 1:
        ret = 0.5 * abs(curr["dL_dt"]) +0.1
    elif z < 2:
        ret = 1 * abs(curr["dL_dt"]) +0.1
    else:
        ret = 2 * abs(curr["dL_dt"]) +0.1
   
    ret = ret * 1/abs(ret)
    
    trace_print(
        10,
        f"t: {t:.2f}, dt: {curr['dt']:.2f}, dL_dt: {curr['dL_dt']:.2f}, "
        f"T: {T:.2f}, Tcible: {Tcible:.2f}, z: {z:.2f}, ret: {ret:.2f}"
    )
    return ret


def auto_L_2(
    t: float,
    p: float,
    L: float,
    Fx_rov: float,
    Fy_rov: float,
    T: float | None,
    Trupt: float | None = None,
    Tcible: float | None = None,
    K: int = 10,
    N: int = 20,
) -> float:
    """
    Copie de auto_L_1 pour essais de lois de commande alternatives.
    """
    if not hasattr(auto_L_2, "_history"):
        auto_L_2._history = []

    if Trupt is None:
        try:
            from src.utils.parameters import get_default_parameters

            params = get_default_parameters()
            Trupt = float(params.get("cable", {}).get("tension_rupture", 50.0))
        except Exception:
            Trupt = 50.0
    if Tcible is None:
        Tcible = Trupt / 2.0

    history = auto_L_2._history
    history.insert(
        0,
        {
            "t": float(t),
            "p": float(p),
            "L": float(L),
            "Fx_rov": float(Fx_rov),
            "Fy_rov": float(Fy_rov),
            "T": None if T is None else float(T),
            "dt": None,
            "dL": None,
            "dT": None,
            "dL_dt": None,
        },
    )

    curr = history[0]
    prev = history[1]
    curr["dL"] = curr["L"] - prev["L"]
    curr["dT"] = curr["T"] - prev["T"]
    curr["dt"] = curr["t"] - prev["t"]
    curr["dL_dt"] = curr["dL"] / curr["dt"] if curr["dt"] != 0 else 0

    if len(history) > K + 1:
        del history[K + 1 :]

    if len(history) < K:
        trace_print(10, f"Moins de K itérations, retourne dL_dt: {curr['dL_dt']:.2f}")
        return curr["dL_dt"] 

    avg_L = sum(item["L"] for item in history[:K]) / K
    avg_T = sum(item["T"] for item in history[:K]) / K
    delta_L = history[-1]["L"] - history[0]["L"]
    delta_T = history[-1]["T"] - history[0]["T"]

    if abs(delta_L) < 1e-1:
        return 0.2
        
    est_dT_dL = delta_T / delta_L
    
    ret  = (Tcible - T) / (2*est_dT_dL) /10
    ret = max(min(ret,2), -2)
    trace_print(
        10,
        f"t: {t:.2f}, L: {L:.2f}, est_dT_dL: {est_dT_dL:.2f}, "
        f"T: {T:.2f}, Tcible: {Tcible:.2f}, ret: {ret:.2f}"
    )
    return ret
