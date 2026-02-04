"""Validation de la syntaxe des scénarios."""
from __future__ import annotations

import re

from src.utils.logger import trace_print


_ALLOWED_PREFIXES = {"t", "l", "p"}
_NUMBER_REGEX = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
_EVT_NAME_REGEX = r"e[A-Za-z0-9_]*"
_CRIT_PREFIX_REGEX = r"[tLlPp]"
_COUPLE_NUMERIC_REGEX = re.compile(
    rf"\(\s*(?P<prefix>{_CRIT_PREFIX_REGEX})\s*(?P<op>[<>])\s*(?P<y>{_NUMBER_REGEX})\s*:"
    rf"\s*(?P<z>{_NUMBER_REGEX})\s*\)"
)
_COUPLE_EMIT_REGEX = re.compile(
    rf"\(\s*(?P<prefix>{_CRIT_PREFIX_REGEX})\s*(?P<op>[<>])\s*(?P<y>{_NUMBER_REGEX})\s*:"
    rf"\s*(?P<evt>{_EVT_NAME_REGEX})\s*\)"
)
_COUPLE_EVENT_REGEX = re.compile(
    rf"\(\s*(?P<evt>{_EVT_NAME_REGEX})\s*:\s*(?P<z>{_NUMBER_REGEX})\s*\)"
)
_COUPLE_EVENT_DELAY_REGEX = re.compile(
    rf"\(\s*(?P<evt>{_EVT_NAME_REGEX})\s*>\s*(?P<y>{_NUMBER_REGEX})\s*:"
    rf"\s*(?P<z>{_NUMBER_REGEX})\s*\)"
)
_COUPLE_ALWAYS_REGEX = re.compile(
    rf"\(\s*:\s*(?P<z>{_NUMBER_REGEX})\s*\)"
)
_COUPLE_ALWAYS_EMIT_REGEX = re.compile(
    rf"\(\s*:\s*(?P<evt>{_EVT_NAME_REGEX})\s*\)"
)


def verifier_syntaxe_scenario(scen: str) -> bool:
    """
    Valide une chaîne de scénario.

    Un scénario est une suite de couples pouvant prendre les formes :
    - (x>y:z) ou (x<y:z) avec x ∈ {"t", "l", "p"} et y/z numériques
    - (x>y:evt) ou (x<y:evt) où evt est un nom d'événement valide
    - (evt>x:z) où evt est un nom d'événement valide et x/z numériques
    - (evt:z) où evt est un nom d'événement valide et z numérique (raccourci de evt>0)
    - (:z) où le critère est vide (toujours vrai) et z numérique
    - (:evt) où le critère est vide (toujours vrai) et evt valide
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

        match = _COUPLE_EMIT_REGEX.match(text, pos)
        if match:
            if match.group("prefix").lower() not in _ALLOWED_PREFIXES:
                return False
            try:
                float(match.group("y"))
            except ValueError:
                return False
            pos = match.end()
            continue

        match = _COUPLE_NUMERIC_REGEX.match(text, pos)
        if match:
            if match.group("prefix").lower() not in _ALLOWED_PREFIXES:
                return False
            try:
                float(match.group("y"))
                float(match.group("z"))
            except ValueError:
                return False
            pos = match.end()
            continue

        match = _COUPLE_EVENT_DELAY_REGEX.match(text, pos)
        if match:
            try:
                float(match.group("y"))
                float(match.group("z"))
            except ValueError:
                return False
            pos = match.end()
            continue

        match = _COUPLE_EVENT_REGEX.match(text, pos)
        if match:
            try:
                float(match.group("z"))
            except ValueError:
                return False
            pos = match.end()
            continue

        match = _COUPLE_ALWAYS_REGEX.match(text, pos)
        if match:
            try:
                float(match.group("z"))
            except ValueError:
                return False
            pos = match.end()
            continue

        match = _COUPLE_ALWAYS_EMIT_REGEX.match(text, pos)
        if match:
            pos = match.end()
            continue

        return False

    return True


def _parse_scenario_criteria(scen: str) -> list[tuple]:
    if scen is None:
        return []

    text = str(scen).strip()
    if text == "":
        return []

    criteria: list[tuple] = []
    pos = 0
    length = len(text)
    while True:
        while pos < length and text[pos].isspace():
            pos += 1
        if pos >= length:
            return criteria

        match = _COUPLE_EMIT_REGEX.match(text, pos)
        if match:
            prefix = match.group("prefix").lower()
            if prefix not in _ALLOWED_PREFIXES:
                return []
            try:
                y_val = float(match.group("y"))
            except ValueError:
                return []
            criteria.append(("emit", prefix, match.group("op"), y_val, match.group("evt")))
            pos = match.end()
            continue

        match = _COUPLE_NUMERIC_REGEX.match(text, pos)
        if match:
            prefix = match.group("prefix").lower()
            if prefix not in _ALLOWED_PREFIXES:
                return []
            try:
                y_val = float(match.group("y"))
                z_val = float(match.group("z"))
            except ValueError:
                return []
            criteria.append(("numeric", prefix, match.group("op"), y_val, z_val))
            pos = match.end()
            continue

        match = _COUPLE_EVENT_DELAY_REGEX.match(text, pos)
        if match:
            try:
                y_val = float(match.group("y"))
                z_val = float(match.group("z"))
            except ValueError:
                return []
            criteria.append(("event_delay", match.group("evt"), y_val, z_val))
            pos = match.end()
            continue

        match = _COUPLE_EVENT_REGEX.match(text, pos)
        if match:
            try:
                z_val = float(match.group("z"))
            except ValueError:
                return []
            criteria.append(("event_delay", match.group("evt"), 0.0, z_val))
            pos = match.end()
            continue

        match = _COUPLE_ALWAYS_REGEX.match(text, pos)
        if match:
            try:
                z_val = float(match.group("z"))
            except ValueError:
                return []
            criteria.append(("always", z_val))
            pos = match.end()
            continue

        match = _COUPLE_ALWAYS_EMIT_REGEX.match(text, pos)
        if match:
            criteria.append(("emit_always", match.group("evt")))
            pos = match.end()
            continue

        return []


def _format_num(value: float) -> str:
    return f"{value:g}"


def _format_criterion(criterion: tuple) -> str:
    kind = criterion[0]
    if kind == "numeric":
        _, prefix, op, y_val, z_val = criterion
        return f"({prefix}{op}{_format_num(y_val)}:{_format_num(z_val)})"
    if kind == "emit":
        _, prefix, op, y_val, evt = criterion
        return f"({prefix}{op}{_format_num(y_val)}:{evt})"
    if kind == "event_delay":
        _, evt_name, delay, z_val = criterion
        if float(delay) == 0.0:
            return f"({evt_name}:{_format_num(z_val)})"
        return f"({evt_name}>{_format_num(delay)}:{_format_num(z_val)})"
    if kind == "always":
        _, z_val = criterion
        return f"(:{_format_num(z_val)})"
    if kind == "emit_always":
        _, evt_name = criterion
        return f"(:{evt_name})"
    return "(crit:val)"


def _scenario_label(param_commande: str) -> str:
    labels = {
        "Fx_rov": "Fx",
        "Fy_rov": "Fy",
        "Vx_bateau": "Bat",
        "dL_dt": "Moul",
    }
    return labels.get(param_commande, param_commande)


def commande_scenario(
    param_commande: str,
    sc: str,
    t: float,
    L: float,
    p: float,
    T_bat: float | None = None,
    trigger_sink: list[str] | None = None,
) -> float | None:
    """
    Calcule une consigne à partir d'un scénario et d'un paramètre de commande.
    """
    if not hasattr(commande_scenario, "cpt_Fx_rov"):
        commande_scenario.cpt_Fx_rov = 1
        commande_scenario.cpt_Fy_rov = 1
        commande_scenario.cpt_dL_dt = 1
        commande_scenario.cpt_Vx_bateau = 1
        commande_scenario._events = {}
        commande_scenario._last_t = None
        commande_scenario._triggered = {}

    if t == 0 and commande_scenario._last_t != 0:
        commande_scenario.cpt_Fx_rov = 1
        commande_scenario.cpt_Fy_rov = 1
        commande_scenario.cpt_dL_dt = 1
        commande_scenario.cpt_Vx_bateau = 1
        commande_scenario._events = {}
        commande_scenario._triggered = {}
    commande_scenario._last_t = t

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

    criterion = criteria[k - 1]
    trigger_key = (param_commande, k)
    scenario_name = _scenario_label(param_commande)
    trigger_text = f"{scenario_name}:{_format_criterion(criterion)}"
    triggered = commande_scenario._triggered
    kind = criterion[0]
    if kind in {"numeric", "emit"}:
        _, prefix, op, y_val, payload = criterion
        values = {"t": t, "l": L, "p": p}
        x_val = values[prefix]
        if (op == ">" and x_val > y_val) or (op == "<" and x_val < y_val):
            setattr(commande_scenario, counter_name, k + 1)
            if kind == "numeric":
                if trigger_key not in triggered:
                    triggered[trigger_key] = float(t)
                    trace_print(
                        10,
                        f"[SCEN] t={t:.2f} scen={scenario_name} "
                        f"crit={_format_criterion(criterion)} T_bat={T_bat} "
                        f"p={p:.2f} L={L:.2f}"
                    )
                    if trigger_sink is not None:
                        trigger_sink.append(trigger_text)
                return float(payload)
            if trigger_key not in triggered:
                triggered[trigger_key] = float(t)
                trace_print(
                    10,
                    f"[SCEN] t={t:.2f} scen={scenario_name} "
                    f"crit={_format_criterion(criterion)} T_bat={T_bat} "
                    f"p={p:.2f} L={L:.2f}"
                )
                if trigger_sink is not None:
                    trigger_sink.append(trigger_text)
            if payload not in commande_scenario._events:
                commande_scenario._events[payload] = float(t)
        return None

    if kind == "event_delay":
        _, evt_name, delay, z_val = criterion
        evt_time = commande_scenario._events.get(evt_name)
        if evt_time is not None and (t - evt_time) > delay:
            setattr(commande_scenario, counter_name, k + 1)
            if trigger_key not in triggered:
                triggered[trigger_key] = float(t)
                trace_print(
                    10,
                    f"[SCEN] t={t:.2f} scen={scenario_name} "
                    f"crit={_format_criterion(criterion)} T_bat={T_bat} "
                    f"p={p:.2f} L={L:.2f}"
                )
                if trigger_sink is not None:
                    trigger_sink.append(trigger_text)
            return z_val

    if kind == "always":
        _, z_val = criterion
        setattr(commande_scenario, counter_name, k + 1)
        if trigger_key not in triggered:
            triggered[trigger_key] = float(t)
            trace_print(
                10,
                f"[SCEN] t={t:.2f} scen={scenario_name} "
                f"crit={_format_criterion(criterion)} T_bat={T_bat} "
                f"p={p:.2f} L={L:.2f}"
            )
            if trigger_sink is not None:
                trigger_sink.append(trigger_text)
        return z_val

    if kind == "emit_always":
        _, evt_name = criterion
        setattr(commande_scenario, counter_name, k + 1)
        if trigger_key not in triggered:
            triggered[trigger_key] = float(t)
            trace_print(
                10,
                f"[SCEN] t={t:.2f} scen={scenario_name} "
                f"crit={_format_criterion(criterion)} T_bat={T_bat} "
                f"p={p:.2f} L={L:.2f}"
            )
            if trigger_sink is not None:
                trigger_sink.append(trigger_text)
        if evt_name not in commande_scenario._events:
            commande_scenario._events[evt_name] = float(t)
        return None

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
) -> tuple[float, str]:
    """
    Calcule une consigne automatique pour dL/dt à partir de l'historique.
    Retourne (ret, explication).
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

    if len(history) == 1:
        return 0.0, "INIT"


    curr = history[0]
    prev = history[1]
    curr["dL"] = curr["L"] - prev["L"]
    curr["dT"] = curr["T"] - prev["T"]
    curr["dt"] = curr["t"] - prev["t"]
    curr["dL_dt"] = curr["dL"] / curr["dt"] if curr["dt"] != 0 else None

    
    
    if abs(T - Tcible) < 1:
        return 0.0, "T_CLOSE"
    if abs(Tcible - prev["T"]) < 1:
        return 0.0, "TCIBLE_PREV_CLOSE"

    z = (T-Tcible)/abs(prev["T"]-Tcible)
    if z < -2:
        ret = -2 * curr["dL_dt"] - 0.1
        exp = "Z_LT_-2"
    elif z < -1:
        ret = -1 * abs(curr["dL_dt"]) - 0.1
        exp = "Z_LT_-1"
    elif z < -0.5:
        ret = -0.5 * abs(curr["dL_dt"]) - 0.1
        exp = "Z_LT_-0_5"
    elif z < 0:
        ret = -0.25
        exp = "Z_LT_0"
    elif z < 0.5:
        ret = 0.25 * abs(curr["dL_dt"]) + 0.1
        exp = "Z_LT_0_5"
    elif z < 1:
        ret = 0.5 * abs(curr["dL_dt"]) + 0.1
        exp = "Z_LT_1"
    elif z < 2:
        ret = 1 * abs(curr["dL_dt"]) + 0.1
        exp = "Z_LT_2"
    else:
        ret = 2 * abs(curr["dL_dt"]) + 0.1
        exp = "Z_GE_2"
   
    ret = ret * 1/abs(ret)
    
    trace_print(
        3,
        f"t: {t:.2f}, dt: {curr['dt']:.2f}, dL_dt: {curr['dL_dt']:.2f}, "
        f"T: {T:.2f}, Tcible: {Tcible:.2f}, z: {z:.2f}, ret: {ret:.2f}"
    )
    return ret, exp


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
) -> tuple[float, str]:
    """
    Copie de auto_L_1 pour essais de lois de commande alternatives.
    Retourne (ret, explication).
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

    if len(history) < 2:
        return 0.0, "HISTORY_LT2"

    J = min(K, len(history))
    avg_L = sum(item["L"] for item in history[:J]) / J
    avg_T = sum(item["T"] for item in history[:J]) / J
    delta_L = history[-1]["L"] - history[0]["L"]
    delta_T = history[-1]["T"] - history[0]["T"]

    if abs(delta_L) < 1e-2:
        if curr["dL_dt"] > 0:
            trace_print(
                10,
                f"t: {t:.2f}, delta_L proche de 0+: {delta_L:.2f}, -> retourne {curr['dL_dt'] - 0.1:.2f}"
            )
            return curr["dL_dt"] - 0.1, "DELTA_L_0+"
        trace_print(
            10,
            f"t: {t:.2f}, delta_L proche de 0-: {delta_L:.2f}, -> retourne {curr['dL_dt'] + 0.1:.2f}"
        )
        return curr["dL_dt"] + 0.11, "DELTA_L_0-"
        
    est_dT_dL = delta_T / delta_L
    
    ret = abs((T - Tcible) / Tcible) * ((T - Tcible) / (2 * est_dT_dL))
    ret_corrige = max(min(ret, 2.02), -2.02)
    trace_print(
        10,
        f"t: {t:.2f}, L: {L:.2f}, est_dT_dL: {est_dT_dL:.2f}, "
        f"T: {T:.2f}, Tcible: {Tcible:.2f}, ret: {ret:.2f}, ret_corrige: {ret_corrige:.2f}"
    )
    return ret_corrige, "MAIN"


def auto_L_3(
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
) -> tuple[float, str]:
    """
    Copie de auto_L_2 pour essais de lois de commande alternatives.
    Retourne (ret, explication).
    """
    if not hasattr(auto_L_3, "_history"):
        auto_L_3._history = []

    if Trupt is None:
        try:
            from src.utils.parameters import get_default_parameters

            params = get_default_parameters()
            Trupt = float(params.get("cable", {}).get("tension_rupture", 50.0))
        except Exception:
            Trupt = 50.0
    if Tcible is None:
        Tcible = Trupt / 2.0

    history = auto_L_3._history
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

    if len(history) < 2:
        return 0.0, "HISTORY_LT2"

    J = min(K, len(history))
    avg_L = sum(item["L"] for item in history[:J]) / J
    avg_T = sum(item["T"] for item in history[:J]) / J
    delta_L = history[-1]["L"] - history[0]["L"]
    delta_T = history[-1]["T"] - history[0]["T"]

    if abs(delta_L) < 1e-5:
        if curr["dL_dt"] > 0:
            trace_print(
                10,
                f"t: {t:.2f}, delta_L proche de 0+: {delta_L:.2f}, -> retourne {curr['dL_dt'] - 0.1:.2f}"
            )
            return curr["dL_dt"] - 0.1, "DELTA_L_0+"
        trace_print(
            10,
            f"t: {t:.2f}, delta_L proche de 0-: {delta_L:.2f}, -> retourne {curr['dL_dt'] + 0.1:.2f}"
        )
        return curr["dL_dt"] + 0.11, "DELTA_L_0-"

    est_dT_dL = delta_T / delta_L

    ret = abs((T - Tcible) / Tcible) * ((T - Tcible) / (2 * est_dT_dL))
    ret_corrige = max(min(ret, 2.02), -2.02)
    trace_print(
        10,
        f"t: {t:.2f}, L: {L:.2f}, est_dT_dL: {est_dT_dL:.2f}, "
        f"T: {T:.2f}, Tcible: {Tcible:.2f}, ret: {ret:.2f}, ret_corrige: {ret_corrige:.2f}"
    )
    return ret_corrige, "MAIN"
