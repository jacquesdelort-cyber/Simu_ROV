"""Validation de la syntaxe des scénarios."""
from __future__ import annotations

import re
import math
import numpy as np

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


def compute_rov_terminal_vy(Fy: float, params: dict | None = None) -> float:
    """
    Calcule la vitesse verticale limite (vy) pour une force Fy donnée
    en tenant compte de la traînée verticale du ROV.
    """
    if Fy == 0:
        return 0.0
    if params is None:
        try:
            from src.utils.parameters import get_default_parameters

            params = get_default_parameters()
        except Exception:
            return 0.0

    rov = params.get("rov", {})
    env = params.get("environment", {})
    rho = float(env.get("rho_eau", 1025.0))
    cy = float(rov.get("Cy", 1.0))
    a = float(rov.get("a", 0.5))
    b = float(rov.get("b", 1.0))
    area = a * b
    denom = 0.5 * rho * cy * area
    if denom <= 0.0:
        return 0.0

    vy_mag = math.sqrt(abs(Fy) / denom)
    return math.copysign(vy_mag, Fy)


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
    Gamma_moulinet_max: float | None = None,
    data: dict | None = None,
    K: int = 10,
    N: int = 20,
) -> tuple[float, str]:
    """
    Calcule une consigne automatique pour dL/dt à partir de l'historique.
    Priorités:
    1) ne pas dépasser la tension de rupture,
    2) respecter l'intention du pilote (Fy_rov < 0 => descendre, éviter câble tendu),
    3) rester en mode caténaire si possible,
    4) éviter les à-coups (lissage + limite d'accélération).
    Retourne (ret, explication).
    """
    if not hasattr(auto_L_1, "_history"):
        auto_L_1._history = []

    def _apply_moulinet_accel_limit(ret, curr, prev, gamma_max):
        if gamma_max is None:
            return ret, False
        dt = curr.get("dt")
        if not dt:
            return ret, False
        prev_cmd = prev.get("dl_dt_cmd_prev")
        if prev_cmd is None:
            prev_cmd = curr.get("dL_dt")
            if prev_cmd is None:
                prev_cmd = 0.0
        max_delta = float(gamma_max) * dt
        limited = prev_cmd + max(min(ret - prev_cmd, max_delta), -max_delta)
        return limited, abs(limited - ret) > 1e-12

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
            "dL_dt": None,
            "y_rov": None,
            "dy_rov": None,
            "vy_rov": None,
            "dl_dt_cmd_prev": None,
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

    
    
    # Priorité 1-2: sécurité rupture + intention pilote (descente)
    t_soft = 0.90 * Trupt
    t_hard = 0.98 * Trupt
    if curr["T"] is not None:
        if curr["T"] >= t_hard:
            ret, exp = 1.0, "T_HARD"
        elif curr["T"] >= t_soft:
            ramp = (curr["T"] - t_soft) / max(t_hard - t_soft, 1e-6)
            ret, exp = 0.5 + 0.5 * max(min(ramp, 1.0), 0.0), "T_SOFT"
        elif Fy_rov < 0:
            # Éviter le blocage en câble tendu quand le pilote veut descendre
            if curr["T"] >= Tcible:
                ret, exp = 0.3, "PILOT_DESC_TAUT"
            else:
                ret, exp = 0.1, "PILOT_DESC"
        else:
            ret = None
            exp = ""
        if ret is not None:
            ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_max)
            if limited:
                exp = f"{exp}_ACC"
            curr["dl_dt_cmd_prev"] = ret
            return ret, exp

    if abs(T - Tcible) < 1:
        ret, exp = 0.0, "T_CLOSE"
        ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_max)
        if limited:
            exp = f"{exp}_ACC"
        curr["dl_dt_cmd_prev"] = ret
        return ret, exp
    if abs(Tcible - prev["T"]) < 1:
        ret, exp = 0.0, "TCIBLE_PREV_CLOSE"
        ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_max)
        if limited:
            exp = f"{exp}_ACC"
        curr["dl_dt_cmd_prev"] = ret
        return ret, exp

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
    
    ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_max)
    if limited:
        exp = f"{exp}_ACC"
    curr["dl_dt_cmd_prev"] = ret
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
    Gamma_moulinet_max: float | None = None,
    data: dict | None = None,
    K: int = 10,
    N: int = 20,
) -> tuple[float, str]:
    """
    Copie de auto_L_1 pour essais de lois de commande alternatives.
    Priorités:
    1) ne pas dépasser la tension de rupture,
    2) respecter l'intention du pilote (Fy_rov < 0 => descendre, éviter câble tendu),
    3) rester en mode caténaire si possible,
    4) éviter les à-coups (lissage + limite d'accélération).
    Retourne (ret, explication).
    """
    if not hasattr(auto_L_2, "_history"):
        auto_L_2._history = []

    def _apply_moulinet_accel_limit(ret, curr, prev, gamma_max):
        if gamma_max is None:
            return ret, False
        dt = curr.get("dt")
        if not dt:
            return ret, False
        prev_cmd = prev.get("dl_dt_cmd_prev")
        if prev_cmd is None:
            prev_cmd = curr.get("dL_dt")
            if prev_cmd is None:
                prev_cmd = 0.0
        max_delta = float(gamma_max) * dt
        limited = prev_cmd + max(min(ret - prev_cmd, max_delta), -max_delta)
        return limited, abs(limited - ret) > 1e-12

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
            "dl_dt_cmd_prev": None,
        },
    )

    curr = history[0]
    prev = history[1]
    curr["dL"] = curr["L"] - prev["L"]
    curr["dT"] = curr["T"] - prev["T"]
    curr["dt"] = curr["t"] - prev["t"]
    curr["dL_dt"] = curr["dL"] / curr["dt"] if curr["dt"] != 0 else 0

    # Priorité 1-2: sécurité rupture + intention pilote (descente)
    t_soft = 0.90 * Trupt
    t_hard = 0.98 * Trupt
    if curr["T"] is not None:
        if curr["T"] >= t_hard:
            ret, exp = 1.0, "T_HARD"
        elif curr["T"] >= t_soft:
            ramp = (curr["T"] - t_soft) / max(t_hard - t_soft, 1e-6)
            ret, exp = 0.5 + 0.5 * max(min(ramp, 1.0), 0.0), "T_SOFT"
        elif Fy_rov < 0:
            if curr["T"] >= Tcible:
                ret, exp = 0.3, "PILOT_DESC_TAUT"
            else:
                ret, exp = 0.1, "PILOT_DESC"
        else:
            ret = None
            exp = ""
        if ret is not None:
            ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_max)
            if limited:
                exp = f"{exp}_ACC"
            curr["dl_dt_cmd_prev"] = ret
            return ret, exp

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
            ret, exp = curr["dL_dt"] - 0.1, "DELTA_L_0+"
            trace_print(
                10,
                f"t: {t:.2f}, delta_L proche de 0+: {delta_L:.2f}, -> retourne {ret:.2f}"
            )
        else:
            ret, exp = curr["dL_dt"] + 0.11, "DELTA_L_0-"
            trace_print(
                10,
                f"t: {t:.2f}, delta_L proche de 0-: {delta_L:.2f}, -> retourne {ret:.2f}"
            )
        ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_max)
        if limited:
            exp = f"{exp}_ACC"
        curr["dl_dt_cmd_prev"] = ret
        return ret, exp
        
    est_dT_dL = delta_T / delta_L
    
    ret = abs((T - Tcible) / Tcible) * ((T - Tcible) / (2 * est_dT_dL))
    ret_corrige = max(min(ret, 2.02), -2.02)
    ret, limited = _apply_moulinet_accel_limit(ret_corrige, curr, prev, Gamma_moulinet_max)
    exp = "MAIN_ACC" if limited else "MAIN"
    curr["dl_dt_cmd_prev"] = ret
    trace_print(
        10,
        f"t: {t:.2f}, L: {L:.2f}, est_dT_dL: {est_dT_dL:.2f}, "
        f"T: {T:.2f}, Tcible: {Tcible:.2f}, ret: {ret:.2f}, ret_corrige: {ret_corrige:.2f}"
    )
    return ret, exp


def auto_L_3(
    t: float,
    p: float,
    L: float,
    Fx_rov: float,
    Fy_rov: float,
    T: float | None,
    Trupt: float | None = None,
    Tcible: float | None = None,
    Gamma_moulinet_max: float | None = None,
    data: dict | None = None,
    K: int = 10,
    N: int = 20,
) -> tuple[float, str]:
    """
    Copie de auto_L_2 pour essais de lois de commande alternatives.
    Priorités:
    1) ne pas dépasser la tension de rupture,
    2) respecter l'intention du pilote (Fy_rov < 0 => descendre, éviter câble tendu),
    3) rester en mode caténaire si possible,
    4) éviter les à-coups (lissage + limite d'accélération).
    Retourne (ret, explication).
    """
    if not hasattr(auto_L_3, "_history"):
        auto_L_3._history = []

    def _apply_moulinet_accel_limit(ret, curr, prev, gamma_max):
        if gamma_max is None:
            return ret, False
        dt = curr.get("dt")
        if not dt:
            return ret, False
        prev_cmd = prev.get("dl_dt_cmd_prev")
        if prev_cmd is None:
            prev_cmd = curr.get("dL_dt")
            if prev_cmd is None:
                prev_cmd = 0.0
        max_delta = float(gamma_max) * dt
        limited = prev_cmd + max(min(ret - prev_cmd, max_delta), -max_delta)
        return limited, abs(limited - ret) > 1e-12

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
            "dl_dt_cmd_prev": None,
        },
    )

    curr = history[0]
    prev = history[1]
    curr["dL"] = curr["L"] - prev["L"]
    curr["dT"] = curr["T"] - prev["T"]
    curr["dt"] = curr["t"] - prev["t"]
    curr["dL_dt"] = curr["dL"] / curr["dt"] if curr["dt"] != 0 else 0

    # Priorité 1-2: sécurité rupture + intention pilote (descente)
    t_soft = 0.90 * Trupt
    t_hard = 0.98 * Trupt
    if curr["T"] is not None:
        if curr["T"] >= t_hard:
            ret, exp = 1.0, "T_HARD"
        elif curr["T"] >= t_soft:
            ramp = (curr["T"] - t_soft) / max(t_hard - t_soft, 1e-6)
            ret, exp = 0.5 + 0.5 * max(min(ramp, 1.0), 0.0), "T_SOFT"
        elif Fy_rov < 0:
            if curr["T"] >= Tcible:
                ret, exp = 0.3, "PILOT_DESC_TAUT"
            else:
                ret, exp = 0.1, "PILOT_DESC"
        else:
            ret = None
            exp = ""
        if ret is not None:
            ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_max)
            if limited:
                exp = f"{exp}_ACC"
            curr["dl_dt_cmd_prev"] = ret
            return ret, exp

    if len(history) > K + 1:
        del history[K + 1 :]

    if len(history) < 2:
        return 0.0, "HISTORY_LT2"

    J = min(K, len(history))
    avg_L = sum(item["L"] for item in history[:J]) / J
    avg_T = sum(item["T"] for item in history[:J]) / J
    delta_L = history[-1]["L"] - history[0]["L"]
    delta_T = history[-1]["T"] - history[0]["T"]

    if abs(delta_L) < 1e-4:
        if curr["dL_dt"] > 0:
            ret, exp = curr["dL_dt"] - 0.1, "DELTA_L_0+"
            trace_print(
                10,
                f"t: {t:.2f}, delta_L proche de 0+: {delta_L:.2f}, -> retourne {ret:.2f}"
            )
        else:
            ret, exp = curr["dL_dt"] + 0.11, "DELTA_L_0-"
            trace_print(
                10,
                f"t: {t:.2f}, delta_L proche de 0-: {delta_L:.2f}, -> retourne {ret:.2f}"
            )
        ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_max)
        if limited:
            exp = f"{exp}_ACC"
        curr["dl_dt_cmd_prev"] = ret
        return ret, exp

    est_dT_dL = delta_T / delta_L

    ret = abs((T - Tcible) / Tcible) * ((T - Tcible) / (2 * est_dT_dL))
    ret_corrige = max(min(ret, 2.02), -2.02)
    ret, limited = _apply_moulinet_accel_limit(ret_corrige, curr, prev, Gamma_moulinet_max)
    exp = "MAIN_ACC" if limited else "MAIN"
    curr["dl_dt_cmd_prev"] = ret
    trace_print(
        10,
        f"t: {t:.2f}, L: {L:.2f}, est_dT_dL: {est_dT_dL:.2f}, "
        f"T: {T:.2f}, Tcible: {Tcible:.2f}, ret: {ret:.2f}, ret_corrige: {ret_corrige:.2f}"
    )
    return ret, exp


def auto_L_4(
    t: float,
    p: float,
    L: float,
    Fx_rov: float,
    Fy_rov: float,
    T: float | None,
    Trupt: float | None = None,
    Tcible: float | None = None,
    Gamma_moulinet_max: float | None = None,
    data: dict | None = None,
    K: int = 10,
    N: int = 20,
) -> tuple[float, str]:
    """
    Copie de auto_L_3 pour essais de lois de commande alternatives.
    Priorités:
    1) ne pas dépasser la tension de rupture,
    2) respecter l'intention du pilote (Fy_rov < 0 => descendre, éviter câble tendu),
    3) rester en mode caténaire si possible,
    4) éviter les à-coups (lissage + limite d'accélération).
    Retourne (ret, explication).
    """
    if not hasattr(auto_L_4, "_history"):
        auto_L_4._history = []

    def _apply_moulinet_accel_limit(ret, curr, prev, gamma_max):
        if gamma_max is None:
            return ret, False
        dt = curr.get("dt")
        if not dt:
            return ret, False
        prev_cmd = prev.get("dl_dt_cmd_prev")
        if prev_cmd is None:
            prev_cmd = curr.get("dL_dt")
            if prev_cmd is None:
                prev_cmd = 0.0
        max_delta = float(gamma_max) * dt
        limited = prev_cmd + max(min(ret - prev_cmd, max_delta), -max_delta)
        return limited, abs(limited - ret) > 1e-12

    if Trupt is None:
        try:
            from src.utils.parameters import get_default_parameters

            params = get_default_parameters()
            Trupt = float(params.get("cable", {}).get("tension_rupture", 50.0))
        except Exception:
            Trupt = 50.0
    if Tcible is None:
        Tcible = Trupt / 2.0

    history = auto_L_4._history
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
            "y_rov": None,
            "dy_rov": None,
            "vy_rov": None,
            "dl_dt_cmd_prev": None,
        },
    )

    if len(history) < 2:
        trace_print(10, f"t: {t:.2f}, HISTORY < 2  -> retourne {0.0:.2f}")
        return 0.0, "HISTORY_LT2"

    curr = history[0]
    prev = history[1]
    curr["dL"] = curr["L"] - prev["L"]
    curr["dT"] = curr["T"] - prev["T"] if curr["T"] is not None and prev["T"] is not None else 0.0
    curr["dt"] = curr["t"] - prev["t"]
    curr["dL_dt"] = curr["dL"] / curr["dt"] if curr["dt"] != 0 else 0
    curr["y_rov"] = float(p)
    prev["y_rov"] = float(prev.get("p", prev.get("y_rov", p)))
    curr["dy_rov"] = curr["y_rov"] - prev["y_rov"]
    curr["vy_rov"] = curr["dy_rov"] / curr["dt"] if curr["dt"] != 0 else 0

    # Priorité 1-2: sécurité rupture + intention pilote (descente)
    t_soft = 0.90 * Trupt
    t_hard = 0.98 * Trupt
    if curr["T"] is not None:
        if curr["T"] >= t_hard:
            ret, exp = 1.0, "T_HARD"
        elif curr["T"] >= t_soft:
            ramp = (curr["T"] - t_soft) / max(t_hard - t_soft, 1e-6)
            ret, exp = 0.5 + 0.5 * max(min(ramp, 1.0), 0.0), "T_SOFT"
        elif Fy_rov < 0:
            if curr["T"] >= Tcible:
                ret, exp = 0.3, "PILOT_DESC_TAUT"
            else:
                ret, exp = 0.1, "PILOT_DESC"
        else:
            ret = None
            exp = ""
        if ret is not None:
            ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_max)
            if limited:
                exp = f"{exp}_ACC"
            curr["dl_dt_cmd_prev"] = ret
            return ret, exp

    if len(history) > K + 1:
        del history[K + 1 :]

    J = min(K, len(history))
    avg_L = np.mean([item["L"] for item in history[:J]])
    t_vals = [item["T"] for item in history[:J] if item["T"] is not None]
    avg_T = np.mean(t_vals) if t_vals else 0.0
    delta_L = history[-1]["L"] - history[0]["L"]
    t_last = history[-1]["T"]
    t_first = history[0]["T"]
    delta_T = (t_last - t_first) if t_last is not None and t_first is not None else 0.0

    # Cas marginal, on ne va pas pouvoir calculer dT/dL car dL est trop petit
    Vy_lim = compute_rov_terminal_vy(Fy_rov)
    
    if abs(delta_L) < 1e-4:
        if curr["dL_dt"] > 0:
            ret, exp = -Vy_lim - 0.05, "DELTA_L_0+"
            trace_print(
                10,
                f"0+: t: {t:.2f}, delta_L: {delta_L:.2f}, -> retourne {ret:.2f}"
            )
        else:
            ret, exp = -Vy_lim + 0.05, "DELTA_L_0-"
            trace_print(
                10,
                f"0-: t: {t:.2f}, delta_L: {delta_L:.2f}, -> retourne {ret:.2f}"
            )
        ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_max)
        if limited:
            exp = f"{exp}_ACC"
        curr["dl_dt_cmd_prev"] = ret
        return ret, exp

    # Cas principal
    if T is None or Tcible in (None, 0):
        ret, exp = 0.0, "NO_T"
        ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_max)
        if limited:
            exp = f"{exp}_ACC"
        curr["dl_dt_cmd_prev"] = ret
        trace_print(10, f"MAIN: t={t:.2f}, T/Tcible invalide -> retourne {ret:.2f}")
        return ret, exp

    est_dT_dL = delta_T / delta_L
    if abs(est_dT_dL)  < 1e-4:
        ret, exp = 0.0, "DTDL_0"
        ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_max)
        if limited:
            exp = f"{exp}_ACC"
        curr["dl_dt_cmd_prev"] = ret
        trace_print(10, f"MAIN: t={t:.2f}, est_dT_dL=0 -> retourne {ret:.2f}")
        return ret, exp

    ret1 = - 1.1*Vy_lim +  min(1, abs((T - Tcible) / Tcible)) * ((T - Tcible) / (2 * est_dT_dL)) 
    ret2 = curr["dL_dt"] + (ret1 - curr["dL_dt"]) / 5
    ret = max(min(ret2, 1.0), -1.0)

    ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_max)
    exp = "MAIN_ACC" if limited else "MAIN"
    curr["dl_dt_cmd_prev"] = ret
    trace_print(
        10,
        f"MAIN: t={t:.2f}, L={L: 6.2f}, est_dT_dL={est_dT_dL: 7.2f}, y_rov={p: 6.2f}, vy_rov={curr['vy_rov']: 6.2f}, Vy_lim={Vy_lim: 6.2f}, "
        f"T={T: 5.2f}, Tcible={Tcible: 5.2f}, ret1={ret1: 8.2f}, ret2={ret2: 6.2f}, ret={ret: 6.2f}"
    )
    return ret, exp


def auto_L_6(
    t: float,
    p: float,
    L: float,
    Fx_rov: float,
    Fy_rov: float,
    T: float | None,
    Trupt: float | None = None,
    Tcible: float | None = None,
    Gamma_moulinet_max: float | None = None,
    data: dict | None = None,
    K: int = 10,
    N: int = 10,
) -> tuple[float, str]:
    """
    Loi de commande dL/dt avec priorités:
    1) ne pas dépasser la tension de rupture,
    2) respecter l'intention du pilote (Fy_rov < 0 => descendre, éviter câble tendu),
    3) rester en mode caténaire si possible,
    4) éviter les à-coups (lissage + limite d'accélération).
    Retourne (ret, explication).
    """
    if not hasattr(auto_L_6, "_history"):
        auto_L_6._history = []

    if Trupt is None:
        try:
            from src.utils.parameters import get_default_parameters

            params = get_default_parameters()
            Trupt = float(params.get("cable", {}).get("tension_rupture", 50.0))
        except Exception:
            Trupt = 50.0
    if Tcible is None:
        Tcible = Trupt / 2.0
    if Gamma_moulinet_max is None:
        Gamma_moulinet_max = 0.5

    history = auto_L_6._history
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
            "dl_dt_cmd_prev": None,
        },
    )

    if len(history) < 2:
        return 0.0, "HISTORY_LT2"

    curr = history[0]
    prev = history[1]
    curr["dL"] = curr["L"] - prev["L"]
    curr["dT"] = curr["T"] - prev["T"] if curr["T"] is not None and prev["T"] is not None else 0.0
    curr["dt"] = curr["t"] - prev["t"]
    curr["dL_dt"] = curr["dL"] / curr["dt"] if curr["dt"] != 0 else 0.0
    curr["y_rov"] = float(p)
    prev["y_rov"] = float(prev.get("y_rov", p))
    curr["dy_rov"] = curr["y_rov"] - prev["y_rov"]
    curr["vy_rov"] = curr["dy_rov"] / curr["dt"] if curr["dt"] != 0 else 0.0
    prev["vy_rov"] = prev.get("vy_rov", 0.0)
    curr["gy_rov"] = curr["vy_rov"] - prev["vy_rov"]

    if data and data.get("vy_rov") and len(history) > 3:
        assert math.isclose(curr["vy_rov"], data["vy_rov"][-1]), "Erreur vy_rov"
        assert math.isclose(prev["vy_rov"], data["vy_rov"][-2]), "Erreur vy_rov"

    if len(history) > K + 1:
        del history[K + 1 :]

    if curr["T"] is None or Tcible in (None, 0):
        return 0.0, "NO_T"

    # Estimation locale dT/dL (si possible)
    est_dT_dL = None
    delta_L = history[-1]["L"] - history[0]["L"]
    t_last = history[-1]["T"]
    t_first = history[0]["T"]
    if delta_L != 0 and t_last is not None and t_first is not None:
        est_dT_dL = (t_last - t_first) / delta_L

    # Priorité 1: éviter Trupt
    t_soft = 0.90 * Trupt
    t_hard = 0.98 * Trupt
    if curr["T"] >= t_hard:
        desired = 1.0
        exp = "T_HARD"
    elif curr["T"] >= t_soft:
        ramp = (curr["T"] - t_soft) / max(t_hard - t_soft, 1e-6)
        desired = 0.5 + 0.5 * max(min(ramp, 1.0), 0.0)
        exp = "T_SOFT"
    else:
        # Priorité 2: respecter l'intention du pilote (descendre)
        if Fy_rov < 0:
            if curr["T"] >= Tcible:
                desired = 0.4
                exp = "PILOT_DESC_TAUT"
            else:
                desired = 0.2
                exp = "PILOT_DESC"
        else:
            # Priorité 3: rester en mode caténaire (tension modérée)
            T_catenary = 0.8 * Tcible
            if est_dT_dL is None or abs(est_dT_dL) < 1e-4:
                if curr["T"] > T_catenary:
                    desired = 0.2
                    exp = "CATENARY_SIGN_OUT"
                elif curr["T"] < 0.6 * T_catenary:
                    desired = -0.1
                    exp = "CATENARY_SIGN_IN"
                else:
                    desired = 0.0
                    exp = "CATENARY_HOLD"
            else:
                desired = -(curr["T"] - T_catenary) / (2.0 * est_dT_dL)
                exp = "CATENARY_REG"

    # Lissage de la commande
    prev_cmd = prev.get("dl_dt_cmd_prev")
    if prev_cmd is None:
        prev_cmd = curr["dL_dt"] if curr["dL_dt"] is not None else 0.0
    ret1 = prev_cmd + (desired - prev_cmd) / max(N, 1)

    # Limitation d'accélération du moulinet
    if curr["dt"] and Gamma_moulinet_max is not None:
        max_delta = float(Gamma_moulinet_max) * curr["dt"]
        ret1 = prev_cmd + max(min(ret1 - prev_cmd, max_delta), -max_delta)
        exp = f"{exp}_ACC"

    ret = max(min(ret1, 1.0), -1.0)
    curr["dl_dt_cmd_prev"] = ret

    def _last_val(key):
        if not data:
            return None
        vals = data.get(key)
        return vals[-1] if isinstance(vals, list) and vals else None

    def _fmt(val):
        return f"{val: 3.2f}" if isinstance(val, (int, float)) else "None"

    trace_print(
        10,
        "IC_L_6: "
        f"t={_fmt(_last_val('time') if _last_val('time') is not None else t)}, "
        f"y_rov={_fmt(_last_val('y_rov'))}, "
        f"vy_rov={_fmt(_last_val('vy_rov'))}, "
        f"gy_rov={_fmt(curr['gy_rov'])}, "
        f"L={_fmt(_last_val('L'))}, "
        f"dl_dt_cmd={_fmt(_last_val('dl_dt_cmd'))}, "
        f"prev_cmd={_fmt(prev_cmd)}, "
        f"T_bat={_fmt(_last_val('T_boat'))}, "
        f"Fy_tr={_fmt(_last_val('Fy_traction'))}, "
        f"F_app_w={_fmt(_last_val('F_apparent_weight'))}, "
        f"F_buoy_net={_fmt(_last_val('F_buoyancy_net'))}, "
        f"Fy_total={_fmt(_last_val('Fy_total'))}, "
        f"ret={ret: 5.2f}, exp={exp}"
    )
    trace_print(
        1,
        f"AUTO_L_6: t={t:.2f}, L={L: 6.2f}, est_dT_dL={0.0 if est_dT_dL is None else est_dT_dL: 7.2f}, "
        f"y_rov={p: 6.2f}, vy_rov={curr['vy_rov']: 6.2f}, "
        f"T={curr['T']:.2f}, "
        f"prev_cmd={prev_cmd: 6.2f}, ret1={ret1: 6.2f}, ret={ret: 6.2f}, "
        f"dL_dt={curr['dL_dt']: 6.2f}, dt={curr['dt']: 6.2f},  "
        f"desired={desired: 6.2f}, exp={exp}"
    )
    return ret, exp
