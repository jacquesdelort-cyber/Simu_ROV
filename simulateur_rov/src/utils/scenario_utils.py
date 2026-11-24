"""Validation de la syntaxe des scénarios."""
from __future__ import annotations

import re
import math
import numpy as np

from src.utils.logger import trace_print

_ANSI_RESET = "\033[0m"
_ANSI_RED = "\033[91m"
_ANSI_GREEN = "\033[92m"
_ANSI_YELLOW = "\033[93m"
_ANSI_BLUE = "\033[94m"
_ANSI_CYAN = "\033[96m" 


def contrôle_câble(
    x_cable: list[float] | np.ndarray,
    y_cable: list[float] | np.ndarray,
    x_boat: float,
    y_boat: float,
    x_rov: float,
    y_rov: float,
    L: float,
    tol_segment: float = 1e-2,
) -> tuple[bool, bool, bool, bool, bool, list[int], str]:
    """
    Vérifie que la géométrie du câble respecte les contraintes suivantes :
    
    1. Le point 0 correspond au bateau (x_boat, y_boat)
    2. Le point N correspond au ROV (x_rov, y_rov)
    3. La somme des longueurs des N segments = L(t)
    4. Slack(t) = L(t) - L_straight(t) >= 0
    5. Chaque segment a une longueur = L(t)/N à tol_segment près
    
    Parameters:
    -----------
    x_cable : list[float] | np.ndarray
        Coordonnées x des points du câble (N+1 points)
    y_cable : list[float] | np.ndarray
        Coordonnées y des points du câble (N+1 points)
    x_boat : float
        Coordonnée x du bateau
    y_boat : float
        Coordonnée y du bateau (généralement 0.0)
    x_rov : float
        Coordonnée x du ROV
    y_rov : float
        Coordonnée y du ROV
    L : float
        Longueur totale du câble L(t)
    tol_segment : float
        Tolérance pour la longueur des segments (défaut: 1e-2 m)
    
    Returns:
    --------
    tuple[bool, bool, bool, bool, bool, list[int], str]
        (r1, r2, r3, r4, r5, ls, msg_err) où :
        - r1 : True si le point 0 correspond au bateau
        - r2 : True si le point N correspond au ROV
        - r3 : True si la somme des segments = L(t)
        - r4 : True si Slack(t) >= 0
        - r5 : True si tous les segments ont la longueur L(t)/N à tol_segment près
        - ls : Liste des indices de segments ne respectant pas la contrainte 5
        - msg_err : concaténation (séparateur ``\\n``) des messages de violation,
          même libellés que dans ``auto_L_7`` ; chaîne vide si tout est valide.
    """
    # Convertir en arrays numpy si nécessaire
    x_cable = np.asarray(x_cable, dtype=float)
    y_cable = np.asarray(y_cable, dtype=float)
    
    N = len(x_cable) - 1  # Nombre de segments
    if N <= 0:
        msg0 = "Géométrie câble invalide : moins d'un segment (N<=0)."
        return (False, False, False, False, False, [], msg0)
    
    errors: list[str] = []

    # Contrainte 1 : Le point 0 correspond au bateau
    tol_position = tol_segment  
    r1 = (abs(x_cable[0] - x_boat) < tol_position and 
          abs(y_cable[0] - y_boat) < tol_position)
    if not r1:
        errors.append(
            f"Contrainte 1 violée : Le point 0 du câble ({x_cable[0]:.6f}, {y_cable[0]:.6f}) "
            f"ne correspond pas au bateau ({float(x_boat):.6f}, {float(y_boat):.6f})"
        )

    # Contrainte 2 : Le point N correspond au ROV
    r2 = (abs(x_cable[-1] - x_rov) < tol_position and 
          abs(y_cable[-1] - y_rov) < tol_position)
    if not r2:
        errors.append(
            f"Contrainte 2 violée : Le point N du câble ({x_cable[-1]:.6f}, {y_cable[-1]:.6f}) "
            f"ne correspond pas au ROV ({float(x_rov):.6f}, {float(y_rov):.6f})"
        )

    # Contrainte 3 : La somme des longueurs des N segments = L(t)
    tol_total_length = 1e-3 * L
    L_total = 0.0
    segment_lengths = []
    for i in range(N):
        dx = x_cable[i+1] - x_cable[i]
        dy = y_cable[i+1] - y_cable[i]
        ds = np.sqrt(dx**2 + dy**2)
        segment_lengths.append(ds)
        L_total += ds
    
    r3 = abs(L_total - L) < tol_total_length
    if not r3:
        errors.append(
            f"Contrainte 3 violée : La somme des longueurs des segments ({L_total:.6f} m) "
            f"n'est pas égale à L(t) ({float(L):.6f} m)"
        )

    # Contrainte 4 : Slack(t) = L(t) - L_straight(t) >= 0
    tol_slack = -1e-2
    dx_straight = x_rov - x_boat
    dy_straight = y_rov - y_boat
    L_straight = np.sqrt(dx_straight**2 + dy_straight**2)
    slack = L - L_straight
    r4 = slack >= tol_slack  
    if not r4:
        dx_straight = float(x_rov - x_boat)
        dy_straight = float(y_rov - y_boat)
        L_straight = float(np.sqrt(dx_straight**2 + dy_straight**2))
        slack = float(L) - L_straight
        errors.append(
            f"Contrainte 4 violée : Slack(t) = {slack:.6f} m < 0 "
            f"(L={float(L):.6f} m, L_straight={L_straight:.6f} m)"
        )

    # Contrainte 5 : Chaque segment a une longueur = L(t)/N à tol_segment près
    ds_target = L / N if N > 0 else 0.0
    r5 = True
    ls = []  # Liste des segments ne respectant pas la contrainte 5
    for i in range(N):
        ds = segment_lengths[i]
        if abs(ds - ds_target) > tol_segment:
            r5 = False
            ls.append(i)
    if not r5:
        ds_target_msg = float(L) / N if N > 0 else 0.0
        errors.append(
            f"Contrainte 5 violée : {len(ls)} segment(s) ne respectent pas la longueur cible "
            f"(ds_target={ds_target_msg:.6f} m, tol={tol_segment} m). Segments: {ls}"
        )

    msg_err = "\n".join(errors)
    return (r1, r2, r3, r4, r5, ls, msg_err)

# Pattern pour les noms de fonctions auto_L : auto_L_ suivi d'un identifiant (alphanumérique + _)
# Permet auto_L_1, auto_L_7, auto_L_basic, etc. (nom complet doit être un identifiant Python valide)
_AUTO_L_PATTERN = re.compile(r"^auto_L_([a-zA-Z0-9_]+)$")

def list_auto_L_functions() -> list[str]:
    """
    Retourne la liste des noms de fonctions auto_L_xxx présentes dans ce module.
    Chaque nom est de la forme 'auto_L_xxx' où xxx est un identifiant Python valide.
    Triée par ordre alphabétique.
    """
    names = []
    mod_globals = globals()
    for name in mod_globals:
        if name.startswith("auto_L_") and name != "auto_L_":
            m = _AUTO_L_PATTERN.match(name)
            if m and callable(mod_globals.get(name)):
                names.append(name)
    return sorted(names)


def get_auto_L_function(selector: int | str):
    """
    Retourne la fonction auto_L correspondant au sélecteur.
    
    Parameters
    ----------
    selector : int ou str
        - int : indice numérique (ex: 7 -> auto_L_7)
        - str : suffixe (ex: "7") ou nom complet (ex: "auto_L_7")
    
    Returns
    -------
    callable ou None
        La fonction auto_L si trouvée, None sinon.
    """
    if isinstance(selector, int):
        name = f"auto_L_{selector}"
    elif isinstance(selector, str):
        s = selector.strip()
        if s.startswith("auto_L_"):
            name = s
        else:
            name = f"auto_L_{s}"
    else:
        return None
    return globals().get(name) if callable(globals().get(name)) else None


_ALLOWED_PREFIXES = {"t", "l", "y", "x"}
_NUMBER_REGEX = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
_EVT_NAME_REGEX = r"(?:e[A-Za-z0-9_]*|x)"
_MISSION_END_EVENTS = {"x"}
_CRIT_PREFIX_REGEX = r"[tTlLyYxX]"
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
_COUPLE_EVENT_DELAY_EMIT_REGEX = re.compile(
    rf"\(\s*(?P<evt>{_EVT_NAME_REGEX})\s*>\s*(?P<y>{_NUMBER_REGEX})\s*:"
    rf"\s*(?P<evt_out>{_EVT_NAME_REGEX})\s*\)"
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
    - (x>y:z) ou (x<y:z) avec x ∈ {"t", "l", "y", "x"} et y/z numériques
    - (x>y:evt) ou (x<y:evt) où evt est un nom d'événement valide
    - (evt>x:z) où evt est un nom d'événement valide et x/z numériques
    - (evt>x:evt2) où evt/evt2 sont des noms d'événements valides
    - (evt:z) où evt est un nom d'événement valide et z numérique (raccourci de evt>0)
    - (:z) où le critère est vide (toujours vrai) et z numérique
    - (:evt) où le critère est vide (toujours vrai) et evt valide
    - evt = x déclenche la fin de mission quand il est émis
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

        match = _COUPLE_EVENT_DELAY_EMIT_REGEX.match(text, pos)
        if match:
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


def find_first_invalid_couple(scen: str) -> str | None:
    """
    Retourne le premier couple syntaxiquement invalide dans un scénario, si présent.
    """
    if scen is None:
        return None

    text = str(scen).strip()
    if text == "":
        return None

    pos = 0
    length = len(text)
    while True:
        while pos < length and text[pos].isspace():
            pos += 1
        if pos >= length:
            return None

        match = _COUPLE_EMIT_REGEX.match(text, pos)
        if match:
            pos = match.end()
            continue

        match = _COUPLE_NUMERIC_REGEX.match(text, pos)
        if match:
            pos = match.end()
            continue

        match = _COUPLE_EVENT_DELAY_EMIT_REGEX.match(text, pos)
        if match:
            pos = match.end()
            continue

        match = _COUPLE_EVENT_DELAY_REGEX.match(text, pos)
        if match:
            pos = match.end()
            continue

        match = _COUPLE_EVENT_REGEX.match(text, pos)
        if match:
            pos = match.end()
            continue

        match = _COUPLE_ALWAYS_REGEX.match(text, pos)
        if match:
            pos = match.end()
            continue

        match = _COUPLE_ALWAYS_EMIT_REGEX.match(text, pos)
        if match:
            pos = match.end()
            continue

        end = text.find(")", pos)
        if end == -1:
            return text[pos:].strip() or None
        return text[pos:end + 1].strip() or None


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

        match = _COUPLE_EVENT_DELAY_EMIT_REGEX.match(text, pos)
        if match:
            try:
                y_val = float(match.group("y"))
            except ValueError:
                return []
            criteria.append(("event_delay_emit", match.group("evt"), y_val, match.group("evt_out")))
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
    if kind == "event_delay_emit":
        _, evt_name, delay, evt_out = criterion
        if float(delay) == 0.0:
            return f"({evt_name}:{evt_out})"
        return f"({evt_name}>{_format_num(delay)}:{evt_out})"
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
    y: float,
    x: float,
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
        commande_scenario._mission_end = False
        commande_scenario._mission_end_time = None
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
        values = {"t": t, "l": L, "y": y, "x": x}
        x_val = values[prefix]
        if (op == ">" and x_val > y_val) or (op == "<" and x_val < y_val):
            setattr(commande_scenario, counter_name, k + 1)
            if kind == "numeric":
                if trigger_key not in triggered:
                    triggered[trigger_key] = float(t)
                    trace_print(8, f"[SCEN] t={t:.2f} scen={scenario_name} "
                        f"crit={_format_criterion(criterion)} T_bat={T_bat} "
                        f"y={y:.2f} x={x:.2f} L={L:.2f}"
                    )
                    if trigger_sink is not None:
                        trigger_sink.append(trigger_text)
                return float(payload)
            if trigger_key not in triggered:
                triggered[trigger_key] = float(t)
                trace_print(8, f"[SCEN] t={t:.2f} scen={scenario_name} "
                    f"crit={_format_criterion(criterion)} T_bat={T_bat} "
                    f"y={y:.2f} x={x:.2f} L={L:.2f}"
                )
                if trigger_sink is not None:
                    trigger_sink.append(trigger_text)
            if payload not in commande_scenario._events:
                commande_scenario._events[payload] = float(t)
                if payload in _MISSION_END_EVENTS:
                    commande_scenario._mission_end = True
                    commande_scenario._mission_end_time = float(t)
        return None

    if kind == "event_delay":
        _, evt_name, delay, z_val = criterion
        evt_time = commande_scenario._events.get(evt_name)
        if evt_time is not None and (t - evt_time) > delay:
            setattr(commande_scenario, counter_name, k + 1)
            if trigger_key not in triggered:
                triggered[trigger_key] = float(t)
                trace_print(8, f"[SCEN] t={t:.2f} scen={scenario_name} "
                    f"crit={_format_criterion(criterion)} T_bat={T_bat} "
                    f"y={y:.2f} x={x:.2f} L={L:.2f}"
                )
                if trigger_sink is not None:
                    trigger_sink.append(trigger_text)
            return z_val

    if kind == "event_delay_emit":
        _, evt_name, delay, evt_out = criterion
        evt_time = commande_scenario._events.get(evt_name)
        if evt_time is not None and (t - evt_time) > delay:
            setattr(commande_scenario, counter_name, k + 1)
            if trigger_key not in triggered:
                triggered[trigger_key] = float(t)
                trace_print(8, f"[SCEN] t={t:.2f} scen={scenario_name} "
                    f"crit={_format_criterion(criterion)} T_bat={T_bat} "
                    f"y={y:.2f} x={x:.2f} L={L:.2f}"
                )
                if trigger_sink is not None:
                    trigger_sink.append(trigger_text)
            if evt_out not in commande_scenario._events:
                commande_scenario._events[evt_out] = float(t)
                if evt_out in _MISSION_END_EVENTS:
                    commande_scenario._mission_end = True
                    commande_scenario._mission_end_time = float(t)
        return None

    if kind == "always":
        _, z_val = criterion
        setattr(commande_scenario, counter_name, k + 1)
        if trigger_key not in triggered:
            triggered[trigger_key] = float(t)
            trace_print(8, f"[SCEN] t={t:.2f} scen={scenario_name} "
                f"crit={_format_criterion(criterion)} T_bat={T_bat} "
                f"y={y:.2f} x={x:.2f} L={L:.2f}"
            )
            if trigger_sink is not None:
                trigger_sink.append(trigger_text)
        return z_val

    if kind == "emit_always":
        _, evt_name = criterion
        setattr(commande_scenario, counter_name, k + 1)
        if trigger_key not in triggered:
            triggered[trigger_key] = float(t)
            trace_print(8, f"[SCEN] t={t:.2f} scen={scenario_name} "
                f"crit={_format_criterion(criterion)} T_bat={T_bat} "
                f"y={y:.2f} x={x:.2f} L={L:.2f}"
            )
            if trigger_sink is not None:
                trigger_sink.append(trigger_text)
        if evt_name not in commande_scenario._events:
            commande_scenario._events[evt_name] = float(t)
            if evt_name in _MISSION_END_EVENTS:
                commande_scenario._mission_end = True
                commande_scenario._mission_end_time = float(t)
        return None

    return None


def auto_L_1(
    t: float,
    y_rov: float,
    L: float,
    L_straight: float | None = None,
    T_boat: float | None = None,
    Fx_rov: float | None = None,
    Fy_rov: float | None = None,
    x_rov: float | None = None,
    x_boat: float | None = None,
    Trupt: float = 100.0,
    Tcible: float = 50.0,
    Gamma_moulinet_min: float = -0.5,
    Gamma_moulinet_max: float = 0.5,
    dl_dt_min: float = -1.0,
    dl_dt_max: float = 1.0,
    # Nombre d'itérations utilisées dans l'historique.
    K: int = 10,
    # Nombre de pas de temps utilisés pour lisser la commande.
    N: int = 10,
    data: dict | None = None,
) -> tuple[float, str]:
    """
    Calcule une consigne automatique pour dL/dt à partir de l'historique.
    Priorités:
    1) ne pas dépasser la tension de rupture,
    2) respecter l'intention du pilote (Fy_rov < 0 => descendre, éviter câble tendu),
    3) rester en mode caténaire si possible,
    4) éviter les à-coups (lissage + limite d'accélération).
    Retourne (ret, explication).
    Paramètres utilisés: t, L, Fx_rov, Fy_rov, T_boat, Trupt, Tcible,
    Gamma_moulinet_min, Gamma_moulinet_max, dl_dt_min, dl_dt_max, K, N.
    """
    if not hasattr(auto_L_1, "_history"):
        auto_L_1._history = []

    def _apply_moulinet_accel_limit(ret, curr, prev, gamma_min, gamma_max):
        if gamma_min is None and gamma_max is None:
            return ret, False
        dt = curr.get("dt")
        if not dt:
            return ret, False
        prev_cmd = prev.get("dl_dt_cmd_prev")
        if prev_cmd is None:
            prev_cmd = curr.get("dL_dt")
            if prev_cmd is None:
                prev_cmd = 0.0
        gmin = -0.5 if gamma_min is None else float(gamma_min)
        gmax = 0.5 if gamma_max is None else float(gamma_max)
        if gmin > gmax:
            gmin, gmax = gmax, gmin
        max_delta_pos = gmax * dt
        max_delta_neg = gmin * dt
        limited = prev_cmd + min(max(ret - prev_cmd, max_delta_neg), max_delta_pos)
        return limited, abs(limited - ret) > 1e-12

    def _finalize(ret, exp):
        if dl_dt_min is None:
            dl_dt_min_val = -1.0
        else:
            dl_dt_min_val = float(dl_dt_min)
        if dl_dt_max is None:
            dl_dt_max_val = 1.0
        else:
            dl_dt_max_val = float(dl_dt_max)
        if dl_dt_min_val > dl_dt_max_val:
            dl_dt_min_val, dl_dt_max_val = dl_dt_max_val, dl_dt_min_val
        ret = max(min(ret, dl_dt_max_val), dl_dt_min_val)
        return ret, exp

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
            "y_rov": float(y_rov),
            "L": float(L),
            "Fx_rov": float(Fx_rov),
            "Fy_rov": float(Fy_rov),
            "T_boat": None if T_boat is None else float(T_boat),
            "dt": None,
            "dL": None,
            "dT_boat": None,
            "dL_dt": None,
            "dl_dt_cmd_prev": None,
        },
    )
    if len(history) > K + 1:
        del history[K + 1 :]

    if len(history) == 1:
        return _finalize(0.0, "INIT")


    curr = history[0]
    prev = history[1]
    curr["dL"] = curr["L"] - prev["L"]
    curr["dT_boat"] = curr["T_boat"] - prev["T_boat"]
    curr["dt"] = curr["t"] - prev["t"]
    curr["dL_dt"] = curr["dL"] / curr["dt"] if curr["dt"] != 0 else None

    
    
    # Priorité 1-2: sécurité rupture + intention pilote (descente)
    t_soft = 0.90 * Trupt
    t_hard = 0.98 * Trupt
    t_soft_off = 0.88 * Trupt
    t_hard_off = 0.95 * Trupt
    if curr["T_boat"] is not None:
        if curr["T_boat"] >= t_hard:
            ret, exp = 1.0, "T_HARD"
        elif curr["T_boat"] >= t_soft:
            ramp = (curr["T_boat"] - t_soft) / max(t_hard - t_soft, 1e-6)
            ret, exp = 0.5 + 0.5 * max(min(ramp, 1.0), 0.0), "T_SOFT"
        elif Fy_rov < 0:
            # Éviter le blocage en câble tendu quand le pilote veut descendre
            if curr["T_boat"] >= Tcible:
                ret, exp = 0.3, "PILOT_DESC_TAUT"
            else:
                ret, exp = 0.1, "PILOT_DESC"
        else:
            ret = None
            exp = ""
        if ret is not None:
            ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
            if limited:
                exp = f"{exp}_ACC"
            curr["dl_dt_cmd_prev"] = ret
            return _finalize(ret, exp)

    if abs(T_boat - Tcible) < 1:
        ret, exp = 0.0, "T_CLOSE"
        ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
        if limited:
            exp = f"{exp}_ACC"
        curr["dl_dt_cmd_prev"] = ret
        return _finalize(ret, exp)
    if abs(Tcible - prev["T_boat"]) < 1:
        ret, exp = 0.0, "TCIBLE_PREV_CLOSE"
        ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
        if limited:
            exp = f"{exp}_ACC"
        curr["dl_dt_cmd_prev"] = ret
        return _finalize(ret, exp)

    z = (T_boat-Tcible)/abs(prev["T_boat"]-Tcible)
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
    
    ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
    if limited:
        exp = f"{exp}_ACC"
    curr["dl_dt_cmd_prev"] = ret
    trace_print(2, f"t: {t:.2f}, dt: {curr['dt']:.2f}, dL_dt: {curr['dL_dt']:.2f}, "
        f"T: {T_boat:.2f}, Tcible: {Tcible:.2f}, z: {z:.2f}, ret: {ret:.2f}"
    )
    return _finalize(ret, exp)


def auto_L_2(
    t: float,
    y_rov: float,
    L: float,
    L_straight: float | None = None,
    T_boat: float | None = None,
    Fx_rov: float | None = None,
    Fy_rov: float | None = None,
    x_rov: float | None = None,
    x_boat: float | None = None,
    Trupt: float = 100.0,
    Tcible: float = 50.0,
    Gamma_moulinet_min: float = -0.5,
    Gamma_moulinet_max: float = 0.5,
    dl_dt_min: float = -1.0,
    dl_dt_max: float = 1.0,
    # Nombre d'itérations utilisées dans l'historique.
    K: int = 10,
    # Nombre de pas de temps utilisés pour lisser la commande.
    N: int = 10,
    data: dict | None = None,
) -> tuple[float, str]:
    """
    Copie de auto_L_1 pour essais de lois de commande alternatives.
    Priorités:
    1) ne pas dépasser la tension de rupture,
    2) respecter l'intention du pilote (Fy_rov < 0 => descendre, éviter câble tendu),
    3) rester en mode caténaire si possible,
    4) éviter les à-coups (lissage + limite d'accélération).
    Retourne (ret, explication).
    Paramètres utilisés: t, L, Fx_rov, Fy_rov, T_boat, Trupt, Tcible,
    Gamma_moulinet_min, Gamma_moulinet_max, dl_dt_min, dl_dt_max, K.
    """
    if not hasattr(auto_L_2, "_history"):
        auto_L_2._history = []

    def _apply_moulinet_accel_limit(ret, curr, prev, gamma_min, gamma_max):
        if gamma_min is None and gamma_max is None:
            return ret, False
        dt = curr.get("dt")
        if not dt:
            return ret, False
        prev_cmd = prev.get("dl_dt_cmd_prev")
        if prev_cmd is None:
            prev_cmd = curr.get("dL_dt")
            if prev_cmd is None:
                prev_cmd = 0.0
        gmin = -0.5 if gamma_min is None else float(gamma_min)
        gmax = 0.5 if gamma_max is None else float(gamma_max)
        if gmin > gmax:
            gmin, gmax = gmax, gmin
        max_delta_pos = gmax * dt
        max_delta_neg = gmin * dt
        limited = prev_cmd + min(max(ret - prev_cmd, max_delta_neg), max_delta_pos)
        return limited, abs(limited - ret) > 1e-12

    def _finalize(ret, exp):
        if dl_dt_min is None:
            dl_dt_min_val = -1.0
        else:
            dl_dt_min_val = float(dl_dt_min)
        if dl_dt_max is None:
            dl_dt_max_val = 1.0
        else:
            dl_dt_max_val = float(dl_dt_max)
        if dl_dt_min_val > dl_dt_max_val:
            dl_dt_min_val, dl_dt_max_val = dl_dt_max_val, dl_dt_min_val
        ret = max(min(ret, dl_dt_max_val), dl_dt_min_val)
        return ret, exp

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
            "y_rov": float(y_rov),
            "L": float(L),
            "Fx_rov": float(Fx_rov),
            "Fy_rov": float(Fy_rov),
            "T_boat": None if T_boat is None else float(T_boat),
            "dt": None,
            "dL": None,
            "dT_boat": None,
            "dL_dt": None,
            "dl_dt_cmd_prev": None,
        },
    )

    curr = history[0]
    prev = history[1]
    curr["dL"] = curr["L"] - prev["L"]
    curr["dT_boat"] = curr["T_boat"] - prev["T_boat"]
    curr["dt"] = curr["t"] - prev["t"]
    curr["dL_dt"] = curr["dL"] / curr["dt"] if curr["dt"] != 0 else 0

    # Priorité 1-2: sécurité rupture + intention pilote (descente)
    t_soft = 0.90 * Trupt
    t_hard = 0.98 * Trupt
    if curr["T_boat"] is not None:
        if curr["T_boat"] >= t_hard:
            ret, exp = 1.0, "T_HARD"
        elif curr["T_boat"] >= t_soft:
            ramp = (curr["T_boat"] - t_soft) / max(t_hard - t_soft, 1e-6)
            ret, exp = 0.5 + 0.5 * max(min(ramp, 1.0), 0.0), "T_SOFT"
        elif Fy_rov < 0:
            if curr["T_boat"] >= Tcible:
                ret, exp = 0.3, "PILOT_DESC_TAUT"
            else:
                ret, exp = 0.1, "PILOT_DESC"
        else:
            ret = None
            exp = ""
        if ret is not None:
            ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
            if limited:
                exp = f"{exp}_ACC"
            curr["dl_dt_cmd_prev"] = ret
            return _finalize(ret, exp)

    if len(history) > K + 1:
        del history[K + 1 :]

    if len(history) < 2:
        return _finalize(0.0, "HISTORY_LT2")

    J = min(K, len(history))
    avg_L = sum(item["L"] for item in history[:J]) / J
    avg_T = sum(item["T_boat"] for item in history[:J]) / J
    delta_L = history[-1]["L"] - history[0]["L"]
    delta_T = history[-1]["T_boat"] - history[0]["T_boat"]

    if abs(delta_L) < 1e-2:
        if curr["dL_dt"] > 0:
            ret, exp = curr["dL_dt"] - 0.1, "DELTA_L_0+"
            trace_print(8, f"t: {t:.2f}, delta_L proche de 0+: {delta_L:.2f}, -> retourne {ret:.2f}"
            )
        else:
            ret, exp = curr["dL_dt"] + 0.11, "DELTA_L_0-"
            trace_print(8, f"t: {t:.2f}, delta_L proche de 0-: {delta_L:.2f}, -> retourne {ret:.2f}"
            )
        ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
        if limited:
            exp = f"{exp}_ACC"
        curr["dl_dt_cmd_prev"] = ret
        return _finalize(ret, exp)
        
    est_dT_dL = delta_T / delta_L
    
    ret = abs((T_boat - Tcible) / Tcible) * ((T_boat - Tcible) / (2 * est_dT_dL))
    ret_corrige = max(min(ret, 2.02), -2.02)
    ret, limited = _apply_moulinet_accel_limit(ret_corrige, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
    exp = "MAIN_ACC" if limited else "MAIN"
    curr["dl_dt_cmd_prev"] = ret
    trace_print(8, f"t: {t:.2f}, L: {L:.2f}, est_dT_dL: {est_dT_dL:.2f}, "
        f"T: {T_boat:.2f}, Tcible: {Tcible:.2f}, ret: {ret:.2f}, ret_corrige: {ret_corrige:.2f}"
    )
    return _finalize(ret, exp)


def auto_L_3(
    t: float,
    y_rov: float,
    L: float,
    L_straight: float | None = None,
    T_boat: float | None = None,
    Fx_rov: float | None = None,
    Fy_rov: float | None = None,
    x_rov: float | None = None,
    x_boat: float | None = None,
    Trupt: float = 100.0,
    Tcible: float = 50.0,
    Gamma_moulinet_min: float = -0.5,
    Gamma_moulinet_max: float = 0.5,
    dl_dt_min: float = -1.0,
    dl_dt_max: float = 1.0,
    # Nombre d'itérations utilisées dans l'historique.
    K: int = 10,
    # Nombre de pas de temps utilisés pour lisser la commande.
    N: int = 10,
    data: dict | None = None,
) -> tuple[float, str]:
    """
    Copie de auto_L_2 pour essais de lois de commande alternatives.
    Priorités:
    1) ne pas dépasser la tension de rupture,
    2) respecter l'intention du pilote (Fy_rov < 0 => descendre, éviter câble tendu),
    3) rester en mode caténaire si possible,
    4) éviter les à-coups (lissage + limite d'accélération).
    Retourne (ret, explication).
    Paramètres utilisés: t, L, Fx_rov, Fy_rov, T_boat, Trupt, Tcible,
    Gamma_moulinet_min, Gamma_moulinet_max, dl_dt_min, dl_dt_max, K.
    """
    if not hasattr(auto_L_3, "_history"):
        auto_L_3._history = []

    def _apply_moulinet_accel_limit(ret, curr, prev, gamma_min, gamma_max):
        if gamma_min is None and gamma_max is None:
            return ret, False
        dt = curr.get("dt")
        if not dt:
            return ret, False
        prev_cmd = prev.get("dl_dt_cmd_prev")
        if prev_cmd is None:
            prev_cmd = curr.get("dL_dt")
            if prev_cmd is None:
                prev_cmd = 0.0
        gmin = -0.5 if gamma_min is None else float(gamma_min)
        gmax = 0.5 if gamma_max is None else float(gamma_max)
        if gmin > gmax:
            gmin, gmax = gmax, gmin
        max_delta_pos = gmax * dt
        max_delta_neg = gmin * dt
        limited = prev_cmd + min(max(ret - prev_cmd, max_delta_neg), max_delta_pos)
        return limited, abs(limited - ret) > 1e-12

    def _finalize(ret, exp):
        if dl_dt_min is None:
            dl_dt_min_val = -1.0
        else:
            dl_dt_min_val = float(dl_dt_min)
        if dl_dt_max is None:
            dl_dt_max_val = 1.0
        else:
            dl_dt_max_val = float(dl_dt_max)
        if dl_dt_min_val > dl_dt_max_val:
            dl_dt_min_val, dl_dt_max_val = dl_dt_max_val, dl_dt_min_val
        ret = max(min(ret, dl_dt_max_val), dl_dt_min_val)
        return ret, exp

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
            "y_rov": float(y_rov),
            "L": float(L),
            "Fx_rov": float(Fx_rov),
            "Fy_rov": float(Fy_rov),
            "T_boat": None if T_boat is None else float(T_boat),
            "dt": None,
            "dL": None,
            "dT_boat": None,
            "dL_dt": None,
            "dl_dt_cmd_prev": None,
        },
    )

    curr = history[0]
    prev = history[1]
    curr["dL"] = curr["L"] - prev["L"]
    curr["dT_boat"] = curr["T_boat"] - prev["T_boat"]
    curr["dt"] = curr["t"] - prev["t"]
    curr["dL_dt"] = curr["dL"] / curr["dt"] if curr["dt"] != 0 else 0

    # Priorité 1-2: sécurité rupture + intention pilote (descente)
    t_soft = 0.90 * Trupt
    t_hard = 0.98 * Trupt
    if curr["T_boat"] is not None:
        if curr["T_boat"] >= t_hard:
            ret, exp = 1.0, "T_HARD"
        elif curr["T_boat"] >= t_soft:
            ramp = (curr["T_boat"] - t_soft) / max(t_hard - t_soft, 1e-6)
            ret, exp = 0.5 + 0.5 * max(min(ramp, 1.0), 0.0), "T_SOFT"
        elif Fy_rov < 0:
            if curr["T_boat"] >= Tcible:
                ret, exp = 0.3, "PILOT_DESC_TAUT"
            else:
                ret, exp = 0.1, "PILOT_DESC"
        else:
            ret = None
            exp = ""
        if ret is not None:
            ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
            if limited:
                exp = f"{exp}_ACC"
            curr["dl_dt_cmd_prev"] = ret
            return _finalize(ret, exp)

    if len(history) > K + 1:
        del history[K + 1 :]

    if len(history) < 2:
        return _finalize(0.0, "HISTORY_LT2")

    J = min(K, len(history))
    avg_L = sum(item["L"] for item in history[:J]) / J
    avg_T = sum(item["T_boat"] for item in history[:J]) / J
    delta_L = history[-1]["L"] - history[0]["L"]
    delta_T = history[-1]["T_boat"] - history[0]["T_boat"]

    if abs(delta_L) < 1e-4:
        if curr["dL_dt"] > 0:
            ret, exp = curr["dL_dt"] - 0.1, "DELTA_L_0+"
            trace_print(8, f"t: {t:.2f}, delta_L proche de 0+: {delta_L:.2f}, -> retourne {ret:.2f}"
            )
        else:
            ret, exp = curr["dL_dt"] + 0.11, "DELTA_L_0-"
            trace_print(8, f"t: {t:.2f}, delta_L proche de 0-: {delta_L:.2f}, -> retourne {ret:.2f}"
            )
        ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
        if limited:
            exp = f"{exp}_ACC"
        curr["dl_dt_cmd_prev"] = ret
        return _finalize(ret, exp)

    est_dT_dL = delta_T / delta_L

    ret = abs((T_boat - Tcible) / Tcible) * ((T_boat - Tcible) / (2 * est_dT_dL))
    ret_corrige = max(min(ret, 2.02), -2.02)
    ret, limited = _apply_moulinet_accel_limit(ret_corrige, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
    exp = "MAIN_ACC" if limited else "MAIN"
    curr["dl_dt_cmd_prev"] = ret
    trace_print(8, f"t: {t:.2f}, L: {L:.2f}, est_dT_dL: {est_dT_dL:.2f}, "
        f"T: {T_boat:.2f}, Tcible: {Tcible:.2f}, ret: {ret:.2f}, ret_corrige: {ret_corrige:.2f}"
    )
    return _finalize(ret, exp)


def auto_L_4(
    t: float,
    y_rov: float,
    L: float,
    L_straight: float | None = None,
    T_boat: float | None = None,
    Fx_rov: float | None = None,
    Fy_rov: float | None = None,
    x_rov: float | None = None,
    x_boat: float | None = None,
    Trupt: float = 100.0,
    Tcible: float = 50.0,
    Gamma_moulinet_min: float = -0.5,
    Gamma_moulinet_max: float = 0.5,
    dl_dt_min: float = -1.0,
    dl_dt_max: float = 1.0,
    # Nombre d'itérations utilisées dans l'historique.
    K: int = 10,
    # Nombre de pas de temps utilisés pour lisser la commande.
    N: int = 10,
    data: dict | None = None,
) -> tuple[float, str]:
    """
    Copie de auto_L_3 pour essais de lois de commande alternatives.
    Priorités:
    1) ne pas dépasser la tension de rupture,
    2) respecter l'intention du pilote (Fy_rov < 0 => descendre, éviter câble tendu),
    3) rester en mode caténaire si possible,
    4) éviter les à-coups (lissage + limite d'accélération).
    Retourne (ret, explication).
    Paramètres utilisés: t, y_rov, L, Fx_rov, Fy_rov, T_boat, Trupt, Tcible,
    Gamma_moulinet_min, Gamma_moulinet_max, dl_dt_min, dl_dt_max, K, N.
    """
    if not hasattr(auto_L_4, "_history"):
        auto_L_4._history = []

    def _apply_moulinet_accel_limit(ret, curr, prev, gamma_min, gamma_max):
        if gamma_min is None and gamma_max is None:
            return ret, False
        dt = curr.get("dt")
        if not dt:
            return ret, False
        prev_cmd = prev.get("dl_dt_cmd_prev")
        if prev_cmd is None:
            prev_cmd = curr.get("dL_dt")
            if prev_cmd is None:
                prev_cmd = 0.0
        gmin = -0.5 if gamma_min is None else float(gamma_min)
        gmax = 0.5 if gamma_max is None else float(gamma_max)
        if gmin > gmax:
            gmin, gmax = gmax, gmin
        max_delta_pos = gmax * dt
        max_delta_neg = gmin * dt
        limited = prev_cmd + min(max(ret - prev_cmd, max_delta_neg), max_delta_pos)
        return limited, abs(limited - ret) > 1e-12

    def _finalize(ret, exp):
        if dl_dt_min is None:
            dl_dt_min_val = -1.0
        else:
            dl_dt_min_val = float(dl_dt_min)
        if dl_dt_max is None:
            dl_dt_max_val = 1.0
        else:
            dl_dt_max_val = float(dl_dt_max)
        if dl_dt_min_val > dl_dt_max_val:
            dl_dt_min_val, dl_dt_max_val = dl_dt_max_val, dl_dt_min_val
        ret = max(min(ret, dl_dt_max_val), dl_dt_min_val)
        return ret, exp

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
            "y_rov": float(y_rov),
            "L": float(L),
            "Fx_rov": float(Fx_rov),
            "Fy_rov": float(Fy_rov),
            "T_boat": None if T_boat is None else float(T_boat),
            "dt": None,
            "dL": None,
            "dT_boat": None,
            "dL_dt": None,
            "dl_dt_cmd_prev": None,
        },
    )

    if len(history) < 2:
        trace_print(8, f"t: {t:.2f}, HISTORY < 2  -> retourne {0.0:.2f}")
        return _finalize(0.0, "HISTORY_LT2")

    curr = history[0]
    prev = history[1]
    curr["dL"] = curr["L"] - prev["L"]
    curr["dT_boat"] = curr["T_boat"] - prev["T_boat"] if curr["T_boat"] is not None and prev["T_boat"] is not None else 0.0
    curr["dt"] = curr["t"] - prev["t"]
    curr["dL_dt"] = curr["dL"] / curr["dt"] if curr["dt"] != 0 else 0
    curr["y_rov"] = float(y_rov)
    prev["y_rov"] = float(prev.get("y_rov", y_rov))
    curr["dy_rov"] = curr["y_rov"] - prev["y_rov"]
    curr["vy_rov"] = curr["dy_rov"] / curr["dt"] if curr["dt"] != 0 else 0

    # Priorité 1-2: sécurité rupture + intention pilote (descente)
    t_soft = 0.90 * Trupt
    t_hard = 0.98 * Trupt
    if curr["T_boat"] is not None:
        if curr["T_boat"] >= t_hard:
            ret, exp = 1.0, "T_HARD"
        elif curr["T_boat"] >= t_soft:
            ramp = (curr["T_boat"] - t_soft) / max(t_hard - t_soft, 1e-6)
            ret, exp = 0.5 + 0.5 * max(min(ramp, 1.0), 0.0), "T_SOFT"
        elif Fy_rov < 0:
            if curr["T_boat"] >= Tcible:
                ret, exp = 0.3, "PILOT_DESC_TAUT"
            else:
                ret, exp = 0.1, "PILOT_DESC"
        else:
            ret = None
            exp = ""
        if ret is not None:
            ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
            if limited:
                exp = f"{exp}_ACC"
            curr["dl_dt_cmd_prev"] = ret
            return _finalize(ret, exp)

    if len(history) > K + 1:
        del history[K + 1 :]

    J = min(K, len(history))
    avg_L = np.mean([item["L"] for item in history[:J]])
    t_vals = [item["T_boat"] for item in history[:J] if item["T_boat"] is not None]
    avg_T = np.mean(t_vals) if t_vals else 0.0
    delta_L = history[-1]["L"] - history[0]["L"]
    t_last = history[-1]["T_boat"]
    t_first = history[0]["T_boat"]
    delta_T = (t_last - t_first) if t_last is not None and t_first is not None else 0.0

    # Cas marginal, on ne va pas pouvoir calculer dT/dL car dL est trop petit
    Vy_lim = compute_rov_terminal_vy(Fy_rov)
    
    if abs(delta_L) < 1e-4:
        if curr["dL_dt"] > 0:
            ret, exp = -Vy_lim - 0.05, "DELTA_L_0+"
            trace_print(8, f"0+: t: {t:.2f}, delta_L: {delta_L:.2f}, -> retourne {ret:.2f}")
        else:
            ret, exp = -Vy_lim + 0.05, "DELTA_L_0-"
            trace_print(8,f"0-: t: {t:.2f}, delta_L: {delta_L:.2f}, -> retourne {ret:.2f}")
        ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
        if limited:
            exp = f"{exp}_ACC"
        curr["dl_dt_cmd_prev"] = ret
        return _finalize(ret, exp)
        return _finalize(ret, exp)

    # Cas principal
    if T_boat is None or Tcible in (None, 0):
        ret, exp = 0.0, "NO_T"
        ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
        if limited:
            exp = f"{exp}_ACC"
        curr["dl_dt_cmd_prev"] = ret
        trace_print(8, f"MAIN: t={t:.2f}, T/Tcible invalide -> retourne {ret:.2f}")
        return _finalize(ret, exp)

    est_dT_dL = delta_T / delta_L
    if abs(est_dT_dL)  < 1e-4:
        ret, exp = 0.0, "DTDL_0"
        ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
        if limited:
            exp = f"{exp}_ACC"
        curr["dl_dt_cmd_prev"] = ret
        trace_print(8, f"MAIN: t={t:.2f}, est_dT_dL=0 -> retourne {ret:.2f}")
        return _finalize(ret, exp)

    ret1 = - 1.1*Vy_lim +  min(1, abs((T_boat - Tcible) / Tcible)) * ((T_boat - Tcible) / (2 * est_dT_dL)) 
    ret2 = curr["dL_dt"] + (ret1 - curr["dL_dt"]) / 5
    ret = ret2

    ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
    exp = "MAIN_ACC" if limited else "MAIN"
    curr["dl_dt_cmd_prev"] = ret
    trace_print(8, f"MAIN: t={t:.2f}, L={L: 6.2f}, est_dT_dL={est_dT_dL: 7.2f}, y_rov={y_rov: 6.2f}, vy_rov={curr['vy_rov']: 6.2f}, Vy_lim={Vy_lim: 6.2f}, "
        f"T={T_boat: 5.2f}, Tcible={Tcible: 5.2f}, ret1={ret1: 8.2f}, ret2={ret2: 6.2f}, ret={ret: 6.2f}"
    )
    return _finalize(ret, exp)


def auto_L_6(
    t: float,
    y_rov: float,
    L: float,
    L_straight: float | None = None,
    T_boat: float | None = None,
    Fx_rov: float | None = None,
    Fy_rov: float | None = None,
    x_rov: float | None = None,
    x_boat: float | None = None,
    Trupt: float = 100.0,
    Tcible: float = 50.0,
    Gamma_moulinet_min: float = -0.5,
    Gamma_moulinet_max: float = 0.5,
    dl_dt_min: float = -1.0,
    dl_dt_max: float = 1.0,
    # Nombre d'itérations utilisées dans l'historique.
    K: int = 10,
    # Nombre de pas de temps utilisés pour lisser la commande.
    N: int = 10,
    data: dict | None = None,
) -> tuple[float, str]:
    """
    Loi de commande dL/dt avec priorités:
    1) ne pas dépasser la tension de rupture,
    2) respecter l'intention du pilote (Fy_rov < 0 => descendre, éviter câble tendu),
    3) rester en mode caténaire si possible,
    4) éviter les à-coups (lissage + limite d'accélération).
    Retourne (ret, explication).
    Paramètres utilisés: t, y_rov, L, Fx_rov, Fy_rov, T_boat, Trupt, Tcible,
    Gamma_moulinet_min, Gamma_moulinet_max, dl_dt_min, dl_dt_max, data, K, N.
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
    if Gamma_moulinet_min is None:
        Gamma_moulinet_min = -0.5
    if Gamma_moulinet_max is None:
        Gamma_moulinet_max = 0.5

    def _finalize(ret, exp):
        if dl_dt_min is None:
            dl_dt_min_val = -1.0
        else:
            dl_dt_min_val = float(dl_dt_min)
        if dl_dt_max is None:
            dl_dt_max_val = 1.0
        else:
            dl_dt_max_val = float(dl_dt_max)
        if dl_dt_min_val > dl_dt_max_val:
            dl_dt_min_val, dl_dt_max_val = dl_dt_max_val, dl_dt_min_val
        ret = max(min(ret, dl_dt_max_val), dl_dt_min_val)
        return ret, exp

    history = auto_L_6._history
    history.insert(
        0,
        {
            "t": float(t),
            "y_rov": float(y_rov),
            "L": float(L),
            "Fx_rov": float(Fx_rov),
            "Fy_rov": float(Fy_rov),
            "T_boat": None if T_boat is None else float(T_boat),
            "dt": None,
            "dL": None,
            "dT_boat": None,
            "dL_dt": None,
            "dl_dt_cmd_prev": None,
        },
    )

    if len(history) < 2:
        return _finalize(0.0, "HISTORY_LT2")

    curr = history[0]
    prev = history[1]
    curr["dL"] = curr["L"] - prev["L"]
    curr["dT_boat"] = curr["T_boat"] - prev["T_boat"] if curr["T_boat"] is not None and prev["T_boat"] is not None else 0.0
    curr["dt"] = curr["t"] - prev["t"]
    curr["dL_dt"] = curr["dL"] / curr["dt"] if curr["dt"] != 0 else 0.0
    if data and isinstance(data.get("T_rov"), list) and data["T_rov"]:
        curr["T_rov"] = data["T_rov"][-1]
    else:
        curr["T_rov"] = None
    curr["y_rov"] = float(y_rov)
    prev["y_rov"] = float(prev.get("y_rov", y_rov))
    curr["dy_rov"] = curr["y_rov"] - prev["y_rov"]
    curr["vy_rov"] = curr["dy_rov"] / curr["dt"] if curr["dt"] != 0 else 0.0
    prev["vy_rov"] = prev.get("vy_rov", 0.0)
    curr["gy_rov"] = curr["vy_rov"] - prev["vy_rov"]

    if data and data.get("vy_rov") and len(history) > 3:
        assert math.isclose(curr["vy_rov"], data["vy_rov"][-1]), "Erreur vy_rov"
        assert math.isclose(prev["vy_rov"], data["vy_rov"][-2]), "Erreur vy_rov"

    if len(history) > K + 1:
        del history[K + 1 :]

    if curr["T_boat"] is None or Tcible in (None, 0):
        return _finalize(0.0, "NO_T")

    # Estimation locale dT/dL (si possible)
    est_dT_dL = None
    delta_L = history[-1]["L"] - history[0]["L"]
    t_last = history[-1]["T_boat"]
    t_first = history[0]["T_boat"]
    if delta_L != 0 and t_last is not None and t_first is not None:
        est_dT_dL = (t_last - t_first) / delta_L

    # Priorité 1: éviter Trupt (avec hystérésis)
    t_soft = 0.90 * Trupt
    t_hard = 0.98 * Trupt
    t_soft_off = 0.88 * Trupt
    t_hard_off = 0.95 * Trupt
    if curr["T_boat"] >= t_hard:
        desired = 1.0
        exp = "T_HARD"
    elif curr["T_boat"] >= t_soft:
        ramp = (curr["T_boat"] - t_soft) / max(t_hard - t_soft, 1e-6)
        desired = 0.5 + 0.5 * max(min(ramp, 1.0), 0.0)
        exp = "T_SOFT"
    else:
        # Priorité 2: respecter l'intention du pilote (descendre)
        if Fy_rov < 0:
            if curr["T_boat"] >= Tcible:
                desired = 0.4
                exp = "PILOT_DESC_TAUT"
            else:
                desired = 0.2
                exp = "PILOT_DESC"
        else:
            # Priorité 3: rester en mode caténaire (tension modérée)
            T_catenary = 0.8 * Tcible
            if est_dT_dL is None or abs(est_dT_dL) < 1e-4:
                if curr["T_boat"] > T_catenary:
                    desired = 0.2
                    exp = "CATENARY_SIGN_OUT"
                elif curr["T_boat"] < 0.6 * T_catenary:
                    desired = -0.1
                    exp = "CATENARY_SIGN_IN"
                else:
                    desired = 0.0
                    exp = "CATENARY_HOLD"
            else:
                desired = -(curr["T_boat"] - T_catenary) / (2.0 * est_dT_dL)
                exp = "CATENARY_REG"

    # Lissage de la commande
    prev_cmd = prev.get("dl_dt_cmd_prev")
    if prev_cmd is None:
        prev_cmd = curr["dL_dt"] if curr["dL_dt"] is not None else 0.0
    ret1 = prev_cmd + (desired - prev_cmd) / max(N, 1)

    # Limitation d'accélération du moulinet
    if curr["dt"] and (Gamma_moulinet_min is not None or Gamma_moulinet_max is not None):
        gmin = -0.5 if Gamma_moulinet_min is None else float(Gamma_moulinet_min)
        gmax = 0.5 if Gamma_moulinet_max is None else float(Gamma_moulinet_max)
        if gmin > gmax:
            gmin, gmax = gmax, gmin
        max_delta_pos = gmax * curr["dt"]
        max_delta_neg = gmin * curr["dt"]
        ret1 = prev_cmd + min(max(ret1 - prev_cmd, max_delta_neg), max_delta_pos)
        exp = f"{exp}_ACC"

    ret = ret1
    curr["dl_dt_cmd_prev"] = ret

    def _last_val(key):
        if not data:
            return None
        vals = data.get(key)
        return vals[-1] if isinstance(vals, list) and vals else None

    def _fmt(val):
        return f"{val: 3.2f}" if isinstance(val, (int, float)) else "None"
    
    def _fmt_vec(val):
        if isinstance(val, (list, tuple)) and len(val) == 2:
            return f"({val[0]: 3.2f}, {val[1]: 3.2f})"
        return "None"
    
    def _fmt_vec(val):
        if isinstance(val, (list, tuple)) and len(val) == 2:
            return f"({val[0]: 3.2f}, {val[1]: 3.2f})"
        return "None"
    
    def _fmt_vec(val):
        if isinstance(val, (list, tuple)) and len(val) == 2:
            return f"({val[0]: 3.2f}, {val[1]: 3.2f})"
        return "None"
    
    def _fmt_vec(val):
        if isinstance(val, (list, tuple)) and len(val) == 2:
            return f"({val[0]: 3.2f}, {val[1]: 3.2f})"
        return "None"
    
    def _fmt_vec(val):
        if isinstance(val, (list, tuple)) and len(val) == 2:
            return f"({val[0]: 3.2f}, {val[1]: 3.2f})"
        return "None"
    
    def _fmt_vec(val):
        if isinstance(val, (list, tuple)) and len(val) == 2:
            return f"({val[0]: 3.2f}, {val[1]: 3.2f})"
        return "None"

    trace_print(8, "IC_L_6: "
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
        f"desired={desired: 6.2f}, "
        f"ret={ret: 5.2f}, exp={exp}"
    )
    trace_print(1, f"AUTO_L_6: t={t:.2f}, L={L: 6.2f}, est_dT_dL={0.0 if est_dT_dL is None else est_dT_dL: 7.2f}, "
        f"y_rov={y_rov: 6.2f}, vy_rov={curr['vy_rov']: 6.2f}, "
        f"T={curr['T_boat']:.2f}, "
        f"prev_cmd={prev_cmd: 6.2f}, ret1={ret1: 6.2f}, ret={ret: 6.2f}, "
        f"dL_dt={curr['dL_dt']: 6.2f}, dt={curr['dt']: 6.2f},  "
        f"desired={desired: 6.2f}, exp={exp}"
    )
    return _finalize(ret, exp)


def auto_L_7(
    t: float,
    y_rov: float,
    L: float,
    L_straight: float | None = None,
    T_boat: float | None = None,
    Fx_rov: float | None = None,
    Fy_rov: float | None = None,
    x_rov: float | None = None,
    x_boat: float | None = None,
    Trupt: float = 100.0,
    Tcible: float = 50.0,
    Gamma_moulinet_min: float = -0.5,
    Gamma_moulinet_max: float = 0.5,
    dl_dt_min: float = -1.0,
    dl_dt_max: float = 1.0,
    # Nombre d'itérations utilisées dans l'historique.
    K: int = 10,
    # Nombre de pas de temps utilisés pour lisser la commande.
    N: int = 10,
    data: dict | None = None,
) -> tuple[float, str]:
    """
    Loi de commande dL/dt avec accès à la distance droite bateau-ROV.
    Priorités:
    1) ne pas dépasser la tension de rupture,
    2) respecter l'intention du pilote (Fy_rov < 0 => descendre, éviter câble tendu),
    3) rester en mode caténaire si possible,
    4) éviter les à-coups (lissage + limite d'accélération).
    Retourne (ret, explication).
    Paramètres utilisés: t, y_rov, L, Fx_rov, Fy_rov, T_boat, x_rov, x_boat,
    Trupt, Tcible, Gamma_moulinet_min, Gamma_moulinet_max, dl_dt_min, dl_dt_max, data, K, N.
    """

    debug_auto_L_7 = 9

    if not hasattr(auto_L_7, "_history"):
        auto_L_7._history = []

    if Trupt is None:
        try:
            from src.utils.parameters import get_default_parameters

            params = get_default_parameters()
            Trupt = float(params.get("cable", {}).get("tension_rupture", 50.0))
        except Exception:
            Trupt = 50.0
    if Tcible is None:
        Tcible = Trupt / 2.0
    if Gamma_moulinet_min is None:
        Gamma_moulinet_min = -0.5
    if Gamma_moulinet_max is None:
        Gamma_moulinet_max = 0.5
    
    # Contrôle de la géométrie du câble
    if data is not None:
        x_cable_curr = data.get('x_cable_curr')
        y_cable_curr = data.get('y_cable_curr')
        x_boat_curr = data.get('x_boat')
        y_boat_curr = 0.0  # Le bateau est toujours à y=0
        
        if (x_cable_curr is not None and y_cable_curr is not None and 
            x_boat_curr is not None and len(x_boat_curr) > 0):
            # Utiliser les dernières valeurs
            x_boat_val = float(x_boat_curr[-1]) if isinstance(x_boat_curr, list) else float(x_boat_curr)
            x_cable_val = x_cable_curr if isinstance(x_cable_curr, list) else list(x_cable_curr)
            y_cable_val = y_cable_curr if isinstance(y_cable_curr, list) else list(y_cable_curr)
            
            # Vérifier les contraintes
            r1, r2, r3, r4, r5, ls, msg_err = contrôle_câble(
                x_cable_val, y_cable_val,
                x_boat_val, y_boat_curr,
                x_rov, y_rov,
                L,
                tol_segment=1e-2
            )
            
            if msg_err:
                error_msg = f"IC_L_7 :  t={t:.3f} s    {_ANSI_RED}Erreur contrôle_câble:{_ANSI_RESET}\n" + msg_err
                trace_print(debug_auto_L_7, error_msg)
                # Ne pas lever d'exception pour ne pas interrompre la simulation,
                # mais afficher l'erreur dans les logs
    
    # Calculer les valeurs de dl_dt_min et dl_dt_max une seule fois au début
    if dl_dt_min is None:
        dl_dt_min_val = -1.0
    else:
        dl_dt_min_val = float(dl_dt_min)
    if dl_dt_max is None:
        dl_dt_max_val = 1.0
    else:
        dl_dt_max_val = float(dl_dt_max)
    if dl_dt_min_val > dl_dt_max_val:
        dl_dt_min_val, dl_dt_max_val = dl_dt_max_val, dl_dt_min_val

    def _finalize(ret, exp):
        ret = max(min(ret, dl_dt_max_val), dl_dt_min_val)
        return ret, exp

    history = auto_L_7._history
    history.insert(
        0,
        {
            "t": float(t),
            "y_rov": float(y_rov),
            "L": float(L),
            "Fx_rov": float(Fx_rov),
            "Fy_rov": float(Fy_rov),
            "T_boat": None if T_boat is None else float(T_boat),
            "x_rov": float(x_rov),
            "x_boat": float(x_boat),
            "dt": None,
            "dL": None,
            "dT_boat": None,
            "dL_dt": None,
            "dl_dt_cmd_prev": None,
        },
    )

    if len(history) < 2:
        return 0.0, "HISTORY_LT2"

    curr = history[0]
    prev = history[1]
    curr["dL"] = curr["L"] - prev["L"]
    curr["dT_boat"] = curr["T_boat"] - prev["T_boat"] if curr["T_boat"] is not None and prev["T_boat"] is not None else 0.0
    curr["dt"] = curr["t"] - prev["t"]
    curr["dL_dt"] = curr["dL"] / curr["dt"] if curr["dt"] != 0 else 0.0
    if data and isinstance(data.get("T_rov"), list) and data["T_rov"]:
        curr["T_rov"] = data["T_rov"][-1]
    else:
        curr["T_rov"] = None

    if len(history) > K + 1:
        del history[K + 1 :]

    # Estimation locale dT/dL (si possible)
    est_dT_dL = None
    delta_L = history[-1]["L"] - history[0]["L"]
    delta_T = history[-1]["T_boat"] - history[0]["T_boat"]
    if delta_L != 0 and delta_T is not None:
        est_dT_dL = delta_T / delta_L

    # Distance droite bateau-ROV et marge de flèche minimale
    L_straight = math.hypot(curr["x_rov"] - curr["x_boat"], curr["y_rov"] - 0.0)
    prev_L_straight = math.hypot(prev["x_rov"] - prev["x_boat"], prev["y_rov"] - 0.0)
    curr["dL_straight"] = L_straight - prev_L_straight
    curr["dL_straight_dt"] = curr["dL_straight"] / curr["dt"] if curr["dt"] else 0.0
    slack = curr["L"] - L_straight
    
    # Marge de flèche volontairement plus élevée pour éviter les bascules fréquentes en "straight"
    slack_min = max(0.3, 0.005 * max(L_straight, 1.0))
    slack_max = max(slack_min * 4.0, 2.0)

    # Hystérésis pour éviter les bascules rapides autour de slack_min
    if not hasattr(auto_L_7, "_slack_mode"):
        auto_L_7._slack_mode = None
    if slack < slack_min:
        auto_L_7._slack_mode = "ADD"
    elif slack > slack_min * 1.3:
        auto_L_7._slack_mode = None

    # Atténuation douce proche du plafond de slack pour éviter le pompage
    slack_cap_ratio = None
    if slack_max > 1e-6:
        slack_cap_ratio = slack / slack_max

    def _last_val(key):
        if not data:
            return None
        vals = data.get(key)
        return vals[-1] if isinstance(vals, list) and vals else None

    def _fmt(val):
        # Renvoie un formatage de la valeur val avec 2 chiffres après la virgule
        return f"{val: 3.2f}" if isinstance(val, (int, float)) else "None"
    
    def _fmt_vec(val):
        # Renvoie un formatage de la valeur val avec 2 chiffres après la virgule
        if isinstance(val, (list, tuple)) and len(val) == 2:
            return f"({val[0]: 3.2f}, {val[1]: 3.2f})"
        return "None"

    # Utiliser t (qui est t_current) pour afficher le temps de l'itération en cours
    # t_current est le même pour auto_L_7, intégration et vérification invariants
    t_display = curr.get('t') if curr.get('t') is not None else t

    # Priorité 1: éviter Trupt
    t_soft = 0.80 * Trupt
    t_hard = 0.90 * Trupt
    if curr["T_boat"] is None or Tcible in (None, 0):
        trace_print(debug_auto_L_7, "IC_L_7: "
            f"t={_fmt(t_display)}, "
            f"L={_fmt(curr.get('L'))}, "  # Utiliser curr['L'] qui correspond à L passé en paramètre
            f"L_straight={L_straight: 6.2f}, "
            f"slack={slack: 6.2f}, "
            f"T_bat={_fmt(_last_val('T_boat'))}, "
            f"Fy={_fmt(_last_val('Fy_traction'))}, "
            "desired=None, ret= 0.00, exp=NO_T"
        )
        return _finalize(0.0, "NO_T")

    # Priorité 1: éviter Trupt (avec hystérésis)
    t_soft = 0.90 * Trupt
    t_hard = 0.98 * Trupt
    t_soft_off = 0.88 * Trupt
    t_hard_off = 0.95 * Trupt

    # Si le slack est important, on évite de déclencher T_HARD/T_SOFT
    # (physiquement, un câble très mou ne peut pas être en tension "hard")
    slack_ratio = slack / max(L_straight, 1.0)
    tension_block_ratio = 0.02  # 2% de la distance droite

    # Mode tension persistant pour éviter le pompage
    if not hasattr(auto_L_7, "_tension_mode"):
        auto_L_7._tension_mode = None
    if not hasattr(auto_L_7, "_tension_mode_age"):
        auto_L_7._tension_mode_age = 0

    # Filtrage léger de T pour éviter les bascules rapides
    t_raw = 0.0 if curr["T_boat"] is None else float(curr["T_boat"])
    t_prev_f = float(prev.get("T_f", t_raw))
    t_f = t_prev_f + 0.3 * (t_raw - t_prev_f)
    curr["T_f"] = t_f

    if slack_ratio <= tension_block_ratio:
        if auto_L_7._tension_mode == "HARD":
            auto_L_7._tension_mode_age += 1
            if auto_L_7._tension_mode_age < 3 or t_f >= t_hard_off:
                desired = dl_dt_max_val
                exp = "T_HARD"
            else:
                auto_L_7._tension_mode = None
                auto_L_7._tension_mode_age = 0
        if auto_L_7._tension_mode == "SOFT":
            auto_L_7._tension_mode_age += 1
            if auto_L_7._tension_mode_age < 3 or t_f >= t_soft_off:
                ramp = (t_f - t_soft) / max(t_hard - t_soft, 1e-6)
                desired = 0.5 * dl_dt_max_val + 0.5 * dl_dt_max_val * max(min(ramp, 1.0), 0.0)
                exp = "T_SOFT"
            else:
                auto_L_7._tension_mode = None
                auto_L_7._tension_mode_age = 0

    if slack_ratio <= tension_block_ratio and auto_L_7._tension_mode is None:
        if t_f >= t_hard:
            auto_L_7._tension_mode = "HARD"
            auto_L_7._tension_mode_age = 0
            desired = dl_dt_max_val
            exp = "T_HARD"
        elif t_f >= t_soft:
            auto_L_7._tension_mode = "SOFT"
            auto_L_7._tension_mode_age = 0
            ramp = (t_f - t_soft) / max(t_hard - t_soft, 1e-6)
            desired = 0.5 * dl_dt_max_val + 0.5 * dl_dt_max_val * max(min(ramp, 1.0), 0.0)
            exp = "T_SOFT"
        else:
            auto_L_7._tension_mode = None
            auto_L_7._tension_mode_age = 0
    if slack_ratio > tension_block_ratio:
        auto_L_7._tension_mode = None
        auto_L_7._tension_mode_age = 0

    if auto_L_7._tension_mode is None:
        # Priorité 2: intention pilote (descendre => éviter câble tendu)
        if Fy_rov < 0:
            # Si le slack est insuffisant, augmenter dL/dt de manière proportionnelle
            slack_err = slack_min - slack
            if auto_L_7._slack_mode == "ADD":
                slack_err = max(slack_err, 1e-6)
            if slack_err > 0:
                err_ratio = slack_err / max(slack_min, 1e-6)
                desired = min(dl_dt_max_val, 0.2 + 1.2 * err_ratio)
                exp = "PILOT_DESC_ADD_SLACK"
            else:
                desired = 0.1
                exp = "PILOT_DESC"
            # Assurer un débit minimal quand la distance droite augmente (descente/éloignement)
            if slack < slack_max and curr["dt"] and (slack_cap_ratio is None or slack_cap_ratio < 0.9):
                # Forcer un débit mini plus fort quand la distance droite augmente
                # (descente rapide / éloignement), pour éviter une tension "bloquée".
                dL_straight_dt = max(0.0, curr.get("dL_straight_dt", 0.0))
                down_speed = max(0.0, -curr.get("vy_rov", 0.0))
                fy_boost = min(0.3, 0.005 * abs(Fy_rov))
                t_boost = 0.0
                if curr.get("T_boat") is not None and Tcible not in (None, 0):
                    t_boost = min(0.3, 0.01 * max(curr["T_boat"] - Tcible, 0.0))
                min_payout_raw = max(dL_straight_dt, 0.5 * down_speed) + 0.1 + fy_boost + t_boost
                prev_minpay = prev.get("min_payout_prev")
                if prev_minpay is None:
                    min_payout = min_payout_raw
                else:
                    min_payout = prev_minpay + 0.3 * (min_payout_raw - prev_minpay)
                curr["min_payout_prev"] = min_payout

                # Hystérésis MINPAY pour éviter les bascules rapides
                if not hasattr(auto_L_7, "_minpay_active"):
                    auto_L_7._minpay_active = False
                if auto_L_7._minpay_active:
                    if min_payout < desired - 0.05:
                        auto_L_7._minpay_active = False
                else:
                    if min_payout > desired + 0.05:
                        auto_L_7._minpay_active = True

                if auto_L_7._minpay_active and desired < min_payout:
                    desired = min_payout
                    exp = f"{exp}_MINPAY"
            else:
                # Slack proche/au-dessus du plafond -> éviter les bascules MINPAY
                if not hasattr(auto_L_7, "_minpay_active"):
                    auto_L_7._minpay_active = False
                auto_L_7._minpay_active = False
        else:
            # Priorité 3: remontée (Fy >= 0) -> réduire le slack sans passer en straight
            if Fy_rov >= 0:
                slack_err = slack - slack_min
                if slack_err > 0:
                    err_ratio = slack_err / max(slack_min, 1e-6)
                    desired = -min(dl_dt_max_val, 0.1 + 1.0 * err_ratio)
                    exp = "PILOT_ASC_REMOVE_SLACK"
                else:
                    desired = 0.0
                    exp = "PILOT_ASC_HOLD"
            else:
                # Priorité 4: rester en caténaire si possible
                if slack < slack_min:
                    slack_err = slack_min - slack
                    err_ratio = slack_err / max(slack_min, 1e-6)
                    desired = min(dl_dt_max_val, 0.1 + 1.0 * err_ratio)
                    exp = "CATENARY_ADD_SLACK"
                elif slack > slack_max:
                    desired = -min((slack - slack_max) / max(slack_max, 1e-6), dl_dt_max_val)
                    exp = "CATENARY_REMOVE_SLACK"
                else:
                    # Régulation douce de tension autour de Tcible
                    if est_dT_dL is None or abs(est_dT_dL) < 1e-4:
                        desired = 0.0
                        exp = "CATENARY_HOLD"
                    else:
                        desired = -(curr["T_boat"] - Tcible) / (2.0 * est_dT_dL)
                        exp = "CATENARY_REG"

    # Anti-bagottement doux quand le slack est proche du plafond
    if slack_cap_ratio is not None and slack_cap_ratio >= 0.8:
        k = min(max((slack_cap_ratio - 0.8) / 0.2, 0.0), 1.0)
        desired = (1.0 - k) * desired + k * (-0.1)
        exp = f"{exp}_SLACK_CAP"

    # Lissage de la commande
    prev_cmd = prev.get("dl_dt_cmd_prev")
    if prev_cmd is None:
        prev_cmd = curr["dL_dt"] if curr["dL_dt"] is not None else 0.0
    prev_desired = prev.get("desired_cmd_prev")
    if prev_desired is None:
        prev_desired = desired
    # Limiter la vitesse de variation de desired (slew-rate)
    max_rate = 2.0
    if slack_cap_ratio is not None and slack_cap_ratio >= 0.8:
        max_rate = 0.6
    if curr.get("dt"):
        max_step = max_rate * curr["dt"]
        delta = max(min(desired - prev_desired, max_step), -max_step)
        desired = prev_desired + delta
    else:
        desired = prev_desired + 0.3 * (desired - prev_desired)
    curr["desired_cmd_prev"] = desired
    ret1 = prev_cmd + (desired - prev_cmd) / max(N, 1)

    # Limitation d'accélération du moulinet
    if curr["dt"] and (Gamma_moulinet_min is not None or Gamma_moulinet_max is not None):
        gmin = -0.5 if Gamma_moulinet_min is None else float(Gamma_moulinet_min)
        gmax = 0.5 if Gamma_moulinet_max is None else float(Gamma_moulinet_max)
        if gmin > gmax:
            gmin, gmax = gmax, gmin
        max_delta_pos = gmax * curr["dt"]
        max_delta_neg = gmin * curr["dt"]
        ret1 = prev_cmd + min(max(ret1 - prev_cmd, max_delta_neg), max_delta_pos)
        exp = f"{exp}_ACC"

    ret = ret1

    # On borne si besoin ret.
    ret, exp = _finalize(ret, exp)

    curr["dl_dt_cmd_prev"] = ret

    # DEBUG: Vérifier cohérence entre L passé en paramètre et curr['L']
    L_param = L  # L passé en paramètre à auto_L_7
    L_curr = curr.get('L')
    if L_curr is not None and abs(L_param - L_curr) > 1e-2:
        trace_print(debug_auto_L_7, f"[DEBUG L] auto_L_7 INCOHÉRENCE: "
            f"L_param={L_param:.6f} m, curr['L']={L_curr:.6f} m, "
            f"ÉCART={abs(L_param - L_curr):.6f} m"
        )
    
    # Utiliser les valeurs passées en paramètre plutôt que _last_val() pour garantir la cohérence
    # T_boat est passé directement en paramètre
    T_boat_display = T_boat if T_boat is not None else curr.get("T_boat")
    # T_rov et cable_drag viennent de data (itération précédente) car non disponibles en paramètre
    T_rov_display = curr.get('T_rov')
    cable_drag_display = _last_val('cable_drag')
    
    trace_print(debug_auto_L_7, "\nIC_L_7: "
        f"t={_fmt(t_display)}, "
        f"L={_fmt(curr.get('L'))}, "  # Utiliser curr['L'] qui correspond à L passé en paramètre
        f"L_straight={L_straight: 6.2f}, "
        f"slack={slack: 6.2f}, "
        f"T_bat={_fmt(T_boat_display)}, "  # Utiliser T_boat passé en paramètre
        f"T_rov={_fmt(T_rov_display)}, "  # Depuis data (itération précédente)
        f"F_drag_cable(prev_step)={_fmt_vec(cable_drag_display)}, "  # Depuis data (itération précédente)
        f"Fy_cmd={Fy_rov: 6.2f}, "
        f"desired={desired: 6.2f}, ret={ret: 6.2f}, exp={exp}"
    )
    # DEBUG: Traçage supplémentaire pour diagnostic
    trace_print(debug_auto_L_7-2, f"[DEBUG L] IC_L_7: t={_fmt(t_display)}, "
        f"L_param={L_param:.6f} m, curr['L']={_fmt(L_curr)}, "
        f"L_last_data={_fmt(_last_val('L'))} m"
    )

    return ret, exp


def auto_L_8(
    t: float,
    y_rov: float,
    L: float,
    L_straight: float | None = None,
    T_boat: float | None = None,
    T_rov: float | None = None,
    Fx_rov: float | None = None,
    Fy_rov: float | None = None,
    x_rov: float | None = None,
    x_boat: float | None = None,
    y_boat: float = 0.0, 
    z_rov: float | None = None,
    z_boat: float | None = None,
    Trupt: float = 100.0,
    Tcible: float = 50.0,
    Gamma_moulinet_min: float = -0.5,
    Gamma_moulinet_max: float = 0.5,
    dl_dt_min: float = -1.0,
    dl_dt_max: float = 1.0,
    # Nombre d'itérations utilisées dans l'historique.
    K: int = 10,
    # Nombre de pas de temps utilisés pour lisser la commande.
    N: int = 10,
    data: dict | None = None,
) -> tuple[float, str]:
    """
    Loi de commande dL/dt avec accès à la distance droite bateau-ROV.
    Priorités:
    1) ne pas dépasser la tension de rupture,
    2) respecter l'intention du pilote (Fy_rov < 0 => descendre, éviter câble tendu),
    3) rester en mode caténaire si possible,
    4) éviter les à-coups (lissage + limite d'accélération).
    Retourne (ret, explication).
    Paramètres utilisés: t, y_rov, L, L_straight, Fx_rov, Fy_rov, T_boat, x_rov, x_boat,
    Trupt, Tcible, Gamma_moulinet_min, Gamma_moulinet_max, dl_dt_min, dl_dt_max, data, K, N.
    """

    debug_auto_L_8 = 9

    if not hasattr(auto_L_8, "_history"):
        auto_L_8._history = []

    # Contrôle de la géométrie du câble
    if data is not None:
        x_cable_curr = data.get('x_cable_curr')
        y_cable_curr = data.get('y_cable_curr')
        x_boat_curr = data.get('x_boat')
        y_boat_curr = 0.0  # Le bateau est toujours à y=0
        if (x_cable_curr is not None and y_cable_curr is not None and
            x_boat_curr is not None and len(x_boat_curr) > 0):
            x_boat_val = float(x_boat_curr[-1]) if isinstance(x_boat_curr, list) else float(x_boat_curr)
            x_cable_val = x_cable_curr if isinstance(x_cable_curr, list) else list(x_cable_curr)
            y_cable_val = y_cable_curr if isinstance(y_cable_curr, list) else list(y_cable_curr)
            r1, r2, r3, r4, r5, ls, msg_err = contrôle_câble(
                x_cable_val, y_cable_val,
                x_boat_val, y_boat_curr,
                x_rov, y_rov,
                L,
                tol_segment=1e-2
            )
            
            if msg_err:
                error_msg = f"IC_L_8 :  t={t:.3f} s    {_ANSI_RED}Erreur contrôle_câble:{_ANSI_RESET}\n" + msg_err
                trace_print(debug_auto_L_8, error_msg)
                # Ne pas lever d'exception pour ne pas interrompre la simulation,
                # mais afficher l'erreur dans les logs

    def _finalize(ret, exp):
        ret = max(min(ret, dl_dt_max), dl_dt_min)
        return ret, exp

    history = auto_L_8._history
    # 0n insère une nouvelle entrée en début d'historique
    history.insert(
        0,
        {
            "t": float(t),
            "y_rov": float(y_rov),
            "L": float(L),
            "L_straight": None if L_straight is None else float(L_straight),
            "dy_rov": None,
            "vy_rov": None,
            "slack": None,
            "Fx_rov": None if Fx_rov is None else float(Fx_rov),
            "Fy_rov": None if Fy_rov is None else float(Fy_rov),
            "T_boat": None if T_boat is None else float(T_boat),
            "T_rov": None if T_rov is None else float(T_rov),
            "x_rov": None if x_rov is None else float(x_rov),
            "x_boat": None if x_boat is None else float(x_boat),
            "dt": None,
            "dL": None,
            "dT_boat": None,
            "dL_dt": None,
            "dL_straight": None,
            "dL_straight_dt": None,
            "dl_dt_cmd_prev": None,
        },
    )

    if len(history) < 2:
        return 0.0, "HISTORY_LT2"

    if len(history) > K + 1:
        del history[K + 1 :]

    curr = history[0]
    prev = history[1]

    curr["dt"] = curr["t"] - prev["t"]
    dt = curr["dt"]
    if curr["dt"] <= 0:
        return 0.0, "DT_NON_POSITIF"
    
    curr["dL"] = curr["L"] - prev["L"]
    curr["dT_boat"] = curr["T_boat"] - prev["T_boat"]      
    curr["dL_dt"] = curr["dL"] / curr["dt"]

    # Estimation de dT/dL sur l'ensemble de la fenêtre historisée
    est_dT_dL = None
    delta_L = history[0]["L"] - history[-1]["L"]
    delta_T = history[0]["T_boat"] - history[-1]["T_boat"]
    est_dT_dL = delta_T / delta_L if delta_L != 0 else None

    # Estimation de dy_rov/dt sur l'ensemble de la fenêtre historisée
    if not prev["y_rov"] is None:
        dy_rov = y_rov - prev["y_rov"]
        curr["dy_rov"] = dy_rov
        curr["vy_rov"] = dy_rov / curr["dt"]

    # Calcul de L_straight si c'est possible
    if L_straight is None:
        if (not x_boat is None and not y_boat is None and not z_boat is None 
            and not x_rov is None and not y_rov is None and not z_rov is None):
            L_straight = math.hypot(x_rov - x_boat, y_rov - y_boat, z_rov - z_boat)
            curr["L_straight"] = L_straight
    else :
        curr["L_straight"] = L_straight

    # Cas dégradé: on ne connait pas L_straight.
    if L_straight is None:
        if T_boat > Tcible:
            new_dL_dt = dl_dt_max
        else:
            new_dL_dt = dl_dt_min
        # Lissage de la commande
        new_dL_dt = ((N-1)*curr["dL_dt"] + new_dL_dt) / N
        return _finalize(new_dL_dt, "NO_L_STRAIGHT")


    # Distance droite bateau-ROV
    if not prev["L_straight"] is None:
        curr["dL_straight"] = L_straight - prev["L_straight"]
        curr["dL_straight_dt"] = curr["dL_straight"] / curr["dt"]

    # Slack
    if not L_straight is None:
        curr["slack"] = curr["L"] - L_straight
        slack = curr["slack"]
    else:
        slack = None

    # Marge de flèche volontairement plus élevée pour éviter les bascules fréquentes en "straight"
    slack_min = max(0.3, 0.005 * max(L_straight, 1.0))
    slack_max = max(slack_min * 4.0, 2.0)

    # Hystérésis pour éviter les bascules rapides autour de slack_min
    if not hasattr(auto_L_8, "_slack_mode"):
        auto_L_8._slack_mode = None
    if slack < slack_min:
        auto_L_8._slack_mode = "ADD"
    elif slack > slack_min * 1.3:
        auto_L_8._slack_mode = None

    # Atténuation douce proche du plafond de slack pour éviter le pompage
    slack_cap_ratio = None
    if slack_max > 1e-6:
        slack_cap_ratio = slack / slack_max

    def _last_val(key):
        if not data:
            return None
        vals = data.get(key)
        return vals[-1] if isinstance(vals, list) and vals else None

    def _fmt(val):
        return f"{val: 3.2f}" if isinstance(val, (int, float)) else "None"

    def _fmt_vec(val):
        if isinstance(val, (list, tuple)) and len(val) == 2:
            return f"({val[0]: 3.2f}, {val[1]: 3.2f})"
        return "None"

    t_display = curr.get("t") if curr.get("t") is not None else t

    if curr["T_boat"] is None or Tcible == 0:
        trace_print(
            debug_auto_L_8,
            "IC_L_8: "
            f"t={_fmt(t_display)}, "
            f"L={_fmt(curr.get('L'))}, "
            f"L_straight={L_straight: 6.2f}, "
            f"slack={slack: 6.2f}, "
            f"T_bat={_fmt(_last_val('T_boat'))}, "
            f"Fy={_fmt(_last_val('Fy_traction'))}, "
            "desired=None, ret= 0.00, exp=NO_T",
        )
        return _finalize(0.0, "NO_T")

    # Priorité 1: éviter Trupt (avec hystérésis)
    t_soft = 0.90 * Trupt
    t_hard = 0.98 * Trupt
    t_soft_off = 0.88 * Trupt
    t_hard_off = 0.95 * Trupt

    # Si le slack est important, on évite de déclencher T_HARD/T_SOFT
    slack_ratio = slack / max(L_straight, 1.0)
    tension_block_ratio = 0.02  # 2% de la distance droite

    if not hasattr(auto_L_8, "_tension_mode"):
        auto_L_8._tension_mode = None
    if not hasattr(auto_L_8, "_tension_mode_age"):
        auto_L_8._tension_mode_age = 0

    t_raw = float(curr["T_boat"])
    t_prev_f = float(prev.get("T_f", t_raw))
    t_f = t_prev_f + 0.3 * (t_raw - t_prev_f)
    curr["T_f"] = t_f

    if slack_ratio <= tension_block_ratio:
        if auto_L_8._tension_mode == "HARD":
            auto_L_8._tension_mode_age += 1
            if auto_L_8._tension_mode_age < 3 or t_f >= t_hard_off:
                desired = dl_dt_max
                exp = "T_HARD"
            else:
                auto_L_8._tension_mode = None
                auto_L_8._tension_mode_age = 0
        if auto_L_8._tension_mode == "SOFT":
            auto_L_8._tension_mode_age += 1
            if auto_L_8._tension_mode_age < 3 or t_f >= t_soft_off:
                ramp = (t_f - t_soft) / max(t_hard - t_soft, 1e-6)
                desired = 0.5 * dl_dt_max + 0.5 * dl_dt_max * max(min(ramp, 1.0), 0.0)
                exp = "T_SOFT"
            else:
                auto_L_8._tension_mode = None
                auto_L_8._tension_mode_age = 0

    if slack_ratio <= tension_block_ratio and auto_L_8._tension_mode is None:
        if t_f >= t_hard:
            auto_L_8._tension_mode = "HARD"
            auto_L_8._tension_mode_age = 0
            desired = dl_dt_max
            exp = "T_HARD"
        elif t_f >= t_soft:
            auto_L_8._tension_mode = "SOFT"
            auto_L_8._tension_mode_age = 0
            ramp = (t_f - t_soft) / max(t_hard - t_soft, 1e-6)
            desired = 0.5 * dl_dt_max + 0.5 * dl_dt_max * max(min(ramp, 1.0), 0.0)
            exp = "T_SOFT"
        else:
            auto_L_8._tension_mode = None
            auto_L_8._tension_mode_age = 0
    if slack_ratio > tension_block_ratio:
        auto_L_8._tension_mode = None
        auto_L_8._tension_mode_age = 0

    fy_cmd = float(Fy_rov or 0.0)
    if auto_L_8._tension_mode is None:
        if fy_cmd < 0:
            slack_err = slack_min - slack
            if auto_L_8._slack_mode == "ADD":
                slack_err = max(slack_err, 1e-6)
            if slack_err > 0:
                err_ratio = slack_err / max(slack_min, 1e-6)
                desired = min(dl_dt_max, 0.2 + 1.2 * err_ratio)
                exp = "PILOT_DESC_ADD_SLACK"
            else:
                desired = 0.1
                exp = "PILOT_DESC"
            if slack < slack_max and curr["dt"] and (slack_cap_ratio is None or slack_cap_ratio < 0.9):
                dL_straight_dt = max(0.0, curr.get("dL_straight_dt", 0.0))
                down_speed = max(0.0, -curr.get("vy_rov", 0.0))
                fy_boost = min(0.3, 0.005 * abs(fy_cmd))
                t_boost = min(0.3, 0.01 * max(curr["T_boat"] - Tcible, 0.0)) if Tcible != 0 else 0.0
                min_payout_raw = max(dL_straight_dt, 0.5 * down_speed) + 0.1 + fy_boost + t_boost
                prev_minpay = prev.get("min_payout_prev")
                if prev_minpay is None:
                    min_payout = min_payout_raw
                else:
                    min_payout = prev_minpay + 0.3 * (min_payout_raw - prev_minpay)
                curr["min_payout_prev"] = min_payout

                if not hasattr(auto_L_8, "_minpay_active"):
                    auto_L_8._minpay_active = False
                if auto_L_8._minpay_active:
                    if min_payout < desired - 0.05:
                        auto_L_8._minpay_active = False
                else:
                    if min_payout > desired + 0.05:
                        auto_L_8._minpay_active = True

                if auto_L_8._minpay_active and desired < min_payout:
                    desired = min_payout
                    exp = f"{exp}_MINPAY"
            else:
                if not hasattr(auto_L_8, "_minpay_active"):
                    auto_L_8._minpay_active = False
                auto_L_8._minpay_active = False
        elif fy_cmd >= 0:
            if slack < slack_min:
                slack_err = slack_min - slack
                err_ratio = slack_err / max(slack_min, 1e-6)
                desired = min(dl_dt_max, 0.2 + 1.2 * err_ratio)
                exp = "PILOT_ASC_ADD_SLACK"
            else:
                slack_err = slack - slack_min
                if slack_err > 0:
                    err_ratio = slack_err / max(slack_min, 1e-6)
                    desired = -min(dl_dt_max, 0.1 + 1.0 * err_ratio)
                    exp = "PILOT_ASC_REMOVE_SLACK"
                else:
                    desired = 0.0
                    exp = "PILOT_ASC_HOLD"
        else:
            if slack < slack_min:
                slack_err = slack_min - slack
                err_ratio = slack_err / max(slack_min, 1e-6)
                desired = min(dl_dt_max, 0.1 + 1.0 * err_ratio)
                exp = "CATENARY_ADD_SLACK"
            elif slack > slack_max:
                desired = -min((slack - slack_max) / max(slack_max, 1e-6), dl_dt_max)
                exp = "CATENARY_REMOVE_SLACK"
            elif est_dT_dL is None or abs(est_dT_dL) < 1e-4:
                desired = 0.0
                exp = "CATENARY_HOLD"
            else:
                desired = -(curr["T_boat"] - Tcible) / (2.0 * est_dT_dL)
                exp = "CATENARY_REG"

    if slack_cap_ratio is not None and slack_cap_ratio >= 0.8:
        k = min(max((slack_cap_ratio - 0.8) / 0.2, 0.0), 1.0)
        desired = (1.0 - k) * desired + k * (-0.1)
        exp = f"{exp}_SLACK_CAP"

    prev_cmd = prev.get("dl_dt_cmd_prev")
    if prev_cmd is None:
        prev_cmd = curr["dL_dt"] if curr["dL_dt"] is not None else 0.0
    prev_desired = prev.get("desired_cmd_prev")
    if prev_desired is None:
        prev_desired = desired
    max_rate = 2.0
    if slack_cap_ratio is not None and slack_cap_ratio >= 0.8:
        max_rate = 0.6
    if curr.get("dt"):
        max_step = max_rate * curr["dt"]
        delta = max(min(desired - prev_desired, max_step), -max_step)
        desired = prev_desired + delta
    else:
        desired = prev_desired + 0.3 * (desired - prev_desired)
    curr["desired_cmd_prev"] = desired
    ret1 = prev_cmd + (desired - prev_cmd) / max(N, 1)

    if curr["dt"]:
        gmin = float(Gamma_moulinet_min)
        gmax = float(Gamma_moulinet_max)
        if gmin > gmax:
            gmin, gmax = gmax, gmin
        max_delta_pos = gmax * curr["dt"]
        max_delta_neg = gmin * curr["dt"]
        ret1 = prev_cmd + min(max(ret1 - prev_cmd, max_delta_neg), max_delta_pos)
        exp = f"{exp}_ACC"

    ret, exp = _finalize(ret1, exp)

    # Garde-fou : L doit rester >= L_straight (le ROV peut rapprocher le bateau plus vite que le moulinet)
    if L_straight is not None and slack is not None:
        slack_margin = max(0.05, 0.001 * max(L_straight, 1.0))
        if slack < slack_margin:
            dt_eff = max(float(curr.get("dt") or 0.0), 1e-6)
            d_ls_dt = float(curr.get("dL_straight_dt") or 0.0)
            floor = d_ls_dt + (slack_margin - slack) / dt_eff
            if ret < floor:
                ret = min(max(floor, dl_dt_min), dl_dt_max)
                exp = f"{exp}_SLACK_FLOOR"

    curr["dl_dt_cmd_prev"] = ret

    L_param = L
    L_curr = curr.get("L")
    if L_curr is not None and abs(L_param - L_curr) > 1e-2:
        trace_print(
            debug_auto_L_8,
            f"[DEBUG L] auto_L_8 INCOHÉRENCE: L_param={L_param:.6f} m, "
            f"curr['L']={L_curr:.6f} m, ÉCART={abs(L_param - L_curr):.6f} m",
        )

    T_boat_display = T_boat if T_boat is not None else curr.get("T_boat")
    T_rov_display = curr.get("T_rov")
    cable_drag_display = _last_val("cable_drag")

    trace_print(
        debug_auto_L_8,
        "\nIC_L_8: "
        f"t={_fmt(t_display)}, "
        f"L={_fmt(curr.get('L'))}, "
        f"L_straight={L_straight: 6.2f}, "
        f"slack={slack: 6.2f}, "
        f"T_bat={_fmt(T_boat_display)}, "
        f"T_rov={_fmt(T_rov_display)}, "
        f"F_drag_cable(prev_step)={_fmt_vec(cable_drag_display)}, "
        f"Fy_cmd={fy_cmd: 6.2f}, "
        f"desired={desired: 6.2f}, ret={ret: 6.2f}, exp={exp}",
    )
    trace_print(
        debug_auto_L_8 - 2,
        f"[DEBUG L] IC_L_8: t={_fmt(t_display)}, L_param={L_param:.6f} m, "
        f"curr['L']={_fmt(L_curr)}, L_last_data={_fmt(_last_val('L'))} m",
    )

    return ret, exp

def consigne_moulinet(
    
    t: float, # Le temps exprimé en secondes
    y_rov: float| None = None, # La profondeur du rov exprimée en mètres. ELle doit être négative
    T_boat: float| None = None, # La tension du câble au niveau du bateau exprimée en Newtons. Elle doit être positive.
    L: float| None = None, # La longueur de câble entre le bateau et le ROV exprimée en mètres
    L_straight: float | None = None, # La distance en ligne droite bateau-ROV exprimée en mètres.
    T_rupture: float = 60.0, # La tension de rupture du câble exprimée en Newtons. 
    T_cible: float = 20.0, # La tension cible du câble au niveau du bateau exprimée en Newtons
    dL_dt_max: float = 0.81,  # La consigne maximale de vitesse du moulinet exprimée en m/s
    dL_dt_min: float = -0.81, # La consigne minimale de vitesse du moulinet exprimée en m/s
    d2L_dt2_min: float = -0.2478, # Décélération limite du fil (en m/s2)
    d2L_dt2_max: float = 0.1239, # Accélération limite du fil (en m/s2)
    K: int = 10, # Nombre d'itérations utilisées dans l'historique.
    N: int = 10, # Nombre de pas de temps utilisés pour lisser la commande.
    delta_t: float = 0.1, # Le pas de temps entre deux calculs de la consigne, exprimé en secondes.
) -> tuple[float, str]:
    f"""
    Loi de commande dL/dt avec ou sans utilisation de la distance droite bateau-ROV.
    Les paramètres pour lesquels une valeur par défaut est fournie peuvent être omis lors de l'appel.
    
    Les deux facteurs clés qui doivent être combinés pour générer la consigne de pilotage du moulinet sont 
    la tension du fil (T_boat) et le mou sur le câble (slack).  

    On calcule la contribution des deux facteurs et on les combine grace à une matrice empirique matrice_consigne[i_slack][i_T_boat] 
    - Plus la valeur donnée par la matrice est positive et élevée (jusqu'à +1), plus il faut donner du mou rapidement.
    - Plus la valeur donnée par la matrice est négative et faible (jusqu'à -1), plus il faut reprendre du mou rapidement.
    - La tension reste le facteur prioritaire.

    La fonction renvoie un tuple (consigne, expl) où :
    - consigne est la consigne de vitesse du moulinet (dL/dt) en m/s (elle est comprise entre dL_dt_max et dL_dt_min m/s)
    - expl est une chaîne de caractères qui explique la commande retournée.
    """

    debug_consigne_moulinet = 9
    print(f"consigne_moulinet: t={t:.2f}")

    def _moyenne_glissante(hist, name, N):
        """
        Moyenne des N valeurs les plus récentes.

        hist[0] est la plus récente. Les None sont ignorés. Le poids décroît
        avec l'âge : la valeur retenue la plus récente pèse n, la plus ancienne pèse 1.
        None s'il n'y a aucune valeur.
        """
        n_fenetre = min(int(N), len(hist))
        valeurs = []
        for i in range(n_fenetre):
            val = hist[i].get(name)
            if val is not None:
                valeurs.append(float(val))
        n = len(valeurs)
        if n == 0:
            return None
        sigma_poids = 0.0
        total = 0.0
        for rang, val in enumerate(valeurs):
            poids = float(n - rang)
            sigma_poids += poids
            total += val * poids
        return total / sigma_poids

    # Contrôle de vraisemblance sur les paramètres d'entrée:

    if y_rov is not None and y_rov > 0 :
        return 0.0, "ERR_Y_ROV_STRICTEMENT_POSITIF"
    if T_boat is not None and T_boat < 0:
        return 0.0, "ERR_T_BOAT_NEGATIF"
    if L is not None and L < 0:
        return 0.0, "ERR_L_NEGATIF"
    if L_straight is not None and L_straight < 0:
        return 0.0, "ERR_L_STRAIGHT_NEGATIF"
    if T_rupture <= 5.0:
        return 0.0, "ERR_T_RUPTURE_TROP_FAIBLE"
    if T_cible <= 2.0:
        return 0.0, "ERR_T_CIBLE_TROP_FAIBLE"
    if T_cible > T_rupture*2/3:
        return 0.0, "ERR_T_CIBLE_SUPERIEURE_A_66%_T_RUPTURE"
    if dL_dt_max <= dL_dt_min:
        return 0.0, "ERR_D_L_DT_MAX_INFERIEUR_A_D_L_DT_MIN"
    if K <= 0:
        return 0.0, "ERR_K_NON_POSITIF"
    if N < 5:
        return 0.0, "ERR_N_INFERIEUR_A_5"

    # Contrôle de cohérence entre L et L_straigh - TBI
    # if L_straight > L:
        pass

    if L_straight is not None:
        slack = L - L_straight
    else:
        slack = None

    # Gestion de l'historique des données reçues ou calculées. 
    # Cet historique est utilisé pour le lissage de la commande.
    # Il est conservé entre chaque appel de la fonction.

    if not hasattr(consigne_moulinet, "_history"):
        consigne_moulinet._history = []
    history = consigne_moulinet._history

    # 0n insère une nouvelle entrée en début d'historique
    history.insert(
        0,
        {
            "t": float(t),
            "dt": None,    
            "y_rov": float(y_rov) if y_rov is not None else None,
            "dy_rov": None,
            "vy_rov": None,
            "L": float(L) if L is not None else None,
            "dL": None,
            "dL_dt": None,
            "L_straight": float(L_straight) if L_straight is not None else None,
            "dL_straight_dt": None,
            "dL_straight_dt_prev": None,
            "T_boat": float(T_boat),
            "slack": slack,
            "consigne": None,
        },
    )

    # Contrôle vraisemblance dt

    if len(history) < 2:
        return 0.0, "HISTORY_LT2"

    if len(history) > K + 1:
        del history[K + 1 :]

    curr = history[0]
    prev = history[1]
    
    # Historique de t et dt
    dt= curr["t"] - prev["t"]
    curr["dt"] = dt
    if dt <= 0:
        return 0.0, "ERR_DT_NON_POSITIF"

    # Historique de y_rov et insertion dans le dictionnaire curr.
    dy_rov = None
    vy_rov = None
    prev_y_rov = prev.get("y_rov")
    if y_rov is not None and prev_y_rov is not None:
        dy_rov = (y_rov - prev_y_rov)/dt
        curr["dy_rov"] = dy_rov
    if dy_rov is not None  and dt > 0:
        vy_rov = dy_rov / dt
        curr["vy_rov"] = vy_rov

    # Historique de L
    dL = None
    dL_dt = None
    prev_L = prev.get("L")
    if L is not None and prev_L is not None:
        dL = L - prev_L
        curr["dL"] = dL
    if dL is not None and dt > 0:
        dL_dt = dL / dt
        curr["dL_dt"] = dL_dt

    # Historique de L_straight
    dL_straight = None
    dL_straight_dt = None
    prev_L_straight = prev.get("L_straight")
    if L_straight is not None and prev_L_straight is not None:
        dL_straight = L_straight - prev_L_straight
        curr["dL_straight"] = dL_straight
    if dL_straight is not None and dt > 0:
        dL_straight_dt = dL_straight / dt
        curr["dL_straight_dt"] = dL_straight_dt


    # Matrice de consigne.
    # matrice_consigne[i_slack][i_T_boat] 

    # i_slack = 0: Slack en dessus du slack max - Le câble est totalement détendu.
    # i_slack = 1: Slack très en dessus du slack cible 
    # i_slack = 2: Significativement au dessus du slack cible
    # i_slack = 3 Egal ou juste au dessus du slack cible - OK
    # i_slack = 4: Juste en dessous du slack cible - OK
    # i_slack = 5: Slack significativement en dessous du slack cible
    # i_slack = 6: Slack très en dessous du slack cible
    # i_slack = 7: Slack en dessous du slack min - On est en mode straight, ou presque.
    # i_slack = 8: Slack unknown - On ne connait pas le slack.

    # i_T_boat = 0: Tension en dessous de la tension min
    # i_T_boat = 1: Très en dessous de la cible
    # i_T_boat = 2: Significativement en dessous de la cible
    # i_T_boat = 3: Juste en dessous de la cible - OK
    # i_T_boat = 4: Egal ou juste au dessus de la cible
    # i_T_boat = 5: Significativement au dessus de la cible
    # i_T_boat = 6: Risque fort de rupture
    # i_T_boat = 7: Limite rupture atteinte ou dépassée

    # La matrice vérifie les règles de cohérence suivantes : 
    #     
    #     s' > s  ==> consigne[s'][T] >= consigne[s][T]   
    #     ie: plus le calable est en mode rectiligne, plus il faut donner du mou.
    #
    #     T' > T  ==> consigne[s][T'] >= consigne[s][T]
    #     ie: plus la tension est forte, plus il faut donner du mou.
    
    matrice_consigne = [
        [-1.00, -0.75, -0.50, -0.25, 0.25, 0.50, 0.75, 1.00],  
        [-0.75, -0.75, -0.50, -0.25, 0.25, 0.50, 0.75, 1.00],
        [-0.50, -0.50, -0.50, -0.25, 0.25, 0.50, 0.75, 1.00],
        [-0.25, -0.25, -0.25,  0.00, 0.25, 0.50, 0.75, 1.00],
        [ 0.25,  0.25,  0.25,  0.25, 0.25, 0.50, 0.75, 1.00],
        [ 0.25,  0.25,  0.25,  0.25, 0.50, 0.50, 0.75, 1.00],
        [ 0.50,  0.50,  0.50,  0.50, 0.50, 0.50, 0.75, 1.00],  
        [ 0.50,  0.50,  0.50,  0.50, 0.50, 0.75, 0.75, 1.00],  
        [-1.00, -1.00, -1.00,  0.00, 0.00, 1.00, 1.00, 1.00],  
    ]                                                         

    # Impact de la tension sur la future consigne
    # On calcule un indice i_T_boat
    T_min = T_cible /2
    t1 = T_min
    t2 = T_min + (T_cible-  T_min) / 3
    t3 = T_min + (T_cible - T_min) * 2 / 3
    t4 = T_cible
    t5 = T_cible + (T_rupture - T_cible) / 3         
    t6 = T_cible + (T_rupture - T_cible) * 2 / 3     
    t7 = T_rupture                                   
    # print(f"t1: {t1:.2f}, t2: {t2:.2f}, t3: {t3:.2f}, t4: {t4:.2f}, t5: {t5:.2f}, t6: {t6:.2f}, t7: {t7:.2f}")

    if T_boat < t1:
        i_T_boat = 0    # Tension en dessous de la tension min
    elif T_boat < t2:
        i_T_boat = 1    # Très en dessous de la cible
    elif T_boat < t3:
        i_T_boat = 2    # Significativement en dessous de la cible
    elif T_boat < t4:
        i_T_boat = 3    # Juste en dessous de la cible - OK
    elif T_boat < t5:
        i_T_boat = 4    # Egal ou juste au dessus de la cible
    elif T_boat < t6:
        i_T_boat = 5    # Significativement au dessus de la cible
    elif T_boat < t7:
        i_T_boat = 6    # Risque fort de rupture
    else:
        i_T_boat = 7    # Limite rupture atteibnte ou dépassée

    # Cas dégradé: on ne connait pas L_straight.
    if L_straight is None:
        # On ne connait pas L_straight, donc on en connait pas le slack.
            # On retient une valeur par défaut i_slack = 0 : Juste au dessus du slack cible - OK.
        i_slack = 8
        exp = "L_STRAIGHT_UNKNOWN"

    # Cas nominal, on connait L_straight
    else :
        slack_min   = 2 + 0.02 * L_straight
        slack_cible = 4 + 0.03 * L_straight
        slack_max   = 6 + 0.04 * L_straight

        # Impact de la tension sur la future consigne
        # On calcule un indice i_slack

        s6 = slack_min
        s5 = slack_min + (slack_cible - slack_min) / 3
        s4 = slack_min  + (slack_cible - slack_min ) * 2 / 3
        s3 = slack_cible
        s2 = slack_cible + (slack_max - slack_cible) / 3
        s1 = slack_cible + (slack_max - slack_cible) * 2 / 3
        s0 = slack_max

        # print(f"s0: {s0:.2f}, s1: {s1:.2f}, s2: {s2:.2f}, s3: {s3:.2f}, s4: {s4:.2f}, s5: {s5:.2f}, s6: {s6:.2f}")

        if slack < s6:
            i_slack = 7   # Slack en dessous dd slack min - On est en mode straight, ou presque.
        elif slack < s5:
            i_slack = 6    # Slack très en dessous du slack cible
        elif slack < s4:
            i_slack = 5    # Slack significativement en dessous du slack cible
        elif slack < s3:
            i_slack = 4    # Juste en dessous du slack cible - OK
        elif slack < s2:
            i_slack = 3    # Egal ou juste au dessus du slack cible - OK
        elif slack < s1:
            i_slack = 2    # Significativement au dessus du slack cible
        elif slack < s0:
            i_slack = 1    # Slack très en dessus du slack cible
        else:
            i_slack = 0    # Slack en dessus du slack max - Le câble est totalement détendu.

        exp = f"{_ANSI_GREEN}L_STRAIGHT_KNOWN{_ANSI_RESET}"

    # Calcul de la consigne brute souhaitée
    mat = matrice_consigne[i_slack][i_T_boat]
    if mat > 0:
        consigne = dL_dt_max * mat
    else:
        consigne = abs(dL_dt_min) *mat 
    exp = exp + f" - Matrice_consigne {mat:.2f} --> Consigne brute: {consigne:.2f}"

    # Prise en compte de la vitesse du ROV
    if dL_straight_dt is not None:
        consigne = consigne + dL_straight_dt
        exp = exp + f"{_ANSI_GREEN} + dL_straight_dt {dL_straight_dt:.2f}{_ANSI_RESET}"
    else:
        if vy_rov is not None:
            consigne = consigne - vy_rov
            exp = exp + f"{_ANSI_CYAN} - vy_rov {vy_rov:.2f}{_ANSI_RESET}"

    # Lissage de la consigne
    m_consigne = _moyenne_glissante(history, "consigne", N)
    if m_consigne is not None:
        consigne_lissée = (consigne + 3*m_consigne) / 4
        exp = exp + f" - Lissage {consigne:.2f} --> {consigne_lissée:.2f} "
        consigne = consigne_lissée

    # Bridage vitesse de la consigne
    if consigne  < dL_dt_min:
        exp = exp + f" - Bridage vitesse min {consigne:.2f} --> {dL_dt_min:.2f}"
        consigne = dL_dt_min
        if consigne  > dL_dt_max:
            exp = exp + f" - Bridage vitesse max {consigne:.2f} --> {dL_dt_max:.2f}"
            consigne = dL_dt_max
            
    # Bridage  accélération de la consigne
    if dL_dt is not None:
        acc = (consigne - dL_dt)/dt
        if acc < d2L_dt2_min:
            exp = exp + f" - Bridage accélaration min {acc:.2f} --> {d2L_dt2_min:.2f}"
            acc = d2L_dt2_min
        if acc > d2L_dt2_max:
            exp = exp + f" - Bridage accélération max {acc:.2f} --> {d2L_dt2_max:.2f}"
            acc  = d2L_dt2_max
            
        consigne = dL_dt + acc*dt

    # Enregistrement de la consigne

    curr["consigne"] = consigne
    if slack is not None:
        slack_str = f"Slack: {slack:.2f} ({slack_min:.2f}, {slack_cible:.2f}, {slack_max:.2f})"
    else:
        slack_str = "Slack: UNKNOWN"

    print(f"dt: {dt:.2f}, {slack_str}, Tension: {T_boat:.2f}, i_slack: {i_slack}, i_T_boat: {i_T_boat}, {exp} --> consigne: {consigne:.2f} ")
    return consigne, exp
    


def auto_L_min(
    t: float,
    y_rov: float,
    L: float,
    L_straight: float | None = None,
    T_boat: float | None = None,
    T_rov: float | None = None,
    Fx_rov: float | None = None,
    Fy_rov: float | None = None,
    x_rov: float | None = None,
    x_boat: float | None = None,
    y_boat: float = 0.0, 
    z_rov: float = 0.0,
    z_boat: float = 0.0,
    Trupt: float = 100.0,
    Tcible: float = 50.0,
    Gamma_moulinet_min: float = -0.5,
    Gamma_moulinet_max: float = 0.5,
    dl_dt_min: float = -1.0,
    dl_dt_max: float = 1.0,
    # Nombre d'itérations utilisées dans l'historique.
    K: int = 10,
    # Nombre de pas de temps utilisés pour lisser la commande.
    N: int = 10,
    data: dict | None = None,
) -> tuple[float, str]:
    """
    Loi de commande dL/dt avec accès à la distance droite bateau-ROV.
    Priorités:
    1) ne pas dépasser la tension de rupture,
    2) respecter l'intention du pilote (Fy_rov < 0 => descendre, éviter câble tendu),
    3) rester en mode caténaire si possible,
    4) éviter les à-coups (lissage + limite d'accélération).
    Retourne (ret, explication).
    Paramètres utilisés: t, y_rov, L, L_straight, Fx_rov, Fy_rov, T_boat, x_rov, x_boat,
    Trupt, Tcible, Gamma_moulinet_min, Gamma_moulinet_max, dl_dt_min, dl_dt_max, data, K, N.
    """
    if L_straight is None:
        if (not x_boat is None and not y_boat is None and not z_boat is None 
        and not x_rov is None and not y_rov is None and not z_rov is None):
            L_straight = math.hypot(x_rov - x_boat, y_rov - y_boat, z_rov - z_boat)

    ret , exp = consigne_moulinet(t, y_rov=y_rov, T_boat=T_boat, L=L, L_straight= L_straight, T_rupture= Trupt, T_cible= Tcible, K= 10, N= 10)
    return ret, exp

