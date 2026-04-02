"""Validation de la syntaxe des scénarios."""
from __future__ import annotations

import re
import math
import numpy as np

from src.utils.logger import trace_print


def contrôle_câble(
    x_cable: list[float] | np.ndarray,
    y_cable: list[float] | np.ndarray,
    x_boat: float,
    y_boat: float,
    x_rov: float,
    y_rov: float,
    L: float,
    tol_segment: float = 1e-2,
) -> tuple[bool, bool, bool, bool, bool, list[int]]:
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
        Tolérance pour la longueur des segments (défaut: 1e-3 m)
    
    Returns:
    --------
    tuple[bool, bool, bool, bool, bool, list[int]]
        (r1, r2, r3, r4, r5, ls) où :
        - r1 : True si le point 0 correspond au bateau
        - r2 : True si le point N correspond au ROV
        - r3 : True si la somme des segments = L(t)
        - r4 : True si Slack(t) >= 0
        - r5 : True si tous les segments ont la longueur L(t)/N à tol_segment près
        - ls : Liste des indices de segments ne respectant pas la contrainte 5
    """
    # Convertir en arrays numpy si nécessaire
    x_cable = np.asarray(x_cable, dtype=float)
    y_cable = np.asarray(y_cable, dtype=float)
    
    N = len(x_cable) - 1  # Nombre de segments
    if N <= 0:
        return (False, False, False, False, False, [])
    
    # Contrainte 1 : Le point 0 correspond au bateau
    tol_position = 1e-2  # Tolérance pour la position
    r1 = (abs(x_cable[0] - x_boat) < tol_position and 
          abs(y_cable[0] - y_boat) < tol_position)
    
    # Contrainte 2 : Le point N correspond au ROV
    r2 = (abs(x_cable[-1] - x_rov) < tol_position and 
          abs(y_cable[-1] - y_rov) < tol_position)
    
    # Contrainte 3 : La somme des longueurs des N segments = L(t)
    L_total = 0.0
    segment_lengths = []
    for i in range(N):
        dx = x_cable[i+1] - x_cable[i]
        dy = y_cable[i+1] - y_cable[i]
        ds = np.sqrt(dx**2 + dy**2)
        segment_lengths.append(ds)
        L_total += ds
    
    tol_length = 1e-2  # Tolérance pour la longueur totale
    r3 = abs(L_total - L) < tol_length
    
    # Contrainte 4 : Slack(t) = L(t) - L_straight(t) >= 0
    dx_straight = x_rov - x_boat
    dy_straight = y_rov - y_boat
    L_straight = np.sqrt(dx_straight**2 + dy_straight**2)
    slack = L - L_straight
    r4 = slack >= -1e-2  # Tolérance de 1e-6 m pour le slack
    
    # Contrainte 5 : Chaque segment a une longueur = L(t)/N à tol_segment près
    ds_target = L / N if N > 0 else 0.0
    r5 = True
    ls = []  # Liste des segments ne respectant pas la contrainte 5
    for i in range(N):
        ds = segment_lengths[i]
        if abs(ds - ds_target) > tol_segment:
            r5 = False
            ls.append(i)
    
    return (r1, r2, r3, r4, r5, ls)

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
    Fx_rov: float,
    Fy_rov: float,
    T_boat: float | None,
    x_rov: float,
    x_boat: float,
    Trupt: float | None = None,
    Tcible: float | None = None,
    Gamma_moulinet_min: float | None = None,
    Gamma_moulinet_max: float | None = None,
    dl_dt_min: float | None = None,
    dl_dt_max: float | None = None,
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
            "T": None if T_boat is None else float(T_boat),
            "dt": None,
            "dL": None,
            "dT": None,
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
    curr["dT"] = curr["T"] - prev["T"]
    curr["dt"] = curr["t"] - prev["t"]
    curr["dL_dt"] = curr["dL"] / curr["dt"] if curr["dt"] != 0 else None

    
    
    # Priorité 1-2: sécurité rupture + intention pilote (descente)
    t_soft = 0.90 * Trupt
    t_hard = 0.98 * Trupt
    t_soft_off = 0.88 * Trupt
    t_hard_off = 0.95 * Trupt
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
    if abs(Tcible - prev["T"]) < 1:
        ret, exp = 0.0, "TCIBLE_PREV_CLOSE"
        ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
        if limited:
            exp = f"{exp}_ACC"
        curr["dl_dt_cmd_prev"] = ret
        return _finalize(ret, exp)

    z = (T_boat-Tcible)/abs(prev["T"]-Tcible)
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
    Fx_rov: float,
    Fy_rov: float,
    T_boat: float | None,
    x_rov: float,
    x_boat: float,
    Trupt: float | None = None,
    Tcible: float | None = None,
    Gamma_moulinet_min: float | None = None,
    Gamma_moulinet_max: float | None = None,
    dl_dt_min: float | None = None,
    dl_dt_max: float | None = None,
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
            "T": None if T_boat is None else float(T_boat),
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
    avg_T = sum(item["T"] for item in history[:J]) / J
    delta_L = history[-1]["L"] - history[0]["L"]
    delta_T = history[-1]["T"] - history[0]["T"]

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
    Fx_rov: float,
    Fy_rov: float,
    T_boat: float | None,
    x_rov: float,
    x_boat: float,
    Trupt: float | None = None,
    Tcible: float | None = None,
    Gamma_moulinet_min: float | None = None,
    Gamma_moulinet_max: float | None = None,
    dl_dt_min: float | None = None,
    dl_dt_max: float | None = None,
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
            "T": None if T_boat is None else float(T_boat),
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
    avg_T = sum(item["T"] for item in history[:J]) / J
    delta_L = history[-1]["L"] - history[0]["L"]
    delta_T = history[-1]["T"] - history[0]["T"]

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
    Fx_rov: float,
    Fy_rov: float,
    T_boat: float | None,
    x_rov: float,
    x_boat: float,
    Trupt: float | None = None,
    Tcible: float | None = None,
    Gamma_moulinet_min: float | None = None,
    Gamma_moulinet_max: float | None = None,
    dl_dt_min: float | None = None,
    dl_dt_max: float | None = None,
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
            "T": None if T_boat is None else float(T_boat),
            "dt": None,
            "dL": None,
            "dT": None,
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
    curr["dT"] = curr["T"] - prev["T"] if curr["T"] is not None and prev["T"] is not None else 0.0
    curr["dt"] = curr["t"] - prev["t"]
    curr["dL_dt"] = curr["dL"] / curr["dt"] if curr["dt"] != 0 else 0
    curr["y_rov"] = float(y_rov)
    prev["y_rov"] = float(prev.get("y_rov", y_rov))
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
            ret, limited = _apply_moulinet_accel_limit(ret, curr, prev, Gamma_moulinet_min, Gamma_moulinet_max)
            if limited:
                exp = f"{exp}_ACC"
            curr["dl_dt_cmd_prev"] = ret
            return _finalize(ret, exp)

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
    Fx_rov: float,
    Fy_rov: float,
    T_boat: float | None,
    x_rov: float,
    x_boat: float,
    Trupt: float | None = None,
    Tcible: float | None = None,
    Gamma_moulinet_min: float | None = None,
    Gamma_moulinet_max: float | None = None,
    dl_dt_min: float | None = None,
    dl_dt_max: float | None = None,
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
            "T": None if T_boat is None else float(T_boat),
            "dt": None,
            "dL": None,
            "dT": None,
            "dL_dt": None,
            "dl_dt_cmd_prev": None,
        },
    )

    if len(history) < 2:
        return _finalize(0.0, "HISTORY_LT2")

    curr = history[0]
    prev = history[1]
    curr["dL"] = curr["L"] - prev["L"]
    curr["dT"] = curr["T"] - prev["T"] if curr["T"] is not None and prev["T"] is not None else 0.0
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

    if curr["T"] is None or Tcible in (None, 0):
        return _finalize(0.0, "NO_T")

    # Estimation locale dT/dL (si possible)
    est_dT_dL = None
    delta_L = history[-1]["L"] - history[0]["L"]
    t_last = history[-1]["T"]
    t_first = history[0]["T"]
    if delta_L != 0 and t_last is not None and t_first is not None:
        est_dT_dL = (t_last - t_first) / delta_L

    # Priorité 1: éviter Trupt (avec hystérésis)
    t_soft = 0.90 * Trupt
    t_hard = 0.98 * Trupt
    t_soft_off = 0.88 * Trupt
    t_hard_off = 0.95 * Trupt
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
        f"T={curr['T']:.2f}, "
        f"prev_cmd={prev_cmd: 6.2f}, ret1={ret1: 6.2f}, ret={ret: 6.2f}, "
        f"dL_dt={curr['dL_dt']: 6.2f}, dt={curr['dt']: 6.2f},  "
        f"desired={desired: 6.2f}, exp={exp}"
    )
    return _finalize(ret, exp)


def auto_L_7(
    t: float,
    y_rov: float,
    L: float,
    Fx_rov: float,
    Fy_rov: float,
    T_boat: float | None,
    x_rov: float,
    x_boat: float,
    Trupt: float | None = None,
    Tcible: float | None = None,
    Gamma_moulinet_min: float | None = None,
    Gamma_moulinet_max: float | None = None,
    dl_dt_min: float | None = None,
    dl_dt_max: float | None = None,
    data: dict | None = None,
    # Nombre d'itérations utilisées dans l'historique.
    K: int = 10,
    # Nombre de pas de temps utilisés pour lisser la commande.
    N: int = 10,
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
            r1, r2, r3, r4, r5, ls = contrôle_câble(
                x_cable_val, y_cable_val,
                x_boat_val, y_boat_curr,
                x_rov, y_rov,
                L,
                tol_segment=1e-2
            )
            
            # Produire un message d'erreur si une contrainte n'est pas respectée
            errors = []
            if not r1:
                errors.append(f"Contrainte 1 violée : Le point 0 du câble ({x_cable_val[0]:.6f}, {y_cable_val[0]:.6f}) "
                            f"ne correspond pas au bateau ({x_boat_val:.6f}, {y_boat_curr:.6f})")
            if not r2:
                errors.append(f"Contrainte 2 violée : Le point N du câble ({x_cable_val[-1]:.6f}, {y_cable_val[-1]:.6f}) "
                            f"ne correspond pas au ROV ({x_rov:.6f}, {y_rov:.6f})")
            if not r3:
                L_total = sum(np.sqrt((x_cable_val[i+1] - x_cable_val[i])**2 + 
                                      (y_cable_val[i+1] - y_cable_val[i])**2) 
                             for i in range(len(x_cable_val) - 1))
                errors.append(f"Contrainte 3 violée : La somme des longueurs des segments ({L_total:.6f} m) "
                            f"n'est pas égale à L(t) ({L:.6f} m)")
            if not r4:
                dx_straight = x_rov - x_boat_val
                dy_straight = y_rov - y_boat_curr
                L_straight = np.sqrt(dx_straight**2 + dy_straight**2)
                slack = L - L_straight
                errors.append(f"Contrainte 4 violée : Slack(t) = {slack:.6f} m < 0 "
                            f"(L={L:.6f} m, L_straight={L_straight:.6f} m)")
            if not r5:
                N_seg = len(x_cable_val) - 1
                ds_target = L / N_seg if N_seg > 0 else 0.0
                errors.append(f"Contrainte 5 violée : {len(ls)} segment(s) ne respectent pas la longueur cible "
                            f"(ds_target={ds_target:.6f} m, tol=1e-3 m). Segments: {ls}")
            
            if errors:
                error_msg = f"[ERREUR contrôle_câble à t={t:.3f} s]\n" + "\n".join(errors)
                trace_print(8, error_msg)
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
            "T": None if T_boat is None else float(T_boat),
            "x_rov": float(x_rov),
            "x_boat": float(x_boat),
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
    if data and isinstance(data.get("T_rov"), list) and data["T_rov"]:
        curr["T_rov"] = data["T_rov"][-1]
    else:
        curr["T_rov"] = None

    if len(history) > K + 1:
        del history[K + 1 :]

    # Estimation locale dT/dL (si possible)
    est_dT_dL = None
    delta_L = history[-1]["L"] - history[0]["L"]
    delta_T = history[-1]["T"] - history[0]["T"]
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
        return f"{val: 3.2f}" if isinstance(val, (int, float)) else "None"
    
    def _fmt_vec(val):
        if isinstance(val, (list, tuple)) and len(val) == 2:
            return f"({val[0]: 3.2f}, {val[1]: 3.2f})"
        return "None"

    # Utiliser t (qui est t_current) pour afficher le temps de l'itération en cours
    # t_current est le même pour auto_L_7, intégration et vérification invariants
    t_display = curr.get('t') if curr.get('t') is not None else t

    # Priorité 1: éviter Trupt
    t_soft = 0.80 * Trupt
    t_hard = 0.90 * Trupt
    if curr["T"] is None or Tcible in (None, 0):
        trace_print(8, "IC_L_7: "
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
    t_raw = 0.0 if curr["T"] is None else float(curr["T"])
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
                if curr.get("T") is not None and Tcible not in (None, 0):
                    t_boost = min(0.3, 0.01 * max(curr["T"] - Tcible, 0.0))
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
                        desired = -(curr["T"] - Tcible) / (2.0 * est_dT_dL)
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
        trace_print(8, f"[DEBUG L] auto_L_7 INCOHÉRENCE: "
            f"L_param={L_param:.6f} m, curr['L']={L_curr:.6f} m, "
            f"ÉCART={abs(L_param - L_curr):.6f} m"
        )
    
    # Utiliser les valeurs passées en paramètre plutôt que _last_val() pour garantir la cohérence
    # T_boat est passé directement en paramètre
    T_boat_display = T_boat if T_boat is not None else curr.get("T")
    # T_rov et cable_drag viennent de data (itération précédente) car non disponibles en paramètre
    T_rov_display = curr.get('T_rov')
    cable_drag_display = _last_val('cable_drag')
    
    trace_print(8, "\nIC_L_7: "
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
    trace_print(8, f"[DEBUG L] IC_L_7: t={_fmt(t_display)}, "
        f"L_param={L_param:.6f} m, curr['L']={_fmt(L_curr)}, "
        f"L_last_data={_fmt(_last_val('L'))} m"
    )

    return ret, exp
