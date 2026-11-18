"""Lois continues pour le câble presque tendu, la tension et la surface.

Les bascules binaires (slack < 0, gel de tension, clip de vitesse) sont
remplacées par des fonctions dont la dérivée s'annule aux raccordements
(smoothstep). Le solveur non linéaire n'est pas appelé ici.
"""
from __future__ import annotations

import numpy as np

# Bande de mou relatif sur laquelle la caténaire rejoint la ligne droite.
# r = L / distance droite. En dessous de 1 : droite. Au-delà de NEAR_TAUT_R_MAX : caténaire.
NEAR_TAUT_R_MAX = 1.05

TAU_TAUT_S = 0.05
TAU_SLACK_S = 0.20
SLACK_RATIO_FOR_FULL_TAU = 0.05

# Épaisseur de la zone où la vitesse verticale des nœuds est atténuée vers la surface.
SURFACE_BLEND_M = 0.05
SURFACE_RESTORE_RATE_S = 2.0

# Largeur (fraction de L) du mou négatif sur laquelle l'accélération du ROV s'éteint.
TAUT_QUENCH_FRACTION = 0.02

# Bande de mou (m) sur laquelle la réaction du câble inextensible s'établit.
# 1 à mou nul, 0 au-delà, raccord en smoothstep.
CONSTRAINT_SLACK_BAND_M = 0.05
# Pulsation et amortissement de la contrainte d(distance)/dt = dL/dt.
CONSTRAINT_OMEGA_RAD_S = 2.5
CONSTRAINT_ZETA = 1.0


def smoothstep(x: np.ndarray | float) -> np.ndarray | float:
    """Polynôme C1 de [0, 1] vers [0, 1], dérivée nulle aux extrémités."""
    z = np.clip(np.asarray(x, dtype=float), 0.0, 1.0)
    out = z * z * (3.0 - 2.0 * z)
    if np.isscalar(x):
        return float(out)
    return out


def straight_blend_weight(length_ratio: float) -> float:
    """
    Poids de la ligne droite dans le mélange quasi-tendu.

    1 si ``r <= 1``, 0 si ``r >= NEAR_TAUT_R_MAX``, smoothstep entre les deux.
    ``r = L / distance droite``.
    """
    r = float(length_ratio)
    if not np.isfinite(r):
        return 0.0
    if r <= 1.0:
        return 1.0
    if r >= NEAR_TAUT_R_MAX:
        return 0.0
    s = (r - 1.0) / (NEAR_TAUT_R_MAX - 1.0)
    return float(1.0 - smoothstep(s))


def continuous_tau_tension(slack_ratio: float) -> float:
    """
    Temps de relaxation de la tension.

    0,05 s quand le mou relatif est nul ou négatif, 0,20 s à partir de 5 % de mou.
    Le raccord est un smoothstep (dérivée nulle aux deux bouts).
    """
    s = float(slack_ratio)
    if not np.isfinite(s) or s <= 0.0:
        return TAU_TAUT_S
    gate = float(smoothstep(min(s / SLACK_RATIO_FOR_FULL_TAU, 1.0)))
    return TAU_TAUT_S + (TAU_SLACK_S - TAU_TAUT_S) * gate


def taut_penalty_newton(t_static_rov: float, slack: float, length: float) -> float:
    """
    Supplément de tension cible quand le mou est négatif.

    Nul à mou nul, ainsi que sa dérivée par rapport au mou (déficit au carré).
    Plafonné à cinq fois la tension statique, comme l'ancien multiplicateur.
    """
    deficit = max(0.0, -float(slack))
    if deficit <= 0.0 or not np.isfinite(deficit):
        return 0.0
    rel = deficit / max(float(length), 1e-9)
    multiplier = min(5.0, (rel / TAUT_QUENCH_FRACTION) ** 2)
    base = max(float(t_static_rov), 0.0)
    return float(base * multiplier)


def payout_matched_velocity(
    vx: float,
    vy: float,
    cos_in: float,
    sin_in: float,
    dL_dt: float,
    slack: float,
) -> tuple[float, float]:
    """
    Vitesse du ROV compatible avec un câble inextensible.

    La composante le long du câble qui écarte le ROV du bateau est ramenée
    sur ``dL/dt``. La composante tangentielle est conservée. Hors de la bande
    de mou, la vitesse d'entrée est renvoyée telle quelle.
    """
    slack_f = float(slack)
    band = CONSTRAINT_SLACK_BAND_M
    vx_f = float(vx)
    vy_f = float(vy)
    if not np.isfinite(slack_f) or slack_f >= band:
        return vx_f, vy_f
    gate = 1.0 if slack_f <= 0.0 else float(1.0 - smoothstep(slack_f / band))
    cos_f = float(cos_in)
    sin_f = float(sin_in)
    v_in = vx_f * cos_f + vy_f * sin_f
    vx_t = vx_f - v_in * cos_f
    vy_t = vy_f - v_in * sin_f
    vx_c = vx_t - float(dL_dt) * cos_f
    vy_c = vy_t - float(dL_dt) * sin_f
    return (1.0 - gate) * vx_f + gate * vx_c, (1.0 - gate) * vy_f + gate * vy_c


def taut_constraint_tension(
    fx_other: float,
    fy_other: float,
    cos_in: float,
    sin_in: float,
    vx: float,
    vy: float,
    dL_dt: float,
    slack: float,
    mass: float,
    d2L_dt2: float = 0.0,
) -> float:
    """
    Tension au ROV imposée par un câble inextensible déjà tendu.

    ``cos_in``, ``sin_in`` pointent du ROV vers le bateau. Les forces passées
    n'incluent pas la tension. La caténaire du fil n'équilibre que son propre
    poids : quand le mou est nul, cette réaction est celle qui empêche le ROV
    de s'éloigner plus vite que le dévidage.

    Nulle dès que le mou dépasse la bande. À mou nul et vitesse radiale égale
    à ``dL/dt``, elle vaut la composante des autres forces qui écarte le ROV.
    Un excès de vitesse d'écartement ou un mou négatif l'augmente.
    """
    mass_f = float(mass)
    if not np.isfinite(mass_f) or mass_f <= 1e-6:
        return 0.0
    slack_f = float(slack)
    band = CONSTRAINT_SLACK_BAND_M
    if not np.isfinite(slack_f) or slack_f >= band:
        return 0.0
    if slack_f <= 0.0:
        gate = 1.0
    else:
        gate = float(1.0 - smoothstep(slack_f / band))

    v_out = -(float(vx) * float(cos_in) + float(vy) * float(sin_in))
    s_dot = v_out - float(dL_dt)
    stretch = max(0.0, -slack_f)
    # Amortissement seulement quand le câble se tend ou est déjà trop court.
    # À l'équilibre (mou nul, vitesse = dL/dt) il est nul, la tension reste
    # la force d'écartement.
    if stretch <= 0.0 and s_dot < 0.0:
        s_dot_active = 0.0
    else:
        s_dot_active = s_dot

    omega = CONSTRAINT_OMEGA_RAD_S
    a_out_des = (
        float(d2L_dt2)
        - 2.0 * CONSTRAINT_ZETA * omega * s_dot_active
        - (omega ** 2) * stretch
    )
    f_out = -(float(fx_other) * float(cos_in) + float(fy_other) * float(sin_in))
    tension = f_out - mass_f * a_out_des
    if not np.isfinite(tension) or tension <= 0.0:
        return 0.0
    return float(gate * tension)


def rov_accel_scale(slack: float, length: float) -> float:
    """
    Facteur d'accélération du ROV.

    1 dès que le mou est positif. 0 quand le déficit atteint 2 % de L.
    Smoothstep sur la bande intermédiaire : le câble tendu freine le ROV
    sans annuler l'accélération d'un seul coup au passage par zéro.
    """
    band = TAUT_QUENCH_FRACTION * max(float(length), 1e-9)
    return float(smoothstep((float(slack) + band) / band))


def smooth_vertical_node_velocity(y: np.ndarray, vy: np.ndarray) -> np.ndarray:
    """
    Vitesse verticale nodale compatible avec la surface, sans seuil dur.

    Plus profond que ``SURFACE_BLEND_M`` : ``vy`` est inchangé.
    Dans la bande ``[-SURFACE_BLEND_M, 0]`` : la composante vers le haut
    s'éteint par smoothstep.
    Au-dessus de la surface : vitesse de rappel ``-k y`` (pénalité quadratique
    sur la position, force de rappel linéaire).
    """
    y_arr = np.asarray(y, dtype=float)
    vy_arr = np.asarray(vy, dtype=float).copy()
    if y_arr.shape != vy_arr.shape:
        return vy_arr
    above = y_arr > 0.0
    if np.any(above):
        vy_arr = np.where(above, -SURFACE_RESTORE_RATE_S * y_arr, vy_arr)
    h = SURFACE_BLEND_M
    in_band = (y_arr > -h) & (y_arr <= 0.0)
    if np.any(in_band):
        s = smoothstep((-y_arr[in_band]) / h)
        vy_down = np.minimum(vy_arr[in_band], 0.0)
        vy_arr[in_band] = s * vy_arr[in_band] + (1.0 - s) * vy_down
    return vy_arr


def blend_polyline_near_taut(
    x: np.ndarray,
    y: np.ndarray,
    tension: np.ndarray,
    x_boat: float,
    x_rov: float,
    y_rov: float,
    length: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Mélange la polyligne avec la corde bateau–ROV quand le câble est presque tendu.

    Le même poids sert à la géométrie et à la tension, donc les deux champs
    restent continus quand ``L`` traverse la distance droite.
    Aucun rééchantillonnage : la longueur curviligne est rétablie par la
    projection de fin de pas.
    """
    x_arr = np.asarray(x, dtype=float)
    y_arr = np.asarray(y, dtype=float)
    t_arr = np.asarray(tension, dtype=float)
    n = int(x_arr.size)
    if n < 2 or y_arr.size != n or t_arr.size != n:
        return x_arr, y_arr, t_arr
    if not np.isfinite(length) or float(length) <= 0.0:
        return x_arr, y_arr, t_arr
    straight = float(np.hypot(float(x_rov) - float(x_boat), float(y_rov)))
    if straight <= 1e-9:
        return x_arr, y_arr, t_arr
    weight_straight = straight_blend_weight(float(length) / straight)
    if weight_straight <= 0.0:
        return x_arr, y_arr, t_arr
    weight_curve = 1.0 - weight_straight
    x_straight = np.linspace(float(x_boat), float(x_rov), n)
    y_straight = np.linspace(0.0, float(y_rov), n)
    x_out = weight_curve * x_arr + weight_straight * x_straight
    y_out = weight_curve * y_arr + weight_straight * y_straight
    x_out[0] = float(x_boat)
    y_out[0] = 0.0
    x_out[-1] = float(x_rov)
    y_out[-1] = float(y_rov)
    t_straight = np.linspace(float(t_arr[0]), float(t_arr[-1]), n)
    t_out = weight_curve * t_arr + weight_straight * t_straight
    return x_out, y_out, t_out
