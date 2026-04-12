"""
Initialisation géométrique du câble à flottabilité positive (ρ_cable < ρ_eau), sans courant.

Repère auxiliaire : z = -(y - y_boat) (profondeur positive vers le bas à partir de la ligne d'eau du bateau).
Dans ce repère, la chaînette « pendante » avec sommet à tangente horizontale s'écrit :
    z(x) = z_H + a * (cosh((x - x_H) / a) - 1),  a > 0.

Les coordonnées simulateur sont y <= y_boat sous la surface (y = y_boat à la surface).

Cas (après translation x' = x - x_bateau) :
  1) L >= |x_r| + |z_r| : slack en surface + vertical (délégué au polyline existant).
  2) L_straight < L < Manhattan : chaînette pure ou chaînette + segment horizontal à z=0.
  3) |L - L_straight| <= tol : droite bateau–ROV.
  4) L < L_straight : impossible.

Cas 2a (L >= L0, sommet H sur la surface z=0, tangente horizontale en H) :
  Le polyline est toujours tracé dans l'ordre **bateau (0,0) -> H (x_H, 0) -> ROV** dans le repère x'
  (puis réflexion si x_rov < x_bateau). La longueur horizontale est |x_H| (distance le long de z=0).

  Deux topologies admissibles pour la même (x_r, z_r, L) :
  - **opposite** : slack du côté opposé au ROV en x' (ex. x_r' > 0 => x_H' < 0, segment horizontal
    visible de H vers le bateau vers les x' négatifs).
  - **toward_rov** : slack entre le bateau et le ROV (0 < x_H' < x_r' si x_r' > 0).
  - **auto** : tente d'abord **toward_rov** (slack entre bateau et ROV en x', sans x' négatif si x_r'>0) ;
    en échec numérique, retombe sur **opposite**.
"""
from __future__ import annotations

from typing import Literal

import numpy as np
from scipy.optimize import fsolve, root_scalar

from src.utils.logger import trace_print

# ---------------------------------------------------------------------------
# Repère et invariants
# ---------------------------------------------------------------------------


def _to_xz(x: float, y: float, x_boat: float, y_boat: float) -> tuple[float, float]:
    """Simulateur (x,y) -> (x', z) avec x' = x - x_boat, z = profondeur positive."""
    return float(x - x_boat), float(-(y - y_boat))


def _to_xy(xp: float, z: float, x_boat: float, y_boat: float) -> tuple[float, float]:
    """Repère (x', z) -> simulateur (x, y)."""
    return float(x_boat + xp), float(y_boat - z)


def manhattan_length(x_r: float, z_r: float) -> float:
    return abs(x_r) + abs(z_r)


def straight_length(x_r: float, z_r: float) -> float:
    return float(np.hypot(x_r, z_r))


def catenary_z(x: np.ndarray, x_H: float, z_H: float, a: float) -> np.ndarray:
    """z(x) = z_H + a * (cosh((x - x_H) / a) - 1)."""
    return z_H + a * (np.cosh((x - x_H) / a) - 1.0)


def arc_catenary(xa: float, xb: float, a: float, x_H: float) -> float:
    """Longueur d'arc le long de z = z_H + a(cosh((x-x_H)/a)-1) entre xa et xb."""
    return float(
        abs(a * (np.sinh((xb - x_H) / a) - np.sinh((xa - x_H) / a)))
    )


# ---------------------------------------------------------------------------
# Référence L0 : sommet au bateau (x_H = 0, z_H = 0)
# ---------------------------------------------------------------------------


def solve_a_vertex_at_boat(x_r: float, z_r: float) -> tuple[float, float]:
    """
    Sommet en (0, 0) : z_r = a * (cosh(x_r / a) - 1), retourne (a, L0) avec
    L0 = a * |sinh(x_r / a)|.
    """
    xr = float(abs(x_r))
    zr = float(z_r)
    if zr <= 0.0 or xr <= 1e-15:
        raise ValueError("solve_a_vertex_at_boat: z_r > 0 et |x_r| > 0 requis")

    def f_a(ap: float) -> float:
        if ap <= 1e-12:
            return 1e30
        if xr / ap > 700.0:
            return 1e30
        return ap * (np.cosh(xr / ap) - 1.0) - zr

    # a trop petit => overflow cosh ; partir d'un lo raisonnable
    lo = max(xr**2 / (2.0 * zr) * 0.01, 1e-4)
    hi = max(1.0, xr**2 / max(zr, 1e-9))
    f_lo, f_hi = f_a(lo), f_a(hi)
    expand = 0
    while np.sign(f_lo) == np.sign(f_hi) and expand < 40:
        hi *= 2.0
        f_hi = f_a(hi)
        expand += 1
    if np.sign(f_lo) == np.sign(f_hi):
        raise ValueError("solve_a_vertex_at_boat: bracket introuvable")

    sol = root_scalar(f_a, bracket=(lo, hi), method="brentq", xtol=1e-10, rtol=1e-10)
    if not sol.converged:
        raise ValueError("solve_a_vertex_at_boat: non convergence")
    a = float(sol.root)
    L0 = float(a * abs(np.sinh(xr / a)))
    return a, L0


# ---------------------------------------------------------------------------
# Cas 2b : L_straight < L < L0 — trois équations (x_H, z_H, a)
# ---------------------------------------------------------------------------


def _solve_three_unknowns(
    x_r: float, z_r: float, L: float, a_guess: float
) -> tuple[float, float, float]:
    """
    0 = z_H + a(cosh(x_H/a) - 1)   [bateau x'=0 sur la courbe]
    z_r = z_H + a(cosh((x_r - x_H)/a) - 1)
    L = a * (sinh((x_r - x_H)/a) - sinh(-x_H/a)) = a * (sinh((x_r - x_H)/a) + sinh(x_H/a))
    """
    xr, zr = float(x_r), float(z_r)

    def residuals(v: np.ndarray) -> np.ndarray:
        x_H, z_H, a = float(v[0]), float(v[1]), float(v[2])
        if a <= 1e-9:
            return np.array([1e12, 1e12, 1e12])
        r0 = z_H + a * (np.cosh(x_H / a) - 1.0)
        r1 = z_H + a * (np.cosh((xr - x_H) / a) - 1.0) - zr
        r2 = (
            a * (np.sinh((xr - x_H) / a) + np.sinh(x_H / a))
        ) - L
        return np.array([r0, r1, r2], dtype=float)

    # Estimation : sommet entre bateau et ROV en x, légèrement en profondeur
    x_H0 = 0.35 * xr
    a0 = max(a_guess * 0.8, 1e-3)
    z_H0 = -a0 * (np.cosh(x_H0 / a0) - 1.0) + 0.01
    v0 = np.array([x_H0, z_H0, a0], dtype=float)
    sol, infodict, ier, _ = fsolve(residuals, v0, full_output=True)
    if ier != 1 or np.max(np.abs(residuals(sol))) > 1e-5:
        v0 = np.array([0.5 * xr, -0.05 * zr, a_guess], dtype=float)
        sol, infodict, ier, _ = fsolve(residuals, v0, full_output=True)
    if ier != 1 or np.max(np.abs(residuals(sol))) > 1e-4:
        raise ValueError("_solve_three_unknowns: fsolve n'a pas convergé")
    x_H, z_H, a = float(sol[0]), float(sol[1]), float(sol[2])
    if a <= 1e-9:
        raise ValueError("_solve_three_unknowns: a invalide")
    return x_H, z_H, a


# ---------------------------------------------------------------------------
# Cas 2a : L >= L0 — segment horizontal à z=0 + arc chaînette H -> ROV
# ---------------------------------------------------------------------------

_BranchSlack = Literal["opposite", "toward_rov"]


def _solve_L_ge_L0_single(
    x_r: float,
    z_r: float,
    L: float,
    L0_ref: float,
    a_ref: float,
    slack_side: _BranchSlack,
) -> tuple[float, float, float]:
    """
    Une branche : opposite (x_H du côté opposé au ROV en x') ou toward_rov (x_H entre 0 et x_r si x_r>0).
    """
    xr, zr = float(x_r), float(z_r)
    if L + 1e-12 < L0_ref:
        raise ValueError("_solve_L_ge_L0_single: L < L0")

    s_h0 = max(0.0, float(L - L0_ref))
    env = 1e-9

    if slack_side == "opposite":
        if xr >= 0.0:
            x_H0 = -s_h0
        else:
            x_H0 = s_h0
    else:
        if xr > env:
            x_H0 = 0.5 * min(s_h0, xr - env)
        elif xr < -env:
            x_H0 = 0.5 * max(-s_h0, xr + env)
        else:
            x_H0 = s_h0 * 0.1

    def res(v: np.ndarray) -> np.ndarray:
        x_H, ap = float(v[0]), float(v[1])
        if ap <= 1e-12:
            return np.array([1e12, 1e12])
        if xr >= 0.0 and x_H >= xr - 1e-12:
            return np.array([1e12, 1e12])
        if xr < 0.0 and x_H <= xr + 1e-12:
            return np.array([1e12, 1e12])

        if abs(xr) > 1e-10:
            if slack_side == "opposite":
                if xr > 0.0 and x_H >= -env:
                    return np.array([1e12, 1e12])
                if xr < 0.0 and x_H <= env:
                    return np.array([1e12, 1e12])
            else:
                if xr > 0.0:
                    if x_H <= env or x_H >= xr - env:
                        return np.array([1e12, 1e12])
                else:
                    if x_H >= -env or x_H <= xr + env:
                        return np.array([1e12, 1e12])

        arc = arc_catenary(x_H, xr, ap, x_H)
        len_h = abs(x_H)
        r0 = len_h + arc - L
        r1 = ap * (np.cosh((xr - x_H) / ap) - 1.0) - zr
        return np.array([r0, r1], dtype=float)

    a_min = max(a_ref, 1e-6)
    x_guesses: list[float]
    a_guesses: list[float]

    if slack_side == "toward_rov" and abs(xr) > 1e-10:
        cap = abs(xr) - env
        s_abs = abs(s_h0)
        x_guesses = [
            0.5 * min(s_abs, cap),
            0.35 * cap,
            0.22 * cap,
            0.12 * cap,
            min(s_abs * 0.65, 0.5 * cap),
        ]
        if xr < 0.0:
            x_guesses = [-xg for xg in x_guesses]
        a_guesses = [
            a_ref,
            max(a_ref * 0.55, 1.5),
            a_ref * 1.35,
            max(a_ref * 0.3, 1.2),
            min(a_ref * 2.0, 40.0),
        ]
    else:
        x_guesses = [x_H0, x_H0 * 0.5]
        a_guesses = [a_min, a_ref * 1.2]

    best: tuple[np.ndarray, float] | None = None
    for xg in x_guesses:
        for ag in a_guesses:
            v0 = np.array([float(xg), float(max(ag, 1e-6))], dtype=float)
            sol, _, ier, _ = fsolve(res, v0, full_output=True)
            if ier != 1:
                continue
            rnorm = float(np.max(np.abs(res(sol))))
            if rnorm > 1e-4:
                continue
            if best is None or rnorm < best[1]:
                best = (sol, rnorm)

    if best is None:
        raise ValueError("_solve_L_ge_L0_single: fsolve")
    sol = best[0]
    x_H, a = float(sol[0]), float(sol[1])
    if a <= 1e-9:
        raise ValueError("_solve_L_ge_L0_single: a invalide")
    return x_H, 0.0, a


def _solve_L_ge_L0_branch(
    x_r: float,
    z_r: float,
    L: float,
    L0_ref: float,
    a_ref: float,
    slack_side: Literal["opposite", "toward_rov", "auto"] = "opposite",
) -> tuple[float, float, float]:
    """
    Sommet chaînette en H = (x_H, 0), tangente horizontale. Bateau en (0,0).
    |x_H| + arc(H->ROV) = L, z_r = a(cosh((x_r-x_H)/a)-1).

    Voir docstring du module pour **opposite** / **toward_rov** / **auto**.
    """
    if slack_side != "auto":
        return _solve_L_ge_L0_single(x_r, z_r, L, L0_ref, a_ref, slack_side)

    try:
        x_H, z_H, a = _solve_L_ge_L0_single(
            x_r, z_r, L, L0_ref, a_ref, "toward_rov"
        )
        trace_print(
            5,
            f"[INIT buoyant] cas=2a slack_side=auto -> branche 'toward_rov' (x_H'={x_H:.6f})",
        )
        return x_H, z_H, a
    except ValueError:
        x_H, z_H, a = _solve_L_ge_L0_single(
            x_r, z_r, L, L0_ref, a_ref, "opposite"
        )
        trace_print(
            5,
            "[INIT buoyant] cas=2a slack_side=auto -> fallback 'opposite' "
            f"(x_H'={x_H:.6f})",
        )
        return x_H, z_H, a


# ---------------------------------------------------------------------------
# Chaînette pure sommet bateau (L proche de L0)
# ---------------------------------------------------------------------------


def _sample_catenary_vertex_boat(
    x_r: float, z_r: float, L: float, n_pts: int, a: float
) -> tuple[np.ndarray, np.ndarray]:
    """Échantillonne z = a(cosh(x/a)-1) pour x de 0 à x_r, en abscisse curviligne totale L."""
    xr = float(x_r)
    n = max(int(n_pts), 1) + 1
    xs = np.linspace(0.0, xr, n)
    zs = catenary_z(xs, 0.0, 0.0, a)
    # Renormaliser légèrement la longueur d'arc par interpolation sur s
    s_tab = np.zeros(n)
    for i in range(1, n):
        xa, xb = xs[i - 1], xs[i]
        s_tab[i] = s_tab[i - 1] + arc_catenary(xa, xb, a, 0.0)
    s_tot = float(s_tab[-1])
    if s_tot <= 1e-12:
        return xs, zs
    s_target = np.linspace(0.0, s_tot, n)
    xs_u = np.interp(s_target, s_tab, xs)
    zs_u = catenary_z(xs_u, 0.0, 0.0, a)
    return xs_u, zs_u


def _sample_path_horizontal_then_catenary(
    x_H: float,
    z_H: float,
    a: float,
    x_r: float,
    z_r: float,
    n_pts: int,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Ordre des points : bateau (0,0) -> H (x_H,z_H) en ligne droite, puis chaînette H -> ROV (repère x').
    Si x_H < 0 et x_r > 0, le segment horizontal va d'abord vers les x' négatifs ; si 0 < x_H < x_r,
    le segment suit la surface vers le ROV avant l'arc.
    """
    n = max(int(n_pts), 1) + 1
    len_h = float(np.hypot(x_H, z_H))
    len_c = arc_catenary(x_H, x_r, a, x_H)
    s_tot = len_h + len_c
    if s_tot <= 1e-12:
        return np.array([0.0, x_r]), np.array([0.0, z_r])

    # Résolution curviligne ~5 cm pour limiter l'écart corde / arc avant normalisation UI.
    n_dense = max(n, int(min(8000, max(400, s_tot / 0.05))))
    s_grid = np.linspace(0.0, s_tot, n_dense)
    xd = np.zeros(n_dense)
    zd = np.zeros(n_dense)
    for i, s in enumerate(s_grid):
        if s <= len_h + 1e-15:
            t = s / max(len_h, 1e-15)
            xd[i] = t * x_H
            zd[i] = t * z_H
        else:
            sc = min(max(s - len_h, 0.0), len_c)
            xi = x_H + a * np.arcsinh(sc / a)
            xd[i] = float(xi)
            zd[i] = float(catenary_z(np.array([xi]), x_H, z_H, a)[0])

    xd[0], zd[0] = 0.0, 0.0
    xd[-1], zd[-1] = x_r, z_r
    zd[-1] = z_r
    if n_dense <= n:
        return xd, zd
    return _subsample_polyline_uniform_arclength(xd, zd, n)


def _sample_catenary_boat_to_rov(
    x0: float,
    x_r: float,
    x_H: float,
    z_H: float,
    a: float,
    n_pts: int,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Échantillon uniforme en abscisse curviligne le long de la chaînette de x0 à x_r
    (sommet en (x_H, z_H)), avec s(x) = a * (sinh((x-x_H)/a) - sinh((x0-x_H)/a)).
    """
    n_out = max(int(n_pts), 1) + 1
    sh0 = np.sinh((x0 - x_H) / a)
    sh1 = np.sinh((x_r - x_H) / a)
    s0 = float(a * (sh1 - sh0))
    if abs(s0) <= 1e-12:
        return np.array([x0, x_r]), np.array(
            [
                float(catenary_z(np.array([x0]), x_H, z_H, a)[0]),
                float(catenary_z(np.array([x_r]), x_H, z_H, a)[0]),
            ]
        )
    n_dense = max(n_out, int(min(5000, max(200, s0 / 0.02))))
    s_grid = np.linspace(0.0, s0, n_dense)
    xd = np.zeros(n_dense)
    zd = np.zeros(n_dense)
    for i, s in enumerate(s_grid):
        val = s / a + sh0
        xi = x_H + a * np.arcsinh(val)
        xd[i] = float(xi)
        zd[i] = float(catenary_z(np.array([xi]), x_H, z_H, a)[0])
    xd[0], xd[-1] = x0, x_r
    zd[0] = float(catenary_z(np.array([x0]), x_H, z_H, a)[0])
    zd[-1] = float(catenary_z(np.array([x_r]), x_H, z_H, a)[0])
    if n_dense <= n_out:
        return xd, zd
    return _subsample_polyline_uniform_arclength(xd, zd, n_out)


def _subsample_polyline_uniform_arclength(
    xp: np.ndarray, zp: np.ndarray, n_out: int
) -> tuple[np.ndarray, np.ndarray]:
    """n_out points espacés uniformément en abscisse curviligne le long de la polyligne (xp,zp)."""
    xp = np.asarray(xp, dtype=float)
    zp = np.asarray(zp, dtype=float)
    ds = np.hypot(np.diff(xp), np.diff(zp))
    s = np.concatenate(([0.0], np.cumsum(ds)))
    s_tot = float(s[-1])
    if s_tot <= 1e-12 or n_out < 2:
        return xp, zp
    targets = np.linspace(0.0, s_tot, n_out)
    j = np.searchsorted(s, targets, side="right") - 1
    j = np.clip(j, 0, len(s) - 2)
    s0 = s[j]
    s1 = s[j + 1]
    t = np.zeros(n_out)
    m = s1 - s0 > 1e-15
    t[m] = (targets[m] - s0[m]) / (s1[m] - s0[m])
    t = np.clip(t, 0.0, 1.0)
    xn = (1.0 - t) * xp[j] + t * xp[j + 1]
    zn = (1.0 - t) * zp[j] + t * zp[j + 1]
    return xn, zn


def _resample_by_arclength(
    xp: np.ndarray, zp: np.ndarray, n_pts: int
) -> tuple[np.ndarray, np.ndarray]:
    """Rééchantillonne (xp,zp) uniformément en abscisse curviligne."""
    n = max(int(n_pts), 1) + 1
    xp = np.asarray(xp, dtype=float)
    zp = np.asarray(zp, dtype=float)
    ds = np.hypot(np.diff(xp), np.diff(zp))
    s = np.concatenate(([0.0], np.cumsum(ds)))
    s_tot = float(s[-1])
    if s_tot <= 1e-12:
        return xp, zp
    s_new = np.linspace(0.0, s_tot, n)
    x_new = np.interp(s_new, s, xp)
    z_new = np.interp(s_new, s, zp)
    return x_new, z_new


# ---------------------------------------------------------------------------
# API publique
# ---------------------------------------------------------------------------


def build_buoyant_cable_polyline(
    x_boat: float,
    y_boat: float,
    x_rov: float,
    y_rov: float,
    L: float,
    n_segments: int,
    *,
    straight_tol: float = 1e-6,
    slack_side: Literal["opposite", "toward_rov", "auto"] = "opposite",
) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
    """
    Construit (x_cable, y_cable) en coordonnées simulateur, boat -> ROV, N+1 points.

    Retourne (None, None) si le cas doit être traité par le polyline Manhattan / droite
    existant (cas 1 ou 3) ou en cas d'échec numérique du cas 2.

    slack_side : pour le cas **L >= L0** uniquement (segment horizontal à la surface + arc).
    Voir docstring du module.     Défaut **opposite** (comportement historique). L'UI utilise
    **auto** (priorité **toward_rov** si convergence).
    """
    x_r, z_r = _to_xz(x_rov, y_rov, x_boat, y_boat)
    if z_r <= 0.0:
        trace_print(
            5,
            "[INIT buoyant] cas=abandon (z_r<=0, ROV pas plus bas que bateau en profondeur) "
            f"x_r={x_r:.4f} z_r={z_r:.4f} L={float(L):.4f}",
        )
        return None, None

    L_st = straight_length(x_r, z_r)
    L_man = manhattan_length(x_r, z_r)
    L_cmd = float(L)

    if L_cmd < L_st - 1e-9:
        raise ValueError(
            f"build_buoyant_cable_polyline: L={L_cmd} < L_straight={L_st:.6f}"
        )

    if L_cmd >= L_man - 1e-9 or abs(L_cmd - L_st) <= straight_tol:
        trace_print(
            5,
            "[INIT buoyant] cas=délégation (polyline existant cas 1 ou 3) "
            f"L_cmd={L_cmd:.4f} L_straight={L_st:.4f} L_manhattan={L_man:.4f} "
            f"(Manhattan: {L_cmd >= L_man - 1e-9}, droite: {abs(L_cmd - L_st) <= straight_tol})",
        )
        return None, None

    # Symétrie : travailler en x_r positive pour la géométrie, puis refléter
    sign_x = 1.0 if x_r >= 0.0 else -1.0
    xr = abs(x_r)

    try:
        a0, L0 = solve_a_vertex_at_boat(xr, z_r)
    except Exception as e_l0:
        trace_print(
            5,
            f"[INIT buoyant] cas=échec L0/a0 (solve_a_vertex_at_boat): {e_l0} "
            f"xr={xr:.4f} z_r={z_r:.4f} L={L_cmd:.4f}",
        )
        return None, None

    n_pts = max(int(n_segments), 1)

    if L_cmd + 1e-9 >= L0:
        cas = "2a (L >= L0, slack horizontal surface + arc chaînette)"
        try:
            x_H, z_H, a = _solve_L_ge_L0_branch(
                xr, z_r, L_cmd, L0, a0, slack_side=slack_side
            )
        except Exception as e_2a:
            trace_print(
                5,
                f"[INIT buoyant] cas={cas} échec résolution: {e_2a} "
                f"L0={L0:.6f} L={L_cmd:.6f}",
            )
            return None, None
        x_H_sim = float(x_boat + sign_x * x_H)
        y_H_sim = float(y_boat - z_H)
        trace_print(
            5,
            f"[INIT buoyant] cas={cas} slack_side={slack_side} L0={L0:.6f} a={a:.6f} "
            f"x_H={x_H_sim:.6f} y_H={y_H_sim:.6f} "
            f"(repère x': x_H'={x_H:.6f} z_H={z_H:.6f}) "
            f"L={L_cmd:.6f} L_st={L_st:.6f}",
        )
        xp, zp = _sample_path_horizontal_then_catenary(
            x_H, z_H, a, xr, z_r, n_pts
        )
    else:
        cas = "2b (L_straight < L < L0, système x_H, z_H, a)"
        try:
            x_H, z_H, a = _solve_three_unknowns(xr, z_r, L_cmd, a0)
        except Exception as e_2b:
            trace_print(
                5,
                f"[INIT buoyant] cas={cas} échec résolution: {e_2b} "
                f"L0={L0:.6f} L={L_cmd:.6f}",
            )
            return None, None
        x_H_sim = float(x_boat + sign_x * x_H)
        y_H_sim = float(y_boat - z_H)
        trace_print(
            5,
            f"[INIT buoyant] cas={cas} L0={L0:.6f} a={a:.6f} "
            f"x_H={x_H_sim:.6f} y_H={y_H_sim:.6f} "
            f"(repère x': x_H'={x_H:.6f} z_H={z_H:.6f}) "
            f"L={L_cmd:.6f} L_st={L_st:.6f}",
        )
        xp, zp = _sample_catenary_boat_to_rov(
            0.0, xr, x_H, z_H, a, n_pts
        )

    # Refléter x si nécessaire
    xp = sign_x * xp

    x_cable = np.empty(n_pts + 1, dtype=float)
    y_cable = np.empty(n_pts + 1, dtype=float)
    for i in range(n_pts + 1):
        xi, yi = _to_xy(float(xp[i]), float(zp[i]), x_boat, y_boat)
        x_cable[i] = xi
        y_cable[i] = yi

    y_cable = np.clip(y_cable, None, y_boat)
    x_cable[0] = float(x_boat)
    y_cable[0] = float(y_boat)
    x_cable[-1] = float(x_rov)
    y_cable[-1] = float(y_rov)

    L_seg = float(np.sum(np.hypot(np.diff(x_cable), np.diff(y_cable))))
    # La somme des cordes sous-estime l'arc pour un maillage grossier ; la normalisation
    # dans simulation_tab recolle L exact.
    if abs(L_seg - L_cmd) > max(15.0, 0.05 * L_cmd):
        trace_print(
            5,
            "[INIT buoyant] cas=abandon (L_seg cordes trop loin de L_cmd, attendre normalisation UI) "
            f"L_seg={L_seg:.4f} L_cmd={L_cmd:.4f} tol={max(15.0, 0.05 * L_cmd):.4f}",
        )
        return None, None

    trace_print(
        5,
        f"[INIT buoyant] polyline OK (L_seg proche L_cmd): L_seg={L_seg:.4f} L_cmd={L_cmd:.4f}",
    )

    return x_cable, y_cable


def is_buoyant_cable(system) -> bool:
    """True si ρ_cable < ρ_eau (poids linéique apparent < 0)."""
    try:
        w = (
            (system.cable.rho_cable - system.environment.rho_eau)
            * system.cable.A_cable
            * system.environment.g
        )
        return float(w) < 0.0
    except Exception:
        return False
