"""
Projection de l'état sur les contraintes géométriques du câble (chantier DAE / half-explicit).

Utilise le même pipeline que simulation_thread : _normalize_cable_length puis
_compute_catenary_tensions, avec mise à jour inplace du vecteur y.
"""
from __future__ import annotations

from typing import Any

import numpy as np

from src.utils.logger import trace_print

# Même ordre de grandeur que ``cable_solver.STRAIGHT_ROV_SNAP_MAX_M`` (éviter import circulaire).
_STRAIGHT_ROV_SNAP_MAX_M = 0.1

# Câble mou : la caténaire géométrique peut renvoyer des tensions non physiques ; alignement pilotage.
_SLACK_RATIO_TENSION_RELAX = 0.03
_SLACK_SOFT_TENSION_CAP_N = 120.0


def slack_ratio_and_soft_cap_tensions(
    x_boat: float,
    y_boat: float,
    x_rov: float,
    y_rov: float,
    L: float,
    T: np.ndarray,
) -> tuple[float, np.ndarray]:
    """
    slack_ratio = (L - distance droite) / distance droite.
    Si > _SLACK_RATIO_TENSION_RELAX, retourne min(T, _SLACK_SOFT_TENSION_CAP_N) élément par élément.
    """
    T_arr = np.asarray(T, dtype=float).copy()
    d_straight = float(np.hypot(float(x_rov) - float(x_boat), float(y_rov) - float(y_boat)))
    slack = float(L) - d_straight
    slack_ratio = max(slack / max(d_straight, 1.0), 0.0)
    if slack_ratio > _SLACK_RATIO_TENSION_RELAX:
        T_arr = np.minimum(T_arr, _SLACK_SOFT_TENSION_CAP_N)
    return slack_ratio, T_arr


def apply_slack_tension_soft_cap_inplace(
    system: Any,
    y: np.ndarray,
    *,
    t_sim: float | None = None,
) -> None:
    """
    Sécurité runtime : impose le plafond mou sur la tranche tension de ``y`` (bateau en y=0).
    À appeler en fin de pas si la projection a échoué ou si l'intégration contrainte saute la projection UI.
    """
    y_arr = np.asarray(y, dtype=float)
    try:
        (x_rov, y_rov, _vx, _vy, x_boat, _vx_boat, _xc, _yc, T, L) = system.unpack_state(y_arr)
        if L is None or float(L) <= 1e-9:
            return
        slack_ratio, T_capped = slack_ratio_and_soft_cap_tensions(
            float(x_boat),
            0.0,
            float(x_rov),
            float(y_rov),
            float(L),
            T,
        )
        T_old = np.asarray(T, dtype=float)
        if not np.allclose(T_old, T_capped, rtol=0.0, atol=1e-9):
            idx_t = 6 + 2 * (system.N + 1)
            y_arr[idx_t : idx_t + system.N + 1] = T_capped
            t_before = float(np.max(T_old)) if T_old.size > 0 else 0.0
            t_after = float(np.max(T_capped)) if T_capped.size > 0 else 0.0
            if (
                t_sim is not None
                and 39.0 <= float(t_sim) <= 40.6
                and t_before > t_after + 1e-6
            ):
                trace_print(
                    7,
                    (
                        "[PROJ CAP] apply_slack_tension_soft_cap_inplace "
                        f"t={float(t_sim):.2f} slack_ratio={slack_ratio:.4f} "
                        f"Tmax_before={t_before:.2f} Tmax_after={t_after:.2f}"
                    ),
                )
    except Exception:
        pass


def reconcile_straight_mode_after_normalize(
    x_proj: np.ndarray,
    y_proj: np.ndarray,
    straight_mode: bool,
    x_rov: float,
    y_rov: float,
    x_boat: float,
    y_boat: float,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    """
    Après ``_normalize_cable_length`` : recoller le dernier nœud au ROV si pas en mode straight.
    En mode straight, n'adopter la position câble comme ROV que si le saut est ≤ 0,1 m ;
    sinon recoller le câble au ROV (défense — ``_normalize_cable_length`` devrait déjà refuser
    les grands sauts et renvoyer le fallback).
    """
    xp = np.asarray(x_proj, dtype=float).copy()
    yp = np.asarray(y_proj, dtype=float).copy()
    xr, yr = float(x_rov), float(y_rov)
    _ = (float(x_boat), float(y_boat))  # API alignée sur les appelants (bateau déjà en tête de xp/yp)

    if not straight_mode:
        xp[-1] = xr
        yp[-1] = yr
        return xp, yp, xr, yr

    snap = float(np.hypot(float(xp[-1]) - xr, float(yp[-1]) - yr))
    if snap <= _STRAIGHT_ROV_SNAP_MAX_M:
        return xp, yp, float(xp[-1]), float(yp[-1])

    trace_print(
        8,
        "[reconcile_straight_mode] Invariant: straight_mode avec snap="
        f"{snap:.4f} m > {_STRAIGHT_ROV_SNAP_MAX_M} m — recollage câble sur ROV.",
    )
    xp[-1] = xr
    yp[-1] = yr
    return xp, yp, xr, yr


def compute_cable_constraint_metrics(system: Any, y: np.ndarray) -> dict[str, float]:
    """
    Métriques scalaires pour C1 (longueur) et C2 (surface).

    Convention surface : y <= 0 sous la surface ; violation si y_i > 0.
    """
    (
        _x_rov,
        _y_rov,
        _vx,
        _vy,
        _x_boat,
        _vx_boat,
        x_cable,
        y_cable,
        _T,
        L,
    ) = system.unpack_state(y)
    L = float(L)
    L_seg = 0.0
    max_y_cable = -1e300
    ds_max = 0.0
    ds_mean = 0.0
    n_seg = 0
    if x_cable is not None and y_cable is not None and len(x_cable) > 1:
        xc = np.asarray(x_cable, dtype=float)
        yc = np.asarray(y_cable, dtype=float)
        max_y_cable = float(np.max(yc))
        seg_lens = []
        for i in range(len(xc) - 1):
            ds = float(np.hypot(xc[i + 1] - xc[i], yc[i + 1] - yc[i]))
            seg_lens.append(ds)
            L_seg += ds
        n_seg = len(seg_lens)
        if seg_lens:
            ds_max = max(seg_lens)
            ds_mean = L_seg / max(n_seg, 1)
    rel_L = abs(L_seg - L) / max(L, 1e-9) if L > 0 else 0.0
    N = int(system.N)
    ds_target = L / max(N, 1) if L > 0 else 0.0
    ratio_max = (ds_max / ds_target) if ds_target > 1e-12 else 0.0
    return {
        "L": L,
        "L_seg": L_seg,
        "rel_L_mismatch": rel_L,
        "max_y_cable": max_y_cable,
        "ds_max": ds_max,
        "ds_mean": ds_mean,
        "ds_target": ds_target,
        "ds_max_ratio": ratio_max,
        "n_segments": float(n_seg),
    }


def compute_constraint_residuals(system: Any, y: np.ndarray) -> dict[str, float]:
    """Résidus normalisés pour diagnostic (ordre de grandeur ~ O(1) si grave)."""
    m = compute_cable_constraint_metrics(system, y)
    tol_L = 1e-4
    surface_eps = 1e-6
    r_L = max(0.0, m["rel_L_mismatch"] / tol_L - 1.0)
    r_surf = max(0.0, (m["max_y_cable"] - 0.0) / surface_eps) if m["max_y_cable"] > 0 else 0.0
    r_seg = max(0.0, m["ds_max_ratio"] - 2.0)  # au-delà de 2x ds_target = suspect
    return {
        "residual_length": r_L,
        "residual_surface": r_surf,
        "residual_segment": r_seg,
        **m,
    }


def project_cable_state_inplace(
    system: Any,
    y: np.ndarray,
    t_sim: float,
    *,
    k_tail: int = 10,
) -> bool:
    """
    Applique _normalize_cable_length + tensions sur y (modification inplace).

    Retourne False si la projection est ignorée (données insuffisantes) ou lève.
    """
    y_arr = np.asarray(y, dtype=float)
    try:
        (
            x_rov,
            y_rov,
            vx_rov,
            vy_rov,
            x_boat,
            vx_boat,
            x_cable,
            y_cable,
            T,
            L,
        ) = system.unpack_state(y_arr)
    except Exception:
        return False

    if x_cable is None or y_cable is None or len(x_cable) <= 1:
        return False
    if L is None or float(L) <= 1e-9:
        return False

    try:
        x_proj, y_proj, straight_mode, _ = system.cable.solver._normalize_cable_length(
            np.asarray(x_cable, dtype=float),
            np.asarray(y_cable, dtype=float),
            float(L),
            x_boat=float(x_boat),
            y_boat=0.0,
            x_rov=float(x_rov),
            y_rov=float(y_rov),
            k_tail=k_tail,
            t=float(t_sim),
        )
        x_proj = np.asarray(x_proj, dtype=float)
        y_proj = np.asarray(y_proj, dtype=float)
        x_proj[0] = float(x_boat)
        y_proj[0] = 0.0
        x_rov_n = float(x_rov)
        y_rov_n = float(y_rov)
        x_proj, y_proj, x_rov_n, y_rov_n = reconcile_straight_mode_after_normalize(
            x_proj,
            y_proj,
            straight_mode,
            x_rov_n,
            y_rov_n,
            float(x_boat),
            0.0,
        )

        weight_per_unit = (
            (system.cable.rho_cable - system.environment.rho_eau)
            * system.cable.A_cable
            * system.environment.g
        )
        T_proj = system.cable.solver._compute_catenary_tensions(
            x_proj,
            y_proj,
            weight_per_unit,
            rov_m=system.rov.m,
            rov_vol=system.rov.V,
        )
        T_proj = np.asarray(T_proj, dtype=float)
        t_uncapped_max = float(np.max(T_proj)) if T_proj.size > 0 else 0.0
        slack_ratio, T_proj = slack_ratio_and_soft_cap_tensions(
            float(x_boat),
            0.0,
            float(x_rov_n),
            float(y_rov_n),
            float(L),
            T_proj,
        )
        t_after_max = float(np.max(T_proj)) if T_proj.size > 0 else 0.0
        if (
            slack_ratio > _SLACK_RATIO_TENSION_RELAX
            and 39.0 <= float(t_sim) <= 40.6
            and t_uncapped_max > t_after_max + 1e-6
        ):
            trace_print(
                7,
                (
                    "[PROJ CAP] project_cable_state_inplace "
                    f"t={float(t_sim):.2f} slack_ratio={slack_ratio:.4f} "
                    f"Tmax_before={t_uncapped_max:.2f} Tmax_after={t_after_max:.2f}"
                ),
            )

        idx_x = 6
        idx_y = idx_x + system.N + 1
        idx_t = idx_y + system.N + 1
        y_arr[0] = x_rov_n
        y_arr[1] = y_rov_n
        y_arr[idx_x:idx_y] = x_proj
        y_arr[idx_y:idx_t] = y_proj
        y_arr[idx_t : idx_t + system.N + 1] = T_proj
        return True
    except Exception:
        return False


def repack_cable_static_equilibrium_inplace(system: Any, y: np.ndarray) -> bool:
    """
    Remplace la géométrie et les tensions du câble dans ``y`` par une solution statique cohérente
    (même pipeline que l'init : ``solve_equilibrium_static`` + tensions caténaire).

    Utilisé en secours lorsque la projection normale échoue, pour éviter de poursuivre avec
    L_seg ≠ L ou des extrémités décollées.
    """
    y_arr = np.asarray(y, dtype=float)
    try:
        (
            x_rov,
            y_rov,
            _vx,
            _vy,
            x_boat,
            _vx_boat,
            _xc,
            _yc,
            _T,
            L,
        ) = system.unpack_state(y_arr)
    except Exception:
        return False
    if L is None or float(L) <= 1e-9:
        return False
    try:
        xc, yc, _Tn = system.cable.solver.solve_equilibrium_static(
            float(x_rov),
            float(y_rov),
            float(x_boat),
            float(L),
            rov_m=system.rov.m,
            rov_vol=system.rov.V,
        )
        xc = np.asarray(xc, dtype=float).reshape(-1)
        yc = np.asarray(yc, dtype=float).reshape(-1)
        if xc.size != system.N + 1 or yc.size != system.N + 1:
            return False
        xc = xc.copy()
        yc = yc.copy()
        xc[0] = float(x_boat)
        yc[0] = 0.0
        xc[-1] = float(x_rov)
        yc[-1] = float(y_rov)

        weight_per_unit = (
            (system.cable.rho_cable - system.environment.rho_eau)
            * system.cable.A_cable
            * system.environment.g
        )
        T_proj = system.cable.solver._compute_catenary_tensions(
            xc,
            yc,
            weight_per_unit,
            rov_m=system.rov.m,
            rov_vol=system.rov.V,
        )
        T_proj = np.asarray(T_proj, dtype=float).reshape(-1)
        if T_proj.size != system.N + 1:
            return False

        try:
            T_cap = float(system._abs_tension_cap_n())
        except Exception:
            T_cap = 1.8e5
        T_proj = np.clip(T_proj, 0.0, T_cap)
        try:
            t_uncapped_max = float(np.max(T_proj)) if T_proj.size > 0 else 0.0
            slack_ratio, T_proj = slack_ratio_and_soft_cap_tensions(
                float(x_boat),
                0.0,
                float(x_rov),
                float(y_rov),
                float(L),
                T_proj,
            )
            t_after_max = float(np.max(T_proj)) if T_proj.size > 0 else 0.0
            if slack_ratio > _SLACK_RATIO_TENSION_RELAX and t_uncapped_max > t_after_max + 1e-6:
                trace_print(
                    7,
                    (
                        "[PROJ CAP] repack_cable_static_equilibrium_inplace "
                        f"slack_ratio={slack_ratio:.4f} "
                        f"Tmax_before={t_uncapped_max:.2f} Tmax_after={t_after_max:.2f}"
                    ),
                )
        except Exception:
            pass

        idx_x = 6
        idx_y = idx_x + system.N + 1
        idx_t = idx_y + system.N + 1
        y_arr[idx_x:idx_y] = xc
        y_arr[idx_y:idx_t] = yc
        y_arr[idx_t : idx_t + system.N + 1] = T_proj
        return True
    except Exception:
        return False
