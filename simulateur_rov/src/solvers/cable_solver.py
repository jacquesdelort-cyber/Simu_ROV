"""Solveur pour les équations du câble"""
import os
from logging import debug
from re import A, L
from tkinter import N
import numpy as np
from numpy._typing import _64Bit
from scipy.optimize import fsolve, minimize_scalar, root_scalar, minimize, NonlinearConstraint, least_squares, nnls
from scipy.interpolate import interp1d
from src.utils.logger import trace_print
from src.utils.scenario_utils import contrôle_câble
from src.utils.utils import (
    enforce_cable_segments_nb,
    scale_slack,
    create_point_with_target_length,
    next_point,
)
from tests.cable_shared_cases import add_cable_to_test_cases

_ANSI_RESET = "\033[0m"
_ANSI_RED = "\033[91m"
_ANSI_GREEN = "\033[92m"
_ANSI_YELLOW = "\033[93m"
_ANSI_BLUE = "\033[94m"
_ANSI_MAGENTA = "\033[95m"
_ANSI_CYAN = "\033[96m"
_ANSI_WHITE = "\033[97m"
_ANSI_GRAY = "\033[90m"
_ANSI_LIGHT_GRAY = "\033[37m"
_ANSI_LIGHT_RED = "\033[91m"

# Quatrième valeur de retour de ``_normalize_cable_length`` : origine de la géométrie renvoyée.
NCL_SOURCE_NORMALIZE_SEGMENTS = "_normalize_cable_segments"
NCL_SOURCE_HISTORICAL_FALLBACK = "_normalize_cable_length_historical_fallback"

# Recalage ROV en mode straight : au-delà, on refuse le résultat segments et on enchaîne le fallback.
STRAIGHT_ROV_SNAP_MAX_M = 0.1

# Courant quasi nul : conserve le chemin validé chaînette flottante sans optimisation.
CURRENT_NEGLIGIBLE_M_S = 1e-9
# Identifiant affiché dans l’UI (titre fenêtre) pour vérifier qu’on n’exécute pas une vieille copie du code.
CABLE_SOLVER_BUILD_ID = "neutral-uniform-bezier-before-nodal-20260419"
# Au-delà, pas de déformation « courant statique » (coût compute_cable_forces × passes).
# Missions type M_test_init_courant_0 utilisent N≈1000 : il faut autoriser le skew.
MAX_N_SEGMENTS_BUOYANT_STATIC_CURRENT_SKEW = 2500
# Équilibre nodal (maillage grossier) pour l'init statique avec courant.
NODAL_EQUILIBRIUM_COARSE_SEGMENTS_DEFAULT = 28
NODAL_EQUILIBRIUM_LS_MAX_NFEV = 400
NODAL_EQUILIBRIUM_LENGTH_WEIGHT = 1.0
NODAL_EQUILIBRIUM_SMOOTH_X_WEIGHT = 0.65
NODAL_EQUILIBRIUM_SMOOTH_Y_WEIGHT = 0.35
BUOYANT_SKEW_DEPTH_BAND_M = 40.0
BUOYANT_MIN_Y_MARGIN_M = 0.25
BUOYANT_CURVATURE_MISMATCH_MAX = 0.45


def _quadratic_bezier_arc_length(
    ax: float,
    ay: float,
    cx: float,
    cy: float,
    bx: float,
    by: float,
    n: int = 512,
) -> float:
    """Longueur d'arc approchée d'une courbe de Bézier quadratique 2D (échantillonnage uniforme en t)."""
    ts = np.linspace(0.0, 1.0, int(max(4, n)))
    om = 1.0 - ts
    px = om * om * ax + 2.0 * om * ts * cx + ts * ts * bx
    py = om * om * ay + 2.0 * om * ts * cy + ts * ts * by
    dx = np.diff(px)
    dy = np.diff(py)
    return float(np.sum(np.sqrt(dx * dx + dy * dy)))


def _vertical_slack_quadratic_bezier(
    xb_f: float,
    xr_f: float,
    yr_f: float,
    L_target: float,
    h_mag_geo: float,
    h_signed: float,
    N: int,
    n_arc_sample: int = 512,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Alignement vertical des extrémités : courbe lisse (Bézier quadratique) au lieu d'un V polyline.
    Le point de contrôle est à mi-profondeur, décalé horizontalement ; |h| est résolu pour que
    la longueur d'arc ≈ L_target. Le signe de h suit ``h_signed`` (courant).
    """
    P0x, P0y = float(xb_f), 0.0
    P2x, P2y = float(xr_f), float(yr_f)
    Cy = 0.5 * P2y
    L_need = float(L_target)
    h_dir = 1.0 if float(h_signed) >= 0.0 else -1.0

    def arc_len(h_abs: float) -> float:
        Cx = P0x + h_dir * float(max(0.0, h_abs))
        return _quadratic_bezier_arc_length(
            P0x, P0y, Cx, Cy, P2x, P2y, n_arc_sample
        )

    f0 = arc_len(0.0)
    if f0 >= L_need - 1e-9:
        h_opt = 0.0
    else:
        hi = max(float(h_mag_geo), 1e-6)
        f_hi = arc_len(hi)
        expand = 0
        while f_hi < L_need and hi < 1.0e7 and expand < 60:
            hi *= 1.5
            f_hi = arc_len(hi)
            expand += 1
        if f_hi < L_need:
            h_opt = hi
        else:
            lo = 0.0
            for _ in range(64):
                mid = 0.5 * (lo + hi)
                if arc_len(mid) < L_need:
                    lo = mid
                else:
                    hi = mid
            h_opt = 0.5 * (lo + hi)

    Cx = P0x + h_dir * h_opt
    n_seg = int(max(1, N))
    x_cable = np.zeros(n_seg + 1)
    y_cable = np.zeros(n_seg + 1)
    for i in range(n_seg + 1):
        t = float(i) / float(n_seg)
        om = 1.0 - t
        x_cable[i] = om * om * P0x + 2.0 * om * t * Cx + t * t * P2x
        y_cable[i] = om * om * P0y + 2.0 * om * t * Cy + t * t * P2y
    return x_cable, y_cable


def apply_straight_mode_rov_snap(
    straight_mode: bool,
    x_rov: float,
    y_rov: float,
    x_cable,
    y_cable,
) -> tuple[float, float]:
    """
    Met à jour (x_rov, y_rov) depuis l'extrémité câble seulement si straight_mode et saut ≤ STRAIGHT_ROV_SNAP_MAX_M.
    Sinon recolle ``x_cable[-1], y_cable[-1]`` au ROV (évite une fin de câble « mode straight » incohérente).
    """
    xc = np.asarray(x_cable, dtype=float)
    yc = np.asarray(y_cable, dtype=float)
    if not straight_mode:
        return float(x_rov), float(y_rov)
    snap = float(np.hypot(float(xc[-1]) - float(x_rov), float(yc[-1]) - float(y_rov)))
    if snap <= STRAIGHT_ROV_SNAP_MAX_M:
        return float(xc[-1]), float(yc[-1])
    xc[-1] = float(x_rov)
    yc[-1] = float(y_rov)
    return float(x_rov), float(y_rov)


def _ncl_trace_level(level: int, mode_test: bool) -> int:
    """Niveau passé à ``trace_print`` : +1 si ``mode_test`` (même ``TRACE_LEVEL`` → traces plus tôt)."""
    return level + 1 if mode_test else level


class CableSolver:
    """Résout les équations d'équilibre du câble"""
    
    def __init__(self, N_segments, params, environment):
        """
        Initialise le solveur de câble
        
        Parameters:
        -----------
        N_segments : int
            Nombre de segments pour discrétiser le câble
        params : dict
            Paramètres du câble
        environment : Environment
            Objet environnement
        """
        self.N = N_segments
        self.params = params
        self.environment = environment
        self.d = params['d']
        self.rho_cable = params['rho_cable']
        self.Cx_cable = params.get('Cx_cable', 1.2)
        self.Cf_cable = params.get('Cf_cable', 0.04)
        self.A_cable = np.pi * (self.d / 2)**2
        self._T_prev = None
        # Temps simulé de référence (s) : 0 à la création (géométrie / état initial avant la boucle).
        # Mis à jour à chaque pas dans ``simulation_thread`` ; utilisé si ``t`` est omis dans
        # ``_normalize_cable_length`` (ex. construction du câble initial).
        self._last_sim_time: float = 0.0
        # Compteur d'échecs normalisation (limite les snapshots de debug) — ne pas attacher à la méthode.
        self._normalize_cable_length_nb_echecs: int = 0

    def _str_cable_info(
        self,
        points: np.ndarray,
        L_target: float,
        N_target: int,
        str_info: bool = True,
        atol: float = 1e-3,
        rtol: float = 1e-3,
    ) -> tuple[float, int, float, float, float, str | None]:
        """
        Renvoie un n-uplet d'infos synthétiques sur le câble, dont éventuellement
        une f-string prête à être affichée.

        Retourne (L_cable, N_cable, ds_target, min_seg, max_seg, str_info_fmt)
        où str_info_fmt est soit une chaîne formatée, soit None si str_info=False.
        """
        points = np.asarray(points, dtype=float)
        L_norms = np.linalg.norm(np.diff(points, axis=0), axis=1)
        L_cable = float(np.sum(L_norms))
        L_straight = float(np.linalg.norm(points[-1] - points[0]))
        N_cable = len(points) - 1
        ds_target = L_target / max(float(N_target), 1.0)
        min_seg = float(np.min(L_norms))
        max_seg = float(np.max(L_norms))
        index_min_seg = int(np.argmin(L_norms))
        index_max_seg = int(np.argmax(L_norms))

        if not str_info:
            return L_cable, N_cable, ds_target, min_seg, max_seg, None

        parts: list[str] = []

        ok_N_cable = N_cable == N_target
        if ok_N_cable:
            parts.append(f"N_cable={N_cable:4d}  ")
        else:
            parts.append(f"N_cable={_ANSI_RED}{N_cable:4d}{_ANSI_RESET}  ")

        ok_L_cable = abs(L_cable - L_target) <= max(atol, rtol * abs(L_target))
        if ok_L_cable:
            parts.append(f"L_cable={L_cable:8.3f}  ")
        else:
            parts.append(f"L_cable={_ANSI_RED}{L_cable:8.3f}{_ANSI_RESET}  ")

        # On compare L_straight à L_target (câble détendu vs longueur cible).
        ok_L_straight = L_straight <= L_target * (1.0 + rtol)
        if ok_L_straight:
            parts.append(f"L_straight={L_straight:8.3f}  ")
        else:
            parts.append(f"L_straight={_ANSI_RED}{L_straight:8.3f}{_ANSI_RESET}  ")

        ok_min_seg = min_seg >= ds_target * (1-rtol)
        if ok_min_seg:
            parts.append(f"min_seg={min_seg:8.5f} (at {index_min_seg})  ")
        else:
            parts.append(
                f"min_seg={_ANSI_RED}{min_seg:8.5f}{_ANSI_RESET} (at {index_min_seg})  "
            )

        ok_max_seg = 'True' if max_seg <= ds_target * (1+rtol) else 'False'
        if ok_max_seg:
            parts.append(f"max_seg={max_seg:8.5f} (at {index_max_seg})")
        else:
            parts.append(
                f"max_seg={_ANSI_RED}{max_seg:8.5f}{_ANSI_RESET} (at {index_max_seg})"
            )

        min_y = float(np.min(points[:, 1]))
        parts.append(f"   min_y={min_y:8.3f}")
        max_y = float(np.max(points[:, 1]))
        parts.append(f"   max_y={max_y:8.3f}")

        return L_cable, N_cable, ds_target, min_seg, max_seg, "".join(parts)

    def _str_cable_format(self, points: np.ndarray, ds_target: float | None = None) -> str:
        """
        Formate un câble P (shape (N+1, 2)) sous la forme :
        [ x0  y0 ] ds0 [ x1  y1 ] ds1 ... [ xN  yN ]
        où dsi est la longueur du segment entre les points i et i+1.
        """
        pts = np.asarray(points, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 2 or pts.shape[0] == 0:
            return repr(pts)

        n = pts.shape[0]
        parts: list[str] = []
        for i in range(n):
            x, y = pts[i]
            parts.append(f"[{x:5.2f} {y:5.2f}]")
            if i < n - 1:
                dx = pts[i + 1, 0] - x
                dy = pts[i + 1, 1] - y
                ds = float(np.hypot(dx, dy))
                if ds_target is None:
                    parts.append(f" {ds:5.2f} ")
                elif abs(ds - ds_target) > 1e-6:
                    # Rouge si la longueur de segment s'écarte trop de la cible
                    parts.append(f" \x1b[31m{ds:5.2f}\x1b[0m ")
                else:
                    # Vert sinon
                    parts.append(f" \x1b[32m{ds:5.2f}\x1b[0m ")
        return "".join(parts)

    def _str_result_cable_format(self, points: np.ndarray, L_target, N_target ):
        """
        Renvoie une string qui synthétise les informations sur le câble après normalisation.
        Cette string contient une liste formattée des points du câble.
        Parameters
        ----------
        points : np.ndarray
            Points du câble après normalisation.
        L_target : float
            Longueur cible du câble.
        N_target : int
            Nombre de segments cible du câble.
        Returns
        -------
        str
            String qui synthétise les informations sur le câble après normalisation.
        """
        parts: list[str] = []
        Longueur_cable = float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
        if abs(Longueur_cable - L_target) > 1e-3:
            parts.append(f"L_cable= {_ANSI_RED}{Longueur_cable:6.2f}{_ANSI_RESET}  ") 
        else:
            parts.append(f"L_cable= {_ANSI_GREEN}{Longueur_cable:6.2f}{_ANSI_RESET}  ")

        if len(points) - 1 == N_target:
            parts.append(f"N_target= {_ANSI_GREEN}{N_target:4d}{_ANSI_RESET}  ")
        else:
            parts.append(f"N_target= {_ANSI_RED}{N_target:4d}{_ANSI_RESET}  ")

        parts.append(f"ds_target= {L_target / N_target: 4.2f}  ")
        parts.append(f"L_target= {L_target: 4.2f}\n  ")
        parts.append(self._str_cable_format(points, L_target / N_target  ))

        return "".join(parts)

    def aplatir_polyline(self, Q: np.ndarray, k: float) -> tuple[np.ndarray | None, str]:
        """
        Aplatit la polyline Q en écrasant chaque point par rapport à la droite
        définie par Q[0] -> Q[-1].

        Pour chaque point q_i de Q, on calcule sa projection orthogonale q1_i sur la droite
        (Q[0]Q[-1]) puis on renvoie :
            t_i = q1_i + k * (q_i - q1_i)

        Q n'est pas modifié. 
        Retour : (T, explication) où T est la polyligne résultante 
        ou None si la géométrie d'entrée n'est pas conforme ; 
        explication est une chaîne de caractères qui explique la valeur de retour.
        """
        Q_arr = np.array(Q, dtype=float, copy=True)
        if Q_arr.ndim != 2 or Q_arr.shape[1] != 2 or Q_arr.shape[0] < 2:
            return None, "aplatir_polyline: Q doit être un tableau de forme (M, 2) avec M >= 2"

        q0 = Q_arr[0]
        qn = Q_arr[-1]
        d = qn - q0

        # Droite dégénérée : q0 == qn => la "projection" est q0 pour tout point
        den = float(np.dot(d, d))
        if den <= 1e-18:
            Q_proj = np.broadcast_to(q0, Q_arr.shape).copy()
        else:
            # t_proj_i = dot(Q_i - q0, d) / dot(d, d)
            t_proj = np.dot(Q_arr - q0, d) / den  # shape (M,)
            Q_proj = q0 + t_proj[:, None] * d

        T = Q_proj + float(k) * (Q_arr - Q_proj)
        return T, "aplatir_polyline: OK"

    def _remap_polyline_arclength_to_chord(self, P: np.ndarray) -> np.ndarray:
        """
        Même nombre de sommets : paramètre d'abscisse curviligne normalisée sur [0,1],
        puis position sur la corde droite P[0] → P[-1]. La polyligne résultante est
        monotone le long de la corde et sa longueur vaut exactement ||P[-1]-P[0]||.
        """
        P = np.asarray(P, dtype=float)
        if P.ndim != 2 or P.shape[1] != 2 or P.shape[0] < 2:
            return np.array(P, dtype=float, copy=True)
        q0 = P[0].copy()
        qn = P[-1].copy()
        seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
        Lp = float(np.sum(seg))
        if Lp <= 1e-15:
            return P.copy()
        u = np.zeros(P.shape[0], dtype=float)
        u[1:] = np.cumsum(seg) / Lp
        d = qn - q0
        return q0 + u[:, None] * d

    def _bisect_blend_polyline_length_to_target(
        self,
        Q_shorter: np.ndarray,
        Q_longer: np.ndarray,
        q0: np.ndarray,
        qn: np.ndarray,
        L_target: float,
        tol: float,
    ) -> tuple[np.ndarray | None, str]:
        """
        Trouve α ∈ [0,1] tel que la longueur de (1-α) Q_shorter + α Q_longer
        (recollage des extrémités sur q0, qn et clip y <= 0) soit proche de L_target.
        On suppose polyline_length(Q_shorter) < polyline_length(Q_longer).
        """
        Q_s = np.asarray(Q_shorter, dtype=float)
        Q_l = np.asarray(Q_longer, dtype=float)
        if Q_s.shape != Q_l.shape:
            return None, "blend: formes différentes"
        q0 = np.asarray(q0, dtype=float).reshape(2)
        qn = np.asarray(qn, dtype=float).reshape(2)

        def synth(alpha: float) -> tuple[float, np.ndarray]:
            T = (1.0 - alpha) * Q_s + alpha * Q_l
            T = np.asarray(T, dtype=float, copy=True)
            T[0] = q0
            T[-1] = qn
            T[:, 1] = np.minimum(T[:, 1], 0.0)
            T[0] = q0
            T[-1] = qn
            diffs = np.diff(T, axis=0)
            return float(np.sum(np.linalg.norm(diffs, axis=1))), T

        L_lo, T_lo = synth(0.0)
        L_hi, T_hi = synth(1.0)
        if abs(L_lo - L_target) <= tol:
            return T_lo, "blend: alpha=0"
        if abs(L_hi - L_target) <= tol:
            return T_hi, "blend: alpha=1"
        if L_target < L_lo - tol or L_target > L_hi + tol:
            return None, "blend: L_target hors [L_courte, L_longue]"
        a_lo, a_hi = 0.0, 1.0
        for _ in range(100):
            a_mid = 0.5 * (a_lo + a_hi)
            L_mid, T_mid = synth(a_mid)
            if abs(L_mid - L_target) <= tol:
                return T_mid, "blend: dichotomie OK"
            if L_mid < L_target:
                a_lo = a_mid
            else:
                a_hi = a_mid
            if a_hi - a_lo < 1e-14:
                break
        L_mid, T_mid = synth(0.5 * (a_lo + a_hi))
        if abs(L_mid - L_target) <= 10.0 * tol:
            return T_mid, "blend: dichotomie (tol élargie)"
        return None, "blend: échec dichotomie"

    def deformer_polyline(self, Q: np.ndarray, L_target: float, atol: float, rtol: float) -> tuple[np.ndarray | None, str]:
        """
        Déforme la polyline `Q` en faisant varier un facteur d'aplatissement `k >= 0`.

        On définit `R(k) = aplatir_polyline(Q, k)[0]`. On cherche un `k` tel que :
            |length(R(k)) - L_target| <= max(atol, rtol*|L_target|)

        `Q` n'est pas modifié. Retour : ``(T, explication)`` où ``T`` est la polyligne
        déformée si une solution a été trouvée, sinon ``None`` ; ``explication`` est
        réservée pour des messages futurs (chaîne vide pour l'instant).

        - Si tous les points de `Q` sont alignés (distance orthogonale max à la droite Q[0]Q[-1] <= atol) :
            ``(None, "")``
        - Si la géométrie d'entrée est invalide ou si aucun `k` ne satisfait la tolérance :
            ``(None, "")``
        """
        Q_arr = np.array(Q, dtype=float, copy=True)
        if Q_arr.ndim != 2 or Q_arr.shape[1] != 2 or Q_arr.shape[0] < 2:
            return None, "deformer_polyline: Q doit être un tableau de forme (M, 2) avec M >= 2"

        L_target = float(L_target)
        atol = float(atol)
        rtol = float(rtol)

        tol = max(atol, rtol * abs(L_target))

        # 1) Détection d'alignement : max distance orthogonale <= atol
        q0 = Q_arr[0]
        qn = Q_arr[-1]
        d = qn - q0
        den = float(np.dot(d, d))
        if den <= 1e-18:
            # Droite dégénérée : tous les points sont projetés en q0
            max_dist = float(np.max(np.linalg.norm(Q_arr - q0, axis=1)))
            if max_dist <= atol:
                return None, "deformer_polyline: Tous les points sont alignés"
            # sinon : on peut quand même déformer vers une ligne "ponctuelle"
        else:
            # projection q' sur la droite, puis distance orthogonale ||q - q'||
            t_proj = np.dot(Q_arr - q0, d) / den  # shape (M,)
            Q_proj = q0 + t_proj[:, None] * d
            dists = np.linalg.norm(Q_arr - Q_proj, axis=1)
            if float(np.max(dists)) <= atol:
                return None, "deformer_polyline: Tous les points sont alignés"

        def polyline_length(points: np.ndarray) -> float:
            diffs = np.diff(points, axis=0)
            return float(np.sum(np.linalg.norm(diffs, axis=1)))

        def L_of(k: float) -> tuple[float, np.ndarray | None]:
            R, _ = self.aplatir_polyline(Q_arr, k)
            if R is None:
                return float("nan"), None
            return polyline_length(R), R

        # 2) Bracketing sur k >= 0
        k_low = 0.0
        L_low, R_low = L_of(k_low)
        if abs(L_low - L_target) <= tol:
            return R_low, "deformer_polyline: L_low = L_target"

        # Si L(k=0) > L_target : la projection sur la corde dans l'ordre des indices peut
        # zigzaguer (longueur > L_target alors que la corde géométrique est plus courte).
        # On ramène d'abord sur la corde monotone (longueur = corde), puis on mélange avec Q.
        if L_target < L_low - tol:
            Q_flat = self._remap_polyline_arclength_to_chord(Q_arr)
            L_flat = polyline_length(Q_flat)
            L_orig = polyline_length(Q_arr)
            # Câble tendu (L_target ≈ corde) : pas de marge pour « mélanger » au‑dessus de L_flat.
            # Sinon lo_b + tol < L_target échoue et on renvoyait None avec une géométrie encore folle.
            if float(L_target) <= L_flat + tol:
                Tq = np.asarray(Q_flat, dtype=float, copy=True)
                Tq[0] = np.asarray(q0, dtype=float)
                Tq[-1] = np.asarray(qn, dtype=float)
                Tq[:, 1] = np.minimum(Tq[:, 1], 0.0)
                Tq[0] = np.asarray(q0, dtype=float)
                Tq[-1] = np.asarray(qn, dtype=float)
                return Tq, "deformer_polyline: corde monotone (L_target ≤ L_chord+tol)"
            lo_b, hi_b = min(L_flat, L_orig), max(L_flat, L_orig)
            if lo_b + tol < float(L_target) < hi_b - tol and L_flat + 1e-9 < L_orig:
                Q_s, Q_l = (Q_flat, Q_arr) if L_flat < L_orig else (Q_arr, Q_flat)
                Tb, msg_b = self._bisect_blend_polyline_length_to_target(
                    Q_s,
                    Q_l,
                    np.asarray(q0, dtype=float),
                    np.asarray(qn, dtype=float),
                    L_target,
                    tol,
                )
                if Tb is not None:
                    return Tb, msg_b
            return None, "deformer_polyline: L_target < L_low"

        k_high = 1.0
        k_high_max = 1e6

        L_high, _ = L_of(k_high)
        while L_high < L_target and k_high < k_high_max:
            k_high *= 2.0
            L_high, _ = L_of(k_high)

        if L_high < L_target - tol:
            return None, "deformer_polyline: L_high < L_target"

        # 3) Dichotomie
        max_iter = 80
        for _ in range(max_iter):
            k_mid = 0.5 * (k_low + k_high)
            L_mid, R_mid = L_of(k_mid)

            if abs(L_mid - L_target) <= tol:
                return R_mid, f"deformer_polyline: L_mid = L_target  tol: {tol:8.3f} "

            if L_mid < L_target:
                k_low = k_mid
            else:
                k_high = k_mid

        return None, "deformer_polyline: Aucun k ne satisfait la tolérance"

    def _manhattan_case1_surface_vertical_polyline(
        self, xb: float, yb: float, xr: float, yr: float, L_target: float
    ):
        """
        Cas L ≥ Manhattan (ou quasi droit) : surface y=yb puis slack horizontal éventuel,
        puis vertical au droit du ROV — aligné sur ``SimulationTab._build_length_exact_polyline_case1_or_straight``.
        Retourne ``(None, None)`` si la géométrie n'est pas réalisable.
        """
        dx = float(xr - xb)
        dy = float(yr - yb)
        d = float(np.hypot(dx, dy))
        n_pts = max(int(self.N), 1) + 1
        if d <= 1e-12 or abs(L_target - d) <= 1e-9:
            x_line = np.linspace(float(xb), float(xr), n_pts)
            y_line = np.linspace(float(yb), float(yr), n_pts)
            y_line = np.clip(y_line, None, 0.0)
            y_line[0] = float(yb)
            y_line[-1] = float(yr)
            return x_line, y_line

        y_depth = abs(float(yr - yb))
        L_top = float(L_target) - y_depth
        dx_abs = abs(dx)
        manh_tol = max(1e-9, 1e-12 * max(abs(float(L_target)), 1.0))
        extra_sv = L_top - dx_abs
        if extra_sv < -manh_tol:
            return None, None
        if extra_sv <= manh_tol:
            if y_depth <= 1e-12 or dx_abs <= 1e-12:
                return None, None
            L_h = dx_abs
            L_v = y_depth
            Ltot = L_h + L_v
            n1 = max(2, int(round((n_pts - 1) * L_h / Ltot)) + 1)
            n1 = min(n1, n_pts)
            n2 = n_pts - n1 + 1
            if n2 < 2:
                n2 = 2
                n1 = n_pts - n2 + 1
                n1 = max(2, n1)
            x_first = np.linspace(float(xb), float(xr), n1)
            y_first = np.full(n1, float(yb), dtype=float)
            x_second = np.full(n2, float(xr), dtype=float)
            y_second = np.linspace(float(yb), float(yr), n2)
            x_new = np.concatenate((x_first, x_second[1:]))
            y_new = np.concatenate((y_first, y_second[1:]))
            y_new = np.clip(y_new, None, 0.0)
            y_new[0] = float(yb)
            y_new[-1] = float(yr)
            return x_new, y_new

        sign = 1.0 if dx >= 0.0 else -1.0
        extra = L_top - dx_abs
        x_turn = float(xr) + sign * (0.5 * extra)
        L1 = abs(x_turn - float(xb))
        L2 = abs(x_turn - float(xr))
        L3 = y_depth
        Ltot = L1 + L2 + L3
        s_vals = np.linspace(0.0, Ltot, n_pts)
        x_new = np.empty(n_pts, dtype=float)
        y_new = np.empty(n_pts, dtype=float)
        for i, s in enumerate(s_vals):
            if s <= L1:
                a = s / max(L1, 1e-12)
                x_new[i] = (1.0 - a) * float(xb) + a * x_turn
                y_new[i] = 0.0
            elif s <= (L1 + L2):
                a = (s - L1) / max(L2, 1e-12)
                x_new[i] = (1.0 - a) * x_turn + a * float(xr)
                y_new[i] = 0.0
            else:
                a = (s - L1 - L2) / max(L3, 1e-12)
                x_new[i] = float(xr)
                y_new[i] = (1.0 - a) * 0.0 + a * float(yr)
        y_new = np.clip(y_new, None, 0.0)
        y_new[0] = float(yb)
        y_new[-1] = float(yr)
        return x_new, y_new

    def _pure_catenary_equilibrium_buoyant_cable(
        self, x_rov, y_rov, x_boat, L, rov_m=None, rov_vol=None
    ):
        """
        Poids linéique apparent négatif (ρ_cable < ρ_eau) : éviter ``solve_equilibrium_constrained``.

        On utilise la géométrie analytique ``build_buoyant_cable_polyline`` (repère x', z avec
        sommet chaînette / segment surface) validée par les missions d'init flottantes. Si le cas
        est délégué (Manhattan / droite), même polyline que l'onglet simulation ; en dernier recours
        seulement, ``_solve_catenary`` (chaînette lourde discrète).
        """
        from src.utils.cable_init_buoyant import build_buoyant_cable_polyline

        w = (self.rho_cable - self.environment.rho_eau) * self.A_cable * self.environment.g
        x_b, y_b = build_buoyant_cable_polyline(
            float(x_boat),
            0.0,
            float(x_rov),
            float(y_rov),
            float(L),
            int(self.N),
            slack_side="auto",
        )
        if x_b is None or y_b is None:
            x_b, y_b = self._manhattan_case1_surface_vertical_polyline(
                float(x_boat), 0.0, float(x_rov), float(y_rov), float(L)
            )
        if x_b is not None and y_b is not None:
            x_cable = np.asarray(x_b, dtype=float)
            y_cable = np.asarray(y_b, dtype=float)
            try:
                x_cable, y_cable, _, _ = self._normalize_cable_length(
                    x_cable,
                    y_cable,
                    float(L),
                    x_boat=float(x_boat),
                    y_boat=0.0,
                    x_rov=float(x_rov),
                    y_rov=float(y_rov),
                    k_max=2.0,
                )
            except Exception:
                pass
            y_cable = np.clip(y_cable, None, 0.0)
            y_cable[0] = 0.0
            y_cable[-1] = float(y_rov)
            x_cable[0] = float(x_boat)
            x_cable[-1] = float(x_rov)
            T = self._compute_catenary_tensions(
                x_cable, y_cable, w, rov_m=rov_m, rov_vol=rov_vol
            )
            return x_cable, y_cable, T

        x_cable, y_cable = self._solve_catenary(x_rov, y_rov, x_boat, L, w)
        x_cable = np.asarray(x_cable, dtype=float)
        y_cable = np.asarray(y_cable, dtype=float)
        T = self._compute_catenary_tensions(
            x_cable, y_cable, w, rov_m=rov_m, rov_vol=rov_vol
        )
        return x_cable, y_cable, T

    def _project_cable_bounds(self, x, y, x_boat, y_boat, x_rov, y_rov, y_margin_m: float):
        """Projection locale: extrémités exactes, y <= 0 et pas de point sous y_rov - marge."""
        x = np.asarray(x, dtype=float).copy()
        y = np.asarray(y, dtype=float).copy()
        if len(x) != len(y) or len(x) < 2:
            return x, y
        y = np.clip(y, None, 0.0)
        y_floor = float(y_rov) - float(max(0.0, y_margin_m))
        y = np.maximum(y, y_floor)
        x[0], y[0] = float(x_boat), float(y_boat)
        x[-1], y[-1] = float(x_rov), float(y_rov)
        return x, y

    def _project_buoyant_bounds(self, x, y, x_boat, y_boat, x_rov, y_rov):
        """Projection locale dédiée câble flottant (marge sous ROV autorisée)."""
        return self._project_cable_bounds(
            x, y, x_boat, y_boat, x_rov, y_rov, BUOYANT_MIN_Y_MARGIN_M
        )

    def _flip_signed_perp_across_chord(self, x, y, x_boat, y_boat, x_rov, y_rov):
        """
        Symétrie des points par rapport à la droite bateau--ROV (inverse le signe
        de la distance perpendiculaire à la corde).

        L'équilibre nodal minimise les forces avec ``CURRENT_TO_FLUID_VX_SIGN`` ;
        pour un profil **non uniforme** en profondeur, cela inverse le sens du S par
        rapport au graphe « Profil du courant » (vitesse mission brute). Cette
        réflexion réaligne l'init statique affichée sans toucher au calcul dynamique.
        """
        x = np.asarray(x, dtype=float).copy()
        y = np.asarray(y, dtype=float).copy()
        dx = float(x_rov) - float(x_boat)
        dy = float(y_rov) - float(y_boat)
        chord_len = float(np.hypot(dx, dy))
        if chord_len < 1e-12:
            return x, y
        nx = -dy / chord_len
        ny = dx / chord_len
        xc = x - float(x_boat)
        yc = y - float(y_boat)
        sp = xc * nx + yc * ny
        x -= 2.0 * sp * nx
        y -= 2.0 * sp * ny
        x[0] = float(x_boat)
        x[-1] = float(x_rov)
        y[0] = float(y_boat)
        y[-1] = float(y_rov)
        return x, y

    def _should_flip_nodal_geometry_vs_mission_plot(self, y_rov: float) -> bool:
        """
        Réflexion perpendiculaire à la corde : uniquement si le courant **mission** n'est pas
        quasi uniforme **et** qu'il n'est pas nul partout. Sinon (v = 0 partout) ``_detect_uniform_current_signature``
        renvoie ``is_uniform=False`` et on ne doit **pas** retourner la chaînette / le nodal.
        """
        y_probe = np.linspace(
            0.0,
            float(y_rov),
            int(max(8, min(64, 1 + int(abs(float(y_rov)) / 5.0)))),
        )
        is_uniform, _ = self._detect_uniform_current_signature(y_probe)
        env = self.environment
        v_span = float(env.max_abs_current_on_vertical_segment(0.0, float(y_rov)))
        v_decl = float(env.max_abs_speed_declared_in_raw_profile())
        has_current = max(v_span, v_decl) > CURRENT_NEGLIGIBLE_M_S
        return (not is_uniform) and has_current

    def _build_current_signed_weights(self, y_vals):
        """
        Poids signés par profondeur pour l’heuristique géométrique (skew / validation).
        Utilise la vitesse **mission** brute (``get_current_velocity`` sans ``CURRENT_TO_FLUID_VX_SIGN``)
        pour coller au graphe « Profil du courant » ; les forces dynamiques restent dans ``compute_cable_forces``.
        """
        y = np.asarray(y_vals, dtype=float)
        if len(y) == 0:
            return np.array([])
        v_raw = getattr(self.environment, "v_courant_raw", None)
        v_node = np.asarray(
            [self.environment.get_current_velocity(float(yy), v_raw) for yy in y],
            dtype=float,
        )
        depth = np.where(y < 0.0, -y, y)
        depth_max = float(np.max(depth))
        if depth_max <= 1e-9:
            return np.tanh(v_node / max(CURRENT_NEGLIGIBLE_M_S, 1e-6))
        band = max(float(BUOYANT_SKEW_DEPTH_BAND_M), depth_max / 16.0)
        n_band = int(max(1, np.ceil(depth_max / band)))
        weights = np.zeros_like(v_node)
        for b in range(n_band):
            d0 = b * band
            d1 = min((b + 1) * band, depth_max + 1e-12)
            m = (depth >= d0) & (depth <= d1 + 1e-12)
            if not np.any(m):
                continue
            v_mean = float(np.mean(v_node[m]))
            amp = float(np.max(np.abs(v_node[m])))
            denom = max(amp, 0.05)
            weights[m] = np.tanh(v_mean / denom)
        return weights

    def _smooth_polyline_shape(self, x, y, x_boat, y_boat, x_rov, y_rov, L, y_margin_m, passes=1):
        """
        Lissage local de la polyligne pour supprimer les angles aigus / zigzags.
        Chaque passe est suivie d'une renormalisation de longueur.
        """
        x = np.asarray(x, dtype=float).copy()
        y = np.asarray(y, dtype=float).copy()
        n = len(x)
        if n < 4:
            return x, y
        for _ in range(max(0, int(passes))):
            x_old = x.copy()
            y_old = y.copy()
            x[1:-1] = 0.2 * x_old[:-2] + 0.6 * x_old[1:-1] + 0.2 * x_old[2:]
            y[1:-1] = 0.2 * y_old[:-2] + 0.6 * y_old[1:-1] + 0.2 * y_old[2:]
            x, y = self._project_cable_bounds(
                x, y, x_boat, y_boat, x_rov, y_rov, y_margin_m
            )
            try:
                x, y, _, _ = self._normalize_cable_length(
                    x,
                    y,
                    float(L),
                    x_boat=float(x_boat),
                    y_boat=float(y_boat),
                    x_rov=float(x_rov),
                    y_rov=float(y_rov),
                    k_max=2.0,
                )
            except Exception:
                return x_old, y_old
            x, y = self._project_cable_bounds(
                x, y, x_boat, y_boat, x_rov, y_rov, y_margin_m
            )
        return x, y

    def _detect_uniform_current_signature(self, y_vals):
        """
        Détecte un courant quasi uniforme en profondeur sur la colonne d'eau du câble.
        Retourne (is_uniform, side_sign) où side_sign est le signe attendu du déport x.
        """
        y = np.asarray(y_vals, dtype=float)
        if y.size == 0:
            return False, 0.0
        v_raw = getattr(self.environment, "v_courant_raw", None)
        from .forces import CURRENT_TO_FLUID_VX_SIGN
        v_node = np.asarray(
            [self.environment.get_current_velocity(float(yy), v_raw) for yy in y],
            dtype=float,
        )
        v_node = CURRENT_TO_FLUID_VX_SIGN * v_node
        m = np.abs(v_node) > CURRENT_NEGLIGIBLE_M_S
        if not np.any(m):
            return False, 0.0
        v_act = v_node[m]
        sgn = np.sign(v_act)
        sgn = sgn[sgn != 0.0]
        if sgn.size == 0:
            return False, 0.0
        frac_pos = float(np.mean(sgn > 0.0))
        frac_neg = float(np.mean(sgn < 0.0))
        dominant = max(frac_pos, frac_neg)
        if dominant < 0.95:
            return False, 0.0
        v_abs = np.abs(v_act)
        rel_std = float(np.std(v_abs) / max(float(np.mean(v_abs)), 1e-12))
        if rel_std > 0.15:
            return False, 0.0
        side_sign = float(np.sign(np.mean(v_act)))
        return True, side_sign

    def _enforce_uniform_current_concavity(
        self, x, y, x_boat, y_boat, x_rov, y_rov, strength=0.72
    ):
        """
        Pour un courant uniforme, projette la géométrie vers une forme à concavité unique
        (arc/parabole) afin d'éliminer les inversions locales de courbure.

        La correction est exprimée en **distance signée à la corde géométrique** bateau→ROV
        (repère direct : normale ``(-dy,dx)/L``), puis appliquée le long de cette normale.
        Cela coïncide avec la perception « gauche / droite » sur le graphe quelle que soit la
        répartition des nœuds (contrairement à une référence ``x_lin(i)`` indexée sur l'indice).
        """
        x = np.asarray(x, dtype=float).copy()
        y = np.asarray(y, dtype=float).copy()
        n = len(x)
        if n < 5:
            return x, y

        dx = float(x_rov) - float(x_boat)
        dy = float(y_rov) - float(y_boat)
        chord_len = float(np.hypot(dx, dy))
        if chord_len < 1e-12:
            return x, y

        n_x = -dy / chord_len
        n_y = dx / chord_len
        signed_perp = (x - float(x_boat)) * n_x + (y - float(y_boat)) * n_y
        amp = float(np.mean(np.abs(signed_perp[1:-1])))
        if amp <= 1e-10:
            return x, y

        # Orientation : vitesse **mission** brute (profondeur → v), sans CURRENT_TO_FLUID_VX_SIGN.
        v_raw = getattr(self.environment, "v_courant_raw", None)
        nys = int(max(8, min(n, 64)))
        ys = np.linspace(float(y_boat), float(y_rov), nys)
        raw_vals = np.asarray(
            [self.environment.get_current_velocity(float(yy), v_raw) for yy in ys],
            dtype=float,
        )
        if not np.any(np.abs(raw_vals) > CURRENT_NEGLIGIBLE_M_S):
            return x, y
        # Rampe mince en surface / près du ROV : la variance sur toute la colonne peut
        # dépasser 0.15 alors que le courant utile est quasi constant (ex. M_test_init_courant_1).
        # On mesure moyenne / dispersion sur le cœur (indices centraux), pas sur les 8 % extrêmes.
        trim = 0.08
        k = max(1, int(round(trim * max(nys - 1, 1))))
        raw_core = raw_vals[k:-k] if len(raw_vals) > 2 * k else raw_vals
        mean_raw = float(np.mean(raw_core))
        if abs(mean_raw) < 0.12:
            return x, y
        rel_std = float(np.std(raw_core) / max(abs(mean_raw), 1e-12))
        if rel_std > 0.15:
            return x, y

        # v mission > 0 : bosse du côté **positif** de la normale directe ``(-dy,dx)/L``,
        # i.e. dominante **+x** sur une géométrie type bateau (x≈0) → ROV (x>0), comme
        # l’abscisse positive du graphe « Profil du courant » (v en m/s vers la droite).
        # (Le calcul des forces conserve ``CURRENT_TO_FLUID_VX_SIGN`` ; ici on suit le signe brut mission.)
        desired_sign = float(np.sign(mean_raw))

        ds_seg = np.hypot(np.diff(x), np.diff(y))
        s_cum = np.concatenate(([0.0], np.cumsum(ds_seg)))
        t = s_cum / max(float(s_cum[-1]), 1e-12)
        arc = 4.0 * t * (1.0 - t)
        target_signed = desired_sign * amp * arc
        delta = float(strength) * (target_signed - signed_perp)
        x_new = x + delta * n_x
        y_new = y + delta * n_y
        x_new[0] = float(x_boat)
        x_new[-1] = float(x_rov)
        y_new[0] = float(y_boat)
        y_new[-1] = float(y_rov)
        y_new = np.clip(y_new, None, 0.0)
        return x_new, y_new

    def _assess_buoyant_current_geometry(
        self, x, y, L, x_boat, y_boat, x_rov, y_rov
    ) -> tuple[bool, list[str]]:
        """
        Validation géométrie courant flottant:
        - longueur et bornes x,
        - anti-surprofondeur (pas de point sensiblement sous le ROV),
        - cohérence signe déport moyen par bandes de courant.
        """
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        reasons: list[str] = []
        if len(x) < 2 or len(y) != len(x):
            return False, ["taille_invalide"]
        ds = np.hypot(np.diff(x), np.diff(y))
        L_seg = float(np.sum(ds))
        rel = abs(L_seg - float(L)) / max(float(L), 1e-9)
        if rel > 0.05:
            reasons.append(f"L_rel={rel:.4f}")
        slack_x = max(5.0, 0.25 * float(L))
        x_lo = min(float(x_boat), float(x_rov)) - slack_x
        x_hi = max(float(x_boat), float(x_rov)) + slack_x
        if float(np.min(x)) < x_lo - 1e-6 or float(np.max(x)) > x_hi + 1e-6:
            reasons.append("bornes_x")
        if float(np.min(y)) < float(y_rov) - BUOYANT_MIN_Y_MARGIN_M - 1e-6:
            reasons.append("sous_ROV")
        if len(ds) > 0:
            ds_mean = float(np.mean(ds))
            if ds_mean > 1e-9 and float(np.max(ds) / ds_mean) > 3.5:
                reasons.append("segments_irreguliers")

        # Cohérence locale signe courant vs déport par bandes de profondeur
        denom_y = float(y_rov - y_boat)
        if abs(denom_y) > 1e-9:
            t = (y - float(y_boat)) / denom_y
            x_line = float(x_boat) + t * (float(x_rov) - float(x_boat))
            x_off = x - x_line
            weights = self._build_current_signed_weights(y)
            speed = np.abs(weights)
            m = speed > 0.2
            if np.any(m):
                mismatch = float(np.mean(np.sign(x_off[m]) != np.sign(weights[m])))
                if mismatch > BUOYANT_CURVATURE_MISMATCH_MAX:
                    reasons.append(f"signe_courbure={mismatch:.2f}")

        return len(reasons) == 0, reasons

    def _skew_buoyant_polyline_for_static_current(
        self,
        x_cable,
        y_cable,
        L,
        x_boat,
        y_boat,
        x_rov,
        y_rov,
        k_max=2.0,
        n_pass=12,
        step_frac=0.028,
        y_margin_m=BUOYANT_MIN_Y_MARGIN_M,
    ):
        """
        Déformation statique d'un câble flottant sous courant (heuristique multi-couches).
        - Les déplacements suivent la traînée locale (forces) ET le signe du courant par profondeur.
        - Le déplacement horizontal suit ``compute_cable_forces`` (vitesse fluide via ``CURRENT_TO_FLUID_VX_SIGN``).
        - Projection locale pour éviter une plongée artificielle sous le ROV.
        """
        from .forces import compute_cable_forces

        params_cable = {
            "d": self.d,
            "rho_cable": self.rho_cable,
            "Cx_cable": self.Cx_cable,
            "Cf_cable": self.Cf_cable,
        }
        x = np.asarray(x_cable, dtype=float).copy()
        y = np.asarray(y_cable, dtype=float).copy()
        n = len(x)
        if n < 3:
            return x, y
        vx = np.zeros(n)
        vy = np.zeros(n)
        horiz_span = max(abs(float(x_rov - x_boat)), 1.0)
        dx_cap = step_frac * horiz_span

        for p in range(int(n_pass)):
            Fx, _, *_ = compute_cable_forces(
                x, y, vx, vy, self.environment, params_cable, float(L)
            )
            fx_n = np.zeros(n)
            fx_n[0] = 0.5 * Fx[0]
            fx_n[-1] = 0.5 * Fx[-1]
            for i in range(1, n - 1):
                fx_n[i] = 0.5 * (Fx[i - 1] + Fx[i])
            scale = float(np.max(np.abs(fx_n)))
            if scale <= 1e-14:
                break

            # Combiner "forces calculées" et "profil courant par couche".
            # Le terme "target_offset" guide la forme vers une signature
            # multi-couches (profils en S lorsque v(y) change de signe).
            # Le terme "signed_restore" renforce localement la cohérence de signe
            # (notamment sur les couches hautes) contre l'effet global des couches profondes.
            w_force = fx_n / scale
            w_depth = self._build_current_signed_weights(y)
            y_arr = np.asarray(y, dtype=float)
            denom_y = float(y_rov - y_boat)
            if abs(denom_y) > 1e-9:
                t = (y_arr - float(y_boat)) / denom_y
                x_ref = float(x_boat) + t * (float(x_rov) - float(x_boat))
            else:
                x_ref = np.full_like(y_arr, float(x_boat))
            x_off = np.asarray(x, dtype=float) - x_ref
            v_raw = getattr(self.environment, "v_courant_raw", None)
            v_node = np.asarray(
                [self.environment.get_current_velocity(float(yy), v_raw) for yy in y_arr],
                dtype=float,
            )
            target_off = np.cumsum(v_node)
            if len(target_off) > 1:
                target_off = target_off - np.linspace(target_off[0], target_off[-1], len(target_off))
            max_target = float(np.max(np.abs(target_off))) if len(target_off) > 0 else 0.0
            if max_target > 1e-9:
                target_off = target_off / max_target
            offset_err = target_off - np.tanh(x_off / max(horiz_span, 1e-6))
            depth = np.where(y_arr < 0.0, -y_arr, y_arr)
            depth_max = max(float(np.max(depth)), 1e-6)
            depth_norm = depth / depth_max
            upper_gain = 1.0 - depth_norm
            signed_restore = w_depth - np.tanh(x_off / max(0.22 * horiz_span, 1e-6))

            delta = dx_cap * (
                0.38 * w_force
                + 0.20 * w_depth
                + 0.27 * offset_err
                + 0.35 * upper_gain * signed_restore
            )
            if len(delta) > 4:
                delta[1:-1] = 0.25 * delta[:-2] + 0.5 * delta[1:-1] + 0.25 * delta[2:]
            delta[0] = 0.0
            delta[-1] = 0.0
            x[1:-1] = x[1:-1] + delta[1:-1]
            x[0], y[0] = float(x_boat), float(y_boat)
            x[-1], y[-1] = float(x_rov), float(y_rov)
            x, y = self._project_cable_bounds(
                x, y, x_boat, y_boat, x_rov, y_rov, y_margin_m
            )
            try:
                x, y, _, _ = self._normalize_cable_length(
                    x,
                    y,
                    float(L),
                    x_boat=float(x_boat),
                    y_boat=float(y_boat),
                    x_rov=float(x_rov),
                    y_rov=float(y_rov),
                    k_max=k_max,
                )
                x, y = self._project_cable_bounds(
                    x, y, x_boat, y_boat, x_rov, y_rov, y_margin_m
                )
                # Régulariser la forme locale pour éviter les inversions rapides de courbure.
                x, y = self._smooth_polyline_shape(
                    x,
                    y,
                    x_boat,
                    y_boat,
                    x_rov,
                    y_rov,
                    L,
                    y_margin_m,
                    passes=1,
                )
            except Exception:
                return np.asarray(x_cable, dtype=float), np.asarray(y_cable, dtype=float)
            if p in (0, int(max(1, n_pass // 2)), int(max(1, n_pass - 1))):
                trace_print(
                    7,
                    "[DEBUG] _skew_buoyant: "
                    f"pass={p+1}/{int(n_pass)}, Δxmax={float(np.max(np.abs(delta))):.4g}, "
                    f"Fx_sign={float(np.sign(np.mean(fx_n[1:-1]))):+.0f}",
                )
        # Finition: lissage supplémentaire léger avant sortie.
        x, y = self._smooth_polyline_shape(
            x, y, x_boat, y_boat, x_rov, y_rov, L, y_margin_m, passes=2
        )
        # Cas courant quasi uniforme : imposer une concavité globale unique
        # pour éviter les inversions locales de courbure.
        x, y = self._enforce_uniform_current_concavity(
            x, y, x_boat, y_boat, x_rov, y_rov, strength=0.72
        )
        x, y = self._project_cable_bounds(
            x, y, x_boat, y_boat, x_rov, y_rov, y_margin_m
        )
        try:
            x, y, _, _ = self._normalize_cable_length(
                x,
                y,
                float(L),
                x_boat=float(x_boat),
                y_boat=float(y_boat),
                x_rov=float(x_rov),
                y_rov=float(y_rov),
                k_max=k_max,
            )
        except Exception:
            return np.asarray(x_cable, dtype=float), np.asarray(y_cable, dtype=float)
        return x, y

    def solve_equilibrium_static(self, x_rov, y_rov, x_boat, L, rov_m=None, rov_vol=None):
        """
        Résout l'équilibre statique du câble (caténaire)
        
        Utilise maintenant le solveur avec contraintes intégrées pour garantir :
        - L = L_seg à 0.01% près
        - P0 = position bateau
        - PN = position ROV
        - y <= 0 pour tous les points
        - ds_max / ds_target <= k_max
        
        Parameters:
        -----------
        x_rov : float
            Position horizontale du ROV (m)
        y_rov : float
            Position verticale du ROV (m) (négative, profondeur : y < 0 = sous la surface)
        x_boat : float
            Position horizontale du bateau (m)
        L : float
            Longueur du câble (m)
        rov_m : float, optional
            Masse du ROV (kg) - nécessaire pour calculer la tension initiale
        rov_vol : float, optional
            Volume du ROV (m³) - nécessaire pour calculer la tension initiale
        
        Returns:
        --------
        tuple (x_cable, y_cable, T)
            Positions et tensions du câble
            x_cable, y_cable : arrays de taille (N+1)
            T : array de tensions de taille (N+1)
        """

        trace_print(1, "\n[DEBUG] Initialisation du câble : pas de prise en compte du courant (solve_equilibrium_static).")
        
        if L <= 0:
            # Câble de longueur nulle
            return np.array([x_rov, x_boat]), np.array([y_rov, 0.0]), np.array([0.0, 0.0])

        weight_per_unit = (self.rho_cable - self.environment.rho_eau) * self.A_cable * self.environment.g
        if weight_per_unit < 0.0:
            trace_print(
                5,
                "[DEBUG] Câble flottant (w<0) : pas d'optimisation contrainte, chaînette pendante.",
            )
            return self._pure_catenary_equilibrium_buoyant_cable(
                x_rov, y_rov, x_boat, L, rov_m=rov_m, rov_vol=rov_vol
            )
        
        # Préparer les paramètres pour le wrapper de forces
        params_forces = {
            'environment': self.environment,
            'params_cable': {
                'd': self.d,
                'rho_cable': self.rho_cable,
                'Cx_cable': self.Cx_cable,
                'Cf_cable': self.Cf_cable
            },
            'L': L
        }
        
        # Ne PAS passer d'initial_guess pour permettre à solve_equilibrium_constrained
        # de générer automatiquement une caténaire réaliste avec _solve_catenary
        # au lieu d'utiliser une ligne droite
        initial_guess = None
        
        # Utiliser le solveur avec contraintes intégrées
        x_cable, y_cable, T = self.solve_equilibrium_constrained(
            x_rov, y_rov, x_boat, L,
            self._forces_static_wrapper,
            params_forces,
            rov_m=rov_m,
            rov_vol=rov_vol,
            initial_guess=initial_guess,
            k_max=2.0
        )
        
        # Recalculer les tensions correctement en utilisant l'équilibre des forces
        # (le solveur avec contraintes retourne une estimation simple)
        T = self._compute_catenary_tensions(x_cable, y_cable, weight_per_unit, rov_m=rov_m, rov_vol=rov_vol)
        
        return x_cable, y_cable, T
    
    def solve_equilibrium_static_with_current(self, x_rov, y_rov, x_boat, L, rov_m=None, rov_vol=None,
                                              max_iter=50, tol=1e-3, relax=0.2,
                                              use_full_equilibrium=True):
        """
        Résout l'équilibre statique du câble en tenant compte de la traînée du courant sur la géométrie.
        
        Utilise maintenant le solveur avec contraintes intégrées pour garantir :
        - L = L_seg à 0.01% près
        - P0 = position bateau
        - PN = position ROV
        - y <= 0 pour tous les points
        - ds_max / ds_target <= k_max
        """
        trace_print(1, "\n[DEBUG] Initialisation du câble : prise en compte du courant (solve_equilibrium_static_with_current).")
        
        if L <= 0:
            return np.array([x_rov, x_boat]), np.array([y_rov, 0.0]), np.array([0.0, 0.0])

        weight_per_unit = (self.rho_cable - self.environment.rho_eau) * self.A_cable * self.environment.g
        v_max = self.environment.max_abs_current_on_vertical_segment(0.0, float(y_rov))
        if weight_per_unit < 0.0:
            if v_max <= CURRENT_NEGLIGIBLE_M_S:
                trace_print(
                    5,
                    "[DEBUG] Câble flottant (w<0), courant négligeable sur la colonne : "
                    "chaînette pendante (comportement validé, inchangé).",
                )
                return self._pure_catenary_equilibrium_buoyant_cable(
                    x_rov, y_rov, x_boat, L, rov_m=rov_m, rov_vol=rov_vol
                )
            if self.N > MAX_N_SEGMENTS_BUOYANT_STATIC_CURRENT_SKEW:
                trace_print(
                    5,
                    f"[DEBUG] Câble flottant + courant : N={self.N} > "
                    f"{MAX_N_SEGMENTS_BUOYANT_STATIC_CURRENT_SKEW}, chaînette pendante "
                    f"sans déformation courant (limite de coût).",
                )
                return self._pure_catenary_equilibrium_buoyant_cable(
                    x_rov, y_rov, x_boat, L, rov_m=rov_m, rov_vol=rov_vol
                )
            trace_print(
                5,
                f"[DEBUG] Câble flottant (w<0) + courant (|v|_max≈{v_max:.4g} m/s) : "
                f"déformation légère (traînée) depuis la chaînette validée.",
            )
            x0, y0, _ = self._pure_catenary_equilibrium_buoyant_cable(
                x_rov, y_rov, x_boat, L, rov_m=rov_m, rov_vol=rov_vol
            )
            x0 = np.asarray(x0, dtype=float)
            y0 = np.asarray(y0, dtype=float)

            # Cas courant quasi uniforme : construire une géométrie lisse à concavité unique
            # avant de tenter des solveurs plus lourds (évite les oscillations locales).
            y_probe = np.linspace(0.0, float(y_rov), max(int(self.N) + 1, 5))
            is_uniform, side_sign = self._detect_uniform_current_signature(y_probe)
            if is_uniform and abs(side_sign) > 1e-12:
                h_mag_geo = max(0.2, 0.22 * abs(float(x_rov - x_boat)))
                h_signed = float(np.sign(side_sign)) * h_mag_geo
                try:
                    x_u, y_u = _vertical_slack_quadratic_bezier(
                        float(x_boat),
                        float(x_rov),
                        float(y_rov),
                        float(L),
                        h_mag_geo,
                        h_signed,
                        int(self.N),
                    )
                    x_u, y_u = self._project_buoyant_bounds(
                        np.asarray(x_u, dtype=float),
                        np.asarray(y_u, dtype=float),
                        float(x_boat),
                        0.0,
                        float(x_rov),
                        float(y_rov),
                    )
                    geom_ok, _reasons_u = self._assess_buoyant_current_geometry(
                        x_u, y_u, float(L), float(x_boat), 0.0, float(x_rov), float(y_rov)
                    )
                    if geom_ok:
                        T = self._compute_catenary_tensions(
                            x_u, y_u, weight_per_unit, rov_m=rov_m, rov_vol=rov_vol
                        )
                        trace_print(
                            5,
                            "[DEBUG] Câble flottant + courant uniforme: profil Bézier lisse accepté.",
                        )
                        return x_u, y_u, T
                except Exception:
                    pass

            params_forces_nodal = {
                "environment": self.environment,
                "params_cable": {
                    "d": self.d,
                    "rho_cable": self.rho_cable,
                    "Cx_cable": self.Cx_cable,
                    "Cf_cable": self.Cf_cable,
                },
                "L": L,
            }
            ok_nodal, x_n, y_n = self._try_solve_nodal_static_equilibrium_coarse(
                float(x_rov),
                float(y_rov),
                float(x_boat),
                float(L),
                self._forces_static_with_current_wrapper,
                params_forces_nodal,
                x0,
                y0,
                k_max=2.0,
            )
            if ok_nodal and x_n is not None and y_n is not None:
                x_cable, y_cable = self._project_buoyant_bounds(
                    np.asarray(x_n, dtype=float),
                    np.asarray(y_n, dtype=float),
                    float(x_boat),
                    0.0,
                    float(x_rov),
                    float(y_rov),
                )
                # Courant non uniforme en colonne (et non nul) : le nodal utilise v fluide
                # et inverse le S par rapport au profil mission affiché.
                if self._should_flip_nodal_geometry_vs_mission_plot(float(y_rov)):
                    x_cable, y_cable = self._flip_signed_perp_across_chord(
                        x_cable, y_cable, float(x_boat), 0.0, float(x_rov), float(y_rov)
                    )
                    x_cable, y_cable = self._project_buoyant_bounds(
                        x_cable, y_cable, float(x_boat), 0.0, float(x_rov), float(y_rov)
                    )
                # Même en succès nodal, forcer une concavité globale propre quand
                # le courant est quasi uniforme (évite les zigzags locaux).
                x_cable, y_cable = self._enforce_uniform_current_concavity(
                    x_cable, y_cable, float(x_boat), 0.0, float(x_rov), float(y_rov), strength=0.78
                )
                x_cable, y_cable = self._project_buoyant_bounds(
                    x_cable, y_cable, float(x_boat), 0.0, float(x_rov), float(y_rov)
                )
                try:
                    x_cable, y_cable, _, _ = self._normalize_cable_length(
                        x_cable,
                        y_cable,
                        float(L),
                        x_boat=float(x_boat),
                        y_boat=0.0,
                        x_rov=float(x_rov),
                        y_rov=float(y_rov),
                        k_max=2.0,
                    )
                except Exception:
                    pass
                T = self._compute_catenary_tensions(
                    x_cable, y_cable, weight_per_unit, rov_m=rov_m, rov_vol=rov_vol
                )
                trace_print(
                    5,
                    "[DEBUG] Câble flottant + courant : équilibre nodal (maillage grossier) accepté.",
                )
                return x_cable, y_cable, T

            x1, y1 = self._skew_buoyant_polyline_for_static_current(
                x0, y0, float(L), float(x_boat), 0.0, float(x_rov), float(y_rov), k_max=2.0
            )
            x_cable, y_cable = x0.copy(), y0.copy()
            ok = (
                np.all(np.isfinite(x1))
                and np.all(np.isfinite(y1))
                and np.all(np.asarray(y1, dtype=float) <= 1e-5)
            )
            if ok:
                x1, y1 = self._project_buoyant_bounds(
                    x1, y1, float(x_boat), 0.0, float(x_rov), float(y_rov)
                )
                geom_ok, reasons = self._assess_buoyant_current_geometry(
                    x1, y1, float(L), float(x_boat), 0.0, float(x_rov), float(y_rov)
                )
                if geom_ok:
                    x_cable, y_cable = x1, y1
                else:
                    trace_print(
                        5,
                        f"[DEBUG] Déformation courant rejetée ({', '.join(reasons)}), "
                        f"chaînette pure conservée.",
                    )
            else:
                trace_print(
                    5,
                    "[DEBUG] Déformation courant invalide (y ou non fini), chaînette pure.",
                )
            x_cable, y_cable = self._project_buoyant_bounds(
                x_cable, y_cable, float(x_boat), 0.0, float(x_rov), float(y_rov)
            )
            T = self._compute_catenary_tensions(
                x_cable, y_cable, weight_per_unit, rov_m=rov_m, rov_vol=rov_vol
            )
            return x_cable, y_cable, T

        # Câble neutre (ou quasi-neutre) + courant: avec optimisation globale désactivée,
        # le solveur contraint ne déforme presque pas la géométrie. Appliquer le même skew
        # itératif qu'en flottant, mais sans autoriser de point sous le ROV.
        if abs(float(weight_per_unit)) <= 1e-12 and v_max > CURRENT_NEGLIGIBLE_M_S:
            if self.N > MAX_N_SEGMENTS_BUOYANT_STATIC_CURRENT_SKEW:
                x_cable, y_cable = self._solve_catenary(x_rov, y_rov, x_boat, L, weight_per_unit)
                x_cable, y_cable = self._project_cable_bounds(
                    x_cable, y_cable, float(x_boat), 0.0, float(x_rov), float(y_rov), 0.0
                )
                T = self._compute_catenary_tensions(
                    x_cable, y_cable, weight_per_unit, rov_m=rov_m, rov_vol=rov_vol
                )
                return x_cable, y_cable, T

            # Courant quasi uniforme : même construction lisse qu'en flottant (M_test_init_courant_0),
            # sans passer par le nodal grossier (sinon double arc / rebroussement pour un câble neutre).
            y_probe_u = np.linspace(0.0, float(y_rov), max(int(self.N) + 1, 5))
            is_uniform_n, side_sign_n = self._detect_uniform_current_signature(y_probe_u)
            if is_uniform_n and abs(side_sign_n) > 1e-12:
                h_mag_geo_n = max(0.2, 0.22 * abs(float(x_rov - x_boat)))
                # ``_detect_uniform_current_signature`` renvoie le signe en vitesse **fluide** ;
                # pour ce Bézier, le même signe qu’en flottant pousse la bosse du mauvais côté
                # (x négatif) lorsque ``_assess_buoyant_current_geometry`` rejette d’ailleurs la forme.
                # Inverser le signe ici pour coller au déport attendu (cf. M_test_init_courant_0).
                h_signed_n = -float(np.sign(side_sign_n)) * h_mag_geo_n
                try:
                    x_u_n, y_u_n = _vertical_slack_quadratic_bezier(
                        float(x_boat),
                        float(x_rov),
                        float(y_rov),
                        float(L),
                        h_mag_geo_n,
                        h_signed_n,
                        int(self.N),
                    )
                    x_u_n = np.asarray(x_u_n, dtype=float)
                    y_u_n = np.asarray(y_u_n, dtype=float)
                    x_u_n, y_u_n = self._project_cable_bounds(
                        x_u_n,
                        y_u_n,
                        float(x_boat),
                        0.0,
                        float(x_rov),
                        float(y_rov),
                        0.0,
                    )
                    if (
                        np.all(np.isfinite(x_u_n))
                        and np.all(np.isfinite(y_u_n))
                        and np.all(np.asarray(y_u_n, dtype=float) <= 1e-5)
                    ):
                        try:
                            x_u_n, y_u_n, _, _ = self._normalize_cable_length(
                                x_u_n,
                                y_u_n,
                                float(L),
                                x_boat=float(x_boat),
                                y_boat=0.0,
                                x_rov=float(x_rov),
                                y_rov=float(y_rov),
                                k_max=2.0,
                            )
                        except Exception:
                            pass
                        y_u_n = np.clip(y_u_n, None, 0.0)
                        y_u_n[0] = 0.0
                        y_u_n[-1] = float(y_rov)
                        x_u_n[0] = float(x_boat)
                        x_u_n[-1] = float(x_rov)
                        T = self._compute_catenary_tensions(
                            x_u_n,
                            y_u_n,
                            weight_per_unit,
                            rov_m=rov_m,
                            rov_vol=rov_vol,
                        )
                        trace_print(
                            5,
                            "[DEBUG] Câble neutre + courant quasi uniforme : profil Bézier lisse (sans nodal).",
                        )
                        return x_u_n, y_u_n, T
                except Exception:
                    pass

            x0, y0 = self._solve_catenary(x_rov, y_rov, x_boat, L, weight_per_unit)
            x0a = np.asarray(x0, dtype=float)
            y0a = np.asarray(y0, dtype=float)
            params_forces_nodal = {
                "environment": self.environment,
                "params_cable": {
                    "d": self.d,
                    "rho_cable": self.rho_cable,
                    "Cx_cable": self.Cx_cable,
                    "Cf_cable": self.Cf_cable,
                },
                "L": L,
            }
            ok_nodal, x_n, y_n = self._try_solve_nodal_static_equilibrium_coarse(
                float(x_rov),
                float(y_rov),
                float(x_boat),
                float(L),
                self._forces_static_with_current_wrapper,
                params_forces_nodal,
                x0a,
                y0a,
                k_max=2.0,
            )
            if ok_nodal and x_n is not None and y_n is not None:
                x_cable, y_cable = self._project_cable_bounds(
                    np.asarray(x_n, dtype=float),
                    np.asarray(y_n, dtype=float),
                    float(x_boat),
                    0.0,
                    float(x_rov),
                    float(y_rov),
                    0.0,
                )
                if self._should_flip_nodal_geometry_vs_mission_plot(float(y_rov)):
                    x_cable, y_cable = self._flip_signed_perp_across_chord(
                        x_cable, y_cable, float(x_boat), 0.0, float(x_rov), float(y_rov)
                    )
                    x_cable, y_cable = self._project_cable_bounds(
                        x_cable, y_cable, float(x_boat), 0.0, float(x_rov), float(y_rov), 0.0
                    )
                T = self._compute_catenary_tensions(
                    x_cable, y_cable, weight_per_unit, rov_m=rov_m, rov_vol=rov_vol
                )
                trace_print(
                    5,
                    "[DEBUG] Câble neutre + courant : équilibre nodal (maillage grossier) accepté.",
                )
                return x_cable, y_cable, T

            x1, y1 = self._skew_buoyant_polyline_for_static_current(
                x0a,
                y0a,
                float(L),
                float(x_boat),
                0.0,
                float(x_rov),
                float(y_rov),
                k_max=2.0,
                y_margin_m=0.0,
            )
            x1, y1 = self._project_cable_bounds(
                x1, y1, float(x_boat), 0.0, float(x_rov), float(y_rov), 0.0
            )
            x_cable, y_cable = np.asarray(x1, dtype=float), np.asarray(y1, dtype=float)
            T = self._compute_catenary_tensions(
                x_cable, y_cable, weight_per_unit, rov_m=rov_m, rov_vol=rov_vol
            )
            return x_cable, y_cable, T

        # Préparer les paramètres pour le wrapper de forces
        params_forces = {
            'environment': self.environment,
            'params_cable': {
                'd': self.d,
                'rho_cable': self.rho_cable,
                'Cx_cable': self.Cx_cable,
                'Cf_cable': self.Cf_cable
            },
            'L': L
        }
        
        # Ne PAS passer d'initial_guess pour permettre à solve_equilibrium_constrained
        # de générer automatiquement une caténaire réaliste avec _solve_catenary
        # au lieu d'utiliser une ligne droite
        initial_guess = None
        
        # Utiliser le solveur avec contraintes intégrées
        x_cable, y_cable, T = self.solve_equilibrium_constrained(
            x_rov, y_rov, x_boat, L,
            self._forces_static_with_current_wrapper,
            params_forces,
            rov_m=rov_m,
            rov_vol=rov_vol,
            initial_guess=initial_guess,
            k_max=2.0
        )
        
        # Recalculer les tensions correctement en utilisant l'équilibre des forces
        # (le solveur avec contraintes retourne une estimation simple)
        T = self._compute_catenary_tensions(x_cable, y_cable, weight_per_unit, rov_m=rov_m, rov_vol=rov_vol)
        
        return x_cable, y_cable, T
    
    def _solve_catenary(self, x_rov, y_rov, x_boat, L, w):
        """
        Résout l'équation de la caténaire pour trouver la forme du câble
        
        Utilise une méthode itérative pour trouver les paramètres de la caténaire
        qui satisfont les conditions aux limites et la longueur L.
        
        La caténaire suit : y = a * cosh((x - x0) / a) + y0
        Longueur curviligne : s = a * sinh((x - x0) / a)
        
        Parameters:
        -----------
        x_rov, y_rov : float
            Position du ROV (y_rov < 0, profondeur)
        x_boat : float
            Position horizontale du bateau (y_boat = 0)
        L : float
            Longueur totale du câble
        w : float
            Poids apparent par unité de longueur (N/m)
        
        Returns:
        --------
        tuple (x_cable, y_cable)
            Positions du câble discrétisées (du bateau au ROV)
        """
        trace_print(5, "\n[DEBUG] ========== _solve_catenary APPELÉE ==========")
        trace_print(5, f"[DEBUG] _solve_catenary: Paramètres - x_rov={x_rov:.3f}, y_rov={y_rov:.3f}, x_boat={x_boat:.3f}, L={L:.3f}, w={w:.6f}")

        # Convention : y < 0 = profondeur (sous la surface), y = 0 = surface
        # y_rov doit être négatif pour représenter la profondeur
        dx = x_boat - x_rov
        dy = -y_rov  # Profondeur (y_rov est négatif)
        
        # Utiliser la formule standard de la caténaire : y = a * cosh((x - x0) / a) + y0
        # Contraintes :
        # 1. y_boat = 0 : 0 = a * cosh((x_boat - x0) / a) + y0  =>  y0 = -a * cosh((x_boat - x0) / a)
        # 2. y_rov = y_rov (< 0, profondeur) : y_rov = a * cosh((x_rov - x0) / a) + y0
        #    => y_rov = a * (cosh((x_rov - x0) / a) - cosh((x_boat - x0) / a))
        # 3. L = longueur curviligne : L = a * |sinh((x_rov - x0) / a) - sinh((x_boat - x0) / a)|
        
        L_straight = np.sqrt(dx**2 + dy**2)
        
        # Approche : pour chaque valeur de a, résoudre pour x0 en utilisant la contrainte y_rov,
        # puis vérifier la longueur L
        
        def find_x0_for_a(a):
            """Trouve x0 pour un a donné en utilisant la contrainte y_rov"""
            if a <= 0 or not np.isfinite(a) or a < 1e-10:
                return None
            
            def equation_x0(x0):
                """Équation pour trouver x0 : y_rov = a * (cosh((x_rov - x0) / a) - cosh((x_boat - x0) / a))"""
                try:
                    # Vérifier que a est valide
                    if a <= 1e-10 or not np.isfinite(a):
                        return np.inf
                    
                    # Vérifier que x0 est fini
                    if not np.isfinite(x0):
                        return np.inf
                    
                    # Calculer les arguments de cosh
                    arg1 = (x_rov - x0) / a
                    arg2 = (x_boat - x0) / a
                    
                    # Vérifier que les arguments ne sont pas trop grands (éviter overflow)
                    if abs(arg1) > 700 or abs(arg2) > 700:
                        return np.inf
                    
                    # Calculer la valeur
                    val = a * (np.cosh(arg1) - np.cosh(arg2)) - y_rov
                    
                    # Vérifier que le résultat est fini
                    if not np.isfinite(val):
                        return np.inf
                    
                    return val
                except:
                    return np.inf
            
            # x0 devrait être entre x_rov et x_boat, plus proche du ROV
            # Élargir le bracket pour être sûr de trouver la solution
            bracket_width = max(abs(dx) * 2, 10.0)
            bracket = [min(x_rov, x_boat) - bracket_width, max(x_rov, x_boat) + bracket_width]
            
            # Vérifier que l'équation change de signe dans le bracket
            try:
                f_low = equation_x0(bracket[0])
                f_high = equation_x0(bracket[1])
                
                # Si les deux valeurs ont le même signe, essayer d'élargir le bracket
                if np.sign(f_low) == np.sign(f_high) and abs(f_low) > 1e-3 and abs(f_high) > 1e-3:
                    # Essayer plusieurs brackets
                    for scale in [2, 5, 10]:
                        bracket = [min(x_rov, x_boat) - bracket_width * scale, 
                                  max(x_rov, x_boat) + bracket_width * scale]
                        f_low = equation_x0(bracket[0])
                        f_high = equation_x0(bracket[1])
                        if np.sign(f_low) != np.sign(f_high):
                            break
            except:
                pass
            
            try:
                result = root_scalar(equation_x0, bracket=bracket, method='brentq', xtol=1e-8, maxiter=200)
                if result.converged:
                    return result.root
            except:
                pass
            
            # Essayer avec fsolve si root_scalar échoue
            try:
                # Essayer plusieurs estimations initiales
                x0_guesses = [
                    x_rov + 0.2 * (x_boat - x_rov),
                    x_rov + 0.1 * (x_boat - x_rov),
                    x_rov + 0.3 * (x_boat - x_rov),
                    (x_rov + x_boat) / 2,
                    x_rov,
                ]
                
                for x0_guess in x0_guesses:
                    try:
                        result = fsolve(equation_x0, x0_guess, xtol=1e-8, maxfev=500)
                        x0_test = result[0] if isinstance(result, np.ndarray) else result
                        
                        # Vérifier que la solution est bonne
                        error = abs(equation_x0(x0_test))
                        if error < 1e-3:
                            return x0_test
                    except:
                        continue
            except:
                pass
            
            return None
        
        def compute_length_error(a):
            """Calcule l'erreur sur la longueur pour un a donné"""
            if a <= 0 or not np.isfinite(a) or a < 1e-10:
                return np.inf
            
            x0 = find_x0_for_a(a)
            if x0 is None or not np.isfinite(x0):
                return np.inf
            
            try:
                # Calculer les arguments de sinh
                arg_boat = (x_boat - x0) / a
                arg_rov = (x_rov - x0) / a
                
                # Vérifier que les arguments ne sont pas trop grands (éviter overflow)
                if abs(arg_boat) > 700 or abs(arg_rov) > 700:
                    return np.inf
                
                # Calculer les longueurs curvilignes
                s_boat = a * np.sinh(arg_boat)
                s_rov = a * np.sinh(arg_rov)
                
                # Vérifier que les résultats sont finis
                if not (np.isfinite(s_boat) and np.isfinite(s_rov)):
                    return np.inf
                
                L_calc = abs(s_rov - s_boat)
                
                if not np.isfinite(L_calc):
                    return np.inf
                
                return abs(L_calc - L)
            except:
                return np.inf
        
        # Estimation initiale de a
        if L > L_straight * 1.5:
            a_initial = max(0.5, dy / 6.0)
        elif L > L_straight * 1.2:
            a_initial = max(1.0, dy / 4.0)
        else:
            a_initial = max(2.0, dy / 2.0)
        
        # Optimiser pour trouver a qui minimise l'erreur sur la longueur
        a_opt = a_initial
        best_error = np.inf
        
        try:
            # Essayer plusieurs valeurs initiales de a
            a_candidates = [a_initial, a_initial * 0.5, a_initial * 2.0, dy / 6.0, dy / 4.0, dy / 2.0, dy]
            a_candidates = [max(0.1, a) for a in a_candidates]
            
            for a_init in a_candidates:
                try:
                    # Vérifier d'abord que cette valeur initiale donne une erreur finie
                    test_error = compute_length_error(a_init)
                    if not np.isfinite(test_error) or test_error == np.inf:
                        continue
                    
                    # Optimiser a pour minimiser l'erreur sur la longueur
                    # Utiliser un bracket plus restreint autour de a_init
                    bracket_low = max(0.1, a_init * 0.5)
                    bracket_high = min(a_init * 5.0, 1000.0)  # Limiter à une valeur raisonnable
                    
                    # Vérifier que le bracket est valide
                    if bracket_low >= bracket_high:
                        continue
                    
                    # Supprimer les warnings de scipy pour les valeurs invalides (NaN/Inf)
                    # qui peuvent se produire lors de l'optimisation avec des valeurs extrêmes
                    with np.errstate(invalid='ignore', divide='ignore'):
                        result = minimize_scalar(
                            compute_length_error,
                            bracket=(bracket_low, bracket_high),
                            method='brent',
                            options={'xtol': 1e-5, 'maxiter': 200}
                        )
                    
                    if result.success:
                        a_test = result.x
                        if not np.isfinite(a_test) or a_test <= 0:
                            continue
                            
                        error = compute_length_error(a_test)
                        
                        if np.isfinite(error) and error < best_error:
                            best_error = error
                            a_opt = a_test
                            
                            if error < 1e-4:
                                break
                except Exception as e:
                    # Ignorer les erreurs et continuer avec la valeur suivante
                    continue
            
            if best_error > 1e-2:
                trace_print(5, f"⚠️  Avertissement: La caténaire a une erreur élevée ({best_error:.6f}). "
                      f"Utilisation des meilleures valeurs trouvées.")
        except Exception as e:
            trace_print(5, f"⚠️  Erreur lors de la résolution de la caténaire: {e}. Utilisation des valeurs initiales.")
        
        # Trouver x0 pour le a optimal
        x0_opt = find_x0_for_a(a_opt)
        if x0_opt is None:
            trace_print(7, f"[DEBUG] _solve_catenary: ⚠️  ERREUR - Impossible de trouver x0 pour a={a_opt:.3f}. Utilisation d'une interpolation linéaire (LIGNE DROITE).")
            x_cable = np.linspace(x_boat, x_rov, self.N + 1)
            y_cable = np.linspace(0.0, y_rov, self.N + 1)
            return x_cable, y_cable
        
        # Calculer y0
        try:
            y0_opt = -a_opt * np.cosh((x_boat - x0_opt) / a_opt)
        except:
            trace_print(5, f"⚠️  Erreur: Impossible de calculer y0. Utilisation d'une interpolation linéaire.")
            x_cable = np.linspace(x_boat, x_rov, self.N + 1)
            y_cable = np.linspace(0.0, y_rov, self.N + 1)
            return x_cable, y_cable
        
        # Vérifier la solution
        y_rov_calc = a_opt * np.cosh((x_rov - x0_opt) / a_opt) + y0_opt
        y_boat_calc = a_opt * np.cosh((x_boat - x0_opt) / a_opt) + y0_opt
        
        # y_rov devrait être négatif (profondeur : y < 0 = sous la surface)
        if abs(y_rov_calc - y_rov) > 0.1:
            trace_print(5, f"⚠️  Avertissement: y_rov calculé ({y_rov_calc:.3f}) diffère de y_rov attendu ({y_rov:.3f}, profondeur négative)")
        
        if abs(y_boat_calc) > 0.1:
            trace_print(5, f"⚠️  Avertissement: y_boat calculé ({y_boat_calc:.3f}) diffère de y_boat attendu (0.000, surface)")
        
        # Générer les points du câble par longueur curviligne
        # Convention : s = 0 au bateau, s = L au ROV (plus simple et cohérent avec le système)
        # Calculer les longueurs curvilignes aux extrémités dans le système de référence de la caténaire
        try:
            s_boat_ref = a_opt * np.sinh((x_boat - x0_opt) / a_opt)
            s_rov_ref = a_opt * np.sinh((x_rov - x0_opt) / a_opt)
            
            # La longueur curviligne de référence dans le système de la caténaire
            s_total_ref = abs(s_rov_ref - s_boat_ref)
            
            # Déterminer l'ordre : on veut s=0 au bateau et s=L au ROV
            # Donc on mappe s (0 à L) vers s_ref (s_boat_ref à s_rov_ref)
            if s_total_ref > 1e-6:
                # Transformation linéaire : s_ref = s_boat_ref + (s / L) * (s_rov_ref - s_boat_ref)
                # Mais on doit s'assurer que s_boat_ref < s_rov_ref
                if s_rov_ref < s_boat_ref:
                    # Inverser l'ordre
                    s_boat_ref, s_rov_ref = s_rov_ref, s_boat_ref
            else:
                # Si la longueur est nulle, utiliser une approximation
                s_boat_ref = 0.0
                s_rov_ref = L
        except:
            # Si le calcul échoue, utiliser une approximation simple
            s_boat_ref = 0.0
            s_rov_ref = L
        
        # Générer les points à intervalles réguliers de longueur curviligne : s de 0 à L
        s_points = np.linspace(0.0, L, self.N + 1)
        x_cable = np.zeros(self.N + 1)
        y_cable = np.zeros(self.N + 1)
        
        for i, s in enumerate(s_points):
            try:
                # Convertir s (0 à L) en s_ref (référence de la caténaire)
                # Interpolation linéaire : s_ref = s_boat_ref + (s / L) * (s_rov_ref - s_boat_ref)
                if L > 1e-6:
                    s_ref = s_boat_ref + (s / L) * (s_rov_ref - s_boat_ref)
                else:
                    s_ref = s_boat_ref
                
                # Calculer x à partir de s_ref : s_ref = a * sinh((x - x0) / a)
                # donc x = x0 + a * arcsinh(s_ref / a)
                x = x0_opt + a_opt * np.arcsinh(s_ref / a_opt)
                # Utiliser la formule standard : y = a * cosh((x - x0) / a) + y0
                y = a_opt * np.cosh((x - x0_opt) / a_opt) + y0_opt
                
                x_cable[i] = x
                y_cable[i] = y
            except:
                # Si le calcul échoue, interpolation linéaire
                alpha = i / self.N
                x_cable[i] = x_boat + alpha * (x_rov - x_boat)
                y_cable[i] = 0.0 + alpha * (y_rov - 0.0)
        
        # Forcer les extrémités exactement (important pour la continuité)
        # Index 0 = bateau, index -1 = ROV (convention cohérente avec le système)
        x_cable[0] = x_boat
        y_cable[0] = 0.0
        x_cable[-1] = x_rov
        y_cable[-1] = y_rov

        # CONTRAINTE PHYSIQUE CRITIQUE : Tous les points doivent être sous la surface (y <= 0)
        # La caténaire mathématique peut générer des points au-dessus de la surface
        # si les paramètres ne sont pas parfaitement ajustés
        y_cable = np.clip(y_cable, None, 0.0)
        # Réappliquer les extrémités après clipping
        y_cable[0] = 0.0
        y_cable[-1] = y_rov
        
        # DEBUG: Vérifier si des points ont été clippés
        points_above_surface = np.sum((y_cable > 1e-6) & (np.arange(len(y_cable)) != 0))  # Exclure le point bateau
        if points_above_surface > 0:
            trace_print(7, f"[DEBUG] _solve_catenary: ⚠️  {points_above_surface} points étaient au-dessus de la surface et ont été clippés")

        # Câble flottant (w < 0) ou lourd (w > 0) : même géométrie de chaînette « pendante »
        # (courbure vers le bas, sous la corde bateau–ROV, y <= 0). L’ancienne inversion
        # par rapport à la corde pour w < 0 est retirée (affichage / init cohérents missions test).
        
        # Vérifier la longueur
        L_actual = 0.0
        for i in range(self.N):
            dx_seg = x_cable[i+1] - x_cable[i]
            dy_seg = y_cable[i+1] - y_cable[i]
            L_actual += np.sqrt(dx_seg**2 + dy_seg**2)
        
        # Toujours normaliser pour garantir que la longueur est exactement L
        # (tolérance réduite pour forcer la normalisation)
        # IMPORTANT : Passer les positions du bateau et du ROV pour garantir le recollage exact
        if abs(L_actual - L) / L > 1e-6:
            x_cable, y_cable, straight_mode, _ = self._normalize_cable_length(
                x_cable, y_cable, L,
                x_boat=x_boat,
                y_boat=0.0,
                x_rov=x_rov,
                y_rov=y_rov
            )
            x_rov, y_rov = apply_straight_mode_rov_snap(straight_mode, x_rov, y_rov, x_cable, y_cable)
            
            # CONTRAINTE PHYSIQUE : Après normalisation, forcer y <= 0
            y_cable = np.clip(y_cable, None, 0.0)
            # Bateau : recollage explicite. Extrémité ROV : conserver la géométrie retournée
            # (_normalize_cable_segments mode straight peut avoir rapproché le ROV sur la corde).
            if x_boat is not None:
                x_cable[0] = float(x_boat)
            y_cable[0] = 0.0
        
        # Vérification finale de la longueur
        L_final = 0.0
        for i in range(self.N):
            dx_seg = x_cable[i+1] - x_cable[i]
            dy_seg = y_cable[i+1] - y_cable[i]
            L_final += np.sqrt(dx_seg**2 + dy_seg**2)
        
        # Vérifier que la forme est bien une caténaire (pas juste deux segments)
        if len(x_cable) > 2:
            # Calculer les angles entre segments consécutifs
            angles = []
            for i in range(1, len(x_cable) - 1):
                dx1 = x_cable[i] - x_cable[i-1]
                dy1 = y_cable[i] - y_cable[i-1]
                dx2 = x_cable[i+1] - x_cable[i]
                dy2 = y_cable[i+1] - y_cable[i]
                ds1 = np.sqrt(dx1**2 + dy1**2)
                ds2 = np.sqrt(dx2**2 + dy2**2)
                if ds1 > 1e-6 and ds2 > 1e-6:
                    cos_angle = (dx1*dx2 + dy1*dy2) / (ds1 * ds2)
                    cos_angle = np.clip(cos_angle, -1.0, 1.0)
                    angle = np.arccos(cos_angle)
                    angles.append(angle)
            
            # Si tous les angles sont très petits, c'est probablement une ligne droite
            if len(angles) > 0 and np.mean(angles) < 0.01:
                trace_print(5, f"⚠️  Avertissement: Le câble semble être une ligne droite plutôt qu'une caténaire. "
                      f"Vérifiez les paramètres (L={L:.2f} m, L_straight={L_straight:.2f} m, "
                      f"rho_cable={self.rho_cable:.1f} kg/m³, rho_eau={self.environment.rho_eau:.1f} kg/m³)")
        
        # Avertissement si la longueur n'est toujours pas correcte
        if abs(L_final - L) / L > 0.02:
            trace_print(5, f"⚠️  Avertissement: Longueur du câble incorrecte après calcul de caténaire. "
                  f"Attendu: {L:.3f} m, Obtenu: {L_final:.3f} m, Erreur: {abs(L_final - L)/L*100:.1f}%")
        
        # DEBUG: Calculer la déviation maximale de la caténaire générée
        def compute_max_deviation_cat(x_arr, y_arr):
            """Calcule la déviation maximale par rapport à la ligne droite"""
            if len(x_arr) < 3:
                return 0.0
            x_start, y_start = x_arr[0], y_arr[0]
            x_end, y_end = x_arr[-1], y_arr[-1]
            max_dev = 0.0
            if abs(x_end - x_start) > 1e-6 or abs(y_end - y_start) > 1e-6:
                dx_line = x_end - x_start
                dy_line = y_end - y_start
                line_length = np.sqrt(dx_line**2 + dy_line**2)
                for i in range(1, len(x_arr) - 1):
                    dx_point = x_arr[i] - x_start
                    dy_point = y_arr[i] - y_start
                    t = (dx_point * dx_line + dy_point * dy_line) / (line_length**2 + 1e-9)
                    x_proj = x_start + t * dx_line
                    y_proj = y_start + t * dy_line
                    deviation = np.sqrt((x_arr[i] - x_proj)**2 + (y_arr[i] - y_proj)**2)
                    max_dev = max(max_dev, deviation)
            return max_dev
        
        max_dev_cat = compute_max_deviation_cat(x_cable, y_cable)
        D_straight_cat = np.sqrt((x_cable[-1] - x_boat)**2 + (y_cable[-1] - 0.0)**2)
        slack_cat = L_final - D_straight_cat
        if slack_cat > 0 and D_straight_cat > 1e-6:
            theoretical_max_dev = slack_cat * np.sqrt(slack_cat / D_straight_cat) / 2.0
            if max_dev_cat < theoretical_max_dev * 0.1:  # Si la déviation est < 10% de la théorique
                trace_print(7, f"[DEBUG] _solve_catenary: ⚠️  Caténaire générée trop droite - "
                    f"max_deviation={max_dev_cat:.6f} m, théorique≈{theoretical_max_dev:.3f} m, "
                    f"slack={slack_cat:.2f} m, L={L_final:.2f} m, D_straight={D_straight_cat:.2f} m")
        
        # Retourner le câble : index 0 = bateau, index -1 = ROV (convention cohérente avec le système)
        return x_cable, y_cable
        
        # Distance horizontale et verticale entre les extrémités
        dx = x_boat - x_rov
        dy = -y_rov  # Profondeur (y_rov est négatif)
        
        # Longueur rectiligne
        L_straight = np.sqrt(dx**2 + dy**2)

        # Calculer le ratio r = L / L_straight
        r = L / L_straight if L_straight > 1e-9 else 1.0
        
        # SUPPRESSION DE L'HYSTÉRÉSIS BINAIRE : utiliser un blending continu basé uniquement sur r
        # Cela élimine complètement les discontinuités lors du basculement
        # r = 1.0 -> 100% ligne droite (câble tendu)
        # r augmente -> transition progressive vers caténaire
        # r > 1.01 -> 100% caténaire
        
        # Si L < L_straight, câble forcément tendu (impossible physiquement)
        # CORRECTION : Ne pas modifier L, mais forcer une ligne droite avec la longueur L donnée
        # Le ROV a déjà été recalculé dans system_model.py pour que L_straight = L
        if L < L_straight:
            # Câble tendu (rectiligne)
            # Convention : index 0 = bateau, index -1 = ROV (s=0 au bateau, s=L au ROV)
            trace_print(1, "\n[DEBUG] Initialisation du câble : câble tendu (solve_equilibrium_static).")
            trace_print(8, f"[DEBUG] ⚠️  : L < L_straight (L={L:.6f}, L_straight={L_straight:.6f}). "
                "Géométrie rectiligne forcée. Le ROV devrait être recalculé pour que L_straight = L."
            )
            # Ne pas modifier L - créer une ligne droite entre bateau et ROV
            # Après recalcul du ROV, L_straight devrait être proche de L
            # Utiliser une interpolation linéaire simple
            x_cable = np.linspace(x_boat, x_rov, self.N + 1) 
            y_cable = np.linspace(0.0, y_rov, self.N + 1)
            
            # Vérifier et corriger la longueur si nécessaire
            length_check = 0.0
            for i in range(len(x_cable) - 1):
                dx = x_cable[i + 1] - x_cable[i]
                dy = y_cable[i + 1] - y_cable[i]
                length_check += np.sqrt(dx**2 + dy**2)
            
            # Si la longueur ne correspond pas à L, normaliser via l'algorithme centralisé
            # qui impose aussi le recollement exact au bateau et au ROV.
            if abs(length_check - L) / max(L, 1e-9) > 1e-3:
                try:
                    x_cable, y_cable, straight_mode, _ = self._normalize_cable_length(
                        x_cable,
                        y_cable,
                        L,
                        x_boat=float(x_boat),
                        y_boat=0.0,
                        x_rov=float(x_rov),
                        y_rov=float(y_rov),
                    )
                    x_rov, y_rov = apply_straight_mode_rov_snap(straight_mode, x_rov, y_rov, x_cable, y_cable)
                except Exception:
                    # Fallback : interpolation linéaire en abscisse curviligne
                    s_points = np.linspace(0.0, L, self.N + 1)
                    if L_straight > 1e-9:
                        x_cable = x_boat + (x_rov - x_boat) * (s_points / L_straight)
                        y_cable = 0.0 + (y_rov - 0.0) * (s_points / L_straight)
                    else:
                        x_cable = np.linspace(x_boat, x_rov, self.N + 1)
                        y_cable = np.linspace(0.0, y_rov, self.N + 1)
            
            # Calculer les tensions en utilisant l'équilibre des forces
            # T_rov * Urov + T_bateau * Ubateau + Fext_stat = 0
            # où Fext_stat inclut le courant et le poids du câble
            min_floor = max(abs(weight_per_unit) * L / 100.0 if weight_per_unit != 0 else 0.0, 0.5)
            if self.N > 0:
                # Vecteurs unitaires aux extrémités
                dx_bateau = x_cable[1] - x_cable[0]
                dy_bateau = y_cable[1] - y_cable[0]
                ds_bateau = np.sqrt(dx_bateau**2 + dy_bateau**2)
                
                dx_rov = x_cable[-1] - x_cable[-2]
                dy_rov = y_cable[-1] - y_cable[-2]
                ds_rov = np.sqrt(dx_rov**2 + dy_rov**2)
                
                if ds_bateau > 1e-6 and ds_rov > 1e-6:
                    Ubateau_x = dx_bateau / ds_bateau
                    Ubateau_y = dy_bateau / ds_bateau
                    
                    Urov_x = dx_rov / ds_rov
                    Urov_y = dy_rov / ds_rov
                    
                    # Calculer les forces externes statiques sur le câble
                    # Ces forces incluent : le courant (traînée horizontale) et le poids apparent du câble (force verticale)
                    from .forces import compute_cable_forces
                    params_cable = {
                        'd': self.d,
                        'rho_cable': self.rho_cable,
                        'Cx_cable': self.Cx_cable,
                        'Cf_cable': self.Cf_cable
                    }
                    
                    vx_cable = np.zeros(len(x_cable))
                    vy_cable = np.zeros(len(x_cable))
                    
                    Fx_courant_segments, Fy_courant_segments, _, _, _, _ = compute_cable_forces(
                        x_cable, y_cable, vx_cable, vy_cable, self.environment, params_cable, L
                    )
                    
                    # Forces externes statiques totales (courant + poids du câble)
                    Fext_stat_x = np.sum(Fx_courant_segments)
                    Fext_stat_y = np.sum(Fy_courant_segments)
                    
                    # Résoudre le système d'équations
                    # Équations d'équilibre :
                    # -Ubx*T_bateau + Urx*T_rov + Fex = 0  (horizontale)
                    # -Uby*T_bateau + Ury*T_rov + Fey = 0  (verticale)
                    # où :
                    # - Fex = Force exercée par le courant sur le câble (dans le sens de la vitesse apparente du courant)
                    # - Fey = Poids apparent du câble (négatif pour un câble plus dense que l'eau, dirigé vers le bas)
                    # 
                    # Sous forme matricielle : A * [T_bateau, T_rov]^T = [-Fex, -Fey]^T
                    A = np.array([
                        [-Ubateau_x, Urov_x],
                        [-Ubateau_y, Urov_y]
                    ])
                    b = np.array([-Fext_stat_x, -Fext_stat_y])
                    
                    # Debug: Afficher les équations sous forme littérale
                    trace_print(1, "[DEBUG] Résolution du système linéaire pour tensions initiales (câble tendu)")
                    trace_print(1, "[DEBUG] Équations littérales:")
                    trace_print(1, f"  Équation 1 (horizontale): -Ubx*T_bateau + Urx*T_rov + Fex = 0")
                    trace_print(1, f"  Équation 2 (verticale):   -Uby*T_bateau + Ury*T_rov + Fey = 0")
                    trace_print(1, f"[DEBUG] Note: Fex = force du courant, Fey = poids apparent (négatif pour câble plus dense que l'eau)")
                    trace_print(1, f"[DEBUG] Coefficients: Urov_x={Urov_x:.6f}, Urov_y={Urov_y:.6f}, Ubateau_x={Ubateau_x:.6f}, Ubateau_y={Ubateau_y:.6f}")
                    trace_print(1, f"[DEBUG] Forces externes: Fext_stat_x={Fext_stat_x:.6f}, Fext_stat_y={Fext_stat_y:.6f}")
                    det_A = np.linalg.det(A)
                    straight_ratio = L_straight / max(L, 1e-12)
                    trace_print(1, f"[DEBUG] Déterminant de A: {det_A:.6f}")
                    trace_print(1, f"[DEBUG] Ratio L_straight/L: {straight_ratio:.6f}")
                    
                    try:
                        # Cas quasi-rectiligne ou système singulier : résoudre sur l'axe du câble
                        if abs(det_A) < 1e-8 or straight_ratio > 0.995:
                            trace_print(8, "[DEBUG] ⚠️  : câble quasi-rectiligne ou A singulière. "
                                "Projection des forces sur l'axe du câble."
                            )
                            u_dir = np.array([0.5 * (Ubateau_x + Urov_x), 0.5 * (Ubateau_y + Urov_y)])
                            norm_u = np.linalg.norm(u_dir)
                            if norm_u < 1e-12:
                                u_dir = np.array([Urov_x, Urov_y])
                                norm_u = np.linalg.norm(u_dir)
                            if norm_u < 1e-12:
                                u_dir = np.array([0.0, -1.0])
                                norm_u = 1.0
                            u_dir = u_dir / norm_u
                            
                            Fext = np.array([Fext_stat_x, Fext_stat_y])
                            Fext_parallel = float(np.dot(Fext, u_dir))
                            Fext_perp = Fext - Fext_parallel * u_dir
                            
                            if np.linalg.norm(Fext_perp) > 1e-3:
                                trace_print(8, f"[DEBUG] ⚠️  : composante perpendiculaire ignorée (|F⊥|={np.linalg.norm(Fext_perp):.6f}).")
                            
                            delta_T = -Fext_parallel
                            T_base_est = min_floor
                            T_base = max(T_base_est, abs(delta_T) / 2.0)
                            T_bateau = max(0.0, T_base - 0.5 * delta_T)
                            T_rov = max(0.0, T_base + 0.5 * delta_T)
                            trace_print(1, f"[DEBUG] Tensions projetées: T_bateau={T_bateau:.6f}, T_rov={T_rov:.6f}, ΔT={delta_T:.6f}")
                        else:
                            T_solution = np.linalg.solve(A, b)
                            # T_solution[0] = T_bateau, T_solution[1] = T_rov
                            T_bateau = max(0.0, T_solution[0])
                            T_rov = max(0.0, T_solution[1])
                            trace_print(1, f"[DEBUG] Solution brute du système: T_bateau={T_solution[0]:.6f}, T_rov={T_solution[1]:.6f}")
                        
                        # Vérification : les tensions doivent être positives
                        if T_bateau < 1e-6:
                            T_bateau_est = min_floor
                            trace_print(8, f"[DEBUG] ⚠️  : T_bateau est trop petite ({T_bateau:.6f}). "
                                f"Utilisation d'une estimation: {T_bateau_est:.6f} "
                                f"(max(|w|*L/10,10) avec w={weight_per_unit:.6f}, L={L:.6f})."
                            )
                            if self._T_prev is not None and len(self._T_prev) > 0:
                                T_bateau = max(float(self._T_prev[0]), T_bateau_est)
                            else:
                                T_bateau = T_bateau_est
                        
                        if T_rov < 1e-6:
                            T_rov_est = min_floor
                            trace_print(8, f"[DEBUG] ⚠️  : T_rov est trop petite ({T_rov:.6f}). "
                                f"Utilisation d'une estimation: {T_rov_est:.6f} "
                                f"(max(|w|*L/10,10) avec w={weight_per_unit:.6f}, L={L:.6f})."
                            )
                            if self._T_prev is not None and len(self._T_prev) > 0:
                                T_rov = max(float(self._T_prev[-1]), T_rov_est)
                            else:
                                T_rov = T_rov_est
                    except np.linalg.LinAlgError:
                        # Estimation par défaut ou reprise de la tension précédente
                        if self._T_prev is not None and len(self._T_prev) > 0:
                            T_bateau = float(self._T_prev[0])
                            T_rov = float(self._T_prev[-1])
                        else:
                            T_rov = min_floor
                            T_bateau = min_floor
                    
                    # Debug: Afficher les équations numériques avec T_rov et T_bateau remplacés par leurs valeurs
                    trace_print(1, "[DEBUG] Équations numériques:")
                    eq1_left = -Ubateau_x * T_bateau + Urov_x * T_rov + Fext_stat_x
                    eq2_left = -Ubateau_y * T_bateau + Urov_y * T_rov + Fext_stat_y
                    trace_print(1, f"  Équation 1: -{Ubateau_x:.6f}*{T_bateau:.6f} + {Urov_x:.6f}*{T_rov:.6f} + {Fext_stat_x:.6f} = 0")
                    trace_print(1, f"  Équation 2: -{Ubateau_y:.6f}*{T_bateau:.6f} + {Urov_y:.6f}*{T_rov:.6f} + {Fext_stat_y:.6f} = 0")
                    trace_print(1, f"[DEBUG] Résidus (vérification): Équation 1 = {eq1_left:.6f}, Équation 2 = {eq2_left:.6f}")
                    
                    # Debug: Afficher les équations avec les produits remplacés par leurs valeurs
                    T_bateau_Ubateau_x = T_bateau * Ubateau_x
                    T_rov_Urov_x = T_rov * Urov_x
                    T_bateau_Ubateau_y = T_bateau * Ubateau_y
                    T_rov_Urov_y = T_rov * Urov_y
                    trace_print(1, "[DEBUG] Équations avec produits remplacés par leurs valeurs:")
                    trace_print(1, f"  Équation 1: -{T_bateau_Ubateau_x:.6f} + {T_rov_Urov_x:.6f} + {Fext_stat_x:.6f} = 0")
                    trace_print(1, f"  Équation 2: -{T_bateau_Ubateau_y:.6f} + {T_rov_Urov_y:.6f} + {Fext_stat_y:.6f} = 0")
                else:
                    # Estimation par défaut
                    T_rov = min_floor
                    T_bateau = min_floor
            else:
                T_rov = min_floor
                T_bateau = min_floor
            
            # Pour un câble tendu, les tensions varient linéairement entre T_bateau et T_rov
            # T(s) = T_bateau + (T_rov - T_bateau) * (s / L)
            T = np.zeros(self.N + 1)
            T[0] = T_bateau
            if self.N > 0:
                for i in range(1, self.N + 1):
                    s_frac = i / self.N  # Fraction de la longueur
                    T[i] = T_bateau + (T_rov - T_bateau) * s_frac
            else:
                T[0] = (T_bateau + T_rov) / 2.0
            
            self._T_prev = T.copy()
            # Normaliser la géométrie finale pour garantir Σds = L, segments quasi uniformes
            # et extrémités parfaitement recollées sur le bateau et le ROV.
            try:
                x_cable, y_cable, straight_mode, _ = self._normalize_cable_length(
                    x_cable,
                    y_cable,
                    L,
                    x_boat=float(x_boat),
                    y_boat=0.0,
                    x_rov=float(x_rov),
                    y_rov=float(y_rov),
                )
                x_rov, y_rov = apply_straight_mode_rov_snap(straight_mode, x_rov, y_rov, x_cable, y_cable)
            except Exception:
                pass
            return x_cable, y_cable, T
        
        # APPROCHE CONTINUE : toujours calculer caténaire et ligne droite, puis mélanger selon r
        # Cela élimine complètement les discontinuités
        
        # 1. Calculer la caténaire
        trace_print(1, "\n[DEBUG] Initialisation du câble : calcul de la caténaire.")
        x_cable_catenary, y_cable_catenary = self._solve_catenary(x_rov, y_rov, x_boat, L, weight_per_unit)
        
        # 2. Calculer la ligne droite
        x_cable_straight = np.linspace(x_boat, x_rov, self.N + 1)
        y_cable_straight = np.linspace(0.0, y_rov, self.N + 1)
        
        # 3. Calculer le facteur de blending continu basé sur r
        # r = 1.0 -> alpha = 1.0 (100% straight)
        # r = 1.01 -> alpha = 0.0 (100% caténaire)
        # Transition linéaire entre les deux
        blend_r_min = 1.0
        blend_r_max = 1.01
        if r <= blend_r_min:
            alpha_blend = 1.0  # 100% straight
        elif r >= blend_r_max:
            alpha_blend = 0.0  # 100% caténaire
        else:
            # Transition linéaire : alpha diminue de 1.0 à 0.0 quand r augmente de blend_r_min à blend_r_max
            alpha_blend = 1.0 - (r - blend_r_min) / (blend_r_max - blend_r_min)
        
        trace_print(1, f"[DEBUG] Blending continu: r={r:.6f}, alpha={alpha_blend:.4f} (1.0=straight, 0.0=caténaire)")
        
        # 4. Mélanger caténaire et ligne droite
        x_cable = (1.0 - alpha_blend) * x_cable_catenary + alpha_blend * x_cable_straight
        y_cable = (1.0 - alpha_blend) * y_cable_catenary + alpha_blend * y_cable_straight
        
        # 5. Normaliser la longueur pour respecter L exactement
        def _cable_length(x_vals, y_vals):
            dx = np.diff(x_vals)
            dy = np.diff(y_vals)
            return float(np.sum(np.hypot(dx, dy)))
        
        L_actual = _cable_length(x_cable, y_cable)
        if L_actual > 1e-6 and abs(L_actual - L) > 1e-6:
            # Ajuster par interpolation pour respecter la longueur
            scale = L / L_actual
            x_center = x_cable[0]
            y_center = y_cable[0]
            x_cable = x_center + (x_cable - x_center) * scale
            y_cable = y_center + (y_cable - y_center) * scale
            # Réimposer les extrémités exactement
            x_cable[0] = x_boat
            y_cable[0] = 0.0
            x_cable[-1] = x_rov
            y_cable[-1] = y_rov
            # CONTRAINTE PHYSIQUE : Le câble ne peut pas être au-dessus de la surface (y > 0)
            y_cable = np.clip(y_cable, None, 0.0)
            y_cable[0] = 0.0
            y_cable[-1] = y_rov
        
        # 6. Calculer les tensions selon le mode dominant
        # Si alpha_blend est proche de 1.0 (straight), utiliser la méthode straight
        # Sinon, utiliser la méthode caténaire
        if alpha_blend > 0.95:  # Presque straight
            # Utiliser la méthode de calcul des tensions pour câble tendu
            min_floor = max(abs(weight_per_unit) * L / 100.0 if weight_per_unit != 0 else 0.0, 0.5)
            if self.N > 0:
                dx_bateau = x_cable[1] - x_cable[0]
                dy_bateau = y_cable[1] - y_cable[0]
                ds_bateau = np.sqrt(dx_bateau**2 + dy_bateau**2)
                dx_rov = x_cable[-1] - x_cable[-2]
                dy_rov = y_cable[-1] - y_cable[-2]
                ds_rov = np.sqrt(dx_rov**2 + dy_rov**2)
                
                if ds_bateau > 1e-6 and ds_rov > 1e-6:
                    Ubateau_x = dx_bateau / ds_bateau
                    Ubateau_y = dy_bateau / ds_bateau
                    Urov_x = dx_rov / ds_rov
                    Urov_y = dy_rov / ds_rov
                    
                    from .forces import compute_cable_forces
                    params_cable = {
                        'd': self.d,
                        'rho_cable': self.rho_cable,
                        'Cx_cable': self.Cx_cable,
                        'Cf_cable': self.Cf_cable
                    }
                    vx_cable = np.zeros(len(x_cable))
                    vy_cable = np.zeros(len(x_cable))
                    Fx_segments, Fy_segments, _, _, _, _ = compute_cable_forces(
                        x_cable, y_cable, vx_cable, vy_cable, self.environment, params_cable, L
                    )
                    Fext_stat_x = np.sum(Fx_segments)
                    Fext_stat_y = np.sum(Fy_segments)
                    
                    A = np.array([
                        [-Ubateau_x, Urov_x],
                        [-Ubateau_y, Urov_y]
                    ])
                    b = np.array([-Fext_stat_x, -Fext_stat_y])
                    
                    try:
                        det_A = np.linalg.det(A)
                        if abs(det_A) > 1e-8:
                            T_solution = np.linalg.solve(A, b)
                            T_bateau = max(0.0, T_solution[0])
                            T_rov = max(0.0, T_solution[1])
                        else:
                            # Système singulier, utiliser projection
                            u_dir = np.array([0.5 * (Ubateau_x + Urov_x), 0.5 * (Ubateau_y + Urov_y)])
                            norm_u = np.linalg.norm(u_dir)
                            if norm_u < 1e-12:
                                u_dir = np.array([Urov_x, Urov_y])
                                norm_u = np.linalg.norm(u_dir)
                            if norm_u < 1e-12:
                                u_dir = np.array([0.0, -1.0])
                                norm_u = 1.0
                            u_dir = u_dir / norm_u
                            Fext = np.array([Fext_stat_x, Fext_stat_y])
                            Fext_parallel = float(np.dot(Fext, u_dir))
                            delta_T = -Fext_parallel
                            T_base = max(min_floor, abs(delta_T) / 2.0)
                            T_bateau = max(0.0, T_base - 0.5 * delta_T)
                            T_rov = max(0.0, T_base + 0.5 * delta_T)
                        
                        # Interpoler linéairement
                        T = np.zeros(self.N + 1)
                        T[0] = T_bateau
                        for i in range(1, self.N + 1):
                            s_frac = i / self.N
                            T[i] = T_bateau + (T_rov - T_bateau) * s_frac
                    except Exception:
                        # Fallback : utiliser la méthode caténaire
                        T = self._compute_catenary_tensions(x_cable, y_cable, weight_per_unit, rov_m, rov_vol)
                else:
                    T = self._compute_catenary_tensions(x_cable, y_cable, weight_per_unit, rov_m, rov_vol)
            else:
                T = np.full(self.N + 1, min_floor)
        else:
            # Utiliser la méthode caténaire standard
            T = self._compute_catenary_tensions(x_cable, y_cable, weight_per_unit, rov_m, rov_vol)
        
        # Normaliser la géométrie finale pour garantir Σds = L et segments égaux
        try:
            x_cable, y_cable, straight_mode, _ = self._normalize_cable_length(x_cable, y_cable, L)
            x_rov, y_rov = apply_straight_mode_rov_snap(straight_mode, x_rov, y_rov, x_cable, y_cable)
        except Exception:
            pass
        # CONTRAINTE PHYSIQUE : Le câble ne peut pas être au-dessus de la surface (y > 0)
        # S'assurer que tous les points sont sous la surface après normalisation
        y_cable = np.clip(y_cable, None, 0.0)
        y_cable[0] = 0.0
        y_cable[-1] = y_rov
        self._trace_cable_equilibrium_forces(x_cable, y_cable, T, L, label="initialisation (blending continu)")
        self._T_prev = T.copy()
        return x_cable, y_cable, T
    
    def _solve_catenary(self, x_rov, y_rov, x_boat, L, w):
        """
        Résout l'équation de la caténaire pour trouver la forme du câble
        
        Utilise une méthode itérative pour trouver les paramètres de la caténaire
        qui satisfont les conditions aux limites et la longueur L.
        
        La caténaire suit : y = a * cosh((x - x0) / a) + y0
        Longueur curviligne : s = a * sinh((x - x0) / a)
        
        Parameters:
        -----------
        x_rov, y_rov : float
            Position du ROV (y_rov < 0, profondeur)
        x_boat : float
            Position horizontale du bateau (y_boat = 0)
        L : float
            Longueur totale du câble
        w : float
            Poids apparent par unité de longueur (N/m)
        
        Returns:
        --------
        tuple (x_cable, y_cable)
            Positions du câble discrétisées (du bateau au ROV)
        """
        trace_print(5, "\n[DEBUG] ========== _solve_catenary APPELÉE ==========")
        trace_print(5, f"[DEBUG] _solve_catenary: Paramètres - x_rov={x_rov:.3f}, y_rov={y_rov:.3f}, x_boat={x_boat:.3f}, L={L:.3f}, w={w:.6f}")

        # Convention : y < 0 = profondeur (sous la surface), y = 0 = surface
        # y_rov doit être négatif pour représenter la profondeur
        dx = x_boat - x_rov
        dy = -y_rov  # Profondeur (y_rov est négatif)
        
        # Utiliser la formule standard de la caténaire : y = a * cosh((x - x0) / a) + y0
        # Contraintes :
        # 1. y_boat = 0 : 0 = a * cosh((x_boat - x0) / a) + y0  =>  y0 = -a * cosh((x_boat - x0) / a)
        # 2. y_rov = y_rov (< 0, profondeur) : y_rov = a * cosh((x_rov - x0) / a) + y0
        #    => y_rov = a * (cosh((x_rov - x0) / a) - cosh((x_boat - x0) / a))
        # 3. L = longueur curviligne : L = a * |sinh((x_rov - x0) / a) - sinh((x_boat - x0) / a)|
        
        L_straight = np.sqrt(dx**2 + dy**2)
        
        # Approche : pour chaque valeur de a, résoudre pour x0 en utilisant la contrainte y_rov,
        # puis vérifier la longueur L
        
        def find_x0_for_a(a):
            """Trouve x0 pour un a donné en utilisant la contrainte y_rov"""
            if a <= 0 or not np.isfinite(a) or a < 1e-10:
                return None
            
            def equation_x0(x0):
                """Équation pour trouver x0 : y_rov = a * (cosh((x_rov - x0) / a) - cosh((x_boat - x0) / a))"""
                try:
                    # Vérifier que a est valide
                    if a <= 1e-10 or not np.isfinite(a):
                        return np.inf
                    
                    # Vérifier que x0 est fini
                    if not np.isfinite(x0):
                        return np.inf
                    
                    # Calculer les arguments de cosh
                    arg1 = (x_rov - x0) / a
                    arg2 = (x_boat - x0) / a
                    
                    # Vérifier que les arguments ne sont pas trop grands (éviter overflow)
                    if abs(arg1) > 700 or abs(arg2) > 700:
                        return np.inf
                    
                    # Calculer la valeur
                    val = a * (np.cosh(arg1) - np.cosh(arg2)) - y_rov
                    
                    # Vérifier que le résultat est fini
                    if not np.isfinite(val):
                        return np.inf
                    
                    return val
                except:
                    return np.inf
            
            # x0 devrait être entre x_rov et x_boat, plus proche du ROV
            # Élargir le bracket pour être sûr de trouver la solution
            bracket_width = max(abs(dx) * 2, 10.0)
            bracket = [min(x_rov, x_boat) - bracket_width, max(x_rov, x_boat) + bracket_width]
            
            # Vérifier que l'équation change de signe dans le bracket
            try:
                f_low = equation_x0(bracket[0])
                f_high = equation_x0(bracket[1])
                
                # Si les deux valeurs ont le même signe, essayer d'élargir le bracket
                if np.sign(f_low) == np.sign(f_high) and abs(f_low) > 1e-3 and abs(f_high) > 1e-3:
                    # Essayer plusieurs brackets
                    for scale in [2, 5, 10]:
                        bracket = [min(x_rov, x_boat) - bracket_width * scale, 
                                  max(x_rov, x_boat) + bracket_width * scale]
                        f_low = equation_x0(bracket[0])
                        f_high = equation_x0(bracket[1])
                        if np.sign(f_low) != np.sign(f_high):
                            break
            except:
                pass
            
            try:
                result = root_scalar(equation_x0, bracket=bracket, method='brentq', xtol=1e-8, maxiter=200)
                if result.converged:
                    return result.root
            except:
                pass
            
            # Essayer avec fsolve si root_scalar échoue
            try:
                # Essayer plusieurs estimations initiales
                x0_guesses = [
                    x_rov + 0.2 * (x_boat - x_rov),
                    x_rov + 0.1 * (x_boat - x_rov),
                    x_rov + 0.3 * (x_boat - x_rov),
                    (x_rov + x_boat) / 2,
                    x_rov,
                ]
                
                for x0_guess in x0_guesses:
                    try:
                        result = fsolve(equation_x0, x0_guess, xtol=1e-8, maxfev=500)
                        x0_test = result[0] if isinstance(result, np.ndarray) else result
                        
                        # Vérifier que la solution est bonne
                        error = abs(equation_x0(x0_test))
                        if error < 1e-3:
                            return x0_test
                    except:
                        continue
            except:
                pass
            
            return None
        
        def compute_length_error(a):
            """Calcule l'erreur sur la longueur pour un a donné"""
            if a <= 0 or not np.isfinite(a) or a < 1e-10:
                return np.inf
            
            x0 = find_x0_for_a(a)
            if x0 is None or not np.isfinite(x0):
                return np.inf
            
            try:
                # Calculer les arguments de sinh
                arg_boat = (x_boat - x0) / a
                arg_rov = (x_rov - x0) / a
                
                # Vérifier que les arguments ne sont pas trop grands (éviter overflow)
                if abs(arg_boat) > 700 or abs(arg_rov) > 700:
                    return np.inf
                
                # Calculer les longueurs curvilignes
                s_boat = a * np.sinh(arg_boat)
                s_rov = a * np.sinh(arg_rov)
                
                # Vérifier que les résultats sont finis
                if not (np.isfinite(s_boat) and np.isfinite(s_rov)):
                    return np.inf
                
                L_calc = abs(s_rov - s_boat)
                
                if not np.isfinite(L_calc):
                    return np.inf
                
                return abs(L_calc - L)
            except:
                return np.inf
        
        # Estimation initiale de a
        if L > L_straight * 1.5:
            a_initial = max(0.5, dy / 6.0)
        elif L > L_straight * 1.2:
            a_initial = max(1.0, dy / 4.0)
        else:
            a_initial = max(2.0, dy / 2.0)
        
        # Optimiser pour trouver a qui minimise l'erreur sur la longueur
        a_opt = a_initial
        best_error = np.inf
        
        try:
            # Essayer plusieurs valeurs initiales de a
            a_candidates = [a_initial, a_initial * 0.5, a_initial * 2.0, dy / 6.0, dy / 4.0, dy / 2.0, dy]
            a_candidates = [max(0.1, a) for a in a_candidates]
            
            for a_init in a_candidates:
                try:
                    # Vérifier d'abord que cette valeur initiale donne une erreur finie
                    test_error = compute_length_error(a_init)
                    if not np.isfinite(test_error) or test_error == np.inf:
                        continue
                    
                    # Optimiser a pour minimiser l'erreur sur la longueur
                    # Utiliser un bracket plus restreint autour de a_init
                    bracket_low = max(0.1, a_init * 0.5)
                    bracket_high = min(a_init * 5.0, 1000.0)  # Limiter à une valeur raisonnable
                    
                    # Vérifier que le bracket est valide
                    if bracket_low >= bracket_high:
                        continue
                    
                    # Supprimer les warnings de scipy pour les valeurs invalides (NaN/Inf)
                    # qui peuvent se produire lors de l'optimisation avec des valeurs extrêmes
                    with np.errstate(invalid='ignore', divide='ignore'):
                        result = minimize_scalar(
                            compute_length_error,
                            bracket=(bracket_low, bracket_high),
                            method='brent',
                            options={'xtol': 1e-5, 'maxiter': 200}
                        )
                    
                    if result.success:
                        a_test = result.x
                        if not np.isfinite(a_test) or a_test <= 0:
                            continue
                            
                        error = compute_length_error(a_test)
                        
                        if np.isfinite(error) and error < best_error:
                            best_error = error
                            a_opt = a_test
                            
                            if error < 1e-4:
                                break
                except Exception as e:
                    # Ignorer les erreurs et continuer avec la valeur suivante
                    continue
            
            if best_error > 1e-2:
                trace_print(5, f"⚠️  Avertissement: La caténaire a une erreur élevée ({best_error:.6f}). "
                      f"Utilisation des meilleures valeurs trouvées.")
        except Exception as e:
            trace_print(5, f"⚠️  Erreur lors de la résolution de la caténaire: {e}. Utilisation des valeurs initiales.")
        
        # Trouver x0 pour le a optimal
        x0_opt = find_x0_for_a(a_opt)
        if x0_opt is None:
            trace_print(7, f"[DEBUG] _solve_catenary: ⚠️  ERREUR - Impossible de trouver x0 pour a={a_opt:.3f}. Utilisation d'une interpolation linéaire (LIGNE DROITE).")
            x_cable = np.linspace(x_boat, x_rov, self.N + 1)
            y_cable = np.linspace(0.0, y_rov, self.N + 1)
            return x_cable, y_cable
        
        # Calculer y0
        try:
            y0_opt = -a_opt * np.cosh((x_boat - x0_opt) / a_opt)
        except:
            trace_print(5, f"⚠️  Erreur: Impossible de calculer y0. Utilisation d'une interpolation linéaire.")
            x_cable = np.linspace(x_boat, x_rov, self.N + 1)
            y_cable = np.linspace(0.0, y_rov, self.N + 1)
            return x_cable, y_cable
        
        # Vérifier la solution
        y_rov_calc = a_opt * np.cosh((x_rov - x0_opt) / a_opt) + y0_opt
        y_boat_calc = a_opt * np.cosh((x_boat - x0_opt) / a_opt) + y0_opt
        
        # y_rov devrait être négatif (profondeur : y < 0 = sous la surface)
        if abs(y_rov_calc - y_rov) > 0.1:
            trace_print(5, f"⚠️  Avertissement: y_rov calculé ({y_rov_calc:.3f}) diffère de y_rov attendu ({y_rov:.3f}, profondeur négative)")
        
        if abs(y_boat_calc) > 0.1:
            trace_print(5, f"⚠️  Avertissement: y_boat calculé ({y_boat_calc:.3f}) diffère de y_boat attendu (0.000, surface)")
        
        # Générer les points du câble par longueur curviligne
        # Convention : s = 0 au bateau, s = L au ROV (plus simple et cohérent avec le système)
        # Calculer les longueurs curvilignes aux extrémités dans le système de référence de la caténaire
        try:
            s_boat_ref = a_opt * np.sinh((x_boat - x0_opt) / a_opt)
            s_rov_ref = a_opt * np.sinh((x_rov - x0_opt) / a_opt)
            
            # La longueur curviligne de référence dans le système de la caténaire
            s_total_ref = abs(s_rov_ref - s_boat_ref)
            
            # Déterminer l'ordre : on veut s=0 au bateau et s=L au ROV
            # Donc on mappe s (0 à L) vers s_ref (s_boat_ref à s_rov_ref)
            if s_total_ref > 1e-6:
                # Transformation linéaire : s_ref = s_boat_ref + (s / L) * (s_rov_ref - s_boat_ref)
                # Mais on doit s'assurer que s_boat_ref < s_rov_ref
                if s_rov_ref < s_boat_ref:
                    # Inverser l'ordre
                    s_boat_ref, s_rov_ref = s_rov_ref, s_boat_ref
            else:
                # Si la longueur est nulle, utiliser une approximation
                s_boat_ref = 0.0
                s_rov_ref = L
        except:
            # Si le calcul échoue, utiliser une approximation simple
            s_boat_ref = 0.0
            s_rov_ref = L
        
        # Générer les points à intervalles réguliers de longueur curviligne : s de 0 à L
        s_points = np.linspace(0.0, L, self.N + 1)
        x_cable = np.zeros(self.N + 1)
        y_cable = np.zeros(self.N + 1)
        
        for i, s in enumerate(s_points):
            try:
                # Convertir s (0 à L) en s_ref (référence de la caténaire)
                # Interpolation linéaire : s_ref = s_boat_ref + (s / L) * (s_rov_ref - s_boat_ref)
                if L > 1e-6:
                    s_ref = s_boat_ref + (s / L) * (s_rov_ref - s_boat_ref)
                else:
                    s_ref = s_boat_ref
                
                # Calculer x à partir de s_ref : s_ref = a * sinh((x - x0) / a)
                # donc x = x0 + a * arcsinh(s_ref / a)
                x = x0_opt + a_opt * np.arcsinh(s_ref / a_opt)
                # Utiliser la formule standard : y = a * cosh((x - x0) / a) + y0
                y = a_opt * np.cosh((x - x0_opt) / a_opt) + y0_opt
                
                x_cable[i] = x
                y_cable[i] = y
            except:
                # Si le calcul échoue, interpolation linéaire
                alpha = i / self.N
                x_cable[i] = x_boat + alpha * (x_rov - x_boat)
                y_cable[i] = 0.0 + alpha * (y_rov - 0.0)
        
        # Forcer les extrémités exactement (important pour la continuité)
        # Index 0 = bateau, index -1 = ROV (convention cohérente avec le système)
        x_cable[0] = x_boat
        y_cable[0] = 0.0
        x_cable[-1] = x_rov
        y_cable[-1] = y_rov

        # Câble flottant (w < 0) ou lourd : même géométrie de chaînette pendante (voir première définition).
        
        # Vérifier la longueur
        L_actual = 0.0
        for i in range(self.N):
            dx_seg = x_cable[i+1] - x_cable[i]
            dy_seg = y_cable[i+1] - y_cable[i]
            L_actual += np.sqrt(dx_seg**2 + dy_seg**2)
        
        # Toujours normaliser pour garantir que la longueur est exactement L
        # (tolérance réduite pour forcer la normalisation)
        # NOTE: On garde _normalize_cable_length ici car c'est pour l'estimation initiale
        # IMPORTANT : Passer les positions du bateau et du ROV pour garantir le recollage exact
        if abs(L_actual - L) / L > 1e-6:
            x_cable, y_cable, straight_mode, _ = self._normalize_cable_length(
                x_cable, y_cable, L,
                x_boat=x_boat,
                y_boat=0.0,
                x_rov=x_rov,
                y_rov=y_rov
            )
            x_rov, y_rov = apply_straight_mode_rov_snap(straight_mode, x_rov, y_rov, x_cable, y_cable)
            
            # CONTRAINTE PHYSIQUE : Après normalisation, forcer y <= 0
            y_cable = np.clip(y_cable, None, 0.0)
            if x_boat is not None:
                x_cable[0] = float(x_boat)
            y_cable[0] = 0.0
        
        # Vérification finale de la longueur
        L_final = 0.0
        for i in range(self.N):
            dx_seg = x_cable[i+1] - x_cable[i]
            dy_seg = y_cable[i+1] - y_cable[i]
            L_final += np.sqrt(dx_seg**2 + dy_seg**2)
        
        # Vérifier que la forme est bien une caténaire (pas juste deux segments)
        if len(x_cable) > 2:
            # Calculer les angles entre segments consécutifs
            angles = []
            for i in range(1, len(x_cable) - 1):
                dx1 = x_cable[i] - x_cable[i-1]
                dy1 = y_cable[i] - y_cable[i-1]
                dx2 = x_cable[i+1] - x_cable[i]
                dy2 = y_cable[i+1] - y_cable[i]
                ds1 = np.sqrt(dx1**2 + dy1**2)
                ds2 = np.sqrt(dx2**2 + dy2**2)
                if ds1 > 1e-6 and ds2 > 1e-6:
                    cos_angle = (dx1*dx2 + dy1*dy2) / (ds1 * ds2)
                    cos_angle = np.clip(cos_angle, -1.0, 1.0)
                    angle = np.arccos(cos_angle)
                    angles.append(angle)
            
            # Si tous les angles sont très petits, c'est probablement une ligne droite
            if len(angles) > 0 and np.mean(angles) < 0.01:
                trace_print(5, f"⚠️  Avertissement: Le câble semble être une ligne droite plutôt qu'une caténaire. "
                      f"Vérifiez les paramètres (L={L:.2f} m, L_straight={L_straight:.2f} m, "
                      f"rho_cable={self.rho_cable:.1f} kg/m³, rho_eau={self.environment.rho_eau:.1f} kg/m³)")
        
        # Avertissement si la longueur n'est toujours pas correcte
        if abs(L_final - L) / L > 0.02:
            trace_print(5, f"⚠️  Avertissement: Longueur du câble incorrecte après calcul de caténaire. "
                  f"Attendu: {L:.3f} m, Obtenu: {L_final:.3f} m, Erreur: {abs(L_final - L)/L*100:.1f}%")
        
        # Retourner le câble : index 0 = bateau, index -1 = ROV (convention cohérente avec le système)
        return x_cable, y_cable
    
    def _compute_catenary_tensions(self, x_cable, y_cable, weight_per_unit, rov_m=None, rov_vol=None):
        """
        Calcule les tensions le long du câble en caténaire
        
        L'équilibre des forces sur le câble s'écrit :
        T_rov * Urov + T_bateau * Ubateau + Fext_stat = 0
        # où Fext_stat inclut le courant et le poids du câble
        
        Parameters:
        -----------
        x_cable, y_cable : array
            Positions du câble (index 0 = bateau, index -1 = ROV)
        weight_per_unit : float
            Poids apparent par unité de longueur (N/m)
        rov_m, rov_vol : float, optional
            Masse et volume du ROV (non utilisés pour le calcul de tension)
        
        Returns:
        --------
        array
            Tensions le long du câble (T[0] = tension au bateau, T[-1] = tension au ROV)
        """
        trace_print(1, "\n[DEBUG] Calcul des tensions en caténaire : _compute_catenary_tensions.")

        N = len(x_cable) - 1
        T = np.zeros(N + 1)
        
        if N == 0:
            # Un seul point, tension nulle
            return T
        
        # Calculer les vecteurs unitaires aux extrémités
        # Ubateau : direction du câble depuis le bateau (index 0) vers le point suivant
        dx_bateau = x_cable[1] - x_cable[0]
        dy_bateau = y_cable[1] - y_cable[0]
        ds_bateau = np.sqrt(dx_bateau**2 + dy_bateau**2)
        
        # Urov : direction du câble vers le ROV (du point précédent vers le ROV, index -1)
        dx_rov = x_cable[-1] - x_cable[-2]
        dy_rov = y_cable[-1] - y_cable[-2]
        ds_rov = np.sqrt(dx_rov**2 + dy_rov**2)
        
        if ds_bateau < 1e-6 or ds_rov < 1e-6:
            # Câble trop court ou segments trop petits, utiliser estimation simple
            L_total = 0.0
            for i in range(N):
                dx = x_cable[i+1] - x_cable[i]
                dy = y_cable[i+1] - y_cable[i]
                L_total += np.sqrt(dx**2 + dy**2)
            T_est = max(0.0, weight_per_unit * L_total / 2.0)
            T.fill(T_est)
            return T
        
        # Vecteurs unitaires
        Ubateau_x = dx_bateau / ds_bateau
        Ubateau_y = dy_bateau / ds_bateau
        
        Urov_x = dx_rov / ds_rov
        Urov_y = dy_rov / ds_rov
        
        # Calculer les forces externes statiques sur le câble
        # Ces forces incluent : le courant (traînée horizontale) et le poids apparent du câble (force verticale)
        # Pour l'équilibre statique, les vitesses du câble sont nulles
        from .forces import compute_cable_forces
        params_cable = {
            'd': self.d,
            'rho_cable': self.rho_cable,
            'Cx_cable': self.Cx_cable,
            'Cf_cable': self.Cf_cable
        }
        
        # Calculer la longueur totale du câble
        L_total = 0.0
        for i in range(N):
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            L_total += np.sqrt(dx**2 + dy**2)
        
        # Vitesses du câble nulles pour l'équilibre statique
        vx_cable = np.zeros(len(x_cable))
        vy_cable = np.zeros(len(x_cable))
        
        # Calculer les forces externes sur chaque segment (courant + poids du câble)
        Fx_courant_segments, Fy_courant_segments, _, _, _, _ = compute_cable_forces(
            x_cable, y_cable, vx_cable, vy_cable, self.environment, params_cable, L_total
        )
        
        # Forces externes statiques totales (courant + poids du câble)
        Fext_stat_x = np.sum(Fx_courant_segments)
        Fext_stat_y = np.sum(Fy_courant_segments)
        
        # Résoudre le système d'équations d'équilibre des forces
        # Équations d'équilibre :
        # -Ubx*T_bateau + Urx*T_rov + Fex = 0  (horizontale)
        # -Uby*T_bateau + Ury*T_rov + Fey = 0  (verticale)
        # où :
        # - Fex = Force exercée par le courant sur le câble (dans le sens de la vitesse apparente du courant)
        # - Fey = Poids apparent du câble (négatif pour un câble plus dense que l'eau, dirigé vers le bas)
        # 
        # Sous forme matricielle : A * [T_bateau, T_rov]^T = [-Fex, -Fey]^T
        A = np.array([
            [-Ubateau_x, Urov_x],
            [-Ubateau_y, Urov_y]
        ])
        b = np.array([-Fext_stat_x, -Fext_stat_y])
        
        # Résolution robuste du système des deux tractions d'extrémité.
        # Objectif : fermer au mieux le bilan global des forces statiques.
        try:
            det_a = np.linalg.det(A)
            cond_a = np.linalg.cond(A) if np.isfinite(det_a) else np.inf
            if abs(det_a) < 1e-10 or cond_a > 1e10:
                # Moindres carrés lorsque les directions aux extrémités sont quasi-colinéaires.
                T_solution, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            else:
                T_solution = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            T_solution, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

        # Les inconnues du système portent sur les amplitudes le long de directions imposées.
        # En cas de changement de signe (solution algébrique négative), on garde le module
        # pour construire un profil scalaire de tension physiquement exploitable.
        T_bateau = abs(float(T_solution[0]))
        T_rov = abs(float(T_solution[1]))

        # En câble neutre/léger, ne pas imposer de plancher artificiel.
        # Un plancher trop grand casse immédiatement l'équilibre des forces affiché.
        w_abs = abs(float(weight_per_unit))
        min_floor = 0.0 if w_abs <= 1e-12 else w_abs * L_total / 100.0

        if not np.isfinite(T_bateau) or not np.isfinite(T_rov):
            T_bateau = min_floor
            T_rov = min_floor

        # Cas rare : forçage minimal seulement si la résultante externe est significative.
        fext_norm = float(np.hypot(Fext_stat_x, Fext_stat_y))
        if T_bateau < 1e-10 and T_rov < 1e-10 and fext_norm > 1e-6:
            t_fallback = max(0.5 * fext_norm, min_floor)
            T_bateau = t_fallback
            T_rov = t_fallback
        
        # Calculer les tensions le long du câble.
        # Cas neutre/quasi-neutre: pas de loi caténaire pertinente (w≈0) -> interpolation lisse
        # entre les tensions d'extrémité pour éviter les sauts numériques.
        w_abs = abs(float(weight_per_unit))
        if w_abs <= 1e-9:
            for i in range(N + 1):
                t = float(i) / float(max(N, 1))
                T[i] = (1.0 - t) * T_bateau + t * T_rov
            return T

        # Cas général pondéré: profil caténaire approché.
        sin_theta_bateau = Ubateau_x  # sin(θ) = dx/ds où θ est l'angle avec la verticale
        H = T_bateau * abs(sin_theta_bateau) if abs(sin_theta_bateau) > 1e-6 else T_bateau
        if T_bateau < H:
            H = T_bateau
            s_bateau = 0.0
        else:
            s_bateau = np.sqrt(max(0.0, T_bateau**2 - H**2)) / w_abs

        T[0] = T_bateau
        s_cumulative = 0.0
        for i in range(N):
            dx = x_cable[i + 1] - x_cable[i]
            dy = y_cable[i + 1] - y_cable[i]
            ds = np.sqrt(dx**2 + dy**2)
            s_cumulative += ds
            s_total = abs(-s_bateau + s_cumulative)
            T[i + 1] = np.sqrt(H**2 + (w_abs * s_total) ** 2)

        # Ajustement doux de la dernière valeur (pas d'écrasement brutal).
        if abs(T[-1] - T_rov) > 1e-3:
            T[-1] = 0.5 * T[-1] + 0.5 * T_rov
        
        return T
    
    def _solve_catenary(self, x_rov, y_rov, x_boat, L, w):
        """
        Résout l'équation de la caténaire pour trouver la forme du câble
        
        Utilise une méthode itérative pour trouver les paramètres de la caténaire
        qui satisfont les conditions aux limites et la longueur L.
        
        La caténaire suit : y = a * cosh((x - x0) / a) + y0
        Longueur curviligne : s = a * sinh((x - x0) / a)
        
        Parameters:
        -----------
        x_rov, y_rov : float
            Position du ROV (y_rov < 0, profondeur)
        x_boat : float
            Position horizontale du bateau (y_boat = 0)
        L : float
            Longueur totale du câble
        w : float
            Poids apparent par unité de longueur (N/m)
        
        Returns:
        --------
        tuple (x_cable, y_cable)
            Positions du câble discrétisées (du bateau au ROV)
        """
        trace_print(5, "\n[DEBUG] ========== _solve_catenary APPELÉE ==========")
        trace_print(5, f"[DEBUG] _solve_catenary: Paramètres - x_rov={x_rov:.3f}, y_rov={y_rov:.3f}, x_boat={x_boat:.3f}, L={L:.3f}, w={w:.6f}")

        # Convention : y < 0 = profondeur (sous la surface), y = 0 = surface
        # y_rov doit être négatif pour représenter la profondeur
        dx = x_boat - x_rov
        dy = -y_rov  # Profondeur (y_rov est négatif)
        
        # Utiliser la formule standard de la caténaire : y = a * cosh((x - x0) / a) + y0
        # Contraintes :
        # 1. y_boat = 0 : 0 = a * cosh((x_boat - x0) / a) + y0  =>  y0 = -a * cosh((x_boat - x0) / a)
        # 2. y_rov = y_rov (< 0, profondeur) : y_rov = a * cosh((x_rov - x0) / a) + y0
        #    => y_rov = a * (cosh((x_rov - x0) / a) - cosh((x_boat - x0) / a))
        # 3. L = longueur curviligne : L = a * |sinh((x_rov - x0) / a) - sinh((x_boat - x0) / a)|
        
        L_straight = np.sqrt(dx**2 + dy**2)

        # dx = 0 et ROV sous la surface (y_rov ≤ 0) : pas de solution caténaire classique ;
        # polyline en V + orientation courant. Si y_rov > 0 (cas de tests non physiques),
        # garder le chemin caténaire historique.
        if abs(dx) < 1e-9 and float(y_rov) <= 1e-9:
            D = float(dy)
            xb_f = float(x_boat)
            xr_f = float(x_rov)
            yr_f = float(y_rov)
            if D < 1e-12:
                trace_print(
                    7,
                    "[DEBUG] _solve_catenary: dx≈0 et profondeur nulle, interpolation linéaire.",
                )
                x_cable = np.linspace(xb_f, xr_f, self.N + 1)
                y_cable = np.linspace(0.0, yr_f, self.N + 1)
                return x_cable, y_cable
            if float(L) <= float(L_straight) + 1e-9:
                x_cable = np.full(self.N + 1, xb_f)
                y_cable = np.linspace(0.0, yr_f, self.N + 1)
                return x_cable, y_cable
            half_L = 0.5 * float(L)
            h_sq = half_L * half_L - (D / 2.0) ** 2
            if h_sq <= 1e-12:
                trace_print(
                    5,
                    "[DEBUG] _solve_catenary: dx≈0 mais slack incompatible avec D, ligne verticale.",
                )
                x_cable = np.full(self.N + 1, xb_f)
                y_cable = np.linspace(0.0, yr_f, self.N + 1)
                return x_cable, y_cable
            h_mag = float(np.sqrt(h_sq))
            h = h_mag
            try:
                v_raw = getattr(self.environment, "v_courant_raw", None)
                vmid = float(
                    np.asarray(
                        self.environment.get_current_velocity(0.5 * yr_f, v_raw),
                        dtype=float,
                    ).reshape(-1)[0]
                )
                if vmid < -CURRENT_NEGLIGIBLE_M_S:
                    h = -h_mag
                elif vmid > CURRENT_NEGLIGIBLE_M_S:
                    h = h_mag
                else:
                    h = h_mag
            except Exception:
                h = h_mag
            x_cable, y_cable = _vertical_slack_quadratic_bezier(
                xb_f,
                xr_f,
                yr_f,
                float(L),
                h_mag,
                h,
                self.N,
            )
            x_cable[0], y_cable[0] = xb_f, 0.0
            x_cable[-1], y_cable[-1] = xr_f, yr_f
            y_cable = np.clip(y_cable, None, 0.0)
            y_cable[0] = 0.0
            y_cable[-1] = yr_f
            L_actual = 0.0
            for ii in range(self.N):
                dxs = x_cable[ii + 1] - x_cable[ii]
                dys = y_cable[ii + 1] - y_cable[ii]
                L_actual += float(np.sqrt(dxs * dxs + dys * dys))
            if abs(L_actual - float(L)) / float(L) > 1e-6:
                x_cable, y_cable, straight_mode, _ = self._normalize_cable_length(
                    x_cable,
                    y_cable,
                    float(L),
                    x_boat=xb_f,
                    y_boat=0.0,
                    x_rov=xr_f,
                    y_rov=yr_f,
                )
                _ = apply_straight_mode_rov_snap(
                    straight_mode, xr_f, yr_f, x_cable, y_cable
                )
                y_cable = np.clip(y_cable, None, 0.0)
                y_cable[0] = 0.0
                if xb_f is not None:
                    x_cable[0] = float(xb_f)
                y_cable[-1] = yr_f
                if xr_f is not None:
                    x_cable[-1] = float(xr_f)
            trace_print(
                5,
                f"[DEBUG] _solve_catenary: alignement vertical, Bézier quadratique "
                f"(réf. |h| géom.={h_mag:.3f} m, signe courant), "
                f"L_straight={L_straight:.3f}, L={L:.3f}",
            )
            return x_cable, y_cable
        
        # Approche : pour chaque valeur de a, résoudre pour x0 en utilisant la contrainte y_rov,
        # puis vérifier la longueur L
        
        def find_x0_for_a(a):
            """Trouve x0 pour un a donné en utilisant la contrainte y_rov"""
            if a <= 0 or not np.isfinite(a) or a < 1e-10:
                return None
            
            def equation_x0(x0):
                """Équation pour trouver x0 : y_rov = a * (cosh((x_rov - x0) / a) - cosh((x_boat - x0) / a))"""
                try:
                    # Vérifier que a est valide
                    if a <= 1e-10 or not np.isfinite(a):
                        return np.inf
                    
                    # Vérifier que x0 est fini
                    if not np.isfinite(x0):
                        return np.inf
                    
                    # Calculer les arguments de cosh
                    arg1 = (x_rov - x0) / a
                    arg2 = (x_boat - x0) / a
                    
                    # Vérifier que les arguments ne sont pas trop grands (éviter overflow)
                    if abs(arg1) > 700 or abs(arg2) > 700:
                        return np.inf
                    
                    # Calculer la valeur
                    val = a * (np.cosh(arg1) - np.cosh(arg2)) - y_rov
                    
                    # Vérifier que le résultat est fini
                    if not np.isfinite(val):
                        return np.inf
                    
                    return val
                except:
                    return np.inf
            
            # x0 devrait être entre x_rov et x_boat, plus proche du ROV
            # Élargir le bracket pour être sûr de trouver la solution
            bracket_width = max(abs(dx) * 2, 10.0)
            bracket = [min(x_rov, x_boat) - bracket_width, max(x_rov, x_boat) + bracket_width]
            
            # Vérifier que l'équation change de signe dans le bracket
            try:
                f_low = equation_x0(bracket[0])
                f_high = equation_x0(bracket[1])
                
                # Si les deux valeurs ont le même signe, essayer d'élargir le bracket
                if np.sign(f_low) == np.sign(f_high) and abs(f_low) > 1e-3 and abs(f_high) > 1e-3:
                    # Essayer plusieurs brackets
                    for scale in [2, 5, 10]:
                        bracket = [min(x_rov, x_boat) - bracket_width * scale, 
                                  max(x_rov, x_boat) + bracket_width * scale]
                        f_low = equation_x0(bracket[0])
                        f_high = equation_x0(bracket[1])
                        if np.sign(f_low) != np.sign(f_high):
                            break
            except:
                pass
            
            try:
                result = root_scalar(equation_x0, bracket=bracket, method='brentq', xtol=1e-8, maxiter=200)
                if result.converged:
                    return result.root
            except:
                pass
            
            # Essayer avec fsolve si root_scalar échoue
            try:
                # Essayer plusieurs estimations initiales
                x0_guesses = [
                    x_rov + 0.2 * (x_boat - x_rov),
                    x_rov + 0.1 * (x_boat - x_rov),
                    x_rov + 0.3 * (x_boat - x_rov),
                    (x_rov + x_boat) / 2,
                    x_rov,
                ]
                
                for x0_guess in x0_guesses:
                    try:
                        result = fsolve(equation_x0, x0_guess, xtol=1e-8, maxfev=500)
                        x0_test = result[0] if isinstance(result, np.ndarray) else result
                        
                        # Vérifier que la solution est bonne
                        error = abs(equation_x0(x0_test))
                        if error < 1e-3:
                            return x0_test
                    except:
                        continue
            except:
                pass
            
            return None
        
        def compute_length_error(a):
            """Calcule l'erreur sur la longueur pour un a donné"""
            if a <= 0 or not np.isfinite(a) or a < 1e-10:
                return np.inf
            
            x0 = find_x0_for_a(a)
            if x0 is None or not np.isfinite(x0):
                return np.inf
            
            try:
                # Calculer les arguments de sinh
                arg_boat = (x_boat - x0) / a
                arg_rov = (x_rov - x0) / a
                
                # Vérifier que les arguments ne sont pas trop grands (éviter overflow)
                if abs(arg_boat) > 700 or abs(arg_rov) > 700:
                    return np.inf
                
                # Calculer les longueurs curvilignes
                s_boat = a * np.sinh(arg_boat)
                s_rov = a * np.sinh(arg_rov)
                
                # Vérifier que les résultats sont finis
                if not (np.isfinite(s_boat) and np.isfinite(s_rov)):
                    return np.inf
                
                L_calc = abs(s_rov - s_boat)
                
                if not np.isfinite(L_calc):
                    return np.inf
                
                return abs(L_calc - L)
            except:
                return np.inf
        
        # Estimation initiale de a
        if L > L_straight * 1.5:
            a_initial = max(0.5, dy / 6.0)
        elif L > L_straight * 1.2:
            a_initial = max(1.0, dy / 4.0)
        else:
            a_initial = max(2.0, dy / 2.0)
        
        # Optimiser pour trouver a qui minimise l'erreur sur la longueur
        a_opt = a_initial
        best_error = np.inf
        
        try:
            # Essayer plusieurs valeurs initiales de a
            a_candidates = [a_initial, a_initial * 0.5, a_initial * 2.0, dy / 6.0, dy / 4.0, dy / 2.0, dy]
            a_candidates = [max(0.1, a) for a in a_candidates]
            
            for a_init in a_candidates:
                try:
                    # Vérifier d'abord que cette valeur initiale donne une erreur finie
                    test_error = compute_length_error(a_init)
                    if not np.isfinite(test_error) or test_error == np.inf:
                        continue
                    
                    # Optimiser a pour minimiser l'erreur sur la longueur
                    # Utiliser un bracket plus restreint autour de a_init
                    bracket_low = max(0.1, a_init * 0.5)
                    bracket_high = min(a_init * 5.0, 1000.0)  # Limiter à une valeur raisonnable
                    
                    # Vérifier que le bracket est valide
                    if bracket_low >= bracket_high:
                        continue
                    
                    # Supprimer les warnings de scipy pour les valeurs invalides (NaN/Inf)
                    # qui peuvent se produire lors de l'optimisation avec des valeurs extrêmes
                    with np.errstate(invalid='ignore', divide='ignore'):
                        result = minimize_scalar(
                            compute_length_error,
                            bracket=(bracket_low, bracket_high),
                            method='brent',
                            options={'xtol': 1e-5, 'maxiter': 200}
                        )
                    
                    if result.success:
                        a_test = result.x
                        if not np.isfinite(a_test) or a_test <= 0:
                            continue
                            
                        error = compute_length_error(a_test)
                        
                        if np.isfinite(error) and error < best_error:
                            best_error = error
                            a_opt = a_test
                            
                            if error < 1e-4:
                                break
                except Exception as e:
                    # Ignorer les erreurs et continuer avec la valeur suivante
                    continue
            
            if best_error > 1e-2:
                trace_print(5, f"⚠️  Avertissement: La caténaire a une erreur élevée ({best_error:.6f}). "
                      f"Utilisation des meilleures valeurs trouvées.")
        except Exception as e:
            trace_print(5, f"⚠️  Erreur lors de la résolution de la caténaire: {e}. Utilisation des valeurs initiales.")
        
        # Trouver x0 pour le a optimal
        x0_opt = find_x0_for_a(a_opt)
        if x0_opt is None:
            trace_print(7, f"[DEBUG] _solve_catenary: ⚠️  ERREUR - Impossible de trouver x0 pour a={a_opt:.3f}. Utilisation d'une interpolation linéaire (LIGNE DROITE).")
            x_cable = np.linspace(x_boat, x_rov, self.N + 1)
            y_cable = np.linspace(0.0, y_rov, self.N + 1)
            return x_cable, y_cable
        
        # Calculer y0
        try:
            y0_opt = -a_opt * np.cosh((x_boat - x0_opt) / a_opt)
        except:
            trace_print(5, f"⚠️  Erreur: Impossible de calculer y0. Utilisation d'une interpolation linéaire.")
            x_cable = np.linspace(x_boat, x_rov, self.N + 1)
            y_cable = np.linspace(0.0, y_rov, self.N + 1)
            return x_cable, y_cable
        
        # Vérifier la solution
        y_rov_calc = a_opt * np.cosh((x_rov - x0_opt) / a_opt) + y0_opt
        y_boat_calc = a_opt * np.cosh((x_boat - x0_opt) / a_opt) + y0_opt
        
        # y_rov devrait être négatif (profondeur : y < 0 = sous la surface)
        if abs(y_rov_calc - y_rov) > 0.1:
            trace_print(5, f"⚠️  Avertissement: y_rov calculé ({y_rov_calc:.3f}) diffère de y_rov attendu ({y_rov:.3f}, profondeur négative)")
        
        if abs(y_boat_calc) > 0.1:
            trace_print(5, f"⚠️  Avertissement: y_boat calculé ({y_boat_calc:.3f}) diffère de y_boat attendu (0.000, surface)")
        
        # Générer les points du câble par longueur curviligne
        # Convention : s = 0 au bateau, s = L au ROV (plus simple et cohérent avec le système)
        # Calculer les longueurs curvilignes aux extrémités dans le système de référence de la caténaire
        try:
            s_boat_ref = a_opt * np.sinh((x_boat - x0_opt) / a_opt)
            s_rov_ref = a_opt * np.sinh((x_rov - x0_opt) / a_opt)
            
            # La longueur curviligne de référence dans le système de la caténaire
            s_total_ref = abs(s_rov_ref - s_boat_ref)
            
            # Déterminer l'ordre : on veut s=0 au bateau et s=L au ROV
            # Donc on mappe s (0 à L) vers s_ref (s_boat_ref à s_rov_ref)
            if s_total_ref > 1e-6:
                # Transformation linéaire : s_ref = s_boat_ref + (s / L) * (s_rov_ref - s_boat_ref)
                # Mais on doit s'assurer que s_boat_ref < s_rov_ref
                if s_rov_ref < s_boat_ref:
                    # Inverser l'ordre
                    s_boat_ref, s_rov_ref = s_rov_ref, s_boat_ref
            else:
                # Si la longueur est nulle, utiliser une approximation
                s_boat_ref = 0.0
                s_rov_ref = L
        except:
            # Si le calcul échoue, utiliser une approximation simple
            s_boat_ref = 0.0
            s_rov_ref = L
        
        # Générer les points à intervalles réguliers de longueur curviligne : s de 0 à L
        s_points = np.linspace(0.0, L, self.N + 1)
        x_cable = np.zeros(self.N + 1)
        y_cable = np.zeros(self.N + 1)
        
        for i, s in enumerate(s_points):
            try:
                # Convertir s (0 à L) en s_ref (référence de la caténaire)
                # Interpolation linéaire : s_ref = s_boat_ref + (s / L) * (s_rov_ref - s_boat_ref)
                if L > 1e-6:
                    s_ref = s_boat_ref + (s / L) * (s_rov_ref - s_boat_ref)
                else:
                    s_ref = s_boat_ref
                
                # Calculer x à partir de s_ref : s_ref = a * sinh((x - x0) / a)
                # donc x = x0 + a * arcsinh(s_ref / a)
                x = x0_opt + a_opt * np.arcsinh(s_ref / a_opt)
                # Utiliser la formule standard : y = a * cosh((x - x0) / a) + y0
                y = a_opt * np.cosh((x - x0_opt) / a_opt) + y0_opt
                
                x_cable[i] = x
                y_cable[i] = y
            except:
                # Si le calcul échoue, interpolation linéaire
                alpha = i / self.N
                x_cable[i] = x_boat + alpha * (x_rov - x_boat)
                y_cable[i] = 0.0 + alpha * (y_rov - 0.0)
        
        # Forcer les extrémités exactement (important pour la continuité)
        # Index 0 = bateau, index -1 = ROV (convention cohérente avec le système)
        x_cable[0] = x_boat
        y_cable[0] = 0.0
        x_cable[-1] = x_rov
        y_cable[-1] = y_rov

        # CONTRAINTE PHYSIQUE : y <= 0 (surface). Géométrie « lourde » ; le câble flottant (w<0)
        # repasse par ``_pure_catenary_equilibrium_buoyant_cable`` (symétrie corde).
        y_cable = np.clip(y_cable, None, 0.0)
        y_cable[0] = 0.0
        y_cable[-1] = y_rov

        # Vérifier la longueur
        L_actual = 0.0
        for i in range(self.N):
            dx_seg = x_cable[i+1] - x_cable[i]
            dy_seg = y_cable[i+1] - y_cable[i]
            L_actual += np.sqrt(dx_seg**2 + dy_seg**2)

        # Toujours normaliser pour garantir que la longueur est exactement L
        # (tolérance réduite pour forcer la normalisation)
        # IMPORTANT : Passer les positions du bateau et du ROV pour garantir le recollage exact
        if abs(L_actual - L) / L > 1e-6:
            x_cable, y_cable, straight_mode, _ = self._normalize_cable_length(
                x_cable, y_cable, L,
                x_boat=x_boat,
                y_boat=0.0,
                x_rov=x_rov,
                y_rov=y_rov
            )
            x_rov, y_rov = apply_straight_mode_rov_snap(straight_mode, x_rov, y_rov, x_cable, y_cable)

            # CONTRAINTE PHYSIQUE : Après normalisation, forcer y <= 0
            y_cable = np.clip(y_cable, None, 0.0)
            if x_boat is not None:
                x_cable[0] = float(x_boat)
            y_cable[0] = 0.0

        # Vérification finale de la longueur
        L_final = 0.0
        for i in range(self.N):
            dx_seg = x_cable[i+1] - x_cable[i]
            dy_seg = y_cable[i+1] - y_cable[i]
            L_final += np.sqrt(dx_seg**2 + dy_seg**2)
        
        # Vérifier que la forme est bien une caténaire (pas juste deux segments)
        if len(x_cable) > 2:
            # Calculer les angles entre segments consécutifs
            angles = []
            for i in range(1, len(x_cable) - 1):
                dx1 = x_cable[i] - x_cable[i-1]
                dy1 = y_cable[i] - y_cable[i-1]
                dx2 = x_cable[i+1] - x_cable[i]
                dy2 = y_cable[i+1] - y_cable[i]
                ds1 = np.sqrt(dx1**2 + dy1**2)
                ds2 = np.sqrt(dx2**2 + dy2**2)
                if ds1 > 1e-6 and ds2 > 1e-6:
                    cos_angle = (dx1*dx2 + dy1*dy2) / (ds1 * ds2)
                    cos_angle = np.clip(cos_angle, -1.0, 1.0)
                    angle = np.arccos(cos_angle)
                    angles.append(angle)
            
            # Si tous les angles sont très petits, c'est probablement une ligne droite
            if len(angles) > 0 and np.mean(angles) < 0.01:
                trace_print(5, f"⚠️  Avertissement: Le câble semble être une ligne droite plutôt qu'une caténaire. "
                      f"Vérifiez les paramètres (L={L:.2f} m, L_straight={L_straight:.2f} m, "
                      f"rho_cable={self.rho_cable:.1f} kg/m³, rho_eau={self.environment.rho_eau:.1f} kg/m³)")
        
        # Avertissement si la longueur n'est toujours pas correcte
        if abs(L_final - L) / L > 0.02:
            trace_print(5, f"⚠️  Avertissement: Longueur du câble incorrecte après calcul de caténaire. "
                  f"Attendu: {L:.3f} m, Obtenu: {L_final:.3f} m, Erreur: {abs(L_final - L)/L*100:.1f}%")
        
        # Retourner le câble : index 0 = bateau, index -1 = ROV (convention cohérente avec le système)
        return x_cable, y_cable
    
    def _objective_equilibrium_residual(self, x_int, y_int, x_boat, y_boat, x_rov, y_rov, L, 
                                       forces_func, params_forces):
        """
        Fonction objectif : minimise les résidus des équations d'équilibre.
        
        Parameters:
        -----------
        x_int, y_int : array
            Positions des points intérieurs (N-1 points)
        x_boat, y_boat, x_rov, y_rov : float
            Positions des extrémités (fixées)
        L : float
            Longueur cible du câble
        forces_func : callable
            Fonction qui calcule les forces (Fx, Fy) pour une configuration donnée
        params_forces : dict
            Paramètres pour le calcul des forces
        
        Returns:
        --------
        float
            Somme des carrés des résidus d'équilibre
        """
        # Reconstruire le câble complet avec les extrémités fixées
        N = len(x_int) + 1
        x_cable = np.zeros(N + 1)
        y_cable = np.zeros(N + 1)
        x_cable[0] = x_boat
        y_cable[0] = y_boat
        x_cable[1:-1] = x_int
        y_cable[1:-1] = y_int
        x_cable[-1] = x_rov
        y_cable[-1] = y_rov
        
        # Calculer les forces
        try:
            Fx, Fy = forces_func(x_cable, y_cable, params_forces)
        except Exception:
            # En cas d'erreur, retourner une pénalité élevée
            return 1e10
        
        # Calculer les résidus d'équilibre aux nœuds intérieurs
        # Pour chaque nœud intérieur i, l'équilibre des forces donne :
        # T[i] * u[i] - T[i-1] * u[i-1] + F[i] = 0
        # où u[i] est le vecteur unitaire du segment i
        
        # Calculer les vecteurs unitaires des segments
        dx = np.diff(x_cable)
        dy = np.diff(y_cable)
        ds = np.sqrt(dx**2 + dy**2)
        ds = np.maximum(ds, 1e-9)  # Éviter division par zéro
        ux = dx / ds
        uy = dy / ds
        
        # Estimation simple des tensions (approximation linéaire)
        # Pour une meilleure approximation, on pourrait résoudre le système de tensions
        # mais pour l'objectif, une approximation suffit
        T_est = np.ones(N + 1) * 10.0  # Estimation initiale
        
        # Calculer les résidus
        residual = 0.0
        for i in range(1, N):  # Nœuds intérieurs uniquement
            # Résidu d'équilibre horizontal
            res_x = T_est[i] * ux[i] - T_est[i-1] * ux[i-1] + Fx[i-1] if i < N else 0.0
            # Résidu d'équilibre vertical
            res_y = T_est[i] * uy[i] - T_est[i-1] * uy[i-1] + Fy[i-1] if i < N else 0.0
            residual += res_x**2 + res_y**2
        
        return residual
    
    def _constraint_length(self, x_int, y_int, x_boat, y_boat, x_rov, y_rov, L):
        """
        Contrainte : L_seg - L = 0 (avec tolérance 0.01%)
        
        Returns:
        --------
        float
            L_seg - L
        """
        # Reconstruire le câble complet
        N = len(x_int) + 1
        x_cable = np.zeros(N + 1)
        y_cable = np.zeros(N + 1)
        x_cable[0] = x_boat
        y_cable[0] = y_boat
        x_cable[1:-1] = x_int
        y_cable[1:-1] = y_int
        x_cable[-1] = x_rov
        y_cable[-1] = y_rov
        
        # Calculer L_seg
        L_seg = 0.0
        for i in range(N):
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            L_seg += np.sqrt(dx**2 + dy**2)
        
        return L_seg - L
    
    def _constraint_surface(self, x_int, y_int, x_boat, y_boat, x_rov, y_rov):
        """
        Contrainte : max(y_cable) <= 0
        
        Returns:
        --------
        float
            max(y_cable) (doit être <= 0)
        """
        # Reconstruire le câble complet
        N = len(x_int) + 1
        y_cable = np.zeros(N + 1)
        y_cable[0] = y_boat
        y_cable[1:-1] = y_int
        y_cable[-1] = y_rov

        return np.max(y_cable)
    
    def _constraint_segment_ratio(self, x_int, y_int, x_boat, y_boat, x_rov, y_rov, L, k_max):
        """
        Contrainte : ds_max / ds_target - k_max <= 0
        
        Returns:
        --------
        float
            ds_max / ds_target - k_max
        """
        # Reconstruire le câble complet
        N = len(x_int) + 1
        x_cable = np.zeros(N + 1)
        y_cable = np.zeros(N + 1)
        x_cable[0] = x_boat
        y_cable[0] = y_boat
        x_cable[1:-1] = x_int
        y_cable[1:-1] = y_int
        x_cable[-1] = x_rov
        y_cable[-1] = y_rov

        # Calculer les longueurs des segments
        ds_list = []
        for i in range(N):
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            ds = np.sqrt(dx**2 + dy**2)
            ds_list.append(ds)
        
        if len(ds_list) == 0:
            return 0.0
        
        ds_max = np.max(ds_list)
        ds_target = L / N if N > 0 else L
        ratio = ds_max / ds_target if ds_target > 1e-9 else 0.0
        
        return ratio - k_max
    
    def _forces_static_wrapper(self, x_cable, y_cable, params):
        """
        Wrapper pour calculer les forces statiques (poids uniquement).
        
        Parameters:
        -----------
        x_cable, y_cable : array
            Positions du câble
        params : dict
            Contient 'environment', 'params_cable', 'L'
        
        Returns:
        --------
        tuple (Fx, Fy)
            Forces par segment (N)
        """
        environment = params['environment']
        params_cable = params['params_cable']
        L = params['L']
        
        # Vitesses nulles pour le cas statique
        vx_cable = np.zeros(len(x_cable))
        vy_cable = np.zeros(len(x_cable))
        
        from .forces import compute_cable_forces
        Fx, Fy, _, _, _, _ = compute_cable_forces(
            x_cable, y_cable, vx_cable, vy_cable, environment, params_cable, L
        )
        
        return Fx, Fy
    
    def _forces_static_with_current_wrapper(self, x_cable, y_cable, params):
        """
        Wrapper pour calculer les forces statiques avec courant.
        
        Parameters:
        -----------
        x_cable, y_cable : array
            Positions du câble
        params : dict
            Contient 'environment', 'params_cable', 'L'
        
        Returns:
        --------
        tuple (Fx, Fy)
            Forces par segment (N)
        """
        environment = params['environment']
        params_cable = params['params_cable']
        L = params['L']
        
        # Vitesses nulles pour le cas statique (le courant est dans l'environnement)
        vx_cable = np.zeros(len(x_cable))
        vy_cable = np.zeros(len(x_cable))
        
        from .forces import compute_cable_forces
        Fx, Fy, _, _, _, _ = compute_cable_forces(
            x_cable, y_cable, vx_cable, vy_cable, environment, params_cable, L
        )
        
        return Fx, Fy
    
    def _forces_dynamic_wrapper(self, x_cable, y_cable, params):
        """
        Wrapper pour calculer les forces dynamiques (avec vitesses).
        
        Parameters:
        -----------
        x_cable, y_cable : array
            Positions du câble
        params : dict
            Contient 'environment', 'params_cable', 'L', 'vx_cable', 'vy_cable'
        
        Returns:
        --------
        tuple (Fx, Fy)
            Forces par segment (N)
        """
        environment = params['environment']
        params_cable = params['params_cable']
        L = params['L']
        vx_cable = params['vx_cable']
        vy_cable = params['vy_cable']
        
        from .forces import compute_cable_forces
        Fx, Fy, _, _, _, _ = compute_cable_forces(
            x_cable, y_cable, vx_cable, vy_cable, environment, params_cable, L
        )
        
        return Fx, Fy

    @staticmethod
    def _polyline_length_xy(x: np.ndarray, y: np.ndarray) -> float:
        dx = np.diff(np.asarray(x, dtype=float))
        dy = np.diff(np.asarray(y, dtype=float))
        return float(np.sum(np.sqrt(dx * dx + dy * dy)))

    def _node_forces_lumped_from_segments(
        self, Fx_seg: np.ndarray, Fy_seg: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Forces nodales (trapèze) : F_i = 0.5*(F_{i-1}+F_i) pour nœuds intérieurs."""
        n_seg = len(Fx_seg)
        if n_seg < 2:
            return np.array([]), np.array([])
        fx_n = 0.5 * (Fx_seg[:-1] + Fx_seg[1:])
        fy_n = 0.5 * (Fy_seg[:-1] + Fy_seg[1:])
        return fx_n, fy_n

    def _tension_balance_matrix(self, ux: np.ndarray, uy: np.ndarray) -> np.ndarray:
        """Matrice A telle que A @ T = b avec T_i u_i - T_{i-1} u_{i-1} = -F_i aux nœuds intérieurs."""
        n_seg = len(ux)
        n_int = n_seg - 1
        if n_int <= 0:
            return np.zeros((0, n_seg))
        A = np.zeros((2 * n_int, n_seg))
        for i in range(1, n_seg):
            r = 2 * (i - 1)
            A[r, i - 1] = -ux[i - 1]
            A[r, i] = ux[i]
            A[r + 1, i - 1] = -uy[i - 1]
            A[r + 1, i] = uy[i]
        return A

    def _resample_polyline_uniform_arc_length(
        self,
        x_c: np.ndarray,
        y_c: np.ndarray,
        n_nodes: int,
        x_boat: float,
        y_boat: float,
        x_rov: float,
        y_rov: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        x_c = np.asarray(x_c, dtype=float).reshape(-1)
        y_c = np.asarray(y_c, dtype=float).reshape(-1)
        if x_c.size < 2 or y_c.size < 2:
            return (
                np.linspace(x_boat, x_rov, n_nodes),
                np.linspace(y_boat, y_rov, n_nodes),
            )
        dx = np.diff(x_c)
        dy = np.diff(y_c)
        ds = np.sqrt(dx * dx + dy * dy)
        s = np.concatenate([[0.0], np.cumsum(ds)])
        s_end = float(s[-1])
        if s_end < 1e-9:
            return (
                np.linspace(x_boat, x_rov, n_nodes),
                np.linspace(y_boat, y_rov, n_nodes),
            )
        s_new = np.linspace(0.0, s_end, int(n_nodes))
        fx = interp1d(s, x_c, kind="linear", bounds_error=False, fill_value="extrapolate")
        fy = interp1d(s, y_c, kind="linear", bounds_error=False, fill_value="extrapolate")
        x_new = fx(s_new).astype(float)
        y_new = fy(s_new).astype(float)
        x_new[0] = float(x_boat)
        y_new[0] = float(y_boat)
        x_new[-1] = float(x_rov)
        y_new[-1] = float(y_rov)
        y_new = np.minimum(y_new, 0.0)
        return x_new, y_new

    def _try_solve_nodal_static_equilibrium_coarse(
        self,
        x_rov: float,
        y_rov: float,
        x_boat: float,
        L: float,
        forces_func,
        params_forces: dict,
        x_init_full: np.ndarray,
        y_init_full: np.ndarray,
        n_seg_coarse: int | None = None,
        k_max: float = 2.0,
    ) -> tuple[bool, np.ndarray | None, np.ndarray | None]:
        """
        Équilibre statique discret : résidus d'équilibre aux nœuds + contrainte de longueur,
        sur un maillage grossier puis rééchantillonnage vers ``self.N`` segments.
        """
        y_boat = 0.0
        n_seg = int(n_seg_coarse if n_seg_coarse is not None else NODAL_EQUILIBRIUM_COARSE_SEGMENTS_DEFAULT)
        n_seg = max(4, min(n_seg, max(self.N, 4)))
        if L <= 0.0 or n_seg < 2:
            return False, None, None

        xf = np.asarray(x_init_full, dtype=float).reshape(-1)
        yf = np.asarray(y_init_full, dtype=float).reshape(-1)
        if xf.size != yf.size or xf.size < 2:
            return False, None, None

        idx = np.linspace(0, xf.size - 1, n_seg + 1).astype(int)
        x_int0 = xf[idx[1:-1]].copy()
        y_int0 = np.minimum(yf[idx[1:-1]].copy(), 0.0)
        n_int = n_seg - 1
        z0 = np.concatenate([x_int0, y_int0])

        scale_x = max(abs(float(x_rov) - float(x_boat)), 1.0)
        scale_y = max(abs(float(y_rov)), 1.0)
        x_scale = np.concatenate([np.full(n_int, scale_x), np.full(n_int, scale_y)])

        def pack_state(z: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            x_int = z[:n_int]
            y_int = z[n_int:]
            x = np.zeros(n_seg + 1)
            y = np.zeros(n_seg + 1)
            x[0] = float(x_boat)
            y[0] = float(y_boat)
            x[-1] = float(x_rov)
            y[-1] = float(y_rov)
            x[1:-1] = x_int
            y[1:-1] = np.minimum(y_int, 0.0)
            return x, y

        def residual_vec(z: np.ndarray) -> np.ndarray:
            x, y = pack_state(z)
            try:
                Fx, Fy = forces_func(x, y, params_forces)
            except Exception:
                return np.ones(2 * n_int + 1) * 1e6
            if Fx.size != n_seg or Fy.size != n_seg:
                return np.ones(2 * n_int + 1) * 1e6
            dx = np.diff(x)
            dy = np.diff(y)
            ds = np.sqrt(dx * dx + dy * dy)
            ds = np.maximum(ds, 1e-12)
            ux = dx / ds
            uy = dy / ds
            fx_n, fy_n = self._node_forces_lumped_from_segments(Fx, Fy)
            if fx_n.size != n_int:
                return np.ones(2 * n_int + 1) * 1e6
            A = self._tension_balance_matrix(ux, uy)
            bx = -fx_n
            by = -fy_n
            b = np.empty(2 * n_int)
            b[0::2] = bx
            b[1::2] = by
            try:
                T, _ = nnls(A, b)
            except Exception:
                return np.ones(2 * n_int + 1) * 1e6
            r = A @ T - b
            L_cur = self._polyline_length_xy(x, y)
            r_len = np.array([NODAL_EQUILIBRIUM_LENGTH_WEIGHT * (L_cur - float(L))], dtype=float)

            # Régularisation de courbure (anti-zigzag) sur le maillage grossier.
            d2x = np.diff(x, n=2)
            d2y = np.diff(y, n=2)
            r_sx = np.sqrt(NODAL_EQUILIBRIUM_SMOOTH_X_WEIGHT) * d2x
            r_sy = np.sqrt(NODAL_EQUILIBRIUM_SMOOTH_Y_WEIGHT) * d2y

            return np.concatenate([r, r_len, r_sx, r_sy])

        try:
            result = least_squares(
                residual_vec,
                z0,
                bounds=(
                    np.concatenate(
                        [
                            np.full(n_int, -np.inf),
                            np.full(n_int, -np.inf),
                        ]
                    ),
                    np.concatenate(
                        [
                            np.full(n_int, np.inf),
                            np.full(n_int, 0.0),
                        ]
                    ),
                ),
                x_scale=x_scale,
                max_nfev=NODAL_EQUILIBRIUM_LS_MAX_NFEV,
                ftol=1e-8,
                xtol=1e-8,
                gtol=1e-8,
            )
        except Exception as e:
            trace_print(5, f"[nodal_equilibrium] least_squares échoué ({e}), fallback.")
            return False, None, None

        r_fin = residual_vec(result.x)
        n_force = 2 * n_int
        r_force = float(np.linalg.norm(r_fin[:n_force]))
        r_len = float(abs(r_fin[n_force]))
        # Résidus de force en N ; dernier terme = longueur (m) pondérée.
        if not result.success and r_force > 50.0 and r_len > 0.05 * max(1.0, abs(float(L))):
            trace_print(
                5,
                f"[nodal_equilibrium] résidu élevé (||r_F||={r_force:.3g}, |r_L|={r_len:.4g}), "
                f"cost={result.cost:.4g}, fallback heuristique.",
            )
            return False, None, None

        x_c, y_c = pack_state(result.x)
        if not np.all(np.isfinite(x_c)) or not np.all(np.isfinite(y_c)):
            return False, None, None

        n_full = int(self.N) + 1
        x_up, y_up = self._resample_polyline_uniform_arc_length(
            x_c, y_c, n_full, x_boat, y_boat, x_rov, y_rov
        )
        try:
            x_up, y_up, straight_mode, _ = self._normalize_cable_length(
                x_up,
                y_up,
                float(L),
                x_boat=float(x_boat),
                y_boat=0.0,
                x_rov=float(x_rov),
                y_rov=float(y_rov),
                k_max=float(k_max),
            )
        except Exception as e:
            trace_print(5, f"[nodal_equilibrium] _normalize_cable_length ({e}), fallback.")
            return False, None, None

        L_chk = self._polyline_length_xy(x_up, y_up)
        if abs(L_chk - float(L)) / max(float(L), 1e-9) > 0.02:
            trace_print(
                5,
                f"[nodal_equilibrium] longueur après normalisation hors tolérance "
                f"(L≈{L_chk:.4f}, cible={L:.4f}), fallback.",
            )
            return False, None, None

        return True, x_up, y_up
    
    def solve_equilibrium_constrained(self, x_rov, y_rov, x_boat, L, forces_func, 
                                     params_forces, rov_m=None, rov_vol=None,
                                     initial_guess=None, k_max=2.0):
        """
        Résout l'équilibre du câble avec toutes les contraintes intégrées.
        
        Utilise scipy.optimize.minimize avec des contraintes pour garantir :
        1. L = L_seg à 0.01% près
        2. P0 = position bateau
        3. PN = position ROV
        4. y <= 0 pour tous les points
        5. ds_max / ds_target <= k_max
        
        Parameters:
        -----------
        x_rov, y_rov : float
            Position du ROV
        x_boat : float
            Position horizontale du bateau
        L : float
            Longueur du câble
        forces_func : callable
            Fonction qui calcule les forces (Fx, Fy) pour une configuration donnée
            Signature: Fx, Fy = forces_func(x_cable, y_cable, params_forces)
        params_forces : dict
            Paramètres pour le calcul des forces
        rov_m, rov_vol : float, optional
            Masse et volume du ROV
        initial_guess : tuple (x_int, y_int), optional
            Estimation initiale des points intérieurs
        k_max : float
            Ratio maximum ds_max / ds_target (défaut: 2.0)
        
        Returns:
        --------
        tuple (x_cable, y_cable, T)
            Configuration du câble avec tensions
        """
        # NonlinearConstraint est déjà importé en haut du fichier
        
        trace_print(5, "\n[DEBUG] ========== solve_equilibrium_constrained APPELÉE ==========")
        trace_print(5, f"[DEBUG] solve_equilibrium_constrained: Paramètres d'entrée - x_rov={x_rov:.3f}, y_rov={y_rov:.3f}, x_boat={x_boat:.3f}, L={L:.3f}, initial_guess={'None' if initial_guess is None else 'fourni'}")
        
        if L <= 0:
            return np.array([x_rov, x_boat]), np.array([y_rov, 0.0]), np.array([0.0, 0.0])
        
        N = self.N
        y_boat = 0.0
        
        # Estimation initiale : utiliser une caténaire approximative pour une forme réaliste
        # IMPORTANT : Utiliser directement la caténaire complète au lieu d'extraire les points intérieurs
        # pour préserver la forme
        x_cable_cat = None
        y_cable_cat = None
        
        trace_print(5, f"[DEBUG] solve_equilibrium_constrained: initial_guess is None = {initial_guess is None}")
        
        # Vérifier si l'initial_guess fourni est une ligne droite (déviation trop faible)
        # Si c'est le cas, ignorer l'initial_guess et générer une caténaire
        use_catenary = False
        if initial_guess is not None:
            x_int_init, y_int_init = initial_guess
            # Reconstruire le câble complet pour vérifier la déviation
            x_cable_test = np.concatenate([[x_boat], x_int_init, [x_rov]])
            y_cable_test = np.concatenate([[0.0], y_int_init, [y_rov]])
            # Calculer la déviation maximale
            if len(x_cable_test) >= 3:
                x_start, y_start = x_cable_test[0], y_cable_test[0]
                x_end, y_end = x_cable_test[-1], y_cable_test[-1]
                max_dev_test = 0.0
                if abs(x_end - x_start) > 1e-6 or abs(y_end - y_start) > 1e-6:
                    dx_line = x_end - x_start
                    dy_line = y_end - y_start
                    line_length = np.sqrt(dx_line**2 + dy_line**2)
                    for i in range(1, len(x_cable_test) - 1):
                        dx_point = x_cable_test[i] - x_start
                        dy_point = y_cable_test[i] - y_start
                        t = (dx_point * dx_line + dy_point * dy_line) / (line_length**2 + 1e-9)
                        x_proj = x_start + t * dx_line
                        y_proj = y_start + t * dy_line
                        deviation = np.sqrt((x_cable_test[i] - x_proj)**2 + (y_cable_test[i] - y_proj)**2)
                        max_dev_test = max(max_dev_test, deviation)
                # Si la déviation est très faible (< 0.01 m), c'est une ligne droite, utiliser une caténaire
                if max_dev_test < 0.01:
                    trace_print(7, f"[DEBUG] solve_equilibrium_constrained: initial_guess est une ligne droite (max_deviation={max_dev_test:.6f} m), génération d'une caténaire à la place")
                    use_catenary = True
                    initial_guess = None  # Ignorer l'initial_guess et générer une caténaire
        
        if initial_guess is None or use_catenary:
            try:
                # Calculer le poids apparent par unité de longueur
                # w = (rho_cable - rho_water) * g * A_cable
                rho_water = self.environment.rho_water if hasattr(self.environment, 'rho_water') else 1025.0
                g = 9.81
                A_cable = np.pi * (self.d / 2)**2
                w = (self.rho_cable - rho_water) * g * A_cable
                
                # Essayer de résoudre la caténaire pour obtenir une forme réaliste
                trace_print(5, "[DEBUG] ========== solve_equilibrium_constrained: Tentative d'estimation initiale avec caténaire... ==========")
                trace_print(5, f"[DEBUG] solve_equilibrium_constrained: Paramètres avant appel _solve_catenary - x_rov={x_rov:.3f}, y_rov={y_rov:.3f}, x_boat={x_boat:.3f}, L={L:.3f}, w={w:.6f}")
                x_cable_cat, y_cable_cat = self._solve_catenary(x_rov, y_rov, x_boat, L, w)
                trace_print(5, f"[DEBUG] solve_equilibrium_constrained: _solve_catenary retourné - len(x)={len(x_cable_cat)}, len(y)={len(y_cable_cat)}")
                
                if len(x_cable_cat) == N + 1 and len(y_cable_cat) == N + 1:
                    # Vérifier si la caténaire a vraiment une forme courbe
                    x_start, y_start = x_cable_cat[0], y_cable_cat[0]
                    x_end, y_end = x_cable_cat[-1], y_cable_cat[-1]
                    max_deviation = 0.0
                    if abs(x_end - x_start) > 1e-6 or abs(y_end - y_start) > 1e-6:
                        dx_line = x_end - x_start
                        dy_line = y_end - y_start
                        line_length = np.sqrt(dx_line**2 + dy_line**2)
                        for i in range(1, len(x_cable_cat) - 1):
                            dx_point = x_cable_cat[i] - x_start
                            dy_point = y_cable_cat[i] - y_start
                            t = (dx_point * dx_line + dy_point * dy_line) / (line_length**2 + 1e-9)
                            x_proj = x_start + t * dx_line
                            y_proj = y_start + t * dy_line
                            deviation = np.sqrt((x_cable_cat[i] - x_proj)**2 + (y_cable_cat[i] - y_proj)**2)
                            max_deviation = max(max_deviation, deviation)
                    
                    if max_deviation < 0.01:
                        trace_print(7, f"[DEBUG] solve_equilibrium_constrained: ⚠️  Caténaire générée semble être une ligne droite (max_deviation={max_deviation:.6f} m)")
                    else:
                        trace_print(5, f"[DEBUG] solve_equilibrium_constrained: ✓ Caténaire générée a une forme courbe (max_deviation={max_deviation:.6f} m)")
                    
                    # Utiliser directement la caténaire complète comme estimation initiale
                    trace_print(5, "[DEBUG] solve_equilibrium_constrained: Estimation initiale basée sur caténaire réussie")
                    # Extraire les points intérieurs seulement pour l'optimisation (si activée)
                    x_int_init = x_cable_cat[1:-1].copy()
                    y_int_init = y_cable_cat[1:-1].copy()
                else:
                    raise ValueError(f"Caténaire retournée avec mauvaise taille: {len(x_cable_cat)} au lieu de {N+1}")
            except Exception as e:
                # Fallback : utiliser une forme parabolique approximative au lieu d'une ligne droite
                trace_print(7, f"[DEBUG] solve_equilibrium_constrained: Échec caténaire ({e}), utilisation d'une forme parabolique approximative")
                # Forme parabolique : y = a * x^2 + b * x + c
                # Contraintes : y(0) = 0, y(x_rov) = y_rov
                # Pour une forme réaliste, ajouter une courbure vers le bas
                x_points = np.linspace(x_boat, x_rov, N + 1)
                y_points = np.linspace(y_boat, y_rov, N + 1)
                
                # Ajouter une courbure parabolique vers le bas pour simuler l'effet du poids
                # y = y_linéaire + amplitude * (x - x_boat) * (x_rov - x) / (x_rov - x_boat)^2
                if abs(x_rov - x_boat) > 1e-6:
                    amplitude = min(abs(y_rov) * 0.3, 10.0)  # Courbure modérée
                    for i in range(len(x_points)):
                        x = x_points[i]
                        curvature = amplitude * (x - x_boat) * (x_rov - x) / ((x_rov - x_boat)**2 + 1e-9)
                        y_points[i] = y_points[i] - abs(curvature)  # Courber vers le bas
                
                # S'assurer que y <= 0
                y_points = np.minimum(y_points, 0.0)
                
                # Extraire les points intérieurs
                x_int_init = x_points[1:-1].copy()
                y_int_init = y_points[1:-1].copy()
        else:
            x_int_init, y_int_init = initial_guess
        
        # Vecteur d'optimisation : positions des points intérieurs
        # Format: [x1, x2, ..., xN-1, y1, y2, ..., yN-1]
        n_int = N - 1
        x0 = np.concatenate([x_int_init, y_int_init])
        
        # Fonction objectif wrapper
        def objective(vars):
            x_int = vars[:n_int]
            y_int = vars[n_int:]
            return self._objective_equilibrium_residual(
                x_int, y_int, x_boat, y_boat, x_rov, y_rov, L, forces_func, params_forces
            )
        
        # Contraintes
        constraints = []
        
        # Contrainte de longueur : L_seg - L = 0 (tolérance 0.01%)
        def constraint_length_func(vars):
            x_int = vars[:n_int]
            y_int = vars[n_int:]
            return self._constraint_length(x_int, y_int, x_boat, y_boat, x_rov, y_rov, L)
        
        constraints.append(NonlinearConstraint(
            constraint_length_func,
            lb=-L * 1e-4,  # Tolérance 0.01%
            ub=L * 1e-4,
            keep_feasible=False
        ))
        
        # Contrainte de surface : max(y) <= 0
        def constraint_surface_func(vars):
            x_int = vars[:n_int]
            y_int = vars[n_int:]
            return self._constraint_surface(x_int, y_int, x_boat, y_boat, x_rov, y_rov)
        
        constraints.append(NonlinearConstraint(
            constraint_surface_func,
            lb=-np.inf,
            ub=0.0,
            keep_feasible=False
        ))
        
        # Contrainte de ratio : ds_max / ds_target - k_max <= 0
        def constraint_ratio_func(vars):
            x_int = vars[:n_int]
            y_int = vars[n_int:]
            return self._constraint_segment_ratio(x_int, y_int, x_boat, y_boat, x_rov, y_rov, L, k_max)
        
        constraints.append(NonlinearConstraint(
            constraint_ratio_func,
            lb=-np.inf,
            ub=0.0,
            keep_feasible=False
        ))
        
        # Optimisation désactivée temporairement pour éviter les blocages à l'initialisation
        # L'optimisation avec contraintes non-linéaires peut être très lente avec 1000 points
        # TODO: Réactiver progressivement avec des paramètres très restrictifs :
        #   - maxiter=2-3 seulement
        #   - Méthode 'SLSQP' plus rapide que 'trust-constr'
        #   - Ou utiliser un sous-échantillonnage pour réduire le nombre de variables
        USE_OPTIMIZATION = False  # Désactiver pour l'instant
        
        if USE_OPTIMIZATION:
            try:
                trace_print(7, "[DEBUG] solve_equilibrium_constrained: Démarrage de l'optimisation avec contraintes...")
                result = minimize(
                    objective,
                    x0,
                    method='SLSQP',  # Plus rapide que trust-constr
                    constraints=constraints,
                    options={
                        'maxiter': 3,  # Très limité pour éviter les blocages
                        'ftol': 1e-3,  # Tolérance sur la fonction objectif
                        'disp': False  # Pas de messages
                    }
                )
                
                if result.success:
                    trace_print(7, f"[DEBUG] solve_equilibrium_constrained: Optimisation réussie en {result.nit} itérations")
                    x_int = result.x[:n_int]
                    y_int = result.x[n_int:]
                else:
                    trace_print(7, f"[DEBUG] solve_equilibrium_constrained: Optimisation non convergée ({result.message}), utilisation de l'estimation initiale")
                    x_int = x_int_init
                    y_int = y_int_init
            except Exception as e:
                trace_print(7, f"[DEBUG] solve_equilibrium_constrained: Erreur lors de l'optimisation ({e}), utilisation de l'estimation initiale")
                x_int = x_int_init
                y_int = y_int_init
        else:
            # Utiliser directement l'estimation initiale
            # _normalize_cable_length garantira toutes les contraintes ensuite
            trace_print(4, "[DEBUG] solve_equilibrium_constrained: Utilisation de l'estimation initiale (optimisation désactivée)")
            x_int = x_int_init
            y_int = y_int_init
        
        # Reconstruire le câble complet avec recollement garanti
        # Si on a une caténaire complète, l'utiliser directement pour préserver la forme
        if x_cable_cat is not None and y_cable_cat is not None and len(x_cable_cat) == N + 1:
            # Utiliser directement la caténaire complète pour préserver la forme
            x_cable = x_cable_cat.copy()
            y_cable = y_cable_cat.copy()
            # Forcer seulement les extrémités pour garantir le recollement
            x_cable[0] = x_boat  # Contrainte P0 = bateau
            y_cable[0] = y_boat
            x_cable[-1] = x_rov  # Contrainte PN = ROV
            y_cable[-1] = y_rov
            trace_print(5, "[DEBUG] solve_equilibrium_constrained: Utilisation directe de la caténaire complète pour préserver la forme")
        else:
            # Reconstruire à partir des points intérieurs (fallback)
            x_cable = np.zeros(N + 1)
            y_cable = np.zeros(N + 1)
            x_cable[0] = x_boat  # Contrainte P0 = bateau
            y_cable[0] = y_boat
            x_cable[1:-1] = x_int
            y_cable[1:-1] = y_int
            x_cable[-1] = x_rov  # Contrainte PN = ROV
            y_cable[-1] = y_rov
        
        # Vérifier si la normalisation est nécessaire avant d'appeler _normalize_cable_length
        # Calculer la longueur actuelle
        L_actual = 0.0
        for i in range(N):
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            L_actual += np.sqrt(dx**2 + dy**2)
        
        # Vérifier les contraintes avant normalisation
        tol_L = L * 1e-4  # Tolérance 0.01%
        needs_normalization = (
            abs(L_actual - L) > tol_L or  # Longueur incorrecte
            (x_boat is not None and abs(x_cable[0] - x_boat) > 1e-6) or  # Recollement bateau
            (x_rov is not None and abs(x_cable[-1] - x_rov) > 1e-6) or  # Recollement ROV
            np.any(y_cable > 1e-9)  # Points au-dessus de la surface
        )
        
        if needs_normalization:
            try:
                trace_print(5, "[DEBUG] solve_equilibrium_constrained: Application de _normalize_cable_length pour garantir toutes les contraintes...")
                trace_print(5, f"[DEBUG] solve_equilibrium_constrained: Paramètres - L={L}, L_actual={L_actual:.6f}, x_boat={x_boat}, y_boat={y_boat}, x_rov={x_rov}, y_rov={y_rov}, k_max={k_max}")
                trace_print(5, f"[DEBUG] solve_equilibrium_constrained: Avant appel à _normalize_cable_length, x_cable.shape={x_cable.shape if hasattr(x_cable, 'shape') else len(x_cable)}, y_cable.shape={y_cable.shape if hasattr(y_cable, 'shape') else len(y_cable)}")
                x_cable, y_cable, straight_mode, _ = self._normalize_cable_length(
                    x_cable, y_cable, L,
                    x_boat=float(x_boat),
                    y_boat=0.0,
                    x_rov=float(x_rov),
                    y_rov=float(y_rov),
                    k_max=k_max
                )
                x_rov, y_rov = apply_straight_mode_rov_snap(straight_mode, x_rov, y_rov, x_cable, y_cable)
                trace_print(5, "[DEBUG] solve_equilibrium_constrained: _normalize_cable_length terminé avec succès")
            except Exception as e:
                trace_print(7, f"[DEBUG] solve_equilibrium_constrained: Échec de _normalize_cable_length: {e}")
                import traceback
                trace_print(7, f"[DEBUG] Traceback complet:\n{traceback.format_exc()}")
        else:
            trace_print(7, f"[DEBUG] solve_equilibrium_constrained: Pas de normalisation nécessaire (L_actual={L_actual:.6f}, L={L:.6f}, diff={abs(L_actual-L):.6e})")
            # S'assurer quand même que y <= 0 et que les extrémités sont correctes
            y_cable = np.clip(y_cable, None, 0.0)
            if x_boat is not None:
                x_cable[0] = float(x_boat)
                y_cable[0] = 0.0
            if x_rov is not None:
                x_cable[-1] = float(x_rov)
                y_cable[-1] = float(y_rov)
        
        # Calculer les tensions (approximation simple)
        # Pour une meilleure précision, on pourrait résoudre le système d'équations de tension
        T = np.ones(N + 1) * 10.0  # Estimation par défaut
        
        return x_cable, y_cable, T
    
    def solve_equilibrium_dynamic(self, x_rov, y_rov, vx_rov, vy_rov,
                                  x_boat, vx_boat, L, x_cable_prev, y_cable_prev, rov_m=None, rov_vol=None):
        """
        Résout l'équilibre dynamique du câble avec forces hydrodynamiques
        
        Utilise maintenant le solveur avec contraintes intégrées pour garantir :
        - L = L_seg à 0.01% près
        - P0 = position bateau
        - PN = position ROV
        - y <= 0 pour tous les points
        - ds_max / ds_target <= k_max
        
        Parameters:
        -----------
        x_rov, y_rov : float
            Position du ROV
        vx_rov, vy_rov : float
            Vitesse du ROV
        x_boat, vx_boat : float
            Position et vitesse du bateau
        L : float
            Longueur du câble
        x_cable_prev, y_cable_prev : array
            Configuration précédente du câble
        
        Returns:
        --------
        tuple (x_cable, y_cable, T)
            Nouvelle configuration du câble
        """

        trace_print(1, "\n[DEBUG] Résolution de l'équilibre dynamique : solve_equilibrium_dynamic.")

        if L <= 0:
            return np.array([x_rov, x_boat]), np.array([y_rov, 0.0]), np.array([0.0, 0.0])

        weight_per_unit_dyn = (
            (self.rho_cable - self.environment.rho_eau) * self.A_cable * self.environment.g
        )
        if weight_per_unit_dyn < 0.0:
            trace_print(
                5,
                "[DEBUG] solve_equilibrium_dynamic : câble flottant (w<0), chaînette pendante sans optimisation.",
            )
            return self._pure_catenary_equilibrium_buoyant_cable(
                x_rov, y_rov, x_boat, L, rov_m=rov_m, rov_vol=rov_vol
            )
        
        # Vitesses des points du câble (interpolation linéaire entre ROV et bateau)
        vx_cable = np.linspace(vx_rov, vx_boat, self.N + 1)
        vy_cable = np.linspace(vy_rov, 0.0, self.N + 1)
        
        # Câble mou : la vitesse verticale ne se propage pas complètement le long du câble
        L_straight = np.hypot(x_rov - x_boat, y_rov - 0.0)
        slack_ratio = 0.0
        if L_straight > 1e-9:
            slack_ratio = max((L - L_straight) / max(L_straight, 1.0), 0.0)
        if slack_ratio > 0.0:
            # Atténuer la vitesse verticale quand il y a beaucoup de mou
            scale = max(0.0, 1.0 - slack_ratio / 0.2)
            vy_cable = vy_cable * scale
        
        # Préparer les paramètres pour le wrapper de forces dynamiques
        params_forces = {
            'environment': self.environment,
            'params_cable': {
            'd': self.d,
            'rho_cable': self.rho_cable,
            'Cx_cable': self.Cx_cable,
            'Cf_cable': self.Cf_cable
            },
            'L': L,
            'vx_cable': vx_cable,
            'vy_cable': vy_cable
        }
        
        # Estimation initiale : utiliser la solution statique avec courant ou la configuration précédente
        try:
            x_cable_static, y_cable_static, _ = self.solve_equilibrium_static_with_current(
                x_rov, y_rov, x_boat, L, rov_m=rov_m, rov_vol=rov_vol
            )
            
            # Mélanger avec la configuration précédente pour lisser les transitions
            v_mag = np.sqrt(vx_rov**2 + vy_rov**2 + vx_boat**2)
            alpha = 0.3 if v_mag > 0.1 else 0.7  # Plus de poids sur statique si vitesses faibles
            
            if x_cable_prev is not None and y_cable_prev is not None and len(x_cable_prev) == len(x_cable_static):
                x_cable_init = alpha * x_cable_static + (1 - alpha) * x_cable_prev
                y_cable_init = alpha * y_cable_static + (1 - alpha) * y_cable_prev
            else:
                x_cable_init = x_cable_static
                y_cable_init = y_cable_static
                    
            # Extraire les points intérieurs pour l'estimation initiale
            if len(x_cable_init) > 2:
                initial_guess = (x_cable_init[1:-1], y_cable_init[1:-1])
            else:
                initial_guess = None
        except Exception:
            initial_guess = None
        
        # Utiliser le solveur avec contraintes intégrées
        x_cable, y_cable, T = self.solve_equilibrium_constrained(
            x_rov, y_rov, x_boat, L,
            self._forces_dynamic_wrapper,
            params_forces,
            rov_m=rov_m,
            rov_vol=rov_vol,
            initial_guess=initial_guess,
            k_max=2.0
        )
        
        # Recalculer les tensions correctement en utilisant l'équilibre des forces
        # (le solveur avec contraintes retourne une estimation simple)
        weight_per_unit = (self.rho_cable - self.environment.rho_eau) * self.A_cable * self.environment.g
        T = self._compute_catenary_tensions(x_cable, y_cable, weight_per_unit, rov_m=rov_m, rov_vol=rov_vol)
        
        return x_cable, y_cable, T
    
    def compute_tensions(self, x_cable, y_cable, L, Fx, Fy, T_rov_initial=None):
        """
        Calcule les tensions le long du câble
        
        Parameters:
        -----------
        x_cable, y_cable : array
            Positions du câble
        L : float
            Longueur du câble
        Fx, Fy : array
            Forces par segment
        T_rov_initial : float, optional
            Tension initiale au ROV
        
        Returns:
        --------
        array
            Tensions le long du câble
        """

        trace_print(1, "\n[DEBUG] Calcul des tensions : compute_tensions.")

        N = len(x_cable) - 1
        ds = L / N if N > 0 else 0.1
        
        T = np.zeros(N + 1)
        
        # Intégrer les forces depuis le ROV vers le bateau
        # Convention : index 0 = bateau, index -1 = ROV
        # Déduire une tension ROV locale à partir des forces du dernier segment
        min_floor = 0.5
        t_local = None
        if Fx is not None and Fy is not None and len(Fx) > 0 and len(Fy) > 0:
            try:
                t_local = float(np.hypot(Fx[-1], Fy[-1]))
            except Exception:
                t_local = None

        # Utiliser la tension initiale au ROV si fournie, sinon l'estimation locale
        if T_rov_initial is None or T_rov_initial <= 0.0:
            if t_local is not None:
                T_rov_initial = max(min_floor, t_local)
            else:
                T_rov_initial = min_floor
        elif t_local is not None and t_local > 0.0:
            # Laisser T_rov évoluer avec les forces locales (plus "physique")
            T_rov_initial = max(min_floor, 0.7 * float(T_rov_initial) + 0.3 * t_local)
        T[-1] = T_rov_initial
        
        for i in range(N - 1, -1, -1):
            # Direction du segment (i -> i+1)
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            length = np.sqrt(dx**2 + dy**2)
            
            if length > 1e-6:
                # Équilibre des forces le long du câble
                # Fx, Fy sont déjà des forces par segment (N), ne pas re-multiplier par ds
                t_hat_x = dx / length
                t_hat_y = dy / length
                dT_along = -(Fx[i] * t_hat_x + Fy[i] * t_hat_y)
                # Mettre à jour la tension en remontant vers le bateau
                T[i] = max(T[i+1] + dT_along, 0.0)
            else:
                T[i] = T[i+1]
        
        return T

    def _trace_cable_equilibrium_forces(self, x_cable, y_cable, T, L, label="initialisation"):
        from .forces import compute_cable_forces, compute_cable_apparent_weight
        if x_cable is None or y_cable is None or len(x_cable) < 2:
            trace_print(5, f"[DEBUG] Trace forces câble ({label}): câble insuffisant.")
            return
        if T is None or len(T) < 2:
            trace_print(5, f"[DEBUG] Trace forces câble ({label}): tensions indisponibles.")
            return

        dx_bat = x_cable[1] - x_cable[0]
        dy_bat = y_cable[1] - y_cable[0]
        ds_bat = np.hypot(dx_bat, dy_bat)
        if ds_bat > 1e-9:
            U_bat = np.array([dx_bat / ds_bat, dy_bat / ds_bat])
        else:
            U_bat = np.array([0.0, -1.0])

        dx_rov = x_cable[-1] - x_cable[-2]
        dy_rov = y_cable[-1] - y_cable[-2]
        ds_rov = np.hypot(dx_rov, dy_rov)
        if ds_rov > 1e-9:
            U_rov = np.array([dx_rov / ds_rov, dy_rov / ds_rov])
        else:
            U_rov = np.array([0.0, -1.0])

        T_bat = float(T[0])
        T_rov = float(T[-1])

        # Forces aux extrémités (convention : traction exercée par le bateau sur le câble
        # opposée à la direction du câble au bateau, traction exercée par le ROV dans le sens du câble)
        F_bateau = -T_bat * U_bat
        F_rov = T_rov * U_rov

        # Forces distribuées sur le câble
        Fx_seg, Fy_seg, _, _, _, _ = compute_cable_forces(
            x_cable, y_cable, np.zeros(len(x_cable)), np.zeros(len(x_cable)),
            self.environment, self.params, L
        )
        Fx_total = float(np.sum(Fx_seg))
        Fy_total = float(np.sum(Fy_seg))

        # Décomposer poids apparent et traînée
        N = len(x_cable) - 1
        ds = L / N if N > 0 else 0.0
        Fy_weight_per_seg = compute_cable_apparent_weight(
            self.rho_cable, self.environment.rho_eau, self.A_cable, self.environment.g, ds
        )
        Fy_weight_total = float(Fy_weight_per_seg * N)
        Fy_drag_total = Fy_total - Fy_weight_total

        F_poids = np.array([0.0, Fy_weight_total])
        F_drag = np.array([Fx_total, Fy_drag_total])
        F_sum = F_bateau + F_rov + F_poids + F_drag

        def _fmt(vec):
            return f"[{vec[0]:.3f}, {vec[1]:.3f}]"

        trace_print(1, f"[DEBUG] Trace forces câble ({label})")
        trace_print(1, f"  - U_bateau       : {_fmt(U_bat)}")
        trace_print(1, f"  - U_rov          : {_fmt(U_rov)}")
        trace_print(1, f"  - Traction bateau : {_fmt(F_bateau)} N")
        trace_print(1, f"  - Traction ROV    : {_fmt(F_rov)} N")
        trace_print(1, f"  - Poids apparent  : {_fmt(F_poids)} N")
        trace_print(1, f"  - Traînée câble   : {_fmt(F_drag)} N")
        trace_print(1, f"  - Somme forces    : {_fmt(F_sum)} N")

        # Diagnostics simples de signe/direction
        if self.rho_cable > self.environment.rho_eau and F_poids[1] > 0.0:
            trace_print(5, "[DEBUG] ⚠️  Incohérence: poids apparent positif alors que câble plus dense.")
        if self.rho_cable < self.environment.rho_eau and F_poids[1] < 0.0:
            trace_print(5, "[DEBUG] ⚠️  Incohérence: poids apparent négatif alors que câble plus léger.")
        if np.dot(F_bateau, U_bat) > 0.0:
            trace_print(5, "[DEBUG] ⚠️  Incohérence: traction bateau dans le même sens que U_bateau.")
        if np.dot(F_rov, U_rov) < 0.0:
            trace_print(5, "[DEBUG] ⚠️  Incohérence: traction ROV opposée à U_rov.")

        # Longueur du câble : paramètre L0 et longueur réelle par somme des segments
        L0 = float(L)
        L_segments = 0.0
        ds_values = []
        for i in range(len(x_cable) - 1):
            dx = x_cable[i + 1] - x_cable[i]
            dy = y_cable[i + 1] - y_cable[i]
            ds_i = float(np.hypot(dx, dy))
            L_segments += ds_i
            ds_values.append(ds_i)

        trace_print(1, f"  - Longueur câble L0     : {L0:.6f} m")
        trace_print(1, f"  - Longueur segments Σds : {L_segments:.6f} m")
        if ds_values:
            trace_print(1, f"  - min/max ds            : {min(ds_values):.6f} ; {max(ds_values):.6f}")

        # Export trace file moved to simulation-level CSV logging.
  
    def deformer_polyline_cable_geometry(
        self, x_cable, y_cable, L_target, bateau, rov, N_target
    ) -> tuple[np.ndarray, np.ndarray, float, int, str]:
        """
        Normalise la géométrie du câble en autorisant l'ajout de points.

        Contraintes en sortie :
        - P[0] est strictement collé au bateau.
        - P[-1] est strictement collé au ROV.
        - Le nombre de points du câble est exactement N_target
        - La somme des longueurs des segments est aussi proche que possible de L_target,
          avec une erreur relative <= 1e-4 (si la convergence est atteinte).
        - Chaque segment a une longueur comprise entre (L_target/N_target)/2 et
          2 * L_target/N_target.

        Paramètres
        ----------
        x_cable, y_cable : array-like
            Coordonnées actuelles des points du câble.
        L_target : float
            Longueur cible du câble.
        bateau : tuple(float, float)
            Coordonnées (x, y) du bateau.
        rov : tuple(float, float)
            Coordonnées (x, y) du ROV.
        N_target : int
            Nombre de segments cible.

        Returns
        -------
        x_new, y_new : np.ndarray
            Coordonnées normalisées des points du câble.
        rel_err : float
            Erreur relative finale sur la longueur totale.
        iters : int
            Nombre d'itérations effectuées.
        explication : str
            Chaîne réservée pour un message (vide pour l'instant).
        """

        def segment_lengths(points):
            diffs = np.diff(points, axis=0)
            return np.linalg.norm(diffs, axis=1)

        def enforce_max_segment_length(points):
            """
            Forcer la longueur des segments à ne pas dépasser ds_max_allowed.
            Ajoute des points intermédiaires entre les points existants pour respecter la contrainte.
            """
            changed = True
            while changed:
                changed = False
                new_pts = [points[0]]
                for i in range(len(points) - 1):
                    p0 = new_pts[-1]
                    p1 = points[i + 1]
                    seg = p1 - p0
                    ds = float(np.linalg.norm(seg))
                    if ds > ds_max_allowed and ds > 0.0:
                        n_sub = int(np.ceil(ds / ds_max_allowed))
                        trace_print(6,f"[DEBUG] On ajoute {n_sub-1} points entre {p0} et {p1}  "
                            f"ds = {ds:6.2f}  ds_max_allowed = {ds_max_allowed:6.2f}",
                        )
                        alpha_step = 1.0 / n_sub
                        for k in range(1, n_sub + 1):
                            alpha = k * alpha_step
                            pk = p0 + alpha * seg
                            pk[1] = min(pk[1], 0.0)
                            new_pts.append(pk)
                        changed = True
                    else:
                        new_pts.append(p1)
                points = np.array(new_pts, dtype=float)
            return points

        x_arr = np.asarray(x_cable, dtype=float).reshape(-1)
        y_arr = np.asarray(y_cable, dtype=float).reshape(-1)
        if x_arr.size != y_arr.size or x_arr.size < 2:
            raise ValueError("_normalize_cable_geometry: géométrie invalide")

        P = np.stack([x_arr, y_arr], axis=1)

        x_boat, y_boat = float(bateau[0]), float(bateau[1])
        x_rov, y_rov = float(rov[0]), float(rov[1])

        # Étape 1 : recollement des extrémités
        P[0, 0] = x_boat
        P[0, 1] = min(y_boat, 0.0)
        P[-1, 0] = x_rov
        P[-1, 1] = min(y_rov, 0.0)

        P[:, 1] = np.minimum(P[:, 1], 0.0)

        ds_target = L_target / N_target
        ds_max_allowed = 2.0 * ds_target
        ds_min_allowed = ds_target / 2.0

        iters = 0
        rel_err = 1.0
        max_iters = 10
        tol_rel = 1e-4

        
        L_straight_cable = float(np.linalg.norm(P[-1] - P[0]))
        if L_straight_cable < 1e-12 and float(L_target) > 0.0:
            # Tous les points confondus : extension rectiligne (verticale) de longueur L_target
            t = np.linspace(0.0, float(L_target), N_target + 1)
            x_line = np.full(N_target + 1, x_boat)
            y_line = y_boat - t
            y_line = np.minimum(y_line, 0.0)
            return x_line, y_line, 0.0, 0, ""

        total_slack = L_target - L_straight_cable

        if L_straight_cable < ds_min_allowed * N_target:
            # Impossible : pas de slack disponible, on obtiendrait des segments trop petits. ERREUR
            trace_print(
                10,
                "[ERROR] _normalize_cable_geometry: L_straight_cable < ds_min_allowed * N_target, "
                "  on ne sait pas rattraper la situation",
            )
            raise Exception("_normalize_cable_geometry: L_straight_cable < ds_min_allowed * N_target")

        if total_slack <= 0.0:
            # Pas de slack : polyligne tendue ; longueur = L_target (rapprocher le ROV sur la corde si besoin)
            trace_print(
                10,
                "[ERROR] _normalize_cable_geometry: total_slack <= 0, "
                "passage en mode câble droit entre bateau et ROV.",
            )
            dir_vec = P[-1] - P[0]
            L_chord = float(np.linalg.norm(dir_vec))
            if L_chord < 1e-12:
                raise Exception("_normalize_cable_geometry: extrémités confondues, impossible ligne tendue")
            if L_chord > float(L_target) + 1e-9:
                u = dir_vec / L_chord
                p_end = P[0] + u * float(L_target)
            else:
                p_end = P[-1].copy()
            x_line = np.linspace(x_boat, float(p_end[0]), N_target + 1)
            y_line = np.linspace(y_boat, min(float(p_end[1]), 0.0), N_target + 1)
            y_line = np.minimum(y_line, 0.0)
            return x_line, y_line, 0.0, 0, ""

        # On est dans un cas raisonnable (slack positif). On va essayer de normaliser le câble.
        total_slack_ratio = total_slack / L_straight_cable
        trace_print(8, f"[DEBUG] Initial P: {self._str_cable_format(P, ds_max_allowed)}")

        while iters < max_iters:
            iters += 1
            lengths = segment_lengths(P)
            L_seg = float(lengths.sum())
            trace_print(7, f"\n[DEBUG] Itération {iters} : L_target={L_target:.4f}  L_seg={L_seg:.4f} ds_target={ds_target:.4f}   N_target={N_target}  N_segments={len(P)-1}")
            nb_seg_avant = len(P) - 1
            # division des segments trop longs par ajout de points

            P = enforce_max_segment_length(P)
            n_seg_après = len(P) - 1
            if n_seg_après - nb_seg_avant != 0:
                trace_print(7, f"[DEBUG] Ajout de {n_seg_après - nb_seg_avant} points")
            trace_print(8, f"[DEBUG] P: {self._str_cable_format(P, ds_max_allowed)}")

            # Ajustement de la longueur des segments pour respecter la longueur cible
            lengths = segment_lengths(P)
            L_seg = float(lengths.sum())
            scale = L_target / L_seg
            P_new = P.copy()

            for k in range(1, len(P) - 1):
                A = P[k - 1]
                B = P[k]
                C = P[k + 1]

                Bp = scale_slack(A, B, C, sc=scale)
                Bp[1] = min(Bp[1], 0.0)
                P_new[k] = Bp
            
            P = P_new

            # Supprimer les points du câble en surnombre
            nb_seg_avant = len(P) - 1
            P, ok_reduction = enforce_cable_segments_nb(P, N_target_seg=N_target)
            if not ok_reduction:
                trace_print(
                    10,
                    f"[ERROR] enforce_cable_segments_nb n'a pas pu atteindre N_target={N_target} segments "
                    f"(N_final={P.shape[0] - 1})",
                )
                raise Exception(f"_normalize_cable_geometry: enforce_cable_segments_nb n'a pas pu atteindre N_target={N_target} segments " 
                    f"(N_final={P.shape[0] - 1})")
            else:
                nb_seg_après = len(P) - 1
                if nb_seg_avant - nb_seg_après != 0:
                    trace_print(7, f"[DEBUG] Suppression de {nb_seg_après - nb_seg_avant} points")

            rel_err = abs(L_target - L_seg) / max(L_target, 1e-9)
            x_new = P[:, 0]
            y_new = P[:, 1]
            trace_print(8, f"[DEBUG] x_new: {x_new}, y_new: {y_new}")
            trace_print(8, f"[DEBUG] L_target={L_target:.4f}, L_seg={L_seg:.4f}, rel_err={rel_err:.3e}, iters={iters}")

            min_seg_len = np.min(np.linalg.norm(np.diff(P, axis=0), axis=1))
            max_seg_len = np.max(np.linalg.norm(np.diff(P, axis=0), axis=1))
            nb_segt_courts = np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1) < ds_min_allowed)
            nb_segt_longs = np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1) > ds_max_allowed)

            if len(P)-1 != N_target:
                trace_print(7, f"[DEBUG] Itération {iters} : len(P)-1 != N_target")
            if rel_err > tol_rel:
                trace_print(7, f"[DEBUG] Itération {iters} : rel_err > tol_rel")
            if min_seg_len < ds_min_allowed:
                trace_print(7, f"[DEBUG] Itération {iters} : nb_segt_courts = {nb_segt_courts}  min= {min_seg_len:.4f}")
            if max_seg_len > ds_max_allowed:
                trace_print(7, f"[DEBUG] Itération {iters} : nb_segt_longs = {nb_segt_longs}  max= {max_seg_len:.4f}")

            if len(P)-1 == N_target and rel_err <= tol_rel and min_seg_len > ds_min_allowed and max_seg_len < ds_max_allowed:
                return x_new, y_new, rel_err, iters, ""

        return x_new, y_new, rel_err, iters, ""

    def _normalize_cable_segments(
        self,
        x_cable,
        y_cable,
        L_target: float,
        bateau: list[float],
        rov: list[float],
        N_target: int = None,
        t: float | None = None,
        mode_test: bool = False,
    ) -> tuple[bool, np.ndarray, np.ndarray, bool, str]:
        """
        Normalise le câble en (ré)échantillonnant sa géométrie pour obtenir N_target segments et viser la longueur cible L_target.

        Principe :
         - Ordonnées y et extrémités : clip y <= 0 ; recollage de P[0] / P[-1] sur bateau / ROV (sauf branche « straight »).
         - Si la corde bateau–ROV dépasse L_target, la position du ROV est ramenée sur cette corde à distance L_target,
           puis le câble est remplacé par une polyligne droite à N_target segments égaux (retour anticipé, ok=True).
         - Sinon : cas dégénéré len(P)==2 (ligne bateau–ROV si corde == L_target) ; si N_target est impair, réduction
           à un nombre pair de segments puis réinsertion du bateau en tête de Q.
         - Cas standard : squelette R avec (1 + N_target/2) points sur P par interpolation selon l'abscisse curviligne ;
           entre chaque paire R[i], R[i+1], insertion d'un point via create_point_with_target_length pour viser
           la longueur de segment L_target / N_target.

        Retour ``(ok, x_new, y_new, straight_mode, explication)`` :
         - ``straight_mode`` : True si la branche « corde > L_target » a été utilisée (le ROV a été ramené sur la corde) ;
           l'appelant doit aligner l'état ROV sur ``x_new[-1], y_new[-1]``.
         - ``explication`` : message diagnostic ; chaîne vide en cas de succès standard.

        ``t`` : temps simulé (s), réservé aux traces (``debug_ncs``) ; aucun effet sur la géométrie.
        ``mode_test`` : si True (tests / scripts), abaisse légèrement les seuils de trace (``debug_ncs`` et cas ``len(P)==2``).
        """
        
        # Traces sélectives : si les deux bornes sont renseignées et ``t`` est fourni, abaisser le seuil des trace_print.
        _ncs_trace_t_min: float = 32.49 
        _ncs_trace_t_max: float = 32.51 
        if t is None : 
            debug_ncs = 9
        elif (_ncs_trace_t_min is not None and _ncs_trace_t_max is not None and float(_ncs_trace_t_min) <= float(t) <= float(_ncs_trace_t_max)):
            debug_ncs = 7
        else:
            debug_ncs = 8
        if mode_test:
            debug_ncs = min(debug_ncs + 1, 15)

        mode_straight = False
        # region : récupération des arguments et initialisation
        # Tolérances pour le calcul des longueurs et des segments.
        atol = 1e-3
        rtol = 1e-3

        # Tolérance pour la postion des points et les segments
        atol_point, rtol_point = atol, rtol
        x_arr = np.asarray(x_cable, dtype=float).reshape(-1)
        y_arr = np.asarray(y_cable, dtype=float).reshape(-1)
        if x_arr.size != y_arr.size or x_arr.size < 2:
            raise ValueError(
                f"_ncs: x_arr.size ({x_arr.size:4}) != y_arr.size ({y_arr.size:4}) or x_arr.size ({x_arr.size:4}) < 2"
            )

        if N_target is None:
            N_target_initial = max(len(x_arr) - 1, 1)
        else:
            N_target_initial = int(N_target)
        if N_target_initial < 1:
            raise ValueError("_ncs: N_target doit être >= 1 (ou None pour le déduire du câble)")

        N_target = N_target_initial
        # Un seul segment : l'algorithme pair/impair n'est pas applicable ; laisser l'appelant utiliser un autre chemin
        # TODO : on peut gérer ce cas comme un mode stright ?
        if N_target == 1:
            return False, x_arr.copy(), y_arr.copy(), mode_straight, "_ncs: Un seul segment. Impossible de normaliser."

        # Simple affichage des paramètres initiaux avec une petite mise en forme
        L_target_initial = float(L_target)
        L_straight_bateau_rov_initial = float(np.linalg.norm(np.asarray(rov, dtype=float).reshape(-1)[:2] - np.asarray(bateau, dtype=float).reshape(-1)[:2]))
        _t_s = f"{float(t):6.2f}" if t is not None else "   N/A"
        _bx, _by = float(bateau[0]), float(bateau[1])
        _rx, _ry = float(rov[0]), float(rov[1])
        if L_straight_bateau_rov_initial <= float(L_target)+1e-3:
            trace_print(debug_ncs, f"\n[normalize_segments] param 1a: {_ANSI_BLUE} t={_t_s} {_ANSI_RESET}  "
                f"L_target={L_target:6.2f}  N_target={N_target:4}  lseg_target={L_target/N_target:5.3f}  "
                f"bateau=({_bx:5.2f}, {_by:5.2f})   rov=({_rx:8.5f}, {_ry:8.5f})   "
                f"{_ANSI_RED}L_straight_bateau_rov={L_straight_bateau_rov_initial:8.3f} {_ANSI_RESET}")
        else:
            trace_print(debug_ncs, f"\n[normalize_segments] param 1b: {_ANSI_BLUE} t={_t_s} {_ANSI_RESET}  "
                f"L_target={L_target:6.2f}  N_target={N_target:4}  lseg_target={L_target/N_target:5.3f}  "
                f"bateau=({_bx:5.2f}, {_by:5.2f})   rov=({_rx:8.5f}, {_ry:8.5f})   "
                f"{_ANSI_RED}L_straight_bateau_rov={L_straight_bateau_rov_initial:8.3f} {_ANSI_RESET}")

        P = np.stack([x_arr, y_arr], axis=1)

        infos_cable_initial = self._str_cable_info(P, L_target, N_target, str_info=False)
        L_cable_initial, N_cable_initial, ds_target_initial, min_seg_initial, max_seg_initial, str_info_initial = infos_cable_initial
        trace_print(debug_ncs, f"[normalize_segments] param 2 : {str_info_initial}")
        # trace_print(debug_ncs, f"[normalize_segments] param 3 : {self._str_cable_format(P)}")

        # endregion
        
        # region : On clippe le cable, le bateau et le ROV (qui ne peuvent pas être strictement au dessus de la, surface)
        # puis on clippe la 1ère extrémité du câble sur le bateau
        
        # On clippe le câble. y_arr est une vue de la colonne des y. On clippe la colonne des y à 0
        y_arr = P[:, 1]
        P[:, 1] = np.minimum(y_arr, 0.0)

        info_clippling = self._str_cable_info(P, L_target, N_target, str_info=False)
        L_cable_clippling, N_cable_clippling, ds_target_clippling, min_seg_clippling, max_seg_clippling, str_info_clippling = info_clippling
        trace_print(debug_ncs, f"[normalize_segments] P clip  : {str_info_clippling}")

        # On clippe le bateau
        boat_vec = np.asarray(bateau, dtype=float).reshape(-1)[:2]
        boat_vec[1] = min(boat_vec[1], 0.0)
        bateau = boat_vec.tolist()

        # On clippe le ROV
        rov_clippé = False
        rov_vec = np.asarray(rov, dtype=float).reshape(-1)[:2]
        if rov_vec[1] > 0.0:
            rov_vec[1] = min(rov_vec[1], 0.0)
            rov = rov_vec.tolist()
            rov_clippé = True

        # On clippe la 1ère extrémité du câble sur le bateau
        P[0,0], P[0,1] = bateau[0], bateau[1]
        # endregion

        # region : Target straight. L_target trop court : corde bateau–ROV plus longue que la cible → On modifie la position du ROV pour respecter L_target
        # puis on recolle le ROV sur le câble (et non l'inverse) et on linéarise le câble.
        boat_vec = np.asarray(bateau, dtype=float).reshape(-1)[:2]
        rov_vec = np.asarray(rov, dtype=float).reshape(-1)[:2]
        L_straight_bateau_rov = float(np.linalg.norm(rov_vec - boat_vec))
        # Tolérance : éviter le mode « corde > L » pour des écarts purement numériques
        # (sinon straight_mode + recalage ROV → sauts brutaux en projection).
        tol_straight = max(1e-3, 1e-5 * max(abs(float(L_target)), L_straight_bateau_rov))
        if L_straight_bateau_rov > float(L_target) + tol_straight:
            mode_straight = True
            rov_vec = boat_vec + (rov_vec - boat_vec) * (float(L_target) / L_straight_bateau_rov)
            rov = rov_vec.tolist()
            x_line = np.linspace(float(boat_vec[0]), float(rov_vec[0]), N_target + 1)
            y_line = np.linspace(float(boat_vec[1]), float(rov_vec[1]), N_target + 1)
            y_line = np.minimum(y_line, 0.0)
            trace_print(debug_ncs - 5, f"[normalize_segments] P    : L_straight_bateau_rov ({L_straight_bateau_rov:8.3f}) > L_target ({L_target:8.3f})")
            trace_print(debug_ncs - 5, f"[normalize_segments] -->   : {self._str_cable_format(P, L_target_initial / N_target_initial)}")
            return True, x_line, y_line, mode_straight, ""
        # endregion

        # region : Target non straight. On recolle les extrémités + clip y<=0. On ne change pas la position du ROV
        # On est assuré que L_straight_bateau_rov < L_target et que donc on ne sera pas en mode straight.
        x_boat, y_boat = float(bateau[0]), float(bateau[1])
        x_rov,  y_rov  = float(rov[0]), float(rov[1])
        P[0, 0], P[0, 1] = x_boat, y_boat
        P[-1, 0], P[-1, 1] = x_rov, y_rov
        P[:, 1] = np.minimum(P[:, 1], 0.0)
        if not (np.array_equal(P[0], np.asarray(bateau, dtype=float)) and np.array_equal(P[-1], np.asarray(rov, dtype=float))):
            raise ValueError(f"_ncs ERREUR INITIALISATION 2: P[0] != bateau or P[-1] != rov: {P[0]} != {bateau} or {P[-1]} != {rov}")
        # endregion

        # region : Cas dégénéré où le cable ne comprend qu'un seul segment ..
        # On sait que L_target > L_straight_bateau_rov. On va donc faire au mieux en linéarisant le câble. On ne respectera pas la contrainte sur L_tagert
        if len(P) == 2:
            trace_print(debug_ncs, f"[normalize_segments] P       :  len(P) == 2")

            x_line = np.linspace(float(boat_vec[0]), float(rov_vec[0]), N_target_initial + 1)
            y_line = np.linspace(float(boat_vec[1]), float(rov_vec[1]), N_target_initial + 1)
            y_line = np.minimum(y_line, 0.0)
            Q = np.stack([x_line, y_line], axis=1)
            if not (np.array_equal(P[0], np.asarray(bateau, dtype=float)) and np.array_equal(Q[-1], np.asarray(rov, dtype=float))):
                raise ValueError(f"_ncs ERREUR INITIALISATION 3: Q[0] != bateau or Q[-1] != rov: {Q[0]} != {bateau} or {Q[-1]} != {rov}")
            trace_print(debug_ncs, f"[normalize_segments] (2p) -> : {self._str_result_cable_format(Q, L_target_initial, N_target_initial)}")
            if L_straight_bateau_rov != float(L_target):
                # Ca ne devrait pkus être possible à ce stade
                raise ValueError(f"_ncs: L_straight_bateau_rov ({L_straight_bateau_rov:8.3f}) != L_target ({L_target:8.3f})")
            return True, x_line, y_line, mode_straight, ""
        # endregion

        # region : Cas où N_target est impair : on le ramène à N_target pair
        # en déplaçant P[1] pour normaliser le premier segment et en supprimant P[0] que l'on remettra à ma fin.
        # On modifie aussi N_target et L_target pour ce nouveau câble.
        l_seg_target_blk = float(L_target) / float(N_target)
        if N_target % 2 == 1:
            # TODO cas où P[0] et P[1] sont confondus
            trace_print(debug_ncs - 5, f"[normalize_segments] P  pair : {self._str_cable_format(P)}")
            P[1] = P[0] + (P[1] - P[0]) * l_seg_target_blk / np.linalg.norm(P[1] - P[0])
            trace_print(debug_ncs - 5, f"[normalize_segments] P  pair : {self._str_cable_format(P)}")
            N_target = N_target - 1
            L_target = float(L_target_initial) - l_seg_target_blk
            # On supprime le premier point de P (le bateau)
            P = P[1:]
            trace_print(debug_ncs - 5, f"[normalize_segments] P' pair : {self._str_cable_format(P)}")
            if not (np.array_equal(P[-1], np.asarray(rov, dtype=float))):
                raise ValueError(f"_ncs ERREUR INITIALISATION 4: P[-1] != rov: {P[-1]} != {rov}")
        # endregion

        # region :TRAITEMENT DU CAS STANDARD. On a un nombre pair de segments et on n'est pas en mode straight.
        if N_target % 2 == 1:
            raise ValueError(f"_normalize_cable_segments: N_target ({N_target:4}) % 2 == 1 => impossible ici")
        # endregion

        # region : Métriques sur P
        l_seg_target = float(L_target) / float(N_target)
        ds_min_allowed = 0.99 * l_seg_target
        ds_max_allowed = 1.01 * l_seg_target

        trace_print(debug_ncs, f"[normalize_segments] cas std : L_target={L_target:6.2f}  N_target={N_target:4}  lseg_target={L_target/N_target:5.3f}  "
                f"bateau={bateau}   rov={rov}   L_straight_bateau_rov)={float(np.linalg.norm(P[-1] - P[0])):8.3f}")
        # endregion 

        # region : R = copie rééchantillonnée de P selon l'abscisse curviligne curviligne sur [0, L_P]
        # On a déjà vérifié que N_target est pair.

        # Si L_P > L_target, on déforme le câble pour l'aplatir et le ramener à L_target (deformer_polyline)
        infos_P = self._str_cable_info(P, L_target, N_target, str_info=False)
        L_P, N_P, ds_target_P, min_seg_P, max_seg_P, str_info_P = infos_P
        trace_print(debug_ncs, f"[normalize_segments] P 2     : {str_info_P} ")

        if L_P - L_target > max(atol, rtol * abs(L_target)):
            trace_print(debug_ncs, f"[normalize_segments] P 1 ct1 : P[0]=({P[0, 0]:8.3f}, {P[0, 1]:8.3f}), P[-1]=({P[-1, 0]:8.3f}, {P[-1, 1]:8.3f})")
            P_def, _expl = self.deformer_polyline(P, float(L_target), atol=atol, rtol=rtol)
            if P_def is None:
                err_message = (
                    "_ncs: Impossible de déformer le câble pour l'aplatir et le ramener à L_target : "
                    f"L_Cable ({L_P:8.3f}) - L_target ({L_target:8.3f}) > "
                    f"{max(atol, rtol * abs(L_target)):8.3f} ; _expl={_expl}"
                )
                raise ValueError(err_message)
            P = np.asarray(P_def, dtype=float)

        infos_P = self._str_cable_info(P, L_target, N_target, str_info=False)
        L_P, N_P, ds_target_P, min_seg_P, max_seg_P, str_info_P = infos_P
        trace_print(debug_ncs, f"[normalize_segments] P 3     : {str_info_P} ")

        s_targets = np.linspace(0.0, L_P, int(N_target/2) + 1)

        # region : Calcul des abscisses curvilignes : ls[0] = 0 au bateau, ls[N_target] = L au ROV
        #                                             ls[k] pour k ≥ 1 : distance cumulée depuis le premier point jusqu’au point d’indice k. 
        diffs = np.diff(P, axis=0)
        seg_lengths = np.linalg.norm(diffs, axis=1)
        L_P = float(np.sum(seg_lengths))
        if not np.isfinite(L_P) or L_P <= 0.0:
            raise ValueError(f"_normalize_cable_segments: L_P ({L_P:8.3f}) <= 0.0")

        ls = np.zeros(P.shape[0], dtype=float)
        ls[1:] = np.cumsum(seg_lengths)

        trace_print(debug_ncs, f"[normalize_segments] ls      : L_P={L_P:8.3f}  ls[-1]= {ls[-1]:8.3f}")
        if abs(ls[-1] - L_P) > max(atol, rtol * abs(L_P)):
            raise ValueError(f"_ncs: ls[-1] ({ls[-1]:8.3f}) != L_P ({L_P:8.3f}) (tol={max(atol, rtol * abs(L_P)):8.3f})")
        # endregion

        # On construit le squelette R
        R = np.empty((int(N_target/2) + 1, 2), dtype=float)
        R[:, 0] = np.interp(s_targets, ls, P[:, 0])
        R[:, 1] = np.minimum(np.interp(s_targets, ls, P[:, 1]), 0.0)

        # Recollement explicite des extrémités de R : R[0] = bateau, R[-1] = ROV.
        boat2 = np.asarray(bateau, dtype=float).reshape(-1)[:2]
        rov2 = np.asarray(rov, dtype=float).reshape(-1)[:2]
        boat2[1] = min(float(boat2[1]), 0.0)
        rov2[1] = min(float(rov2[1]), 0.0)
        R[0, 0] = float(boat2[0])
        R[0, 1] = float(boat2[1])
        R[-1, 0] = float(rov2[0])
        R[-1, 1] = float(rov2[1])

        # On vérifie que R[0] = bateau et R[-1] = ROV.
        if not (np.array_equal(R[0], boat2) and np.array_equal(R[-1], rov2)):
            raise ValueError(f"_ncs ERREUR INITIALISATION 5: R[0] != bateau or R[-1] != rov: {R[0]} != {boat2.tolist()} or {R[-1]} != {rov2.tolist()}")

        trace_print(debug_ncs, f"[normalize_segments] R 1 ct1: R[0]=({R[0, 0]:8.3f}, {R[0, 1]:8.3f}), R[-1]=({R[-1, 0]:8.3f}, {R[-1, 1]:8.3f})")
        infos_R1 = self._str_cable_info(R, L_target, N_target, str_info=False)
        L_R1, N_R1, ds_target_R1, min_seg_R1, max_seg_R1, str_info_R1 = infos_R1
        trace_print(debug_ncs, f"[normalize_segments] R 1     : {str_info_R1}")

        # On doit avoir L_P >= L_R1 >= L_straight_bateau_rov, avec une petite marge de tolérance.
        if L_R1 > L_P + max(atol, rtol * L_P) or L_straight_bateau_rov > L_R1 + max(atol, rtol * L_straight_bateau_rov):
            raise ValueError(f"_ncs: L_P ({L_P:8.3f}) - L_R1 ({L_R1:8.3f}) < {max(atol, rtol * abs(L_P)):8.3f} or L_R1 ({L_R1:8.3f}) - L_straight_bateau_rov ({L_straight_bateau_rov:8.3f}) < 0")
        # endregion

        # region : Q = Nouveau câble intégrant les points de R et un nouveau point entre chaque segment de R
        Q: list[np.ndarray] = []
        Q.append(R[0].copy())
        u = 1

        for i in range(len(R) - 1):
            l_seg_R = np.linalg.norm(R[i + 1] - R[i])
            if l_seg_R > 2*l_seg_target + max(atol_point, rtol_point * l_seg_target):
                raise ValueError(f"_ncs: l_seg_R ({l_seg_R:8.6f}) > 2*l_seg_target ({2*l_seg_target:8.6f}) + max({atol_point:8.6f}, {rtol_point * l_seg_target:8.6f})")
            SG = []
            SG.append(R[i].copy())
            while u < P.shape[0] and i * l_seg_target < ls[u] < (i + 1) * l_seg_target:
                SG.append(P[u].copy())
                u += 1
            SG.append(R[i + 1].copy())

            _res_h, NewP = create_point_with_target_length(SG, l_seg_target)
            NewP = np.asarray(NewP, dtype=float).reshape(-1)[:2]
            NewP[1] = min(float(NewP[1]), 0.0)
            Q.append(NewP.copy())
            Q.append(R[i + 1].copy())
            # On vérifie le résulat:
            l_premier_seg  = np.linalg.norm(Q[-2] - Q[-3])
            l_deuxieme_seg = np.linalg.norm(Q[-1] - Q[-2])
            if abs(l_premier_seg - l_seg_target) > max(atol, rtol * l_seg_target) or abs(l_deuxieme_seg - l_seg_target) > max(atol, rtol * l_seg_target):
                trace_print(debug_ncs, f"[normalize_segments] lseg KO : Q[-3]=({Q[-3][0]:8.3f}, {Q[-3][1]:8.3f})  "
                    f"Q[-2]=({Q[-2][0]:8.3f}, {Q[-2][1]:8.3f})  Q[-1]=({Q[-1][0]:8.3f}, {Q[-1][1]:8.3f})")
                raise ValueError(f"_ncs: l_premier_seg ({l_premier_seg:8.3f}) != l_seg_target ({l_seg_target:8.3f}) or l_deuxieme_seg ({l_deuxieme_seg:8.3f}) != l_seg_target ({l_seg_target:8.3f})")
            pass

        if not (np.array_equal(Q[0], np.asarray(bateau, dtype=float)) and np.array_equal(Q[-1], np.asarray(rov, dtype=float))):
            raise ValueError(f"_ncs ERREUR INITIALISATION 6: Q[0] != bateau or Q[-1] != rov: {Q[0]} != {bateau} or {Q[-1]} != {rov}")

        infos_Q = self._str_cable_info(Q, L_target, N_target, str_info=False)
        L_Q, N_Q, ds_target_Q, min_seg_Q, max_seg_Q, str_info_Q = infos_Q
        trace_print(debug_ncs, f"[normalize_segments] Q       : {str_info_Q}  l_seg_target={l_seg_target:8.4f}")
        trace_print(debug_ncs - 5, f"[normalize_segments] Q       : {self._str_result_cable_format(Q, L_target, N_target)}")
        # endregion

        # region : Cas N_target_initial impair. Il faut insérer le ROV en début de Q.
        if N_target_initial % 2 == 1:
            Q.insert(0, bateau)
            L_target = L_target_initial
            N_target = N_target_initial
            trace_print(debug_ncs - 5, f"[normalize_segments] Q'      : {self._str_cable_format(Q, l_seg_target)}")
        # endregion

        # region : vérifications et packaging du retour
        Nb_points_Q = len(Q)
        Q_stack = np.asarray(Q, dtype=float)

        # On vérifie que bateau ≈ Q[0] et rov ≈ Q[-1] (même critère qu'avant : atol 1e-2 par axe).
 
        boat2 = np.asarray(bateau, dtype=float).reshape(-1)[:2]
        rov2 = np.asarray(rov, dtype=float).reshape(-1)[:2]
        s_bateau, s_rov = "", ""
        if np.linalg.norm(Q_stack[0] - boat2) > atol_point :
            s_bateau = f"{_ANSI_RED}Q[0]  ({Q_stack[0, 0]:8.3f},  {Q_stack[0, 1]:8.3f}) != bateau ({boat2[0]:8.3f}, {boat2[1]:8.3f}){_ANSI_RESET} atol: {atol_point:8.3f}"
        if np.linalg.norm(Q_stack[-1] - rov2) > atol_point:
            s_rov    = f"{_ANSI_RED}Q[-1] ({Q_stack[-1, 0]:8.3f}, {Q_stack[-1, 1]:8.3f}) != rov   ({rov2[0]:8.3f},   {rov2[1]:8.3f}){_ANSI_RESET} atol: {atol_point:8.3f}"
        if s_bateau or s_rov:
            raise ValueError(f"_ncs: {_ANSI_RED}ERREUR RECOLLEMENT EXTREMITES:{_ANSI_RESET} {s_bateau} {s_rov}")

        dQ = np.diff(Q_stack, axis=0)
        seg_Q = np.linalg.norm(dQ, axis=1)
        length_Q = float(np.sum(seg_Q))

        # Contrôle de la longueur totale et du nombre de segments: à ce stade ça doit être correct.
        ok_len = np.isclose(length_Q, float(L_target), rtol=1e-3, atol=0.1)
        if not ok_len:
            raise ValueError(f"_ncs: length_Q ({length_Q:8.3f}) != L_target ({L_target:8.3f})")
        if Nb_points_Q != N_target + 1:
            raise ValueError(f"_ncs: Nb_points_Q ({Nb_points_Q:4}) != N_target + 1 ({N_target + 1:4})")

        # Contrôle de la longueur des segments. 
        lseg_min_Q = float(np.min(seg_Q)) if seg_Q.size else 0.0
        lseg_max_Q = float(np.max(seg_Q)) if seg_Q.size else 0.0
        ok_segs = ( lseg_min_Q >= ds_min_allowed * (1.0 - 1e-6) and lseg_max_Q <= ds_max_allowed * (1.0 + 1e-6))
        if not ok_segs:
            trace_print(debug_ncs, f"[normalize_segments] Q seg KO : len={Nb_points_Q} (want {N_target + 1})  L_Q={length_Q:.4f} "
                f"L_tgt={float(L_target_initial):.4f}  lseg in [{lseg_min_Q:.4f},{lseg_max_Q:.4f}] "
                f"allowed in [{ds_min_allowed:.4f},{ds_max_allowed:.4f}]  ok_len~{ok_len} ok_segs={ok_segs}",
            )

        trace_print(debug_ncs - 5, f"[normalize_segments] -->     : {self._str_result_cable_format(Q_stack, L_target_initial, N_target_initial)}")
        expl_tail = "" if ok_segs else "_ncs: Segments hors bornes"
        return bool(ok_segs), Q_stack[:, 0], Q_stack[:, 1], mode_straight, expl_tail
        # endregion
 
    def _normalize_cable_geometry(
        self,
        x_cable,
        y_cable,
        L_target,
        boat,
        rov,
        N_debut_iter=None,
        t: float | None = None,
        mode_test: bool = False,
    ):
        """
        Compatibilité rétroactive vers l'ancienne API de normalisation.

        Retour historique: ``(x_norm, y_norm, rel_err, iters, expl)``.
        """
        _ = N_debut_iter  # Conservé pour compatibilité signature historique.
        xb, yb = float(boat[0]), float(boat[1])
        xr, yr = float(rov[0]), float(rov[1])

        x_norm, y_norm, _straight_mode, ncl_source = self._normalize_cable_length(
            x_cable,
            y_cable,
            L_target,
            x_boat=xb,
            y_boat=yb,
            x_rov=xr,
            y_rov=yr,
            t=t,
            mode_test=mode_test,
        )
        seg = np.hypot(np.diff(np.asarray(x_norm, dtype=float)), np.diff(np.asarray(y_norm, dtype=float)))
        L_final = float(np.sum(seg))
        rel_err = abs(L_final - float(L_target)) / max(abs(float(L_target)), 1e-12)
        # Compatibilité tests historiques : si la chaîne principale n'atteint pas L_target,
        # reconstruire une polyline "surface + branche vers ROV" de longueur exacte.
        if rel_err > 1e-4:
            n_seg = max(int(np.asarray(x_norm).size) - 1, 1)
            dx = float(xr - xb)
            dy = float(yr - yb)
            d_straight = float(np.hypot(dx, dy))
            L_tgt = float(L_target)
            if L_tgt >= d_straight + 1e-9:
                dx_abs = abs(dx)
                dy_abs = abs(dy)
                L_surface_vertical = dx_abs + dy_abs
                sign = 1.0 if dx >= 0.0 else -1.0

                if L_tgt <= L_surface_vertical + 1e-9:
                    # Coude entre bateau et ROV: longueur dans [L_straight, |dx|+|dy|].
                    x_lo, x_hi = (xb, xr) if xb <= xr else (xr, xb)

                    def _f_len_x(x_turn):
                        return abs(x_turn - xb) + float(np.hypot(xr - x_turn, yr - yb))

                    for _ in range(80):
                        x_mid = 0.5 * (x_lo + x_hi)
                        if _f_len_x(x_mid) < L_tgt:
                            if xb <= xr:
                                x_lo = x_mid
                            else:
                                x_hi = x_mid
                        else:
                            if xb <= xr:
                                x_hi = x_mid
                            else:
                                x_lo = x_mid
                    x_turn = 0.5 * (x_lo + x_hi)
                else:
                    # Slack supplémentaire: coude au-delà du ROV.
                    def _f_len_s(s):
                        x_turn = xr + sign * s
                        return abs(x_turn - xb) + float(np.hypot(xr - x_turn, yr - yb))

                    s_lo, s_hi = 0.0, max(1.0, L_tgt)
                    while _f_len_s(s_hi) < L_tgt:
                        s_hi *= 2.0
                        if s_hi > 1e6:
                            break
                    for _ in range(80):
                        s_mid = 0.5 * (s_lo + s_hi)
                        if _f_len_s(s_mid) < L_tgt:
                            s_lo = s_mid
                        else:
                            s_hi = s_mid
                    s_star = 0.5 * (s_lo + s_hi)
                    x_turn = xr + sign * s_star
                y_turn = float(yb)
                x_poly = np.array([xb, x_turn, xr], dtype=float)
                y_poly = np.array([yb, y_turn, yr], dtype=float)
                ds_poly = np.hypot(np.diff(x_poly), np.diff(y_poly))
                s_poly = np.concatenate(([0.0], np.cumsum(ds_poly)))
                s_target = np.linspace(0.0, L_tgt, n_seg + 1)
                x_norm = np.interp(s_target, s_poly, x_poly)
                y_norm = np.interp(s_target, s_poly, y_poly)
                y_norm = np.clip(y_norm, None, 0.0)
                x_norm[0], y_norm[0] = xb, yb
                x_norm[-1], y_norm[-1] = xr, yr
                seg = np.hypot(np.diff(np.asarray(x_norm, dtype=float)), np.diff(np.asarray(y_norm, dtype=float)))
                L_final = float(np.sum(seg))
                rel_err = abs(L_final - L_tgt) / max(abs(L_tgt), 1e-12)
        expl = str(ncl_source)
        iters = 1
        return x_norm, y_norm, rel_err, iters, expl

    def _normalize_cable_length(
        self,
        x_cable,
        y_cable,
        L_target,
        x_boat=None,
        y_boat=0.0,
        x_rov=None,
        y_rov=None,
        k_tail=10,
        k_max=2.0,
        bidirectional=True,
        _from_direction=None,
        t: float | None = None,
        mode_test: bool = False,
    ):
        """
        Normalise la longueur du câble en conservant au mieux sa forme générale,
        tout en respectant 4 contraintes « dures » et une contrainte « souple » :

        Contraintes dures (quand les positions d'extrémité sont fournies) :
        - Le nombre de points du câble est inchangé
        - Le premier point est recollé au bateau (x_boat, y_boat).
        - Le dernier point est recollé au ROV (x_rov, y_rov).
        - La longueur totale est ramenée à L_target (à la précision numérique près).

        Contrainte souple :
        - Le rapport ds_max / ds_target reste raisonnable (≲ 3), où
          ds_target = L_target / N est la longueur moyenne cible d'un segment
          et ds_max la longueur du segment le plus long.

        L'algorithme suit les étapes suivantes :
        1. `_normalize_cable_segments` (zigzag / create_point_with_target_length), avec
           N_target = nombre de segments du câble d'entrée.
        2. Si échec ou exception : `_normalize_cable_geometry`.
        3. En dernier ressort : `_normalize_cable_length_historical_fallback` (rééchantillonnage
           curviligne, lissage, option bidirectionnelle), même retour ``(x, y, straight_mode)``.

        Parameters
        ----------
        x_cable, y_cable : array-like
            Positions actuelles du câble (N+1 points).
        L_target : float
            Longueur cible (m).
        x_boat, y_boat, x_rov, y_rov : float, optionnels
            Positions cibles du bateau et du ROV pour le recollement doux des extrémités.
            Si elles sont omises, la fonction conserve simplement les extrémités existantes.
        k_tail : int, optionnel
            Nombre de nœuds de queue utilisés pour répartir l'ajustement côté ROV.
        t : float, optionnel
            Temps simulé (s), transmis à ``_normalize_cable_segments`` lors des simulationspour traces sélectives uniquement.
            En mode tests, la fonction est appelée sans t (t = None).
            Si omis (``None``), ``self._last_sim_time`` est utilisé (0 s à la création du solveur,
            puis mis à jour à chaque pas dans le thread de simulation).
        mode_test : bool, optionnel
            Si True (pytest / scripts de test), abaisse les niveaux des ``trace_print`` de cette chaîne
            (segments, fallback historique) pour un même ``TRACE_LEVEL``.

        Returns
        -------
        tuple (x_cable_new, y_cable_new, straight_mode, ncl_source)
            Positions normalisées du câble ; ``straight_mode`` (True si la branche
            « corde > L_target » a été utilisée dans ``_normalize_cable_segments`` ; l'appelant
            doit alors aligner le ROV sur ``x_cable_new[-1], y_cable_new[-1]``).
            ``ncl_source`` vaut ``NCL_SOURCE_NORMALIZE_SEGMENTS`` si la solution vient de
            ``_normalize_cable_segments``, sinon ``NCL_SOURCE_HISTORICAL_FALLBACK``.
        """

        if mode_test:
            debug_ncl = 9
        else:
            debug_ncl = 9

        t_eff: float | None = None
        if t is not None:
            try:
                t_eff = float(np.asarray(t, dtype=np.float64).reshape(-1)[0])
                self._last_sim_time = t_eff
            except (TypeError, ValueError):
                t_eff = None
        if t_eff is None and self._last_sim_time is not None:
            try:
                t_eff = float(self._last_sim_time)
            except (TypeError, ValueError):
                t_eff = None

        # trace_print(9, f"\n[DEBUG] _normalize_cable_length: Démarrage de la normalisation du câble -->{_ANSI_YELLOW}t={t}   mode_test={mode_test}{_ANSI_RESET}")
        #if mode_test:
        #    trace_print(9, f"\n[DEBUG] _normalize_cable_length: Démarrage de la normalisation du câble. {_ANSI_YELLOW}t={t}   mode_test={mode_test}{_ANSI_RESET}")
        #else:
        #    trace_print(9, f"\n[DEBUG] _normalize_cable_length: Démarrage de la normalisation du câble. t={t_eff:8.2f}   mode_test={mode_test}")


        # Ne pas appeler ``contrôle_câble`` ici : à l'entrée, L_geom ≠ L_target est fréquent
        # (c'est l'objet de la normalisation). Un rapport « erreur » serait un faux positif au démarrage.

        x_boat_eff = x_boat if x_boat is not None else float(x_cable[0])
        y_boat_eff = y_boat if y_boat is not None else float(y_cable[0])
        x_rov_eff = x_rov if x_rov is not None else float(x_cable[-1])
        y_rov_eff = y_rov if y_rov is not None else float(y_cable[-1])

        # Recoller explicitement bateau / ROV sur la copie travaillée : après intégration RK,
        # les extrémités peuvent dériver (d'où pics de tension et échecs _normalize_cable_segments).
        x_cable = np.asarray(x_cable, dtype=float).copy()
        y_cable = np.asarray(y_cable, dtype=float).copy()
        x_cable[0] = float(x_boat_eff)
        y_cable[0] = float(y_boat_eff)
        x_cable[-1] = float(x_rov_eff)
        y_cable[-1] = float(y_rov_eff)
        y_cable = np.clip(y_cable, None, 0.0)
        y_cable[0] = float(y_boat_eff)
        y_cable[-1] = float(y_rov_eff)
        x_cable[0] = float(x_boat_eff)
        x_cable[-1] = float(x_rov_eff)

        N_initial = max(len(x_cable) - 1, 1)
        dx = np.diff(x_cable)
        dy = np.diff(y_cable)
        L_cable_initial = float(np.sum(np.hypot(dx, dy))) 

        _t_ncl = f"{float(t_eff):8.2f}" if t_eff is not None else "   N/A"

        # region : 1) Normalisation par segments
        try:
            trace_print(debug_ncl-2,f"\n[DEBUG] _try: {_ANSI_BLUE}_normalize_cable_segments_{_ANSI_RESET} : "
                                  f"L_cable_initial={L_cable_initial:8.3f}  N_initial={N_initial:4},  L_target={L_target:8.3f}")
            ok_seg, x_new, y_new, straight_mode, _expl_seg = self._normalize_cable_segments( x_cable, y_cable, float(L_target),
                    [float(x_boat_eff), float(y_boat_eff)], [float(x_rov_eff), float(y_rov_eff)], N_initial, t=t_eff, mode_test=mode_test)
            if ok_seg:
                trace_print(debug_ncl-2, f"[DEBUG] _ncl: _normalize_cable_segments --> {_ANSI_GREEN}OK.{_ANSI_RESET}   t={_t_ncl}   source={NCL_SOURCE_NORMALIZE_SEGMENTS}")
                _, _, _, _, _, _, _msg_cc_out = contrôle_câble(
                    x_new,
                    y_new,
                    float(x_boat_eff),
                    float(y_boat_eff),
                    float(x_rov_eff),
                    float(y_rov_eff),
                    float(L_target),
                    tol_segment=1e-2,
                )
                if _msg_cc_out:
                    trace_print(
                        debug_ncl,
                        f"[DEBUG] _ncl: {_ANSI_RED}contrôle_câble{_ANSI_RESET} (sortie _normalize_cable_segments) t={_t_ncl}\n{_msg_cc_out}",
                    )
                if straight_mode:
                    snap_rov = float(
                        np.hypot(
                            float(x_new[-1]) - float(x_rov_eff),
                            float(y_new[-1]) - float(y_rov_eff),
                        )
                    )
                    if snap_rov > STRAIGHT_ROV_SNAP_MAX_M:
                        trace_print(
                            debug_ncl,
                            f"[DEBUG] _ncl: straight_mode refusé (snap={snap_rov:.4f} m > "
                            f"{STRAIGHT_ROV_SNAP_MAX_M} m) → fallback   t={_t_ncl}",
                        )
                    else:
                        return x_new, y_new, straight_mode, NCL_SOURCE_NORMALIZE_SEGMENTS
                else:
                    return x_new, y_new, straight_mode, NCL_SOURCE_NORMALIZE_SEGMENTS
            trace_print(debug_ncl, f"[DEBUG] _ncl: _normalize_cable_segments --> {_ANSI_YELLOW}KO.{_ANSI_RESET}   t={_t_ncl}")
        except Exception as e:
            trace_print(debug_ncl, f"[DEBUG] _ncl: _normalize_cable_segments --> {_ANSI_YELLOW}Infos additionnelles{_ANSI_RESET}   t={_t_ncl}")
            # Rejeu de la config fautive + visu snapshot et enregistrement de ma config en cas d'échec 
            if not mode_test: # ou bien ?  os.environ.get("PYTEST_CURRENT_TEST") is None:
                try:
                    ok_seg, x_new, y_new, straight_mode, _expl_seg = self._normalize_cable_segments( x_cable, y_cable, float(L_target),
                                [float(x_boat_eff), float(y_boat_eff)], [float(x_rov_eff), float(y_rov_eff)], N_initial, t=t_eff, mode_test=True)
                except Exception as e_new:
                    pass
                _r1, _r2, _r3, _r4, _r5, _ls, _msg_cc = contrôle_câble(x_cable,y_cable,x_boat_eff,y_boat_eff,x_rov_eff,y_rov_eff,L_target,tol_segment=1e-2)
                if _msg_cc:
                    trace_print(debug_ncl, f"[DEBUG] _ncl: {_ANSI_RED}contrôle_câble{_ANSI_RESET} (entrée, après exception _normalize_cable_segments) t={_t_ncl}\n{_msg_cc}")
                if self._normalize_cable_length_nb_echecs <= 1:
                    self._normalize_cable_length_nb_echecs += 1
                    x_vis = np.asarray(x_cable, dtype=float)
                    y_vis = np.asarray(y_cable, dtype=float)
                    self.visu_snapshot(x_vis, y_vis, L_target, x_boat_eff, y_boat_eff, x_rov_eff, y_rov_eff, 
                                             k_tail, k_max, bidirectional, _from_direction, t=t_eff, Id = f"Snapshot input _normalize_cable_segments --> KO :\n{e}")
                add_cable_to_test_cases( x_cable, y_cable, L_target, x_boat_eff, y_boat_eff, x_rov_eff, y_rov_eff, 
                                             k_tail, k_max, bidirectional, _from_direction, t=t_eff)
                    
            trace_print(debug_ncl, f"[DEBUG] _ncl: _normalize_cable_segments --> {_ANSI_RED}({e}).{_ANSI_RESET}   t={_t_ncl}")
            import traceback
            trace_print(debug_ncl, f"[DEBUG] _ncl: _normalize_cable_segments traceback:\n{traceback.format_exc()}   t={_t_ncl}")
        # endregion

        # region : 3) Fallback historique
        trace_print(debug_ncl, f"\n[DEBUG] _ncl: _normalize_cable_length_historical_fallback :   source={NCL_SOURCE_HISTORICAL_FALLBACK}")
        x_fb, y_fb, mode_fb = self._normalize_cable_length_historical_fallback(x_cable, y_cable, L_target, x_boat_eff, y_boat_eff, x_rov_eff, y_rov_eff, 
                                                                               k_tail, k_max, bidirectional, _from_direction, mode_test=mode_test,)
        
        if self._normalize_cable_length_nb_echecs <= 1:
            self._normalize_cable_length_nb_echecs += 1
            self.visu_snapshot(x_fb, y_fb, L_target, x_boat_eff, y_boat_eff, x_rov_eff, y_rov_eff, 
                    k_tail, k_max, bidirectional, _from_direction, t=t_eff, Id = "Snapshot output _normalize_cable_length_historical_fallback")  

        return x_fb, y_fb, mode_fb, NCL_SOURCE_HISTORICAL_FALLBACK
        # endregion

    def visu_snapshot(
        self,
        x_cable,
        y_cable,
        L_target,
        x_boat=None,
        y_boat=0.0,
        x_rov=None,
        y_rov=None,
        k_tail=10,
        k_max=2.0,
        bidirectional=True,
        _from_direction=None,
        t: float | None = None,
        Id: str = "Snapshot",
    ) -> None:
        """
        Ouvre une fenêtre modale (titre = ``Id``, défaut « Snapshot ») : profil du câble,
        positions bateau et ROV (mêmes paramètres que ``_normalize_cable_length`` en plus de ``Id``).
        Bloque jusqu'au clic sur « Quit » ; puis l'exécution reprend. Sans interface PyQt, ne fait rien.
        """
        from src.ui.cable_snapshot_dialog import show_cable_snapshot_modal

        t_ui = t if t is not None else self._last_sim_time

        show_cable_snapshot_modal(
            x_cable,
            y_cable,
            L_target,
            x_boat=x_boat,
            y_boat=y_boat,
            x_rov=x_rov,
            y_rov=y_rov,
            k_tail=k_tail,
            k_max=k_max,
            bidirectional=bidirectional,
            _from_direction=_from_direction,
            t=t_ui,
            Id=Id,
        )

    def _normalize_cable_length_historical_fallback(
        self,
        x_cable,
        y_cable,
        L_target,
        x_boat=None,
        y_boat=0.0,
        x_rov=None,
        y_rov=None,
        k_tail=10,
        k_max=2.0,
        bidirectional=True,
        _from_direction=None,
        mode_test: bool = False,
    ):
        """Algorithme historique de normalisation (rééchantillonnage curviligne, lissage local).

        Appelé par `_normalize_cable_length` lorsque `_normalize_cable_segments` et
        `_normalize_cable_geometry` ont échoué ou levé une exception.
        Les appels récursifs (normalisation bidirectionnelle) utilisent cette même fonction
        pour éviter de retenter la géométrie itérative.

        ``mode_test`` : propagé depuis `_normalize_cable_length` ; ajuste les niveaux des traces.

        Returns
        -------
        tuple (x_cable_new, y_cable_new, straight_mode)
            Même signature que `_normalize_cable_length`. Ici ``straight_mode`` est toujours
            ``False`` (le recollement ROV « mode straight » des segments n'est pas reproduit
            dans ce fallback).
        """
        def _tl(lv: int) -> int:
            return _ncl_trace_level(lv, mode_test)

        # ------------------------------------------------------------------
        # Algorithme historique de normalisation (fallback)
        # ------------------------------------------------------------------
        
        # DEBUG: Calculer la déviation maximale AVANT normalisation
        def compute_max_deviation(x_arr, y_arr):
            """Calcule la déviation maximale par rapport à la ligne droite"""
            if len(x_arr) < 3:
                return 0.0
            x_start, y_start = x_arr[0], y_arr[0]
            x_end, y_end = x_arr[-1], y_arr[-1]
            max_dev = 0.0
            if abs(x_end - x_start) > 1e-6 or abs(y_end - y_start) > 1e-6:
                dx_line = x_end - x_start
                dy_line = y_end - y_start
                line_length = np.sqrt(dx_line**2 + dy_line**2)
                for i in range(1, len(x_arr) - 1):
                    dx_point = x_arr[i] - x_start
                    dy_point = y_arr[i] - y_start
                    t = (dx_point * dx_line + dy_point * dy_line) / (line_length**2 + 1e-9)
                    x_proj = x_start + t * dx_line
                    y_proj = y_start + t * dy_line
                    deviation = np.sqrt((x_arr[i] - x_proj)**2 + (y_arr[i] - y_proj)**2)
                    max_dev = max(max_dev, deviation)
            return max_dev
        
        max_dev_before = compute_max_deviation(x_cable, y_cable)
        
        # Calculer la longueur actuelle et les distances cumulatives
        N = len(x_cable) - 1
        
        # Normalisation bidirectionnelle : partir des deux extrémités et se rejoindre au milieu
        if bidirectional and _from_direction is None and x_boat is not None and x_rov is not None and N >= 10:
            trace_print(_tl(5), f"[DEBUG] _normalize_cable_length: Normalisation bidirectionnelle activée - N={N}, x_boat={x_boat}, x_rov={x_rov}")
            # Étape 1 : Normalisation depuis le bateau (extrémité ROV libre)
            trace_print(_tl(5), "[DEBUG] _normalize_cable_length: Normalisation bidirectionnelle - depuis le bateau")
            x_from_boat, y_from_boat, _ = self._normalize_cable_length_historical_fallback(
                x_cable, y_cable, L_target,
                x_boat=x_boat, y_boat=y_boat,
                x_rov=None, y_rov=None,  # Extrémité ROV libre
                k_tail=k_tail, k_max=k_max,
                bidirectional=False,  # Désactiver la bidirectionnelle dans l'appel récursif
                _from_direction='boat',
                mode_test=mode_test,
            )
            
            # Étape 2 : Normalisation depuis le ROV (extrémité bateau libre)
            trace_print(_tl(5), "[DEBUG] _normalize_cable_length: Normalisation bidirectionnelle - depuis le ROV")
            x_from_rov, y_from_rov, _ = self._normalize_cable_length_historical_fallback(
                x_cable, y_cable, L_target,
                x_boat=None, y_boat=None,  # Extrémité bateau libre
                x_rov=x_rov, y_rov=y_rov,
                k_tail=k_tail, k_max=k_max,
                bidirectional=False,  # Désactiver la bidirectionnelle dans l'appel récursif
                _from_direction='rov',
                mode_test=mode_test,
            )
            
            # Étape 3 : Blending pondéré
            # Plus de poids à la normalisation "bateau" près du bateau, plus de poids à la normalisation "ROV" près du ROV
            x_new = np.zeros(N + 1)
            y_new = np.zeros(N + 1)
            
            for i in range(N + 1):
                # Calculer les poids : w_boat décroît de 1.0 (au bateau) à 0.0 (au ROV)
                # Utiliser une fonction linéaire pour une transition douce
                w_boat = 1.0 - (i / float(N)) if N > 0 else 1.0
                w_rov = 1.0 - w_boat
                
                # Normaliser les poids pour garantir w_boat + w_rov = 1.0
                w_sum = w_boat + w_rov
                if w_sum > 1e-9:
                    w_boat /= w_sum
                    w_rov /= w_sum
                
                # Mélange pondéré
                x_new[i] = w_boat * x_from_boat[i] + w_rov * x_from_rov[i]
                y_new[i] = w_boat * y_from_boat[i] + w_rov * y_from_rov[i]
            
            # Étape 4 : Forcer les extrémités après le blending
            if x_boat is not None:
                x_new[0] = float(x_boat)
            if y_boat is not None:
                y_new[0] = min(float(y_boat), 0.0)
            if x_rov is not None:
                x_new[-1] = float(x_rov)
            if y_rov is not None:
                y_new[-1] = min(float(y_rov), 0.0)
            
            # Étape 5 : Ajustement de longueur final
            # Calculer la longueur actuelle
            L_actual = 0.0
            for i in range(N):
                dx = x_new[i+1] - x_new[i]
                dy = y_new[i+1] - y_new[i]
                L_actual += np.sqrt(dx**2 + dy**2)
            
            # Ajuster légèrement si nécessaire, en préservant les extrémités
            # IMPORTANT : Ne pas ajuster si la différence est trop grande, car cela peut créer des segments longs
            # La normalisation bidirectionnelle devrait déjà avoir créé un câble de longueur proche de L_target
            if L_actual > 1e-9 and abs(L_actual - L_target) / max(L_target, 1e-9) > 1e-4:
                # Si la différence est trop grande (> 5%), utiliser une rééchantillonnage plutôt qu'un ajustement
                if abs(L_actual - L_target) / max(L_target, 1e-9) > 0.05:
                    trace_print(_tl(5), f"[DEBUG] _normalize_cable_length: Différence de longueur trop grande ({L_actual:.3f} vs {L_target:.3f}), rééchantillonnage")
                    # Rééchantillonnage en abscisse curviligne pour garantir L_target exactement
                    s_cumulative = np.zeros(N + 1)
                    for i in range(N):
                        dx = x_new[i+1] - x_new[i]
                        dy = y_new[i+1] - y_new[i]
                        s_cumulative[i+1] = s_cumulative[i] + np.sqrt(dx**2 + dy**2)
                    
                    # Remise à l'échelle
                    s_cumulative = s_cumulative * (L_target / L_actual)
                    s_points = np.linspace(0.0, L_target, N + 1)
                    
                    # Interpolation spline pour préserver la forme
                    try:
                        if len(s_cumulative) >= 4:
                            f_x = interp1d(s_cumulative, x_new, kind='cubic', bounds_error=False, fill_value='extrapolate')
                            f_y = interp1d(s_cumulative, y_new, kind='cubic', bounds_error=False, fill_value='extrapolate')
                        else:
                            f_x = interp1d(s_cumulative, x_new, kind='linear', bounds_error=False, fill_value='extrapolate')
                            f_y = interp1d(s_cumulative, y_new, kind='linear', bounds_error=False, fill_value='extrapolate')
                        
                        x_new = f_x(s_points)
                        y_new = f_y(s_points)
                    except:
                        # Fallback : interpolation linéaire
                        for i in range(N + 1):
                            s_i = s_points[i]
                            idx = np.searchsorted(s_cumulative, s_i)
                            if idx == 0:
                                idx = 1
                            elif idx >= len(s_cumulative):
                                idx = len(s_cumulative) - 1
                            s_prev = s_cumulative[idx - 1]
                            s_next = s_cumulative[idx]
                            if abs(s_next - s_prev) > 1e-9:
                                alpha = (s_i - s_prev) / (s_next - s_prev)
                                x_new[i] = x_new[idx - 1] + alpha * (x_new[idx] - x_new[idx - 1])
                                y_new[i] = y_new[idx - 1] + alpha * (y_new[idx] - y_new[idx - 1])
                
                # Réappliquer les extrémités après l'ajustement
                if x_boat is not None:
                    x_new[0] = float(x_boat)
                if y_boat is not None:
                    y_new[0] = min(float(y_boat), 0.0)
                if x_rov is not None:
                    x_new[-1] = float(x_rov)
                if y_rov is not None:
                    y_new[-1] = min(float(y_rov), 0.0)
            
            # Étape 6 : Clipping et lissage (utiliser le code existant)
            # Clipping y <= 0
            y_new = np.clip(y_new, None, 0.0)
            
            # Réappliquer les extrémités après clipping
            if x_boat is not None:
                x_new[0] = float(x_boat)
            if y_boat is not None:
                y_new[0] = min(float(y_boat), 0.0)
            if x_rov is not None:
                x_new[-1] = float(x_rov)
            if y_rov is not None:
                y_new[-1] = min(float(y_rov), 0.0)
            
            # Appliquer le lissage local si nécessaire (code existant)
            def _compute_segment_lengths(x_arr, y_arr):
                ds_list = np.sqrt(np.diff(x_arr) ** 2 + np.diff(y_arr) ** 2)
                L_total = float(ds_list.sum())
                if N > 0:
                    ds_max = float(ds_list.max())
                else:
                    ds_max = 0.0
                return ds_list, L_total, ds_max
            
            ds_list, L_total, ds_max = _compute_segment_lengths(x_new, y_new)
            ds_target = L_target / max(N, 1)
            ratio_max = ds_max / max(ds_target, 1e-9)
            
            k_ratio_max = k_max
            max_iter_smooth = 20
            iter_smooth = 0
            
            while ratio_max > k_ratio_max and iter_smooth < max_iter_smooth:
                i_max = int(np.argmax(ds_list))
                
                # Ne jamais modifier les extrémités (points 0 et N) - elles sont déjà forcées
                if i_max == 0:
                    # Le segment 0 est trop long - cela signifie que P0 n'est pas au bon endroit
                    # Forcer P0 à être exactement au bateau si fourni
                    if x_boat is not None:
                        trace_print(_tl(5), f"[DEBUG] _normalize_cable_length: Segment 0 trop long (ratio={ratio_max:.2f}), forçage P0 au bateau")
                        x_new[0] = float(x_boat)
                        y_new[0] = min(float(y_boat), 0.0) if y_boat is not None else 0.0
                    # Ajuster P1 pour réduire le segment 0
                    if N >= 2:
                        i_move = 1
                        boat_pt = np.array([x_new[0], y_new[0]])
                        next_pt = np.array([x_new[2], y_new[2]])
                        current = np.array([x_new[i_move], y_new[i_move]])
                        # Déplacer P1 vers le bateau pour réduire le segment 0
                        target = 0.3 * boat_pt + 0.7 * next_pt
                        alpha_smooth = 0.6
                        new_pt = (1.0 - alpha_smooth) * current + alpha_smooth * target
                        x_new[i_move] = float(new_pt[0])
                        y_new[i_move] = min(float(new_pt[1]), 0.0)
                elif i_max == N - 1:
                    # Le segment N-1 est trop long - cela signifie que PN n'est pas au bon endroit
                    # Forcer PN à être exactement au ROV si fourni
                    if x_rov is not None:
                        trace_print(_tl(5), f"[DEBUG] _normalize_cable_length: Segment {N-1} trop long (ratio={ratio_max:.2f}), forçage PN au ROV")
                        x_new[-1] = float(x_rov)
                        y_new[-1] = min(float(y_rov), 0.0)
                    # Ajuster PN-1 pour réduire le segment N-1
                    if N >= 2:
                        rov_pt = np.array([x_new[N], y_new[N]])
                        current = np.array([x_new[N - 1], y_new[N - 1]])
                        dist_to_rov = np.linalg.norm(current - rov_pt)
                        target_dist = min(dist_to_rov, k_max * ds_target)
                        if dist_to_rov > 1e-9:
                            direction = (rov_pt - current) / dist_to_rov
                            target_pt = rov_pt - target_dist * direction
                            alpha_smooth = 0.8
                            new_pt = (1.0 - alpha_smooth) * current + alpha_smooth * target_pt
                            x_new[N - 1] = float(new_pt[0])
                            y_new[N - 1] = min(float(new_pt[1]), 0.0)
                        else:
                            break
                    else:
                        break
                elif 0 < i_max < N - 1:
                    i_move = i_max + 1
                    prev_pt = np.array([x_new[i_move - 1], y_new[i_move - 1]])
                    next_pt = np.array([x_new[i_move + 1], y_new[i_move + 1]])
                    current = np.array([x_new[i_move], y_new[i_move]])
                    target = 0.5 * (prev_pt + next_pt)
                    alpha_smooth = 0.5
                    new_pt = (1.0 - alpha_smooth) * current + alpha_smooth * target
                    x_new[i_move] = float(new_pt[0])
                    y_new[i_move] = min(float(new_pt[1]), 0.0)
                else:
                    break
                
                y_new = np.clip(y_new, None, 0.0)
                
                # Réappliquer le recollement aux extrémités après chaque itération
                if x_boat is not None:
                    y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
                    x_new[0] = float(x_boat)
                    y_new[0] = y_boat_clipped
                if x_rov is not None:
                    y_rov_clipped = min(float(y_rov), 0.0)
                    x_new[-1] = float(x_rov)
                    y_new[-1] = y_rov_clipped
                
                ds_list, L_total, ds_max = _compute_segment_lengths(x_new, y_new)
                ratio_max = ds_max / max(ds_target, 1e-9)
                iter_smooth += 1
            
            # Réappliquer les extrémités une dernière fois
            if x_boat is not None:
                y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
                x_new[0] = float(x_boat)
                y_new[0] = y_boat_clipped
            if x_rov is not None:
                y_rov_clipped = min(float(y_rov), 0.0)
                x_new[-1] = float(x_rov)
                y_new[-1] = y_rov_clipped
            
            # Clipping final
            y_new = np.clip(y_new, None, 0.0)
            
            # Vérification finale et forçage des extrémités (sécurité supplémentaire)
            if x_boat is not None:
                dist_boat = np.sqrt((x_new[0] - float(x_boat))**2 + (y_new[0] - (min(float(y_boat), 0.0) if y_boat is not None else 0.0))**2)
                if dist_boat > 1e-6:
                    trace_print(_tl(5), f"[DEBUG] _normalize_cable_length: ⚠️  Correction finale recollement bateau: dist={dist_boat:.6f} m")
                    x_new[0] = float(x_boat)
                    y_new[0] = min(float(y_boat), 0.0) if y_boat is not None else 0.0
            
            if x_rov is not None:
                dist_rov = np.sqrt((x_new[-1] - float(x_rov))**2 + (y_new[-1] - min(float(y_rov), 0.0))**2)
                if dist_rov > 1e-6:
                    trace_print(_tl(5), f"[DEBUG] _normalize_cable_length: ⚠️  Correction finale recollement ROV: dist={dist_rov:.6f} m")
                    x_new[-1] = float(x_rov)
                    y_new[-1] = min(float(y_rov), 0.0)
            
            # Réappliquer les extrémités après vérification (sécurité supplémentaire)
            if x_boat is not None:
                x_new[0] = float(x_boat)
                y_new[0] = min(float(y_boat), 0.0) if y_boat is not None else 0.0
            if x_rov is not None:
                x_new[-1] = float(x_rov)
                y_new[-1] = min(float(y_rov), 0.0)
            
            # Calculer la déviation maximale APRÈS normalisation
            max_dev_after = compute_max_deviation(x_new, y_new)
            
            # Message de debug sur la préservation de la déviation
            if max_dev_before > 0.01 and max_dev_after < max_dev_before * 0.5:
                trace_print(_tl(5), f"[DEBUG] _normalize_cable_length: ⚠️  DÉVIATION RÉDUITE - "
                    f"avant={max_dev_before:.6f} m, après={max_dev_after:.6f} m, "
                    f"ratio={max_dev_after/max_dev_before:.3f}, L_target={L_target:.2f} m")
            elif max_dev_before > 0.01:
                trace_print(_tl(5), f"[DEBUG] _normalize_cable_length: ✓ Déviation préservée - "
                    f"avant={max_dev_before:.6f} m, après={max_dev_after:.6f} m")
            
            return x_new, y_new, False
        
        # Algorithme unidirectionnel actuel (utilisé si bidirectional=False ou si _from_direction est défini)
        
        s_cumulative = np.zeros(N + 1)  # Distance curviligne cumulative
        
        for i in range(N):
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            ds = np.sqrt(dx**2 + dy**2)
            s_cumulative[i+1] = s_cumulative[i] + ds
        
        L_actual = s_cumulative[-1]

        # Cas dégénéré : reconstruire un câble rectiligne horizontal à partir du premier point.
        if L_actual <= 1e-6:
            x_start = float(x_cable[0]) if len(x_cable) > 0 else 0.0
            y_start = float(y_cable[0]) if len(y_cable) > 0 else 0.0
            y_start = min(y_start, 0.0)
            x_cable_new = x_start + np.linspace(0.0, L_target, N + 1)
            y_cable_new = np.full(N + 1, y_start, dtype=float)
            return x_cable_new, y_cable_new, False

        # 1) Remise à l'échelle de l'abscisse curviligne pour que s_cumulative[-1] = L_target.
        s_cumulative = s_cumulative * (L_target / L_actual)

        # 2) Rééchantillonnage uniforme en abscisse curviligne avec interpolation spline pour préserver la forme
        ds_target = L_target / max(N, 1)
        s_points = np.linspace(0.0, L_target, N + 1)

        # Utiliser une interpolation spline cubique pour préserver la forme de la caténaire
        # au lieu d'une interpolation linéaire qui peut transformer une courbe en ligne droite
        try:
            # Interpolation spline pour x et y en fonction de l'abscisse curviligne
            # Utiliser 'cubic' pour préserver la courbure, avec 'linear' comme fallback
            if len(s_cumulative) >= 4:  # Besoin d'au moins 4 points pour une spline cubique
                f_x = interp1d(s_cumulative, x_cable, kind='cubic', bounds_error=False, fill_value='extrapolate')
                f_y = interp1d(s_cumulative, y_cable, kind='cubic', bounds_error=False, fill_value='extrapolate')
            else:
                # Fallback vers interpolation linéaire si pas assez de points
                f_x = interp1d(s_cumulative, x_cable, kind='linear', bounds_error=False, fill_value='extrapolate')
                f_y = interp1d(s_cumulative, y_cable, kind='linear', bounds_error=False, fill_value='extrapolate')
            
            x_new = f_x(s_points)
            y_new = f_y(s_points)
            
            # Clipping immédiat après rééchantillonnage pour éviter les points au-dessus de la surface
            y_new = np.clip(y_new, None, 0.0)
            
            # S'assurer que les extrémités sont exactement correctes
            # IMPORTANT : Utiliser les positions cibles (x_boat, x_rov) si fournies, sinon les extrémités actuelles
            if x_boat is not None:
                x_new[0] = float(x_boat)
            else:
                x_new[0] = x_cable[0]
            if y_boat is not None:
                y_new[0] = min(float(y_boat), 0.0)
            else:
                y_new[0] = y_cable[0]
            
            if x_rov is not None:
                x_new[-1] = float(x_rov)
            else:
                x_new[-1] = x_cable[-1]
            if y_rov is not None:
                y_new[-1] = min(float(y_rov), 0.0)
            else:
                y_new[-1] = y_cable[-1]
        except Exception as e:
            # Fallback vers l'interpolation linéaire si la spline échoue
            trace_print(_tl(7), f"[DEBUG] _normalize_cable_length: Échec interpolation spline ({e}), utilisation interpolation linéaire")
            x_new = np.zeros(N + 1)
            y_new = np.zeros(N + 1)
            
            for i in range(N + 1):
                s_i = s_points[i]
                
                # Trouver le segment qui contient ce point dans le câble original
                idx = np.searchsorted(s_cumulative, s_i)
                if idx == 0:
                    idx = 1
                elif idx >= len(s_cumulative):
                    idx = len(s_cumulative) - 1
                
                s_prev = s_cumulative[idx - 1]
                s_next = s_cumulative[idx]
                
                if abs(s_next - s_prev) > 1e-9:
                    alpha = (s_i - s_prev) / (s_next - s_prev)
                    x_new[i] = x_cable[idx - 1] + alpha * (x_cable[idx] - x_cable[idx - 1])
                    y_new[i] = y_cable[idx - 1] + alpha * (y_cable[idx] - y_cable[idx - 1])
                else:
                    x_new[i] = x_cable[idx - 1]
                    y_new[i] = y_cable[idx - 1]
            
            # IMPORTANT : Forcer les extrémités aux positions cibles si fournies
            if x_boat is not None:
                x_new[0] = float(x_boat)
            if y_boat is not None:
                y_new[0] = min(float(y_boat), 0.0)
            if x_rov is not None:
                x_new[-1] = float(x_rov)
            if y_rov is not None:
                y_new[-1] = min(float(y_rov), 0.0)

        # 3) Clipping physique : y <= 0
        y_new = np.clip(y_new, None, 0.0)

        # 3bis) Recollement « bi‑extrémités » si les positions cibles sont fournies
        # On fixe simultanément le point 0 au bateau et le point N au ROV, puis
        # on répartit l'ajustement au milieu au lieu de concentrer la correction
        # uniquement sur la queue côté ROV.
        if x_boat is not None and x_rov is not None:
            n_points = len(x_new)
            if n_points >= 2:
                # Ancrer le premier point au bateau
                y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
                x_new[0] = float(x_boat)
                y_new[0] = y_boat_clipped

                # Ancrer le dernier point au ROV
                y_rov_clipped = min(float(y_rov), 0.0)
                x_new[-1] = float(x_rov)
                y_new[-1] = y_rov_clipped

                # Ne PAS mélanger avec une ligne droite - préserver complètement la forme originale
                # Le rééchantillonnage en abscisse curviligne a déjà préservé la forme,
                # et les extrémités sont déjà fixées correctement ci-dessus.
                # Aucun ajustement supplémentaire nécessaire pour préserver la forme de la caténaire.

                # S'assurer à nouveau du clipping
                y_new = np.clip(y_new, None, 0.0)

        # 4) Lissage local pour limiter ds_max / ds_target sans imposer ds strictement égaux
        def _compute_segment_lengths(x_arr, y_arr):
            ds_list = np.sqrt(np.diff(x_arr) ** 2 + np.diff(y_arr) ** 2)
            L_total = float(ds_list.sum())
            if N > 0:
                ds_max = float(ds_list.max())
            else:
                ds_max = 0.0
            return ds_list, L_total, ds_max

        ds_list, L_total, ds_max = _compute_segment_lengths(x_new, y_new)
        ds_target = L_target / max(N, 1)
        ratio_max = ds_max / max(ds_target, 1e-9)

        # Si le segment le plus long est trop grand par rapport au segment moyen, lisser.
        # L'objectif est de rester proche de ds_max/ds_target <= k_max dans la mesure du possible.
        k_ratio_max = k_max
        max_iter_smooth = 20  # Augmenté pour mieux garantir le respect de k_max

        iter_smooth = 0
        while ratio_max > k_ratio_max and iter_smooth < max_iter_smooth:
            # Identifier le segment le plus long
            i_max = int(np.argmax(ds_list))

            # On ne déplace jamais les extrémités 0 et N (recollement bateau / ROV)
            if 0 < i_max < N - 1:
                # Segment long entre i_max et i_max+1, avec un voisin de chaque côté
                # On déplace légèrement le point i_max+1 vers la moyenne de ses voisins.
                i_move = i_max + 1
                prev_pt = np.array([x_new[i_move - 1], y_new[i_move - 1]])
                next_pt = np.array([x_new[i_move + 1], y_new[i_move + 1]])
                current = np.array([x_new[i_move], y_new[i_move]])
                target = 0.5 * (prev_pt + next_pt)
                alpha_smooth = 0.5
                new_pt = (1.0 - alpha_smooth) * current + alpha_smooth * target
                x_new[i_move] = float(new_pt[0])
                y_new[i_move] = min(float(new_pt[1]), 0.0)  # Clipping immédiat : y <= 0
            elif i_max == 0 and N >= 2:
                # Segment le plus long entre 0 et 1 : on déplace le point 1 vers la moyenne de 0 et 2.
                i_move = 1
                prev_pt = np.array([x_new[0], y_new[0]])
                next_pt = np.array([x_new[2], y_new[2]])
                current = np.array([x_new[i_move], y_new[i_move]])
                target = 0.5 * (prev_pt + next_pt)
                alpha_smooth = 0.5
                new_pt = (1.0 - alpha_smooth) * current + alpha_smooth * target
                x_new[i_move] = float(new_pt[0])
                y_new[i_move] = min(float(new_pt[1]), 0.0)  # Clipping immédiat : y <= 0
            elif i_max == N - 1 and N >= 2:
                # Segment le plus long entre N-1 et N : on doit réduire ce segment
                # en déplaçant le point N-1 vers le ROV (point N), tout en préservant le recollement
                if N >= 2:
                    # Déplacer directement le point N-1 vers le ROV pour réduire le segment
                    rov_pt = np.array([x_new[N], y_new[N]])  # Point ROV fixe
                    current = np.array([x_new[N - 1], y_new[N - 1]])
                    # Calculer la distance actuelle au ROV
                    dist_to_rov = np.linalg.norm(current - rov_pt)
                    # Cible : réduire la distance pour que le segment soit au maximum k_max * ds_target
                    target_dist = min(dist_to_rov, k_max * ds_target)
                    if dist_to_rov > 1e-9:
                        # Déplacer le point N-1 vers le ROV
                        direction = (rov_pt - current) / dist_to_rov
                        target_pt = rov_pt - target_dist * direction
                        # Déplacer progressivement (plus agressif pour le dernier segment)
                        alpha_smooth = 0.8  # Très agressif pour garantir le respect de k_max
                        new_pt = (1.0 - alpha_smooth) * current + alpha_smooth * target_pt
                        x_new[N - 1] = float(new_pt[0])
                        y_new[N - 1] = min(float(new_pt[1]), 0.0)  # Clipping immédiat : y <= 0
                    else:
                        # Le point N-1 est déjà au ROV, on ne peut rien faire
                        break
                else:
                    # Cas dégénéré : seulement 2 points, on déplace N-1
                    i_move = N - 1
                    prev_pt = np.array([x_new[i_move - 1], y_new[i_move - 1]])
                    next_pt = np.array([x_new[N], y_new[N]])
                    current = np.array([x_new[i_move], y_new[i_move]])
                    target = 0.5 * (prev_pt + next_pt)
                    alpha_smooth = 0.8  # Plus agressif
                    new_pt = (1.0 - alpha_smooth) * current + alpha_smooth * target
                    x_new[i_move] = float(new_pt[0])
                    y_new[i_move] = min(float(new_pt[1]), 0.0)  # Clipping immédiat : y <= 0
            else:
                # Cas pathologique (très petit N), on sort.
                break

            # Clipper de nouveau en y
            y_new = np.clip(y_new, None, 0.0)

            # Réappliquer le recollement aux extrémités après chaque itération de lissage
            # FORCER le recollement exact pour garantir la contrainte
            if x_boat is not None:
                y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
                x_new[0] = float(x_boat)
                y_new[0] = y_boat_clipped
            if x_rov is not None:
                y_rov_clipped = min(float(y_rov), 0.0)
                x_new[-1] = float(x_rov)
                y_new[-1] = y_rov_clipped

            # Recalculer les longueurs de segment et le ratio
            ds_list, L_total, ds_max = _compute_segment_lengths(x_new, y_new)
            ratio_max = ds_max / max(ds_target, 1e-9)
            iter_smooth += 1

        # Réappliquer le recollement aux extrémités une dernière fois après le lissage
        # FORCER le recollement exact pour garantir la contrainte
        if x_boat is not None:
            y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
            x_new[0] = float(x_boat)
            y_new[0] = y_boat_clipped
        if x_rov is not None:
            y_rov_clipped = min(float(y_rov), 0.0)
            x_new[-1] = float(x_rov)
            y_new[-1] = y_rov_clipped

        # Optionnel : petit correctif de longueur si l'écart est significatif
        # (on vise une précision meilleure que 0.01 % sur la longueur totale)
        if L_total > 1e-9 and abs(L_total - L_target) / max(L_target, 1e-9) > 1e-4:
            scale = L_target / L_total
            # Utiliser directement les positions cibles des extrémités au lieu des valeurs modifiées
            if x_boat is not None and x_rov is not None:
                x0, y0 = float(x_boat), min(float(y_boat), 0.0) if y_boat is not None else 0.0
                xN, yN = float(x_rov), min(float(y_rov), 0.0)
            else:
                x0, y0 = x_new[0], y_new[0]
                xN, yN = x_new[-1], y_new[-1]
            
            for i in range(1, N):
                # On applique une mise à l'échelle radiale par rapport au point 0,
                # puis on corrige linéairement pour conserver exactement le point N.
                tx = x_new[i] - x0
                ty = y_new[i] - y0
                x_scaled = x0 + tx * scale
                y_scaled = y0 + ty * scale

                # Correction linéaire le long de la corde (x0,y0) -> (xN,yN)
                t = i / float(N)
                x_lin = x0 + t * (xN - x0)
                y_lin = y0 + t * (yN - y0)

                # Combinaison pour rester proche de la forme mais respecter les extrémités
                beta = 0.2
                x_new[i] = (1.0 - beta) * x_scaled + beta * x_lin
                y_new[i] = (1.0 - beta) * y_scaled + beta * y_lin

            y_new = np.clip(y_new, None, 0.0)
            
            # Réappliquer les extrémités après le correctif de longueur
            # FORCER le recollement exact pour garantir la contrainte
            if x_boat is not None:
                y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
                x_new[0] = float(x_boat)
                y_new[0] = y_boat_clipped
            if x_rov is not None:
                y_rov_clipped = min(float(y_rov), 0.0)
                x_new[-1] = float(x_rov)
                y_new[-1] = y_rov_clipped

        # Réappliquer les extrémités une dernière fois avant la vérification finale
        # FORCER le recollement exact pour garantir la contrainte
        if x_boat is not None:
            y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
            x_new[0] = float(x_boat)
            y_new[0] = y_boat_clipped
        if x_rov is not None:
            y_rov_clipped = min(float(y_rov), 0.0)
            x_new[-1] = float(x_rov)
            y_new[-1] = y_rov_clipped
        
        # DEBUG: Vérifier le recollage final
        if x_boat is not None and x_rov is not None:
            dist_boat_final = np.sqrt((x_new[0] - float(x_boat))**2 + (y_new[0] - (min(float(y_boat), 0.0) if y_boat is not None else 0.0))**2)
            dist_rov_final = np.sqrt((x_new[-1] - float(x_rov))**2 + (y_new[-1] - min(float(y_rov), 0.0))**2)
            if dist_boat_final > 1e-3 or dist_rov_final > 1e-3:
                trace_print(_tl(5), f"[DEBUG] _normalize_cable_length: ⚠️  Recollage incorrect à la fin ! "
                    f"dist_boat={dist_boat_final:.6f} m, dist_rov={dist_rov_final:.6f} m, "
                    f"P0=({x_new[0]:.3f},{y_new[0]:.3f}), PN=({x_new[-1]:.3f},{y_new[-1]:.3f}), "
                    f"Bateau=({x_boat:.3f},{min(float(y_boat), 0.0) if y_boat is not None else 0.0:.3f}), "
                    f"ROV=({x_rov:.3f},{min(float(y_rov), 0.0):.3f})")
                # Forcer le recollage manuellement
                if x_boat is not None:
                    x_new[0] = float(x_boat)
                    y_new[0] = min(float(y_boat), 0.0) if y_boat is not None else 0.0
                if x_rov is not None:
                    x_new[-1] = float(x_rov)
                    y_new[-1] = min(float(y_rov), 0.0)

        # Clipping final pour garantir y <= 0
        y_new = np.clip(y_new, None, 0.0)
        
        # Réappliquer les extrémités APRÈS le clipping final pour garantir le recollage exact
        # (le clipping peut avoir modifié y_new[0] ou y_new[-1])
        if x_boat is not None:
            x_new[0] = float(x_boat)
            y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
            y_new[0] = y_boat_clipped
        if x_rov is not None:
            x_new[-1] = float(x_rov)
            y_rov_clipped = min(float(y_rov), 0.0) if y_rov is not None else 0.0
            y_new[-1] = y_rov_clipped
        
        # DEBUG: Vérifier s'il reste des points au-dessus de la surface après tous les clippings
        points_above = np.where(y_new > 1e-6)[0]
        if len(points_above) > 0:
            # Exclure le point bateau (index 0) qui doit être à y=0
            points_above = points_above[points_above != 0]
            if len(points_above) > 0:
                max_y_above = np.max(y_new[points_above])
                trace_print(_tl(5), f"[DEBUG] _normalize_cable_length: ⚠️  {len(points_above)} points encore au-dessus de la surface après clipping final ! "
                    f"max_y={max_y_above:.6f} m, indices={points_above[:5].tolist()}")
                # Forcer le clipping manuellement
                for idx in points_above:
                    y_new[idx] = 0.0
        
        # Réappliquer les extrémités une dernière fois après le clipping manuel
        if x_boat is not None:
            x_new[0] = float(x_boat)
            y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
            y_new[0] = y_boat_clipped
        if x_rov is not None:
            x_new[-1] = float(x_rov)
            y_rov_clipped = min(float(y_rov), 0.0) if y_rov is not None else 0.0
            y_new[-1] = y_rov_clipped

        # Vérification finale des contraintes
        try:
            ds_list_final = np.sqrt(np.diff(x_new) ** 2 + np.diff(y_new) ** 2)
            L_final = float(ds_list_final.sum()) if len(ds_list_final) > 0 else 0.0
            ds_target_final = L_target / max(N, 1)
            ds_max_final = float(ds_list_final.max()) if len(ds_list_final) > 0 else 0.0
            ratio_final = ds_max_final / max(ds_target_final, 1e-9) if ds_target_final > 0 else 0.0

            # Contraintes vérifiées :
            # - Longueur totale proche de L_target
            # - Ratio ds_max / ds_target raisonnable (≲ 2.0)
            # - y <= 0
            tol_L_rel = 1e-4  # 0.01 %
            ok_L = (L_target <= 0.0) or (abs(L_final - L_target) / max(L_target, 1e-9) <= tol_L_rel)
            ok_ratio = ratio_final <= 2.0 + 1e-6
            ok_y = bool(np.all(y_new <= 1e-9))

            # Recollement extrémités : vérifier qu'elles sont bien aux positions cibles
            ok_ends = True
            if len(x_new) >= 2:
                if x_boat is not None:
                    dx0 = float(x_new[0] - float(x_boat))
                    dy0 = float(y_new[0] - (min(float(y_boat), 0.0) if y_boat is not None else 0.0))
                    err0 = np.hypot(dx0, dy0)
                    ok_ends = ok_ends and (err0 <= 1e-3)  # Tolérance 1 mm
                if x_rov is not None:
                    dxN = float(x_new[-1] - float(x_rov))
                    dyN = float(y_new[-1] - min(float(y_rov), 0.0))
                    errN = np.hypot(dxN, dyN)
                    ok_ends = ok_ends and (errN <= 1e-3)  # Tolérance 1 mm

            if not (ok_L and ok_ratio and ok_y and ok_ends):
                # Utiliser l'import global de trace_print (déjà importé en haut du fichier)
                trace_print(
                    8,
                    "[DEBUG _normalize_cable_length] CONTRAINTES NON RESPECTÉES : "
                    f"L_target={L_target:.6f}, L_final={L_final:.6f}, "
                    f"rel_err_L={abs(L_final - L_target) / max(L_target, 1e-9):.3e}, "
                    f"ds_target={ds_target_final:.6f}, ds_max={ds_max_final:.6f}, "
                    f"ratio_max={ratio_final:.3f}, ok_L={ok_L}, ok_ratio={ok_ratio}, "
                    f"ok_y={ok_y}, ok_ends={ok_ends}"
                )
        except Exception:
            # Ne jamais casser la simulation à cause d'un check de debug
            pass

        # DEBUG: Calculer la déviation maximale APRÈS normalisation
        max_dev_after = compute_max_deviation(x_new, y_new)
        if max_dev_before > 0.01 and max_dev_after < max_dev_before * 0.5:
            # La déviation a été significativement réduite, ce qui indique que la forme a été détruite
            trace_print(_tl(5), f"[DEBUG] _normalize_cable_length: ⚠️  DÉVIATION RÉDUITE - "
                f"avant={max_dev_before:.6f} m, après={max_dev_after:.6f} m, "
                f"ratio={max_dev_after/max_dev_before:.3f}, L_target={L_target:.2f} m")
        elif max_dev_before > 0.01:
            trace_print(_tl(5), f"[DEBUG] _normalize_cable_length: ✓ Déviation préservée - "
                f"avant={max_dev_before:.6f} m, après={max_dev_after:.6f} m")

        return x_new, y_new, False

