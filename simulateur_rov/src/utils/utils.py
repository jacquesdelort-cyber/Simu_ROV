"""Fonctions utilitaires générales."""

from __future__ import annotations

from math import e
from typing import Sequence

import numpy as np
from numpy.testing import print_assert_equal

from src.utils.logger import trace_print

def _as_point(p: Sequence[float]) -> np.ndarray:
    """Convertit un point quelconque en vecteur numpy 1D de floats."""
    a = np.asarray(p, dtype=float).reshape(-1)
    if a.size < 2:
        raise ValueError("Un point doit avoir au moins 2 coordonnées.")
    return a


def scale_slack(
    A: Sequence[float],
    B: Sequence[float],
    C: Sequence[float],
    sc: float,
) -> np.ndarray:
    """
    Calcule un point B' vérifiant les contraintes suivantes :

    - M est le milieu de AC ;
    - les vecteurs [M, B] et [M, B'] sont colinéaires et orientés dans le même sens
      (B' est donc sur la droite passant par M et B) ;
    - (AB' + CB') = sc * (AB + CB), avec sc > 0 ;
    - si A, B et C sont alignés, la fonction renvoie le point B inchangé.

    Les points A, B, C peuvent être des séquences de flottants (liste, tuple, array...).
    La fonction renvoie un array numpy 1D.
    """
    if sc <= 0:
        raise ValueError("sc doit être strictement positif.")

    A_v = _as_point(A)
    B_v = _as_point(B)
    C_v = _as_point(C)

    if not (A_v.shape == B_v.shape == C_v.shape):
        raise ValueError("A, B et C doivent avoir la même dimension.")

    # Cas dégénéré : A == B ou B == C -> on ne touche pas à B
    if np.allclose(A_v, B_v) or np.allclose(B_v, C_v):
        return B_v.copy()

    AB = B_v - A_v
    AC = C_v - A_v

    # Test d'alignement A, B, C (cas 2D) : si alignés on renvoie B
    # On considère que la géométrie est plane (x, y).
    if A_v.size >= 2:
        cross = AB[0] * AC[1] - AB[1] * AC[0]
        if abs(cross) < 1e-10:
            return B_v.copy()

    # Milieu M de AC
    M = 0.5 * (A_v + C_v)
    v = B_v - M
    nv = np.linalg.norm(v)
    if nv == 0.0:
        # A et B confondus (déjà traité plus haut) ou très proches
        return B_v.copy()

    # Longueurs de référence
    AB_len = np.linalg.norm(AB)
    BC_len = np.linalg.norm(C_v - B_v)
    S0 = AB_len + BC_len
    if S0 == 0.0:
        # Triangle complètement dégénéré
        return B_v.copy()

    target = sc * S0

    # Si sc ≈ 1, on renvoie directement B
    if abs(sc - 1.0) < 1e-12:
        return B_v.copy()

    def f(t: float) -> float:
        """Somme des longueurs AB'(t) + CB'(t) pour B'(t) = M + t v."""
        Bp = M + t * v
        return np.linalg.norm(Bp - A_v) + np.linalg.norm(C_v - Bp)

    # Recherche d'un encadrement [t_lo, t_hi] tel que f(t_lo) <= target <= f(t_hi)
    if sc < 1.0:
        # On s'attend à t in [0, 1]
        t_lo, t_hi = 0.0, 1.0
    else:
        # On s'attend à t >= 1 (on prolonge le rayon au-delà de B)
        t_lo, t_hi = 1.0, 1.0
        f_hi = f(t_hi)
        it = 0
        while f_hi < target and it < 60:
            t_hi *= 2.0
            f_hi = f(t_hi)
            it += 1
        # Si on n'a pas réussi à dépasser la cible, on abandonne et renvoie B
        if f_hi < target:
            return B_v.copy()

    f_lo = f(t_lo)
    f_hi = f(t_hi)

    # Si déjà très proche en borne, renvoyer la borne correspondante
    if abs(f_lo - target) <= 1e-12 * max(1.0, target):
        return M + t_lo * v
    if abs(f_hi - target) <= 1e-12 * max(1.0, target):
        return M + t_hi * v

    # Bisection pour trouver t tel que f(t) ≈ target
    for _ in range(80):
        t_mid = 0.5 * (t_lo + t_hi)
        f_mid = f(t_mid)
        if f_mid > target:
            t_hi = t_mid
            f_hi = f_mid
        else:
            t_lo = t_mid
            f_lo = f_mid
        if abs(f_mid - target) <= 1e-10 * max(1.0, target):
            break

    t_final = 0.5 * (t_lo + t_hi)
    return M + t_final * v


def deplacer_point(
    A: Sequence[float],
    B: Sequence[float],
    C: Sequence[float],
) -> np.ndarray:
    """
    Calcule un point D à partir de trois points A, B, C.

    La fonction est pensée pour remplacer B par D tout en conservant la longueur
    du câble entre A et C :

    - Si A, B, C sont alignés, on retourne D = (A + C) / 2.
    - Sinon, D est le point situé sur la médiatrice de AC, du côté de B,
      tel que AD + DC = AB + BC.

    Les points sont supposés en 2D (x, y).
    """
    A_v = _as_point(A)
    B_v = _as_point(B)
    C_v = _as_point(C)

    if not (A_v.shape == B_v.shape == C_v.shape):
        raise ValueError("A, B, C doivent avoir la même dimension.")
    if A_v.size < 2:
        raise ValueError("Les points doivent être au moins en 2D (x, y).")

    # On ne travaille qu'en 2D (x, y)
    A2 = A_v[:2]
    B2 = B_v[:2]
    C2 = C_v[:2]

    AB = B2 - A2
    AC = C2 - A2
    AC_len = float(np.linalg.norm(AC))

    # Cas dégénéré : A == C -> on renvoie simplement le milieu de A et C
    if AC_len == 0.0:
        return 0.5 * (A2 + C2)

    # Test d'alignement A, B, C
    cross = AB[0] * AC[1] - AB[1] * AC[0]
    if abs(cross) < 1e-12:
        # A, B, C alignés : D = (A + C)/2
        return 0.5 * (A2 + C2)

    BC = C2 - B2
    AB_len = float(np.linalg.norm(AB))
    BC_len = float(np.linalg.norm(BC))

    # Milieu de AC
    M = 0.5 * (A2 + C2)

    # Distance AD = DC = r sur la médiatrice de AC
    r = 0.5 * (AB_len + BC_len)
    base = 0.5 * AC_len  # distance du milieu M à A (et C) projetée sur AC
    tmp = r * r - base * base
    if tmp <= 0.0:
        # Géométrie impossible numériquement, fallback simple
        return M

    t = float(np.sqrt(tmp))

    # Vecteur normal unitaire à AC (rotation de 90°)
    n = np.array([-AC[1], AC[0]], dtype=float)
    n_norm = float(np.linalg.norm(n))
    if n_norm == 0.0:
        # Cas dégénéré improbable : AC de longueur non nulle mais normal nul
        return M
    n /= n_norm

    # Choisir le côté de la médiatrice : même côté que B par rapport à M
    side = float(np.dot(B2 - M, n))
    sign = 1.0 if side >= 0.0 else -1.0

    D2 = M + sign * t * n
    return D2


def create_point_with_target_length(
    _P: Sequence[Sequence[float]],
    l_seg_target: float,
) -> tuple[bool, np.ndarray]:
    """
    Construit un point C tel que |AC| = |BC| = l_seg_target.

    Notations :
    - A = premier point de _P
    - B = dernier point de _P
    - H = milieu de AB
    - G = barycentre des milieux des segments de _P (après clipping y<=0)

    Règles métier appliquées :
    - Tous les points de _P sont clippés à y <= 0 avant calcul.
    - Si AB > 2*l_seg_target  -> (False, H)
    - Si AB == 2*l_seg_target -> (True, H)
    - Si A == B               -> (True, A + (l_seg_target, 0))
    - Sinon, C est sur la perpendiculaire à AB passant par H, du côté de G.
      Si C est au-dessus de la surface (y > 0), on le réfléchit par rapport à AB.
    """
    # region : fonctions locales et initialisation
    def _is_on_line_ab(U: np.ndarray) -> bool:
        return abs(AB_vec[0] * (U[1] - A[1]) - AB_vec[1] * (U[0] - A[0])) <= tol
    
    P = np.asarray(_P, dtype=float)
    if P.ndim != 2 or P.shape[0] < 2 or P.shape[1] < 2:
        raise ValueError("_P doit contenir au moins 2 points 2D.")
    if l_seg_target < 0:
        raise ValueError("l_seg_target doit etre >= 0.")

    P2 = P[:, :2].copy()
    # Clip physique : tous les points au-dessus de la surface sont projetés sur y=0.
    P2[:, 1] = np.minimum(P2[:, 1], 0.0)

    A = P2[0]
    B = P2[-1]
    H = 0.5 * (A + B)

    AB_vec = B - A
    AB = float(np.linalg.norm(AB_vec))
    tol = 1e-9 * max(1.0, abs(AB)/abs(l_seg_target))

    seg_midpoints = 0.5 * (P2[:-1] + P2[1:])
    G = np.mean(seg_midpoints, axis=0)
    # endregion : fonctions locales et initialisation
    # region :Cas dégénérés 
    if AB > 2.0 * l_seg_target + tol:
        Q = H
        res = False
        explication = f"AB ({AB:6.2f}) > 2*l_seg_target ({2.0 * l_seg_target:6.2f})"

    elif abs(AB - 2.0 * l_seg_target) <= tol:
        Q=H
        res = True
        explication = (
            f"AB ({AB:6.2f}) - 2*l_seg_target ({2.0 * l_seg_target:6.2f}) <= tol ({tol:6.2f})"
        )

    elif np.linalg.norm(A - B) <= tol:
        explication = "A == B"
        res = True
        Q = A + np.array([float(l_seg_target), 0.0], dtype=float)
    else:
    # endregion : Cas dégénérés 
    # region cas standard : on cherche Q sur la perpendiculaire à AB passant par H, du côté de G.
        if _is_on_line_ab(G):
            G = G + np.array([0.0, -1.0], dtype=float)
            if _is_on_line_ab(G):
                G = G + np.array([1.0, 0.0], dtype=float)

        # Direction unitaire de la perpendiculaire a AB. On sait que l'on est pas dans le cas "A == B" donc AB_vec n'est pas nul.
        n = np.array([-AB_vec[1], AB_vec[0]], dtype=float)
        n_norm = float(np.linalg.norm(n))
        n /= n_norm

        # HQ tel que AQ = l_seg_target (triangle isocèle de base AB).
        rad = max(l_seg_target * l_seg_target - (0.5 * AB) * (0.5 * AB), 0.0)
        hq = float(np.sqrt(rad))

        side_g = AB_vec[0] * (G[1] - H[1]) - AB_vec[1] * (G[0] - H[0])
        sign = 1.0 if side_g >= 0.0 else -1.0
        Q = H + sign * hq * n

        # Si Q est au-dessus de la surface, réfléchir Q par rapport à la droite AB.
        if Q[1] > 0.0:
            u = AB_vec / AB  # AB non nul ici
            AH = Q - A
            proj = np.dot(AH, u) * u
            perp = AH - proj
            Q = A + proj - perp
            explication = "Cas standard - miroir" 
        else:
            explication = "Cas standard"
        # On est dzns le cas standard, donc on doit doit avoir AQ == BQ == l_seg_target
        if abs(np.linalg.norm(Q - A) - l_seg_target) > tol or abs(np.linalg.norm(Q - B) - l_seg_target) > tol:
            raise ValueError("AQ or BQ != l_seg_target")
        res = True
    # endregion : cas standard
    # region : contrôle des segments
    L_AQ = float(np.linalg.norm(Q - A))
    L_BQ = float(np.linalg.norm(Q - B))
    _ANSI_RED = "\033[91m"
    _ANSI_RESET = "\033[0m"
    control_segments = True
    L_seg_A = ""
    L_segB = ""
    if abs(L_AQ - l_seg_target) > tol:
        L_seg_A = (f"AQ ( {_ANSI_RED}{L_AQ:5.3f}{_ANSI_RESET})")
        control_segments = False

    if abs(L_BQ - l_seg_target) > tol:
        L_segB = f"BQ ( {_ANSI_RED}{L_BQ:5.3f}{_ANSI_RESET})"
        explication = explication + " " + L_segB
        control_segments = False
    
    if not res or not control_segments:
        trace_print(6, "[create_pnt_with_tgt_length] : "
            f"A=({A[0]:6.2f}, {A[1]:6.2f}) "
            f"B=({B[0]:6.2f}, {B[1]:6.2f}) "
            f"--> Q=({Q[0]:6.2f}, {Q[1]:6.2f})  "
            f"{L_seg_A} "
            f"{L_segB} "
            f"l_seg_target: {l_seg_target:5.3f} "
            f"Exp:  {explication}"
    )
    # endregion : contrôle des segments
    return res, Q


def supprimer_point(
    A: Sequence[float],
    B: Sequence[float],
    C: Sequence[float],
    D: Sequence[float],
) -> np.ndarray | None:
    """
    Calcule un point E qui remplace B et C par un unique point sur le câble
    tout en conservant la longueur locale et en préservant au mieux la géométrie.

    Notations (en 2D) :
    - L = |AB| + |BC| + |CD|
    - M = milieu de AD
    - M' = milieu de BC

    Si A, M et M' sont alignés, la fonction renvoie None (cas dégénéré).

    Sinon, E est défini comme l'unique point situé sur la droite (M, M'),
    du côté de M', tel que :
        |AE| + |DE| = L

    Si aucune solution numérique stable n'est trouvée, la fonction renvoie None.

    Les points A, B, C, D peuvent être des séquences de flottants (liste, tuple, array...).
    La fonction renvoie un array numpy 1D (2D) ou None.
    """
    A_v = _as_point(A)
    B_v = _as_point(B)
    C_v = _as_point(C)
    D_v = _as_point(D)

    if not (A_v.shape == B_v.shape == C_v.shape == D_v.shape):
        raise ValueError("A, B, C, D doivent avoir la même dimension.")
    if A_v.size < 2:
        raise ValueError("Les points doivent être au moins en 2D (x, y).")

    # Travailler en 2D (x, y)
    A2 = A_v[:2]
    B2 = B_v[:2]
    C2 = C_v[:2]
    D2 = D_v[:2]

    # Longueur totale locale L
    def _dist(P: np.ndarray, Q: np.ndarray) -> float:
        return float(np.linalg.norm(Q - P))

    L_local = _dist(A2, B2) + _dist(B2, C2) + _dist(C2, D2)

    # Milieux
    M = 0.5 * (A2 + D2)
    Mp = 0.5 * (B2 + C2)

    # Échelle de longueur locale pour détecter les cas de micro-géométrie
    L_AD = _dist(A2, D2)
    L_ref = max(L_AD, L_local, 1e-9)
    eps_L = 1e-6 * max(L_ref, 1.0)

    # Si la longueur locale est extrêmement petite, on simplifie :
    # on place E au milieu de AD et on accepte une petite erreur de longueur.
    if L_local < eps_L:
        return 0.5 * (A2 + D2)

    # Test d'alignement A, M, M'
    AM = M - A2
    AMp = Mp - A2
    cross = AM[0] * AMp[1] - AM[1] * AMp[0]

    # Direction le long de la droite (M, M'), orientée vers M'
    d = Mp - M
    d_norm = float(np.linalg.norm(d))

    # Si la géométrie est trop dégénérée (M≈Mp, alignement fort, d_norm≈0),
    # on utilise un fallback simple : E ≈ Mp.
    if np.allclose(M, Mp) or abs(cross) < 1e-12 or d_norm == 0.0:
        return Mp

    u = d / d_norm

    # On cherche E(t) = M + t * u, t > 0 tel que |AE| + |DE| = L_local
    def f(t: float) -> float:
        E = M + t * u
        return _dist(A2, E) + _dist(D2, E) - L_local

    # Encadrement de t : on part de t=0 et on étire vers M' puis au-delà
    t_lo = 0.0
    f_lo = f(t_lo)
    # On veut une solution avec E du côté de M' donc t > 0
    t_hi = d_norm  # d'abord jusqu'à M'
    f_hi = f(t_hi)

    # Si déjà un changement de signe entre 0 et ||M'M||, on utilise cet intervalle
    max_expand = 60
    it = 0
    while f_lo * f_hi > 0 and it < max_expand:
        # Étendre t_hi jusqu'à trouver un changement de signe ou atteindre la limite
        t_hi *= 2.0
        f_hi = f(t_hi)
        it += 1

    if f_lo * f_hi > 0:
        # Pas de changement de signe trouvé : on revient au fallback géométrique
        return Mp

    # Bisection pour trouver t tel que f(t) ≈ 0
    for _ in range(80):
        t_mid = 0.5 * (t_lo + t_hi)
        f_mid = f(t_mid)
        if abs(f_mid) <= 1e-10 * max(1.0, abs(L_local)):
            t_lo = t_hi = t_mid
            break
        if f_lo * f_mid <= 0:
            t_hi = t_mid
            f_hi = f_mid
        else:
            t_lo = t_mid
            f_lo = f_mid

    t_final = 0.5 * (t_lo + t_hi)
    if t_final <= 0.0:
        return Mp

    E2 = M + t_final * u
    return E2


def enforce_cable_segments_nb(
    P: Sequence[Sequence[float]],
    N_target_seg: int,
) -> tuple[np.ndarray, bool]:
    """
    Réduit le nombre de segments d'un câble en supprimant des points intermédiaires
    tout en conservant la longueur totale locale à l'aide de `supprimer_point`.

    - P : séquence de points (x, y) décrivant le câble.
    - N_target_seg : nombre de segments souhaité (>= 1).

    La fonction renvoie un tuple (P_new, ok) :
    - P_new : tableau numpy (n_points, 2) représentant le câble modifié ;
    - ok    : booléen indiquant si le nombre de segments vaut exactement
              N_target_seg à l'issue de l'algorithme.
    """
    pts = np.asarray(P, dtype=float)
    if pts.ndim != 2 or pts.shape[1] < 2:
        raise ValueError("P doit être un tableau (n_points, 2) ou plus.")

    # On ne garde que les 2 premières coordonnées (x, y)
    P_xy = pts[:, :2].copy()

    if N_target_seg < 1:
        raise ValueError("N_target_seg doit être >= 1.")

    def seg_lengths(arr: np.ndarray) -> np.ndarray:
        return np.linalg.norm(arr[1:] - arr[:-1], axis=1)

    # Garde-fou sur le nombre d'itérations : on ne doit pas boucler indéfiniment
    initial_N = P_xy.shape[0] - 1
    max_iter = max(10 * initial_N, 50)
    iter_count = 0

    changed = True
    while True:
        iter_count += 1
        if iter_count > max_iter:
            trace_print(9, f"[DEBUG] enforce_cable_segments_nb: nombre max d'itérations atteint "
                        f"(max_iter={max_iter}, N_actuel={P_xy.shape[0] - 1})")
            break
        N = P_xy.shape[0] - 1  # nombre de segments
        trace_print(8, f"[DEBUG] enforce_cable_segments_nb: N={N}, N_target_seg={N_target_seg}")
        if N <= N_target_seg or N < 3:
            break

        # Recherche du segment le plus court
        lengths = seg_lengths(P_xy)
        order = np.argsort(lengths)

        changed = False

        for k in order:
            i = int(k)
            j = i + 1
            n_pts = P_xy.shape[0]

            # Besoin d'au moins 4 points pour appliquer supprimer_point
            if n_pts < 4:
                break

            # Cas interne : 0 < i et j < n_pts - 1
            if 0 < i and j < n_pts - 1:
                A = P_xy[i - 1]
                B = P_xy[i]
                C = P_xy[j]
                D = P_xy[j + 1]
                E = supprimer_point(A, B, C, D)
                if E is None:
                    trace_print(9, f"[DEBUG] CAS INTERNE supprimer_point(A={A}, B={B}, C={C}, D={D}) is None")
                    continue
                # Construire le nouveau tableau : ... A, E, D ...
                new_pts = []
                new_pts.extend(P_xy[: i])        # jusqu'à A (exclu i-1+1)
                new_pts.append(A)
                new_pts.append(E)
                new_pts.extend(P_xy[j + 1 :])    # à partir de D
                P_xy = np.asarray(new_pts, dtype=float)
                changed = True
                break

            # Cas du premier segment (i == 0), nécessite au moins 4 points
            if i == 0 and n_pts >= 4:
                A = P_xy[0]
                B = P_xy[1]
                C = P_xy[2]
                D = P_xy[3]
                E = supprimer_point(A, B, C, D)
                if E is None:
                    trace_print(6, f"[DEBUG] CAS BATEAU supprimer_point(A={A}, B={B}, C={C}, D={D}) is None")
                    continue    
                new_pts = [A, E]
                new_pts.extend(P_xy[3:])
                P_xy = np.asarray(new_pts, dtype=float)
                changed = True
                break

            # Cas du dernier segment (j == n_pts - 1), nécessite au moins 4 points
            if j == n_pts - 1 and n_pts >= 4:
                A = P_xy[-4]
                B = P_xy[-3]
                C = P_xy[-2]
                D = P_xy[-1]
                E = supprimer_point(A, B, C, D)
                if E is None:
                    trace_print(6, f"[DEBUG] CAS ROV supprimer_point(A={A}, B={B}, C={C}, D={D}) is None")
                    continue
                # ... , A, E, D
                new_pts = []
                new_pts.extend(P_xy[:-4])
                new_pts.append(A)
                new_pts.append(E)
                new_pts.append(D)
                P_xy = np.asarray(new_pts, dtype=float)
                changed = True
                break

        if not changed:
            break

    ok = (P_xy.shape[0] - 1) == N_target_seg
    return P_xy, ok


def next_point(
    _Q: Sequence[Sequence[float]],
    _ls: Sequence[float],
    Lseg_total: float,
    R: Sequence[float],
    num_seg_R: int,
    s_R: float,
    step: float,
) -> tuple[int | None, np.ndarray | None]:
    """
    Renvoie le point T situé à l'abscisse curviligne s_R + step sur la polyline _Q.

    Paramètres
    ----------
    _Q : liste de points (x, y), longueur = N_points.
    _ls : liste des abscisses curvilignes des points de _Q (même longueur que _Q).
          En pratique, _ls[i] correspond au cumul de longueur du sommet _Q[i].
    Lseg_total : somme des longueurs des segments de _Q, soit _ls[-1].
    R : point courant, situé sur le segment num_seg_R.
    num_seg_R : numéro du segment sur lequel R est situé.
    s_R : abscisse curviligne de R.
    step : incrément d'abscisse curviligne.

    Retour
    ------
    (ns, T)
    - Si s_R + step > Lseg_total : (None, None)
    - Sinon : ns est le numéro du segment contenant T et T est le point interpolé.
    """
    Q = np.asarray(_Q, dtype=float)
    if Q.ndim != 2 or Q.shape[1] < 2 or Q.shape[0] < 2:
        raise ValueError("_Q doit contenir au moins 2 points en 2D (x, y).")
    Q = Q[:, :2]

    ls = np.asarray(_ls, dtype=float).reshape(-1)
    if ls.size != Q.shape[0]:
        raise ValueError("_ls doit avoir une taille égale à len(_Q).")

    L_total = float(Lseg_total)
    step_f = float(step)
    s_target = float(s_R) + step_f

    if s_target > L_total:
        trace_print(9, f"[next_point] s_target ({s_target:8.2f}) > L_total ({L_total:8.2f}) --> (None, None)")
        return None, None

    n_seg = Q.shape[0] - 1
    if num_seg_R < 0 or num_seg_R >= n_seg:
        raise ValueError("num_seg_R doit être dans [0, len(_Q)-2].")

    # Contrôle de cohérence minimal (on n'utilise pas R directement pour le calcul).
    s_seg0 = float(ls[num_seg_R])
    s_seg1 = float(ls[num_seg_R + 1])
    if not (min(s_seg0, s_seg1) - 1e-9 <= float(s_R) <= max(s_seg0, s_seg1) + 1e-9):
        # On ne bloque pas : l'utilisateur peut fournir s_R approximatif.
        pass

    # clamp éventuel sur la borne gauche
    if s_target <= float(ls[0]):
        trace_print(9, f"[next_point] : Q[0]=({Q[0][0]:8.2f}, {Q[0][1]:8.2f}) s_target ({s_target:8.2f}) <= ls[0] ({ls[0]:8.2f}) --> (0, Q[0])")
        return 0, Q[0].copy()

    if s_target >= float(ls[-1]):
        trace_print(9, f"[next_point] : Q[-1]=({Q[-1][0]:8.2f}, {Q[-1][1]:8.2f}) s_target ({s_target:8.2f}) >= ls[-1] ({ls[-1]:8.2f}) --> ({n_seg - 1}, Q[-1])")
        return n_seg - 1, Q[-1].copy()

    ns = int(np.searchsorted(ls, s_target, side="right") - 1)
    ns = max(0, min(ns, n_seg - 1))

    seg_start = float(ls[ns])
    seg_end = float(ls[ns + 1])
    seg_len = seg_end - seg_start

    if abs(seg_len) <= 1e-15:
        return ns, Q[ns].copy()

    t = (s_target - seg_start) / seg_len
    t = float(np.clip(t, 0.0, 1.0))
    T = (1.0 - t) * Q[ns] + t * Q[ns + 1]
    trace_print(6, f"[next_point] : ns = {ns} T=({T[0]:6.2f}, {T[1]:6.2f}) = (1.0 - t) * Q[ns] + t * Q[ns + 1] avec t = {t:4.2f}")
    return ns, T
