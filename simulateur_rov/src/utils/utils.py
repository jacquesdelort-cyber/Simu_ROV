"""Fonctions utilitaires générales."""

from __future__ import annotations

from typing import Sequence

import numpy as np

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

    # Si M' confondu avec M, géométrie trop dégénérée
    if np.allclose(M, Mp):
        return None

    # Test d'alignement A, M, M'
    AM = M - A2
    AMp = Mp - A2
    cross = AM[0] * AMp[1] - AM[1] * AMp[0]
    if abs(cross) < 1e-12:
        # Cas dégénéré : on ne sait pas construire E de façon fiable
        return None

    # Direction unitaire le long de la droite (M, M'), orientée vers M'
    d = Mp - M
    d_norm = float(np.linalg.norm(d))
    if d_norm == 0.0:
        return None
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
        # Pas de changement de signe trouvé : pas de solution évidente
        return None

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
        return None

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

    changed = True
    while True:
        N = P_xy.shape[0] - 1  # nombre de segments
        trace_print(9, f"[DEBUG]     N={N}, N_target_seg={N_target_seg}")
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
                    trace_print(9, f"[DEBUG] CAS INTERNE supprimer_point(A, B, C, D) is None")
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
                    trace_print(9, f"[DEBUG] CAS BATEAU supprimer_point(A, B, C, D) is None")
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
                    trace_print(9, f"[DEBUG] CAS ROV supprimer_point(A, B, C, D) is None")
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
