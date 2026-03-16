"""Fonctions utilitaires générales."""

from __future__ import annotations

from typing import Sequence

import numpy as np


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

    - M est le milieu de AB ;
    - les vecteurs [M, B] et [M, B'] sont colinéaires et orientés dans le même sens
      (B' est donc sur le rayon issu de M passant par B) ;
    - (AB' + CB') = sc * (AB + BC), avec sc > 0 ;
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

    # Milieu M de AB
    M = 0.5 * (A_v + B_v)
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

