"""Bornes d'axes pour les rapports Plotly (câbles 2D).

Sans égalité d'échelle x/y : les anciens scripts utilisaient scaleanchor + même
« half » pour x et y, ce qui écrasait visuellement la profondeur lorsque le câble
était beaucoup plus long en x qu'en |y|.
"""

from __future__ import annotations

import numpy as np


def data_ranges_for_cable_view(
    xs: np.ndarray,
    ys: np.ndarray,
    *,
    pad_frac: float = 0.08,
    include_y_zero: bool = True,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """
    Retourne (x_min, x_max), (y_min, y_max) avec marges proportionnelles à chaque direction.

    Si include_y_zero est True, y_max est au moins 0 (surface) pour garder le repère.
    """
    xs = np.asarray(xs, dtype=float).ravel()
    ys = np.asarray(ys, dtype=float).ravel()
    if xs.size == 0 or ys.size == 0:
        return (-1.0, 1.0), (-1.0, 1.0)
    x_min, x_max = float(np.min(xs)), float(np.max(xs))
    y_min, y_max = float(np.min(ys)), float(np.max(ys))
    if include_y_zero:
        y_max = max(y_max, 0.0)
    dx = max(x_max - x_min, 1e-9)
    dy = max(y_max - y_min, 1e-9)
    px = pad_frac * dx
    py = pad_frac * dy
    return (x_min - px, x_max + px), (y_min - py, y_max + py)


def figure_layout_square_subplots(
    n_rows: int,
    *,
    screen_ref_px: int = 1920,
    fraction_side: float = 2.0 / 3.0,
    margin_l: int = 72,
    margin_r: int = 120,
    margin_t: int = 120,
    margin_b: int = 64,
    subplot_title_px: int = 40,
    inter_row_gap_px: int | None = None,
) -> tuple[int, int, dict]:
    """
    Dimensions (px) et marges pour un rapport HTML Plotly à une colonne : chaque
    ligne de sous-graphique occupe environ un carré de côté ``fraction_side * screen_ref_px``
    (par défaut ~2/3 d'un écran 1920 px), avec ``margin_r`` réservé à droite (encart,
    légende, etc.).

    Retourne (width_px, height_px, margin_dict) à passer à ``fig.update_layout``.
    """
    if n_rows < 1:
        n_rows = 1
    plot_side = max(400, int(screen_ref_px * fraction_side))
    if inter_row_gap_px is None:
        inter_row_gap_px = max(8, int(0.015 * plot_side))
    width = margin_l + plot_side + margin_r
    height = (
        margin_t
        + margin_b
        + n_rows * (plot_side + subplot_title_px)
        + (n_rows - 1) * inter_row_gap_px
    )
    return width, height, dict(l=margin_l, r=margin_r, t=margin_t, b=margin_b)


def subplot_vertical_spacing(n_rows: int, *, max_spacing: float = 0.08, min_row_frac: float = 0.04) -> float:
    """
    Fraction d'espacement vertical pour ``make_subplots`` (Plotly).

    Si ``vertical_spacing`` est trop grand alors qu'il y a beaucoup de lignes,
    la somme ``(n_rows - 1) * vertical_spacing`` occupe presque toute la figure
    et chaque sous-graphique n'a plus qu'une bande très basse — aspect « aplati ».

    On borne l'espacement pour que chaque ligne garde au minimum environ
    ``min_row_frac`` de la hauteur utile (hors marges Plotly).
    """
    if n_rows <= 1:
        return max_spacing
    # (1 - (n_rows-1)*vs) / n_rows >= min_row_frac  =>  vs <= (1 - n_rows*min_row_frac) / (n_rows-1)
    numer = 1.0 - n_rows * min_row_frac
    if numer <= 0:
        # Trop de lignes pour la cible : on réserve ~15 % au total des interlignes
        min_row_frac = 0.85 / n_rows
        numer = 1.0 - n_rows * min_row_frac
    cap = numer / (n_rows - 1)
    return float(max(0.004, min(max_spacing, cap)))
