"""Infobulles Plotly normalisées pour les points de câble (plans XY, abscisse curviligne)."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

CABLE_XY_HOVERTEMPLATE = "%{hovertext}<extra></extra>"


def cable_vertex_tooltip(
    num_point: int | str,
    s: float,
    lseg: float | None,
    x: float,
    y: float,
) -> str:
    """Une ligne d'infobulle complète (sans balise extra Plotly)."""
    lseg_s = f"{lseg:.2f}" if lseg is not None else "N/A"
    return (
        f"Num point: {num_point}<br>"
        f"s: {float(s):.2f}<br>"
        f"lseg: {lseg_s}<br>"
        f"X: {float(x):.4f}<br>"
        f"Y: {float(y):.4f}"
    )


def cable_polyline_hover_texts(
    x: Sequence[float] | np.ndarray,
    y: Sequence[float] | np.ndarray,
    *,
    point_numbers: Sequence[int | str] | None = None,
) -> list[str]:
    """
    Infobulles pour chaque sommet d'une polyligne : s et lseg déduits de la géométrie.
    Num point : 1..n par défaut, ou ``point_numbers`` (même longueur que x).
    """
    xa = np.asarray(x, dtype=float).ravel()
    ya = np.asarray(y, dtype=float).ravel()
    if xa.size != ya.size:
        raise ValueError("x et y doivent avoir la même longueur")
    n = int(xa.size)
    if n == 0:
        return []
    if point_numbers is not None:
        if len(point_numbers) != n:
            raise ValueError("point_numbers doit avoir la même longueur que x/y")
        nums: list[int | str] = list(point_numbers)
    else:
        nums = [i + 1 for i in range(n)]
    s = np.zeros(n, dtype=float)
    for i in range(1, n):
        s[i] = s[i - 1] + float(np.hypot(xa[i] - xa[i - 1], ya[i] - ya[i - 1]))
    out: list[str] = []
    for i in range(n):
        if i < n - 1:
            lseg = float(np.hypot(xa[i + 1] - xa[i], ya[i + 1] - ya[i]))
        else:
            lseg = None
        out.append(cable_vertex_tooltip(nums[i], float(s[i]), lseg, float(xa[i]), float(ya[i])))
    return out


def cable_polyline_hover_plotly_kwargs(
    x: Sequence[float] | np.ndarray,
    y: Sequence[float] | np.ndarray,
    *,
    point_numbers: Sequence[int | str] | None = None,
) -> dict:
    """Arguments à étaler dans ``go.Scatter`` (lignes+marqueurs)."""
    texts = cable_polyline_hover_texts(x, y, point_numbers=point_numbers)
    if not texts:
        return {}
    return {
        "hovertext": texts,
        "hovertemplate": CABLE_XY_HOVERTEMPLATE,
        "hoverinfo": "text",
    }


def cable_markers_along_s_hover_texts(
    s_arr: Sequence[float] | np.ndarray,
    *,
    x_arr: Sequence[float] | np.ndarray | None = None,
    y_arr: Sequence[float] | np.ndarray | None = None,
    point_numbers: Sequence[int | str] | None = None,
) -> list[str]:
    """
    Infobulles lorsque l'abscisse ``s`` est fournie par les données (ex. graphe T(s)).
    lseg : longueur géométrique si x,y disponibles, sinon Δs entre points consécutifs.
    X/Y : quatre décimales si fournis, sinon ``N/A``.
    """
    s_ = np.asarray(s_arr, dtype=float).ravel()
    n = int(s_.size)
    if point_numbers is not None and len(point_numbers) != n:
        raise ValueError("point_numbers doit avoir la même longueur que s_arr")
    nums: list[int | str] = list(point_numbers) if point_numbers is not None else [i + 1 for i in range(n)]
    xa = ya = None
    if x_arr is not None and y_arr is not None:
        xa = np.asarray(x_arr, dtype=float).ravel()
        ya = np.asarray(y_arr, dtype=float).ravel()
        if xa.size != n or ya.size != n:
            raise ValueError("x_arr et y_arr doivent avoir la même longueur que s_arr")
    out: list[str] = []
    for i in range(n):
        if i < n - 1:
            if xa is not None and ya is not None:
                lseg = float(np.hypot(xa[i + 1] - xa[i], ya[i + 1] - ya[i]))
            else:
                lseg = float(s_[i + 1] - s_[i])
        else:
            lseg = None
        xi = float(xa[i]) if xa is not None else None
        yi = float(ya[i]) if ya is not None else None
        xs = f"{xi:.4f}" if xi is not None else "N/A"
        ys = f"{yi:.4f}" if yi is not None else "N/A"
        lseg_s = f"{lseg:.2f}" if lseg is not None else "N/A"
        out.append(
            f"Num point: {nums[i]}<br>"
            f"s: {float(s_[i]):.2f}<br>"
            f"lseg: {lseg_s}<br>"
            f"X: {xs}<br>"
            f"Y: {ys}"
        )
    return out


def cable_markers_along_s_hover_plotly_kwargs(
    s_arr: Sequence[float] | np.ndarray,
    *,
    x_arr: Sequence[float] | np.ndarray | None = None,
    y_arr: Sequence[float] | np.ndarray | None = None,
    point_numbers: Sequence[int | str] | None = None,
) -> dict:
    texts = cable_markers_along_s_hover_texts(
        s_arr, x_arr=x_arr, y_arr=y_arr, point_numbers=point_numbers
    )
    if not texts:
        return {}
    return {
        "hovertext": texts,
        "hovertemplate": CABLE_XY_HOVERTEMPLATE,
        "hoverinfo": "text",
    }
