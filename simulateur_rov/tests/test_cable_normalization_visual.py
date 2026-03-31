"""Visualisation graphique des cas de test de normalisation du câble.

Ce script génère un rapport HTML Plotly permettant de comparer, pour chaque
cas élémentaire, le profil du câble avant et après normalisation.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import os
import sys

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.tests.test_catalog import get_visual_default_html_path  # noqa: E402
from src.models.environment import Environment
from src.solvers.cable_solver import CableSolver
from tests._plotly_cable_axes import (
    data_ranges_for_cable_view,
    figure_layout_square_subplots,
    subplot_vertical_spacing,
)
from tests._report_output import write_plotly_html_with_generation_line  # noqa: E402
from tests.cable_shared_cases import ALL_SHARED_CABLE_CASES, SharedCableCase  # noqa: E402


@dataclass(frozen=True)
class CableNormalizationSummary:
    """Résumé numérique d'un cas de normalisation."""

    name: str
    l_target: float
    l_before: float
    ds_min_before: float
    ds_max_before: float
    l_after: float
    ds_min_after: float
    ds_max_after: float


def _make_solver(n_segments: int) -> CableSolver:
    params = {
        "d": 0.01,
        "rho_cable": 1500.0,
        "Cx_cable": 1.2,
        "Cf_cable": 0.04,
    }
    env = Environment({"rho_eau": 1025.0, "g": 9.81})
    return CableSolver(n_segments, params, env)


def _segment_lengths(x_vals: np.ndarray, y_vals: np.ndarray) -> np.ndarray:
    x_vals = np.asarray(x_vals, dtype=float)
    y_vals = np.asarray(y_vals, dtype=float)
    dx = np.diff(x_vals)
    dy = np.diff(y_vals)
    return np.sqrt(dx**2 + dy**2)


def _polyline_length(x_vals: np.ndarray, y_vals: np.ndarray) -> float:
    lengths = _segment_lengths(x_vals, y_vals)
    return float(np.sum(lengths))


def _stats_text(label: str, x_vals: np.ndarray, y_vals: np.ndarray) -> str:
    lengths = _segment_lengths(x_vals, y_vals)
    n_seg = len(lengths)
    if n_seg == 0:
        min_ds = 0.0
        max_ds = 0.0
    else:
        min_ds = float(np.min(lengths))
        max_ds = float(np.max(lengths))
    l_total = _polyline_length(x_vals, y_vals)
    return (
        f"{label}<br>"
        f"N_segments = {n_seg}<br>"
        f"L = {l_total:.4f} m<br>"
        f"ds_min = {min_ds:.4f} m<br>"
        f"ds_max = {max_ds:.4f} m"
    )


def _build_summary(
    case: SharedCableCase,
    x_before: np.ndarray,
    y_before: np.ndarray,
    x_after: np.ndarray,
    y_after: np.ndarray,
) -> CableNormalizationSummary:
    before_lengths = _segment_lengths(x_before, y_before)
    after_lengths = _segment_lengths(x_after, y_after)

    return CableNormalizationSummary(
        name=case.name,
        l_target=case.l_target,
        l_before=_polyline_length(x_before, y_before),
        ds_min_before=float(np.min(before_lengths)) if len(before_lengths) else 0.0,
        ds_max_before=float(np.max(before_lengths)) if len(before_lengths) else 0.0,
        l_after=_polyline_length(x_after, y_after),
        ds_min_after=float(np.min(after_lengths)) if len(after_lengths) else 0.0,
        ds_max_after=float(np.max(after_lengths)) if len(after_lengths) else 0.0,
    )


def _build_cases() -> list[SharedCableCase]:
    """Cas définis dans ``tests/cable_shared_cases.py`` (ordre du tuple)."""
    # Rapport complet : tous les cas. Pour n'en tracer qu'un sous-ensemble :
    # return list(ALL_SHARED_CABLE_CASES[-4:])
    return list(ALL_SHARED_CABLE_CASES)


def generate_visual_report(output_file: str | Path | None = None) -> Path:
    """Génère un rapport HTML de visualisation des tests de normalisation."""
    cases = _build_cases()
    n_cases = len(cases)
    # Une seule colonne de graphiques : un cas par ligne
    n_cols = 1
    n_plot_rows = n_cases

    # Titres de sous-graphiques : nom lisible du cas (HTML pris en charge par Plotly)
    subplot_titles = [f"<b>{case.name}</b>" for case in cases]
    fig = make_subplots(
        rows=n_plot_rows,
        cols=n_cols,
        specs=[[{"type": "scatter"}] for _ in range(n_plot_rows)],
        subplot_titles=subplot_titles,
        horizontal_spacing=0.1,
        vertical_spacing=subplot_vertical_spacing(n_plot_rows),
        row_heights=[1.0] * n_plot_rows,
    )
    # Titres de sous-graphiques : police un peu plus lisible (annotations créées par make_subplots)
    for i in range(min(n_cases, len(fig.layout.annotations))):
        fig.layout.annotations[i].font.size = 14

    summaries: list[CableNormalizationSummary] = []

    for idx, case in enumerate(cases):
        row = idx + 1
        col = 1
        solver = _make_solver(case.n_segments)
        x_before = np.asarray(case.x_cable, dtype=float)
        y_before = np.asarray(case.y_cable, dtype=float)

        # Normalisation géométrique (échec affiché dans l'encart sans interrompre le rapport)
        norm_error: str | None = None
        try:
            x_after, y_after, rel_err, iters, _expl = solver._normalize_cable_geometry(
                x_before,
                y_before,
                case.l_target,
                case.boat,
                case.rov,
                case.n_segments,
            )
            x_after = np.asarray(x_after, dtype=float)
            y_after = np.asarray(y_after, dtype=float)
        except Exception as e:
            norm_error = str(e)
            x_after = np.asarray(x_before, dtype=float).copy()
            y_after = np.asarray(y_before, dtype=float).copy()
            rel_err = float("nan")
            iters = -1
        summaries.append(_build_summary(case, x_before, y_before, x_after, y_after))

        # Abscisse curviligne pour les tooltips
        def _curvilinear_abscissa(x_vals: np.ndarray, y_vals: np.ndarray) -> np.ndarray:
            x_vals = np.asarray(x_vals, dtype=float)
            y_vals = np.asarray(y_vals, dtype=float)
            s = np.zeros_like(x_vals, dtype=float)
            if len(x_vals) > 1:
                s[1:] = np.cumsum(_segment_lengths(x_vals, y_vals))
            return s

        s_before = _curvilinear_abscissa(x_before, y_before)
        idx_before = np.arange(len(x_before), dtype=int)
        custom_before = np.stack([idx_before, s_before], axis=1)

        s_after = _curvilinear_abscissa(x_after, y_after)
        idx_after = np.arange(len(x_after), dtype=int)
        custom_after = np.stack([idx_after, s_after], axis=1)

        # Câble avant normalisation
        fig.add_trace(
            go.Scatter(
                x=x_before,
                y=y_before,
                mode="lines+markers",
                name="Avant",
                legendgroup="Avant",
                showlegend=(idx == 0),
                line=dict(color="#d62728", width=2),
                marker=dict(size=7),
                customdata=custom_before,
                hovertemplate=(
                    "Point #%{customdata[0]}<br>"
                    "s = %{customdata[1]:.3f} m<br>"
                    "x = %{x:.3f} m<br>"
                    "y = %{y:.3f} m<extra>Avant</extra>"
                ),
            ),
            row=row,
            col=col,
        )
        # Câble après normalisation
        fig.add_trace(
            go.Scatter(
                x=x_after,
                y=y_after,
                mode="lines+markers",
                name="Apres",
                legendgroup="Apres",
                showlegend=(idx == 0),
                line=dict(color="#1f77b4", width=2),
                marker=dict(size=7),
                customdata=custom_after,
                hovertemplate=(
                    "Point #%{customdata[0]}<br>"
                    "s = %{customdata[1]:.3f} m<br>"
                    "x = %{x:.3f} m<br>"
                    "y = %{y:.3f} m<extra>Apres</extra>"
                ),
            ),
            row=row,
            col=col,
        )

        # Marqueurs explicites pour bateau et ROV après normalisation
        fig.add_trace(
            go.Scatter(
                x=[x_after[0]],
                y=[y_after[0]],
                mode="markers",
                name="Bateau",
                legendgroup="Bateau",
                showlegend=(idx == 0),
                marker=dict(size=9, symbol="square", color="#ff7f0e"),
                hovertemplate="Bateau<br>x=%{x:.3f} m<br>y=%{y:.3f} m<extra></extra>",
            ),
            row=row,
            col=col,
        )
        fig.add_trace(
            go.Scatter(
                x=[x_after[-1]],
                y=[y_after[-1]],
                mode="markers",
                name="ROV",
                legendgroup="ROV",
                showlegend=(idx == 0),
                marker=dict(size=9, symbol="diamond", color="#2ca02c"),
                hovertemplate="ROV<br>x=%{x:.3f} m<br>y=%{y:.3f} m<extra></extra>",
            ),
            row=row,
            col=col,
        )

        x_min = float(min(np.min(x_before), np.min(x_after)))
        x_max = float(max(np.max(x_before), np.max(x_after)))
        y_min = float(min(np.min(y_before), np.min(y_after)))
        y_max = float(max(np.max(y_before), np.max(y_after), 0.0))

        fig.add_hline(
            y=0.0,
            line_dash="dot",
            line_color="gray",
            line_width=1,
            row=row,
            col=col,
        )

        # Utiliser des tirets longs épaissis (police en gras) pour améliorer la visibilité
        avant_label = "Avant: <span style='color:#d62728;font-weight:bold'>────</span>"
        apres_label = "Après: <span style='color:#1f77b4;font-weight:bold'>────</span>"

        rel_err_str = f"{rel_err:.3e}" if np.isfinite(rel_err) else "n/a"
        err_block = (
            f"<br><br><span style='color:#b00'>Erreur _normalize_cable_geometry : {norm_error}</span>"
            if norm_error
            else ""
        )
        stats = (
            f"<b>Cas : {case.name}</b><br><br>"
            + _stats_text(avant_label, x_before, y_before)
            + "<br><br>"
            + _stats_text(apres_label, x_after, y_after)
            + f"<br><br>L cible = {case.l_target:.4f} m"
            + f"<br>rel_err = {rel_err_str}, iters = {iters}"
            + err_block
        )
        fig.add_annotation(
            # On place l'encart nettement à droite du graphique, dans les coordonnées du papier
            xref="paper",
            yref=f"y{idx + 1}" if idx > 0 else "y",
            x=1.12,
            y=y_max - 0.05 * max(y_max - y_min, 1.0),
            text=stats,
            showarrow=False,
            align="left",
            bgcolor="rgba(255,255,255,0.9)",
            bordercolor="#999",
            borderwidth=1,
            font=dict(size=9),
        )

        (x_axis_rng, y_axis_rng) = data_ranges_for_cable_view(
            np.concatenate([x_before, x_after]),
            np.concatenate([y_before, y_after]),
            pad_frac=0.08,
        )
        fig.update_xaxes(title_text="x (m)", range=list(x_axis_rng), row=row, col=col)
        fig.update_yaxes(title_text="y (m)", range=list(y_axis_rng), row=row, col=col)

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    w_px, h_px, margin = figure_layout_square_subplots(
        n_plot_rows,
        margin_r=400,
        margin_t=140,
    )
    fig.update_layout(
        title=(
            "Tests visuels de normalisation du cable"
            f"<br><sup>Comparaison avant/apres avec longueurs totales et min/max des segments</sup>"
            f"<br><sup>Généré le {timestamp}</sup>"
        ),
        template="plotly_white",
        width=w_px,
        height=h_px,
        margin=margin,
        autosize=False,
        hovermode="closest",
    )

    if output_file is None:
        rel = get_visual_default_html_path("visual_cable_normalization")
        if rel is None:
            raise RuntimeError("Catalogue: entrée visual_cable_normalization introuvable.")
        output_file = Path(rel)
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_plotly_html_with_generation_line(fig, output_path, include_plotlyjs="inline")
    return output_path


def main() -> None:
    output_path = generate_visual_report()
    print("Rapport visuel genere :", output_path)


if __name__ == "__main__":
    main()
