"""
Visualisation graphique pour la fonction deplacer_point.

Ce script génère un rapport HTML Plotly montrant, pour plusieurs cas,
les triangles ABC et le point déplacé D, avec quelques annotations.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import os
import sys

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.tests.test_catalog import get_visual_default_html_path  # noqa: E402
from src.visualization.cable_hover import (  # noqa: E402
    CABLE_XY_HOVERTEMPLATE,
    cable_polyline_hover_plotly_kwargs,
    cable_polyline_hover_texts,
    cable_vertex_tooltip,
)
from src.utils.utils import deplacer_point  # noqa: E402
from tests._plotly_cable_axes import (  # noqa: E402
    data_ranges_for_cable_view,
    figure_layout_square_subplots,
)
from tests._report_output import write_plotly_html_with_generation_line  # noqa: E402


@dataclass(frozen=True)
class DeplacerPointCase:
    name: str
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray


def _build_cases() -> list[DeplacerPointCase]:
    return [
        DeplacerPointCase(
            name="Alignes_x",
            A=np.array([0.0, 0.0]),
            B=np.array([1.0, 0.0]),
            C=np.array([4.0, 0.0]),
        ),
        DeplacerPointCase(
            name="Triangle_isoscele",
            A=np.array([0.0, 0.0]),
            B=np.array([2.0, 2.0]),
            C=np.array([4.0, 0.0]),
        ),
        DeplacerPointCase(
            name="Triangle_quelconque",
            A=np.array([0.0, 0.0]),
            B=np.array([3.0, 2.0]),
            C=np.array([4.0, 0.0]),
        ),
                DeplacerPointCase(
            name="Triangle_étiré",
            A=np.array([0.0, 0.0]),
            B=np.array([5.0, 1.0]),
            C=np.array([4.0, 0.0]),
        ),
    ]


def generate_deplacer_point_report(output_file: str | Path | None = None) -> Path:
    cases = _build_cases()
    n_cases = len(cases)

    fig = make_subplots(
        rows=n_cases,
        cols=1,
        specs=[[{"type": "scatter"}] for _ in range(n_cases)],
        subplot_titles=[case.name for case in cases],
        vertical_spacing=0.08,
    )

    for idx, case in enumerate(cases):
        row = idx + 1
        col = 1
        A = case.A
        B = case.B
        C = case.C
        D = deplacer_point(A, B, C)

        # Segments AB, BC, AD, CD
        fig.add_trace(
            go.Scatter(
                x=[A[0], B[0]],
                y=[A[1], B[1]],
                mode="lines+markers",
                name="AB",
                legendgroup="AB",
                showlegend=(idx == 0),
                line=dict(color="#1f77b4", width=2),
                marker=dict(size=8),
                **cable_polyline_hover_plotly_kwargs([A[0], B[0]], [A[1], B[1]]),
            ),
            row=row,
            col=col,
        )
        fig.add_trace(
            go.Scatter(
                x=[B[0], C[0]],
                y=[B[1], C[1]],
                mode="lines+markers",
                name="BC",
                legendgroup="BC",
                showlegend=(idx == 0),
                line=dict(color="#ff7f0e", width=2),
                marker=dict(size=8),
                **cable_polyline_hover_plotly_kwargs([B[0], C[0]], [B[1], C[1]]),
            ),
            row=row,
            col=col,
        )

        # Segment de la médiatrice de AC passant par D (dans un cadre un peu plus large)
        AC = C - A
        n = np.array([-AC[1], AC[0]], dtype=float)
        if np.linalg.norm(n) > 0:
            n /= np.linalg.norm(n)
            # On choisit une longueur suffisante pour traverser la figure
            lengths = np.array(
                [np.linalg.norm(B - A), np.linalg.norm(C - A), np.linalg.norm(C - B)]
            )
            L_med = max(lengths.max(), 1.0) * 2.0
            P1 = D - L_med * n
            P2 = D + L_med * n
            fig.add_trace(
                go.Scatter(
                    x=[P1[0], P2[0]],
                    y=[P1[1], P2[1]],
                    mode="lines",
                    name="Mediatrice(AB)",
                    legendgroup="Mediatrice",
                    showlegend=(idx == 0),
                    line=dict(color="#9467bd", width=1, dash="dash"),
                ),
                row=row,
                col=col,
            )
        fig.add_trace(
            go.Scatter(
                x=[A[0], D[0]],
                y=[A[1], D[1]],
                mode="lines+markers",
                name="AD",
                legendgroup="AD",
                showlegend=(idx == 0),
                line=dict(color="#2ca02c", width=2, dash="dot"),
                marker=dict(size=8),
                **cable_polyline_hover_plotly_kwargs([A[0], D[0]], [A[1], D[1]]),
            ),
            row=row,
            col=col,
        )
        fig.add_trace(
            go.Scatter(
                x=[C[0], D[0]],
                y=[C[1], D[1]],
                mode="lines+markers",
                name="CD",
                legendgroup="CD",
                showlegend=(idx == 0),
                line=dict(color="#d62728", width=2, dash="dot"),
                marker=dict(size=8),
                **cable_polyline_hover_plotly_kwargs([C[0], D[0]], [C[1], D[1]]),
            ),
            row=row,
            col=col,
        )

        chain = np.stack([A, B, C], axis=0)
        h_abc = cable_polyline_hover_texts(chain[:, 0], chain[:, 1])
        s_d = float(np.linalg.norm(D - A))
        l_dc = float(np.linalg.norm(C - D))
        h_d = cable_vertex_tooltip("D", s_d, l_dc, float(D[0]), float(D[1]))
        point_hover = {"A": h_abc[0], "B": h_abc[1], "C": h_abc[2], "D": h_d}

        # Points A, B, C, D avec labels
        for pt, label, color in [
            (A, "A", "#000000"),
            (B, "B", "#000000"),
            (C, "C", "#000000"),
            (D, "D", "#000000"),
        ]:
            fig.add_trace(
                go.Scatter(
                    x=[pt[0]],
                    y=[pt[1]],
                    mode="markers+text",
                    text=[label],
                    textposition="top center",
                    showlegend=False,
                    marker=dict(size=9, color=color),
                    hovertext=[point_hover[label]],
                    hovertemplate=CABLE_XY_HOVERTEMPLATE,
                    hoverinfo="text",
                ),
                row=row,
                col=col,
            )

        xs = np.array([A[0], B[0], C[0], D[0]])
        ys = np.array([A[1], B[1], C[1], D[1]])
        (x_rng, y_rng) = data_ranges_for_cable_view(xs, ys, pad_frac=0.08)
        fig.update_xaxes(title_text="x", range=list(x_rng), row=row, col=col)
        fig.update_yaxes(title_text="y", range=list(y_rng), row=row, col=col)

    w_px, h_px, margin = figure_layout_square_subplots(n_cases, margin_r=120, margin_t=100)
    fig.update_layout(
        title="Visualisation de la fonction deplacer_point (A, B, C -> D)",
        template="plotly_white",
        width=w_px,
        height=h_px,
        margin=margin,
        autosize=False,
        hovermode="closest",
    )

    if output_file is None:
        rel = get_visual_default_html_path("visual_deplacer_point")
        if rel is None:
            raise RuntimeError("Catalogue: entrée visual_deplacer_point introuvable.")
        output_file = Path(rel)
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_plotly_html_with_generation_line(fig, output_path, include_plotlyjs="inline")
    return output_path


def main() -> None:
    output_path = generate_deplacer_point_report()
    print("Rapport visuel deplacer_point genere :", output_path)


if __name__ == "__main__":
    main()

