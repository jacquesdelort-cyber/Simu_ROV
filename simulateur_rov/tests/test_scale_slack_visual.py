"""
Visualisation graphique pour la fonction scale_slack.

Ce script génère un rapport HTML Plotly montrant, pour plusieurs cas,
les triangles ABC et le point déplacé B', avec comparaison avant/après.
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

from src.utils.utils import scale_slack  # noqa: E402


@dataclass(frozen=True)
class ScaleSlackCase:
    name: str
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    sc: float


def _build_cases() -> list[ScaleSlackCase]:
    return [
        ScaleSlackCase(
            name="sc_1_triangle_quelconque",
            A=np.array([0.0, 0.0]),
            B=np.array([1.0, 2.0]),
            C=np.array([4.0, 0.0]),
            sc=1.0,
        ),
        ScaleSlackCase(
            name="sc_gt_1_plus_slack_a",
            A=np.array([0.0, 0.0]),
            B=np.array([1.0, 2.0]),
            C=np.array([4.0, 0.0]),
            sc=1.5,
        ),
        ScaleSlackCase(
            name="sc_gt_1_plus_slack_b",
            A=np.array([0.0, 0.0]),
            B=np.array([6.0, 3.0]),
            C=np.array([4.0, 0.0]),
            sc=1.5,
        ),
        ScaleSlackCase(
            name="sc_lt_1_moins_slack_a",
            A=np.array([0.0, 0.0]),
            B=np.array([1.0, 2.0]),
            C=np.array([4.0, 0.0]),
            sc=0.7,
        ),
                ScaleSlackCase(
            name="sc_lt_1_moins_slack_b",
            A=np.array([0.0, 0.0]),
            B=np.array([6.0, 3.0]),
            C=np.array([4.0, 0.0]),
            sc=0.7,
        ),
        ScaleSlackCase(
            name="alignes",
            A=np.array([0.0, 0.0]),
            B=np.array([1.0, 0.0]),
            C=np.array([4.0, 0.0]),
            sc=2.0,
        ),
    ]


def _length_sum(A: np.ndarray, B: np.ndarray, C: np.ndarray) -> float:
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    C = np.asarray(C, dtype=float)
    return float(np.linalg.norm(B - A) + np.linalg.norm(C - B))


def generate_scale_slack_report(output_file: str | Path | None = None) -> Path:
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
        sc = case.sc
        Bp = scale_slack(A, B, C, sc=sc)

        # Segments avant : AB, BC
        fig.add_trace(
            go.Scatter(
                x=[A[0], B[0]],
                y=[A[1], B[1]],
                mode="lines+markers",
                name="AB",
                legendgroup="Avant",
                showlegend=(idx == 0),
                line=dict(color="#1f77b4", width=2),
                marker=dict(size=8),
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
                legendgroup="Avant",
                showlegend=(idx == 0),
                line=dict(color="#ff7f0e", width=2),
                marker=dict(size=8),
            ),
            row=row,
            col=col,
        )

        # Segments après : AB', B'C
        fig.add_trace(
            go.Scatter(
                x=[A[0], Bp[0]],
                y=[A[1], Bp[1]],
                mode="lines+markers",
                name="AB'",
                legendgroup="Apres",
                showlegend=(idx == 0),
                line=dict(color="#2ca02c", width=2, dash="dot"),
                marker=dict(size=8),
            ),
            row=row,
            col=col,
        )
        fig.add_trace(
            go.Scatter(
                x=[Bp[0], C[0]],
                y=[Bp[1], C[1]],
                mode="lines+markers",
                name="B'C",
                legendgroup="Apres",
                showlegend=(idx == 0),
                line=dict(color="#d62728", width=2, dash="dot"),
                marker=dict(size=8),
            ),
            row=row,
            col=col,
        )

        # Points A, B, C, M, B' avec labels
        M = 0.5 * (A + C)
        for pt, label, color in [
            (A, "A", "#000000"),
            (B, "B", "#000000"),
            (C, "C", "#000000"),
            (M, "M", "#000000"),
            (Bp, "B'", "#000000"),
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
                ),
                row=row,
                col=col,
            )

        # Affichage du vecteur [M,B'] pour voir la colinéarité (M milieu de AC)
        fig.add_trace(
            go.Scatter(
                x=[M[0], Bp[0]],
                y=[M[1], Bp[1]],
                mode="lines",
                name="MB'",
                legendgroup="Vecteurs",
                showlegend=(idx == 0),
                line=dict(color="#bcbd22", width=1, dash="dash"),
            ),
            row=row,
            col=col,
        )

        # Ajuster les axes avec la même échelle
        xs = np.array([A[0], B[0], C[0], Bp[0]])
        ys = np.array([A[1], B[1], C[1], Bp[1]])
        x_span = xs.max() - xs.min()
        y_span = ys.max() - ys.min()
        span = max(x_span, y_span, 1.0)
        cx = 0.5 * (xs.max() + xs.min())
        cy = 0.5 * (ys.max() + ys.min())
        half_span = 0.7 * span
        x_min = cx - half_span
        x_max = cx + half_span
        y_min = cy - half_span
        y_max = cy + half_span
        fig.update_xaxes(
            title_text="x",
            range=[x_min, x_max],
            row=row,
            col=col,
        )
        fig.update_yaxes(
            title_text="y",
            scaleanchor="x",
            scaleratio=1.0,
            range=[y_min, y_max],
            row=row,
            col=col,
        )

        # Annoter les longueurs avant / après
        L_before = _length_sum(A, B, C)
        L_after = _length_sum(A, Bp, C)
        stats = (
            f"AB+BC = {L_before:.4f} m<br>"
            f"AB'+B'C = {L_after:.4f} m<br>"
            f"sc = {sc:.3f}"
        )
        fig.add_annotation(
            xref="paper",
            yref=f"y{idx + 1}" if idx > 0 else "y",
            x=1.05,
            y=y_max - 0.05 * max(y_max - y_min, 1.0),
            text=stats,
            showarrow=False,
            align="left",
            bgcolor="rgba(255,255,255,0.9)",
            bordercolor="#999",
            borderwidth=1,
            font=dict(size=9),
        )

    fig.update_layout(
        title="Visualisation de la fonction scale_slack (A, B, C, sc -> B')",
        template="plotly_white",
        height=max(350 * n_cases, 600),
        width=900,
        hovermode="closest",
    )

    if output_file is None:
        output_file = Path("results") / "scale_slack_visual_tests.html"
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(output_path), include_plotlyjs="inline")
    return output_path


def main() -> None:
    output_path = generate_scale_slack_report()
    print("Rapport visuel scale_slack genere :", output_path)


if __name__ == "__main__":
    main()

