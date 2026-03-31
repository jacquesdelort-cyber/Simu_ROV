"""
Visualisation graphique pour la fonction supprimer_point.

Ce script génère un rapport HTML Plotly montrant, pour plusieurs cas,
la géométrie avant (A-B-C-D) et après (A-E-D) suppression du point.
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
from src.utils.utils import supprimer_point  # noqa: E402
from tests._plotly_cable_axes import (  # noqa: E402
    data_ranges_for_cable_view,
    figure_layout_square_subplots,
)
from tests._report_output import write_plotly_html_with_generation_line  # noqa: E402


@dataclass(frozen=True)
class SupprimerPointCase:
    name: str
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    D: np.ndarray


def _build_cases() -> list[SupprimerPointCase]:
    return [
        SupprimerPointCase(
            name="Arc_simple",
            A=np.array([0.0, 0.0]),
            B=np.array([1.0, 2.0]),
            C=np.array([3.0, 2.0]),
            D=np.array([4.0, 0.0]),
        ),
        SupprimerPointCase(
            name="Courbe_asymetrique_a",
            A=np.array([0.0, 0.0]),
            B=np.array([1.0, 2.0]),
            C=np.array([3.0, 1.0]),
            D=np.array([4.0, 0.0]),
        ),
                SupprimerPointCase(
            name="Courbe_asymetrique_b",
            A=np.array([0.0, 0.0]),
            B=np.array([2.0, 2.0]),
            C=np.array([3.0, 1.0]),
            D=np.array([4.0, 0.0]),
        ),
        SupprimerPointCase(
            name="Courbe_asymetrique_c",
            A=np.array([0.0, 0.0]),
            B=np.array([3.0, 3.0]),
            C=np.array([6.0, 2.0]),
            D=np.array([5.0, 0.0]),
        ),
        SupprimerPointCase(
            name="Courbe_boucle",
            A=np.array([0.0, 0.0]),
            B=np.array([4.0, 2.0]),
            C=np.array([2.0, 3.0]),
            D=np.array([4.0, 0.0]),
        ),
                SupprimerPointCase(
            name="Courbe_Bateau",
            A=np.array([0.0, 0.0]),
            B=np.array([1.0, 1.0]),
            C=np.array([3.0, -2.0]),
            D=np.array([4.0, 0.0]),
        ),
                        SupprimerPointCase(
            name="Courbe_Z_1",
            A=np.array([0.0, 0.0]),
            B=np.array([2.0, -1.0]),
            C=np.array([3.0, 1.0]),
            D=np.array([4.0, 0.0]),
        ),
                SupprimerPointCase(
            name="Courbe_Z_2",
            A=np.array([0.0, 0.0]),
            B=np.array([2.0, -1.0]),
            C=np.array([3.0, 2.0]),
            D=np.array([4.0, 0.0]),
        ),
        SupprimerPointCase(
            name="Alignes_x",  # devrait renvoyer None
            A=np.array([0.0, 0.0]),
            B=np.array([1.0, 0.0]),
            C=np.array([2.0, 0.0]),
            D=np.array([4.0, 0.0]),
        ),
    ]


def _length_full(A: np.ndarray, B: np.ndarray, C: np.ndarray, D: np.ndarray) -> float:
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    C = np.asarray(C, dtype=float)
    D = np.asarray(D, dtype=float)
    return float(
        np.linalg.norm(B - A) + np.linalg.norm(C - B) + np.linalg.norm(D - C)
    )


def generate_supprimer_point_report(output_file: str | Path | None = None) -> Path:
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
        D = case.D

        E = supprimer_point(A, B, C, D)

        # Segments avant : A-B-C-D
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
        fig.add_trace(
            go.Scatter(
                x=[C[0], D[0]],
                y=[C[1], D[1]],
                mode="lines+markers",
                name="CD",
                legendgroup="Avant",
                showlegend=(idx == 0),
                line=dict(color="#2ca02c", width=2),
                marker=dict(size=8),
            ),
            row=row,
            col=col,
        )

        # Segments après : A-E-D (si E existe)
        if E is not None:
            fig.add_trace(
                go.Scatter(
                    x=[A[0], E[0]],
                    y=[A[1], E[1]],
                    mode="lines+markers",
                    name="AE",
                    legendgroup="Apres",
                    showlegend=(idx == 0),
                    line=dict(color="#d62728", width=2, dash="dot"),
                    marker=dict(size=8),
                ),
                row=row,
                col=col,
            )
            fig.add_trace(
                go.Scatter(
                    x=[E[0], D[0]],
                    y=[E[1], D[1]],
                    mode="lines+markers",
                    name="ED",
                    legendgroup="Apres",
                    showlegend=(idx == 0),
                    line=dict(color="#9467bd", width=2, dash="dot"),
                    marker=dict(size=8),
                ),
                row=row,
                col=col,
            )

        # Points A, B, C, D, E (si défini)
        points = [(A, "A"), (B, "B"), (C, "C"), (D, "D")]
        if E is not None:
            points.append((E, "E"))

        for pt, label in points:
            fig.add_trace(
                go.Scatter(
                    x=[pt[0]],
                    y=[pt[1]],
                    mode="markers+text",
                    text=[label],
                    textposition="top center",
                    showlegend=False,
                    marker=dict(size=9, color="#000000"),
                ),
                row=row,
                col=col,
            )

        xs = [A[0], B[0], C[0], D[0]]
        ys = [A[1], B[1], C[1], D[1]]
        if E is not None:
            xs.append(E[0])
            ys.append(E[1])
        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        (x_rng, y_rng) = data_ranges_for_cable_view(xs, ys, pad_frac=0.08)
        fig.update_xaxes(title_text="x", range=list(x_rng), row=row, col=col)
        fig.update_yaxes(title_text="y", range=list(y_rng), row=row, col=col)

        # Annoter les longueurs avant / après
        L_before = _length_full(A, B, C, D)
        if E is not None:
            L_after = float(np.linalg.norm(E - A) + np.linalg.norm(D - E))
            text = (
                f"L_avant = {L_before:.4f} m<br>"
                f"L_apres = {L_after:.4f} m<br>"
                f"E defini"
            )
        else:
            text = f"L_avant = {L_before:.4f} m<br>E non defini (None)"

        fig.add_annotation(
            xref="paper",
            yref=f"y{idx + 1}" if idx > 0 else "y",
            x=1.05,
            y=y_max - 0.05 * max(y_max - y_min, 1.0),
            text=text,
            showarrow=False,
            align="left",
            bgcolor="rgba(255,255,255,0.9)",
            bordercolor="#999",
            borderwidth=1,
            font=dict(size=9),
        )

    w_px, h_px, margin = figure_layout_square_subplots(n_cases, margin_r=320, margin_t=100)
    fig.update_layout(
        title="Visualisation de la fonction supprimer_point (A, B, C, D -> E)",
        template="plotly_white",
        width=w_px,
        height=h_px,
        margin=margin,
        autosize=False,
        hovermode="closest",
    )

    if output_file is None:
        rel = get_visual_default_html_path("visual_supprimer_point")
        if rel is None:
            raise RuntimeError("Catalogue: entrée visual_supprimer_point introuvable.")
        output_file = Path(rel)
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_plotly_html_with_generation_line(fig, output_path, include_plotlyjs="inline")
    return output_path


def main() -> None:
    output_path = generate_supprimer_point_report()
    print("Rapport visuel supprimer_point genere :", output_path)


if __name__ == "__main__":
    main()

