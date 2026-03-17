"""
Visualisation graphique pour la fonction enforce_cable_segments_nb.

Ce script génère un rapport HTML Plotly montrant, pour plusieurs cas,
le câble avant et après réduction du nombre de segments.
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

from src.utils.utils import enforce_cable_segments_nb  # noqa: E402


@dataclass(frozen=True)
class EnforceCableCase:
    name: str
    P: np.ndarray
    n_target_seg: int


def _build_cases() -> list[EnforceCableCase]:
    return [
        EnforceCableCase(
            name="Arc_souple_5pts_vers_2seg",
            P=np.array(
                [
                    [0.0, 0.0],
                    [1.0, 1.0],
                    [2.0, 1.2],
                    [3.0, 1.0],
                    [4.0, 0.0],
                ]
            ),
            n_target_seg=2,
        ),
        EnforceCableCase(
            name="Courbe_ondulante_8pts_vers_3seg",
            P=np.array(
                [
                    [0.0, 0.0],
                    [1.0, 0.5],
                    [2.0, -0.2],
                    [3.0, 0.8],
                    [4.0, 0.3],
                    [5.0, -0.1],
                    [6.0, 0.4],
                    [7.0, 0.0],
                ]
            ),
            n_target_seg=3,
        ),
        EnforceCableCase(
            name="Courbe_en_S_7pts_vers_4seg",
            P=np.array(
                [
                    [0.0, 0.0],
                    [1.0, -0.5],
                    [2.0, 0.5],
                    [3.0, -0.5],
                    [4.0, 0.5],
                    [5.0, -0.2],
                    [6.0, 0.0],
                ]
            ),
            n_target_seg=4,
        ),
        EnforceCableCase(
            name="Micro_segments_alignes_vers_2seg",
            P=np.array(
                [
                    [0.0, 0.0],
                    [1e-3, 0.0],
                    [2e-3, 0.0],
                    [3e-3, 0.0],
                    [4e-3, 0.0],
                    [5e-3, 0.0],
                    [6e-3, 0.0],
                ]
            ),
            n_target_seg=2,
        ),
    ]


def _length(P: np.ndarray) -> float:
    P = np.asarray(P, dtype=float)
    return float(np.sum(np.linalg.norm(P[1:] - P[:-1], axis=1)))


def generate_enforce_cable_report(output_file: str | Path | None = None) -> Path:
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
        P = case.P
        n_target = case.n_target_seg

        P_new, ok = enforce_cable_segments_nb(P, n_target)

        # Câble avant
        fig.add_trace(
            go.Scatter(
                x=P[:, 0],
                y=P[:, 1],
                mode="lines+markers",
                name="Avant",
                legendgroup="Avant",
                showlegend=(idx == 0),
                line=dict(color="#1f77b4", width=2),
                marker=dict(size=8),
            ),
            row=row,
            col=col,
        )

        # Câble après
        fig.add_trace(
            go.Scatter(
                x=P_new[:, 0],
                y=P_new[:, 1],
                mode="lines+markers",
                name="Apres",
                legendgroup="Apres",
                showlegend=(idx == 0),
                line=dict(color="#d62728", width=2, dash="dot"),
                marker=dict(size=8),
            ),
            row=row,
            col=col,
        )

        # Points indexés
        for arr, prefix, color in [
            (P, "A", "#000000"),
            (P_new, "A'", "#444444"),
        ]:
            for k, pt in enumerate(arr):
                fig.add_trace(
                    go.Scatter(
                        x=[pt[0]],
                        y=[pt[1]],
                        mode="markers+text",
                        text=[f"{prefix}{k}"],
                        textposition="top center",
                        showlegend=False,
                        marker=dict(size=7, color=color),
                    ),
                    row=row,
                    col=col,
                )

        # Axes avec même échelle
        xs = np.concatenate([P[:, 0], P_new[:, 0]])
        ys = np.concatenate([P[:, 1], P_new[:, 1]])
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

        # Stats avant / après
        L_before = _length(P)
        L_after = _length(P_new)
        n_before = P.shape[0] - 1
        n_after = P_new.shape[0] - 1
        text = (
            f"N_avant = {n_before}, N_cible = {n_target}, N_apres = {n_after}<br>"
            f"L_avant = {L_before:.4f} m<br>"
            f"L_apres = {L_after:.4f} m<br>"
            f"ok = {ok}"
        )
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

    fig.update_layout(
        title="Visualisation de enforce_cable_segments_nb",
        template="plotly_white",
        height=max(350 * n_cases, 600),
        width=900,
        hovermode="closest",
    )

    if output_file is None:
        output_file = Path("results") / "enforce_cable_segments_nb_visual_tests.html"
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(output_path), include_plotlyjs="inline")
    return output_path


def main() -> None:
    output_path = generate_enforce_cable_report()
    print("Rapport visuel enforce_cable_segments_nb genere :", output_path)


if __name__ == "__main__":
    main()

