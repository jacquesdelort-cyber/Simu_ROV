"""
Visualisation graphique pour create_point_with_target_length.

Ce script génère un rapport HTML Plotly pour des configurations de _P
comportant entre 2 et 6 points (extrémités incluses).
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

from src.utils.utils import create_point_with_target_length  # noqa: E402


@dataclass(frozen=True)
class CreatePointCase:
    name: str
    P: np.ndarray
    l_seg_target: float


def _build_cases() -> list[CreatePointCase]:
    return [
        # 2 points
        CreatePointCase(
            name="N2_AB_trop_long",
            P=np.array([[0.0, 0.0], [6.0, -1.0]], dtype=float),
            l_seg_target=2.0,
        ),
        CreatePointCase(
            name="N2_AB_egal_2l",
            P=np.array([[0.0, 0.0], [4.0, 0.0]], dtype=float),
            l_seg_target=2.0,
        ),
        # 3 points
        CreatePointCase(
            name="N3_general",
            P=np.array([[0.0, 0.0], [1.2, -1.0], [3.0, -0.5]], dtype=float),
            l_seg_target=2.2,
        ),
        # 4 points
        CreatePointCase(
            name="N4_points_au_dessus_surface",
            P=np.array([[0.0, 1.0], [1.0, 1.3], [2.2, 0.8], [3.2, 0.4]], dtype=float),
            l_seg_target=2.0,
        ),
        # 5 points
        CreatePointCase(
            name="N5_A_egal_B",
            P=np.array([[1.0, -2.0], [1.5, -2.2], [1.0, -2.0], [0.8, -1.9], [1.0, -2.0]], dtype=float),
            l_seg_target=1.5,
        ),
        # 6 points
        CreatePointCase(
            name="N6_general_courbe",
            P=np.array(
                [
                    [0.0, 0.0],
                    [0.8, -0.5],
                    [1.7, -1.2],
                    [2.8, -1.8],
                    [3.7, -1.3],
                    [4.6, -0.6],
                ],
                dtype=float,
            ),
            l_seg_target=2.8,
        ),
    ]


def generate_create_point_report(output_file: str | Path | None = None) -> Path:
    cases = _build_cases()
    n_cases = len(cases)

    fig = make_subplots(
        rows=n_cases,
        cols=1,
        specs=[[{"type": "scatter"}] for _ in range(n_cases)],
        subplot_titles=[case.name for case in cases],
        vertical_spacing=0.06,
    )

    for idx, case in enumerate(cases):
        row = idx + 1
        P = np.asarray(case.P, dtype=float)
        P_clip = P.copy()
        P_clip[:, 1] = np.minimum(P_clip[:, 1], 0.0)
        A = P_clip[0]
        B = P_clip[-1]
        H = 0.5 * (A + B)

        ok, C = create_point_with_target_length(P, case.l_seg_target)

        fig.add_trace(
            go.Scatter(
                x=P[:, 0],
                y=P[:, 1],
                mode="lines+markers",
                name="P_original",
                legendgroup="P_original",
                showlegend=(idx == 0),
                line=dict(color="#7f7f7f", width=1, dash="dot"),
                marker=dict(size=6),
            ),
            row=row,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=P_clip[:, 0],
                y=P_clip[:, 1],
                mode="lines+markers",
                name="P_clipped",
                legendgroup="P_clipped",
                showlegend=(idx == 0),
                line=dict(color="#1f77b4", width=2),
                marker=dict(size=7),
            ),
            row=row,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=[A[0], B[0]],
                y=[A[1], B[1]],
                mode="lines",
                name="AB",
                legendgroup="AB",
                showlegend=(idx == 0),
                line=dict(color="#2ca02c", width=1, dash="dash"),
            ),
            row=row,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=[H[0]],
                y=[H[1]],
                mode="markers+text",
                text=["H"],
                textposition="top center",
                name="H",
                legendgroup="H",
                showlegend=(idx == 0),
                marker=dict(size=8, color="#9467bd"),
            ),
            row=row,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=[C[0]],
                y=[C[1]],
                mode="markers+text",
                text=["C"],
                textposition="top center",
                name="C",
                legendgroup="C",
                showlegend=(idx == 0),
                marker=dict(size=9, color=("#d62728" if ok else "#ff7f0e")),
            ),
            row=row,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=[A[0], C[0], B[0], C[0]],
                y=[A[1], C[1], B[1], C[1]],
                mode="lines",
                name="AC_BC",
                legendgroup="AC_BC",
                showlegend=(idx == 0),
                line=dict(color="#d62728", width=1),
            ),
            row=row,
            col=1,
        )

        ac = float(np.linalg.norm(C - A))
        bc = float(np.linalg.norm(C - B))
        fig.add_annotation(
            xref=f"x{row}" if row > 1 else "x",
            yref=f"y{row}" if row > 1 else "y",
            x=float(C[0]),
            y=float(C[1]),
            text=(
                f"ok={ok}"
                f"<br>A=({A[0]:.3f}, {A[1]:.3f})"
                f"<br>B=({B[0]:.3f}, {B[1]:.3f})"
                f"<br>C=({C[0]:.3f}, {C[1]:.3f})"
                f"<br>AC={ac:.3f}"
                f"<br>BC={bc:.3f}"
                f"<br>l={case.l_seg_target:.3f}"
            ),
            showarrow=True,
            arrowhead=1,
            ax=180,
            ay=0,
            align="left",
        )

        xs = np.concatenate([P[:, 0], P_clip[:, 0], np.array([C[0]])])
        ys = np.concatenate([P[:, 1], P_clip[:, 1], np.array([C[1]])])
        span = max(xs.max() - xs.min(), ys.max() - ys.min(), 1.0)
        cx = 0.5 * (xs.max() + xs.min())
        cy = 0.5 * (ys.max() + ys.min())
        half = 0.7 * span
        fig.update_xaxes(title_text="x", range=[cx - half, cx + half], row=row, col=1)
        fig.update_yaxes(
            title_text="y",
            range=[cy - half, cy + half],
            scaleanchor=f"x{row}" if row > 1 else "x",
            scaleratio=1.0,
            row=row,
            col=1,
        )

    fig.update_layout(
        title="Visualisation create_point_with_target_length",
        template="plotly_white",
        height=max(330 * n_cases, 700),
        width=900,
        hovermode="closest",
    )

    if output_file is None:
        output_file = Path("results") / "create_point_with_target_length_visual_tests.html"
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(output_path), include_plotlyjs="inline")
    return output_path


def main() -> None:
    output_path = generate_create_point_report()
    print("Rapport visuel create_point_with_target_length genere :", output_path)


if __name__ == "__main__":
    main()

