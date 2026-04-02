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

from src.tests.test_catalog import get_visual_default_html_path  # noqa: E402
from src.utils.utils import create_point_with_target_length  # noqa: E402
from src.visualization.cable_hover import (  # noqa: E402
    CABLE_XY_HOVERTEMPLATE,
    cable_polyline_hover_plotly_kwargs,
    cable_vertex_tooltip,
)
from tests._plotly_cable_axes import (  # noqa: E402
    data_ranges_for_cable_view,
    figure_layout_square_subplots,
)
from tests._report_output import write_plotly_html_with_generation_line  # noqa: E402


def _s_along_polyline_closest(P: np.ndarray, pt: np.ndarray) -> float:
    """Abscisse curviligne du projeté de ``pt`` sur la polyligne ``P`` (segments)."""
    P = np.asarray(P, dtype=float)
    pt = np.asarray(pt, dtype=float).ravel()
    s_cum = 0.0
    for i in range(P.shape[0] - 1):
        p0 = P[i]
        p1 = P[i + 1]
        v = p1 - p0
        L = float(np.linalg.norm(v))
        if L < 1e-15:
            continue
        t = float(np.dot(pt - p0, v)) / (L * L)
        t = max(0.0, min(1.0, t))
        proj = p0 + t * v
        if float(np.linalg.norm(pt - proj)) <= 1e-4 * max(L, 1.0):
            return s_cum + t * L
        s_cum += L
    return float(np.linalg.norm(pt - P[0]))


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

        Lp = (
            float(np.sum(np.linalg.norm(np.diff(P_clip, axis=0), axis=1)))
            if P_clip.shape[0] > 1
            else 0.0
        )
        s_c = _s_along_polyline_closest(P_clip, C)

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
                **cable_polyline_hover_plotly_kwargs(P[:, 0], P[:, 1]),
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
                **cable_polyline_hover_plotly_kwargs(P_clip[:, 0], P_clip[:, 1]),
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
                hovertext=[cable_vertex_tooltip("H", 0.5 * Lp, None, float(H[0]), float(H[1]))],
                hovertemplate=CABLE_XY_HOVERTEMPLATE,
                hoverinfo="text",
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
                hovertext=[cable_vertex_tooltip("C", s_c, None, float(C[0]), float(C[1]))],
                hovertemplate=CABLE_XY_HOVERTEMPLATE,
                hoverinfo="text",
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
        (x_rng, y_rng) = data_ranges_for_cable_view(xs, ys, pad_frac=0.08)
        fig.update_xaxes(title_text="x", range=list(x_rng), row=row, col=1)
        fig.update_yaxes(title_text="y", range=list(y_rng), row=row, col=1)

    w_px, h_px, margin = figure_layout_square_subplots(n_cases, margin_r=120, margin_t=100)
    fig.update_layout(
        title="Visualisation create_point_with_target_length",
        template="plotly_white",
        width=w_px,
        height=h_px,
        margin=margin,
        autosize=False,
        hovermode="closest",
    )

    if output_file is None:
        rel = get_visual_default_html_path("visual_create_point_with_target_length")
        if rel is None:
            raise RuntimeError("Catalogue: entrée visual_create_point_with_target_length introuvable.")
        output_file = Path(rel)
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_plotly_html_with_generation_line(fig, output_path, include_plotlyjs="inline")
    return output_path


def main() -> None:
    output_path = generate_create_point_report()
    print("Rapport visuel create_point_with_target_length genere :", output_path)


if __name__ == "__main__":
    main()

