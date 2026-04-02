"""
Rapport visuel Plotly pour CableSolver._normalize_cable_length.

Les cas sont ceux de ``tests/cable_shared_cases.py``. En cas d'exception, l'encart
l'indique et le tracé « Q » reprend le profil d'entrée P.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.tests.test_catalog import get_visual_default_html_path  # noqa: E402
from src.solvers.cable_solver import CableSolver  # noqa: E402
from src.visualization.cable_hover import cable_polyline_hover_plotly_kwargs  # noqa: E402
from tests._plotly_cable_axes import (  # noqa: E402
    data_ranges_for_cable_view,
    figure_layout_square_subplots,
    subplot_vertical_spacing,
)
from tests._report_output import write_plotly_html_with_generation_line  # noqa: E402
from tests.cable_shared_cases import ALL_SHARED_CABLE_CASES  # noqa: E402


class _DummyEnv:
    pass


def _make_solver() -> CableSolver:
    params = {"d": 0.01, "rho_cable": 1500.0}
    return CableSolver(N_segments=50, params=params, environment=_DummyEnv())


def generate_report(output_file: str | Path | None = None) -> Path:
    solver = _make_solver()
    cases = list(ALL_SHARED_CABLE_CASES)
    n_cases = len(cases)

    fig = make_subplots(
        rows=n_cases,
        cols=1,
        specs=[[{"type": "scatter"}] for _ in range(n_cases)],
        subplot_titles=[c.name for c in cases],
        vertical_spacing=subplot_vertical_spacing(n_cases),
    )

    for idx, case in enumerate(cases):
        row = idx + 1
        P = np.asarray(case.P, dtype=float)
        bateau = case.boat
        rov = case.rov

        seg_error: str | None = None
        straight_mode = False
        ncl_source = "—"
        try:
            x_new, y_new, straight_mode, ncl_source = solver._normalize_cable_length(
                P[:, 0],
                P[:, 1],
                case.l_target,
                x_boat=float(bateau[0]),
                y_boat=float(bateau[1]),
                x_rov=float(rov[0]),
                y_rov=float(rov[1]),
                mode_test=True,
            )
            x_new = np.asarray(x_new, dtype=float)
            y_new = np.asarray(y_new, dtype=float)
        except Exception as e:
            seg_error = str(e)
            x_new = P[:, 0].copy()
            y_new = P[:, 1].copy()

        segP = np.linalg.norm(np.diff(P, axis=0), axis=1)
        Lp = float(np.sum(segP)) if segP.size else 0.0
        minP = float(np.min(segP)) if segP.size else 0.0
        maxP = float(np.max(segP)) if segP.size else 0.0

        Q = np.stack([np.asarray(x_new, dtype=float), np.asarray(y_new, dtype=float)], axis=1)
        segQ = np.linalg.norm(np.diff(Q, axis=0), axis=1)
        Lq = float(np.sum(segQ)) if segQ.size else 0.0
        minQ = float(np.min(segQ)) if segQ.size else 0.0
        maxQ = float(np.max(segQ)) if segQ.size else 0.0

        fig.add_trace(
            go.Scatter(
                x=P[:, 0],
                y=P[:, 1],
                mode="lines+markers",
                name="P",
                legendgroup="P",
                showlegend=(idx == 0),
                line=dict(color="#7f7f7f", width=2, dash="dot"),
                marker=dict(size=6),
                **cable_polyline_hover_plotly_kwargs(P[:, 0], P[:, 1]),
            ),
            row=row,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=x_new,
                y=y_new,
                mode="lines+markers",
                name="Q",
                legendgroup="Q",
                showlegend=(idx == 0),
                line=dict(color="#d62728", width=2),
                marker=dict(size=7),
                **cable_polyline_hover_plotly_kwargs(x_new, y_new),
            ),
            row=row,
            col=1,
        )

        xs = np.concatenate([P[:, 0], np.asarray(x_new, dtype=float)])
        ys = np.concatenate([P[:, 1], np.asarray(y_new, dtype=float)])
        (x_rng, y_rng) = data_ranges_for_cable_view(xs, ys, pad_frac=0.08)
        y_min = float(y_rng[0])
        y_max = float(y_rng[1])

        fig.add_annotation(
            xref="paper",
            yref=f"y{row}" if row > 1 else "y",
            x=1.08,
            y=y_max - 0.05 * max(y_max - y_min, 1.0),
            text=(
                f"<b>{case.name}</b><br>straight_mode={straight_mode}"
                + f"<br><b>source</b> = {ncl_source}"
                + (f"<br><span style='color:#b00'>Exception : {seg_error}</span>" if seg_error else "")
                + f"<br>Nb points _P = {P.shape[0]}"
                + f"<br>Longueur _P = {Lp:.3f}"
                + f"<br>Min seg _P = {minP:.3f}"
                + f"<br>Max seg _P = {maxP:.3f}"
                + f"<br>Nb points _Q = {len(x_new)}"
                + f"<br>Longueur _Q = {Lq:.3f}"
                + f"<br>Min seg _Q = {minQ:.3f}"
                + f"<br>Max seg _Q = {maxQ:.3f}"
                + f"<br>n_target (cas) = {case.n_target}"
                + f"<br>L_target = {case.l_target:.3f}"
            ),
            showarrow=False,
            align="left",
            bgcolor="rgba(255,255,255,0.92)",
            bordercolor="#999",
            borderwidth=1,
            font=dict(size=9),
        )

        fig.update_xaxes(title_text="x", range=list(x_rng), row=row, col=1)
        fig.update_yaxes(title_text="y", range=list(y_rng), row=row, col=1)

    w_px, h_px, margin = figure_layout_square_subplots(n_cases, margin_r=400, margin_t=100)
    fig.update_layout(
        title="Visualisation _normalize_cable_length",
        template="plotly_white",
        width=w_px,
        height=h_px,
        margin=margin,
        autosize=False,
        hovermode="closest",
    )

    if output_file is None:
        rel = get_visual_default_html_path("visual_cable_normalize_length")
        if rel is None:
            raise RuntimeError("Catalogue: entrée visual_cable_normalize_length introuvable.")
        output_file = Path(rel)
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_plotly_html_with_generation_line(fig, output_path, include_plotlyjs="inline")
    return output_path


def main() -> None:
    output_path = generate_report()
    print("Rapport visuel _normalize_cable_length genere :", output_path)


if __name__ == "__main__":
    main()
