"""
Rapport visuel Plotly pour next_point.

Génère des configurations de _Q avec un nombre de points de 2 à 12 (extrémités incluses).
Affiche R et T selon :
- s_T = s_R + step
- T est calculé via next_point(_Q, _ls, Lseg_total, R, num_seg_R, s_R, step)
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
from src.utils.utils import next_point  # noqa: E402
from tests._plotly_cable_axes import (  # noqa: E402
    data_ranges_for_cable_view,
    figure_layout_square_subplots,
)
from tests._report_output import write_plotly_html_with_generation_line  # noqa: E402


@dataclass(frozen=True)
class NextPointCase:
    name: str
    Q: np.ndarray
    num_seg: int
    s_R: float
    step: float


def _ls_from_Q(Q: np.ndarray) -> tuple[np.ndarray, float]:
    Q = np.asarray(Q, dtype=float)[:, :2]
    lseg = np.linalg.norm(Q[1:] - Q[:-1], axis=1)
    ls = np.zeros(Q.shape[0], dtype=float)
    ls[1:] = np.cumsum(lseg)
    return ls, float(ls[-1])


def _point_at_s(Q: np.ndarray, ls: np.ndarray, seg_idx: int, s: float) -> np.ndarray:
    Q = np.asarray(Q, dtype=float)[:, :2]
    seg_start = float(ls[seg_idx])
    seg_end = float(ls[seg_idx + 1])
    seg_len = seg_end - seg_start
    if abs(seg_len) <= 1e-15:
        return Q[seg_idx].copy()
    t = (float(s) - seg_start) / seg_len
    t = float(np.clip(t, 0.0, 1.0))
    return (1.0 - t) * Q[seg_idx] + t * Q[seg_idx + 1]


def _build_cases() -> list[NextPointCase]:
    rng = np.random.default_rng(1234)
    cases: list[NextPointCase] = []
    for n in range(2, 13):
        # Points globalement sous la surface : y <= 0
        xs = np.linspace(0.0, float(n - 1), n)
        ys = -rng.random(n) * 3.0
        # Un peu de structure pour éviter les polylignes dégénérées
        ys[0] = 0.0
        ys[-1] = -0.8

        Q = np.stack([xs, ys], axis=1)
        ls, total = _ls_from_Q(Q)

        # Choisir un segment de départ au milieu
        num_seg = min(max((n - 2) // 2, 0), n - 2)
        s_seg0 = float(ls[num_seg])
        s_seg1 = float(ls[num_seg + 1])
        # Choisir R quelque part dans le segment (pas forcément à une extrémité)
        s_R = s_seg0 + 0.35 * (s_seg1 - s_seg0)

        # Choisir step pour arriver dans le futur sans dépasser la fin
        remaining = total - s_R
        step = 0.35 * total
        if step > remaining:
            step = 0.6 * remaining
        if step <= 0:
            step = 0.1 * remaining if remaining > 0 else 0.0

        cases.append(
            NextPointCase(
                name=f"N{n}",
                Q=Q,
                num_seg=num_seg,
                s_R=s_R,
                step=step,
            )
        )
    return cases


def generate_next_point_report(output_file: str | Path | None = None) -> Path:
    cases = _build_cases()
    n_cases = len(cases)

    fig = make_subplots(
        rows=n_cases,
        cols=1,
        specs=[[{"type": "scatter"}] for _ in range(n_cases)],
        subplot_titles=[c.name for c in cases],
        vertical_spacing=0.06,
    )

    for idx, case in enumerate(cases):
        row = idx + 1
        Q = np.asarray(case.Q, dtype=float)
        ls, L_total = _ls_from_Q(Q)

        R = _point_at_s(Q, ls, seg_idx=case.num_seg, s=case.s_R)
        ns, T = next_point(
            Q,
            ls,
            L_total,
            R,
            num_seg_R=case.num_seg,
            s_R=case.s_R,
            step=case.step,
        )
        target_s = case.s_R + case.step

        assert ns is not None and T is not None, "Le cas généré doit rester dans la plage."

        A = Q[case.num_seg]
        B = Q[case.num_seg + 1]
        T0 = Q[ns]
        T1 = Q[ns + 1]

        fig.add_trace(
            go.Scatter(
                x=Q[:, 0],
                y=Q[:, 1],
                mode="lines+markers",
                name="Q",
                legendgroup="Q",
                showlegend=(idx == 0),
                line=dict(color="#7f7f7f", width=2),
                marker=dict(size=7),
            ),
            row=row,
            col=1,
        )

        # Segment de départ (num_seg)
        fig.add_trace(
            go.Scatter(
                x=[A[0], B[0]],
                y=[A[1], B[1]],
                mode="lines",
                name="Segment départ",
                legendgroup="start_seg",
                showlegend=(idx == 0),
                line=dict(color="#ff7f0e", width=3, dash="dash"),
            ),
            row=row,
            col=1,
        )

        # Segment trouvé (ns)
        fig.add_trace(
            go.Scatter(
                x=[T0[0], T1[0]],
                y=[T0[1], T1[1]],
                mode="lines",
                name="Segment cible",
                legendgroup="target_seg",
                showlegend=(idx == 0),
                line=dict(color="#d62728", width=3),
            ),
            row=row,
            col=1,
        )

        # Point R
        fig.add_trace(
            go.Scatter(
                x=[R[0]],
                y=[R[1]],
                mode="markers+text",
                text=["R"],
                textposition="top center",
                showlegend=False,
                marker=dict(size=12, color="#d62728"),
            ),
            row=row,
            col=1,
        )

        # Point T
        fig.add_trace(
            go.Scatter(
                x=[T[0]],
                y=[T[1]],
                mode="markers+text",
                text=["T"],
                textposition="top center",
                showlegend=False,
                marker=dict(size=12, color="#2ca02c"),
            ),
            row=row,
            col=1,
        )

        # Encarts : info à droite
        xs = Q[:, 0]
        ys = Q[:, 1]
        (x_rng, y_rng) = data_ranges_for_cable_view(xs, ys, pad_frac=0.08)
        fig.update_xaxes(title_text="x", range=list(x_rng), row=row, col=1)
        fig.update_yaxes(title_text="y", range=list(y_rng), row=row, col=1)

        fig.add_annotation(
            xref=f"x{row}" if row > 1 else "x",
            yref=f"y{row}" if row > 1 else "y",
            x=float(T[0]),
            y=float(T[1]),
            text=(
                f"Coordonnées R = ({R[0]:.3f}, {R[1]:.3f})"
                f"<br>Numéro segment R = {case.num_seg}"
                f"<br>Abscisse curviligne R = {case.s_R:.3f}"
                f"<br>step = {case.step:.3f}"
                f"<br>Coordonnées T = ({T[0]:.3f}, {T[1]:.3f})"
                f"<br>Numéro segment T = {ns}"
                f"<br>Abscisse curviligne T = {target_s:.3f}"
            ),
            showarrow=True,
            arrowhead=1,
            ax=190,
            ay=0,
            align="left",
        )

    w_px, h_px, margin = figure_layout_square_subplots(n_cases, margin_r=120, margin_t=100)
    fig.update_layout(
        title="Visualisation next_point",
        template="plotly_white",
        width=w_px,
        height=h_px,
        margin=margin,
        autosize=False,
        hovermode="closest",
    )

    if output_file is None:
        rel = get_visual_default_html_path("visual_next_point")
        if rel is None:
            raise RuntimeError("Catalogue: entrée visual_next_point introuvable.")
        output_file = Path(rel)
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_plotly_html_with_generation_line(fig, output_path, include_plotlyjs="inline")
    return output_path


def main() -> None:
    output_path = generate_next_point_report()
    print("Rapport visuel next_point genere :", output_path)


if __name__ == "__main__":
    main()

