"""
Rapport visuel Plotly pour CableSolver._normalize_cable_segments.

Génère des polylignes _P avec 2 à 12 points (extrémités incluses) et affiche le câble
normalisé (x_new, y_new) pour un N_target donné.
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

from src.solvers.cable_solver import CableSolver  # noqa: E402


class _DummyEnv:
    pass


def _make_solver() -> CableSolver:
    params = {"d": 0.01, "rho_cable": 1500.0}
    return CableSolver(N_segments=10, params=params, environment=_DummyEnv())


@dataclass(frozen=True)
class Case:
    name: str
    P: np.ndarray
    L_target: float
    N_target: int


def _build_cases() -> list[Case]:
    rng = np.random.default_rng(2026)
    cases: list[Case] = []
    for n in range(2, 13):
        xs = np.linspace(0.0, 10.0, n)
        ys = -rng.random(n) * 3.0
        ys[0] = 0.0
        ys[-1] = -2.0
        P = np.stack([xs, ys], axis=1)
        # longueur actuelle
        L_seg = float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))
        # cible : un peu plus longue pour avoir du slack
        L_target = L_seg * 1.15
        N_target = max(3, min(10, n + 1))
        cases.append(Case(name=f"N{n}", P=P, L_target=L_target, N_target=N_target))
    # Cas fallback (L_target trop court)
    P = np.array([[0.0, 0.0], [10.0, -2.0]], dtype=float)
    cases.append(Case(name="Fallback_L_too_short", P=P, L_target=5.0, N_target=6))
    return cases


def generate_report(output_file: str | Path | None = None) -> Path:
    solver = _make_solver()
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
        P = np.asarray(case.P, dtype=float)
        bateau = (float(P[0, 0]), float(P[0, 1]))
        rov = (float(P[-1, 0]), float(P[-1, 1]))

        ok, x_new, y_new = solver._normalize_cable_segments(
            x_cable=P[:, 0],
            y_cable=P[:, 1],
            L_target=case.L_target,
            bateau=bateau,
            rov=rov,
            N_target=case.N_target,
        )
        # Métriques P/Q pour l'encart
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
            ),
            row=row,
            col=1,
        )

        # encart
        fig.add_annotation(
            xref=f"x{row}" if row > 1 else "x",
            yref=f"y{row}" if row > 1 else "y",
            x=float(x_new[-1]),
            y=float(y_new[-1]),
            text=(
                f"ok={ok}"
                f"<br>Nb points _P = {P.shape[0]}"
                f"<br>Longueur _P = {Lp:.3f}"
                f"<br>Min seg _P = {minP:.3f}"
                f"<br>Max seg _P = {maxP:.3f}"
                f"<br>Nb points _Q = {len(x_new)}"
                f"<br>Longueur _Q = {Lq:.3f}"
                f"<br>Min seg _Q = {minQ:.3f}"
                f"<br>Max seg _Q = {maxQ:.3f}"
                f"<br>N_target = {case.N_target}"
                f"<br>L_target = {case.L_target:.3f}"
            ),
            showarrow=True,
            arrowhead=1,
            ax=190,
            ay=0,
            align="left",
        )

        xs = np.concatenate([P[:, 0], np.asarray(x_new, dtype=float)])
        ys = np.concatenate([P[:, 1], np.asarray(y_new, dtype=float)])
        span = max(float(xs.max() - xs.min()), float(ys.max() - ys.min()), 1.0)
        cx = 0.5 * float(xs.max() + xs.min())
        cy = 0.5 * float(ys.max() + ys.min())
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
        title="Visualisation _normalize_cable_segments",
        template="plotly_white",
        height=max(320 * n_cases, 700),
        width=900,
        hovermode="closest",
    )

    if output_file is None:
        output_file = Path("results") / "cable_normalize_segments_visual_tests.html"
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(output_path), include_plotlyjs="inline")
    return output_path


def main() -> None:
    output_path = generate_report()
    print("Rapport visuel _normalize_cable_segments genere :", output_path)


if __name__ == "__main__":
    main()

