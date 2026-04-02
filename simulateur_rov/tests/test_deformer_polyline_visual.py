"""
Visualisation graphique pour la fonction CableSolver.deformer_polyline.

Ce script génère un rapport HTML Plotly montrant :
- la polyligne d'entrée Q,
- sa projection orthogonale sur la droite Q[0]Q[-1] (cas k = 0),
- la polyligne déformée R trouvée (si possible) pour atteindre une longueur cible L_target.
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

from src.models.environment import Environment  # noqa: E402
from src.solvers.cable_solver import CableSolver  # noqa: E402
from src.tests.test_catalog import get_visual_default_html_path  # noqa: E402
from src.visualization.cable_hover import cable_polyline_hover_plotly_kwargs  # noqa: E402
from tests._plotly_cable_axes import (  # noqa: E402
    data_ranges_for_cable_view,
    figure_layout_square_subplots,
    subplot_vertical_spacing,
)
from tests._report_output import write_plotly_html_with_generation_line  # noqa: E402


def _make_solver() -> CableSolver:
    params = {"d": 0.01, "rho_cable": 1500.0}
    env = Environment({"rho_eau": 1025.0, "g": 9.81})
    return CableSolver(N_segments=10, params=params, environment=env)


def _polyline_length(points: np.ndarray) -> float:
    points = np.asarray(points, dtype=float)
    diffs = np.diff(points, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def _projection_orthogonale(Q: np.ndarray) -> np.ndarray:
    Q = np.asarray(Q, dtype=float)
    q0 = Q[0]
    qn = Q[-1]
    d = qn - q0
    den = float(np.dot(d, d))
    if den <= 1e-18:
        return np.broadcast_to(q0, Q.shape)
    t_proj = np.dot(Q - q0, d) / den
    return q0 + t_proj[:, None] * d


def _estimate_k_from_Q_Qproj_R(Q: np.ndarray, Q_proj: np.ndarray, R: np.ndarray) -> float | None:
    """
    Estime k dans R = Q_proj + k*(Q - Q_proj) par moindres carrés.
    Retourne None si non identifiable (Q ~= Q_proj).
    """
    Q = np.asarray(Q, dtype=float)
    Q_proj = np.asarray(Q_proj, dtype=float)
    R = np.asarray(R, dtype=float)
    V = (Q - Q_proj).reshape(-1)
    W = (R - Q_proj).reshape(-1)
    den = float(np.dot(V, V))
    if den <= 1e-18:
        return None
    return float(np.dot(V, W) / den)


@dataclass(frozen=True)
class DeformerPolylineCase:
    name: str
    Q: np.ndarray
    L_target: float


def _build_cases(solver: CableSolver) -> list[DeformerPolylineCase]:
    # Cas non aligné "simple" : endpoints sur y=0, point milieu à y=1.
    Q_base = np.array([[0.0, 0.0], [0.5, 1.0], [1.0, 0.0]], dtype=float)
    R_k2, _ = solver.aplatir_polyline(Q_base, 2.0)
    assert R_k2 is not None
    L_k2 = _polyline_length(R_k2)

    # Cas non aligné mais cible plus proche de la longueur minimale (k proche de 0).
    R_k02, _ = solver.aplatir_polyline(Q_base, 0.2)
    assert R_k02 is not None
    L_k02 = _polyline_length(R_k02)

    # Cas aligné : deformer_polyline doit retourner (None, "").
    Q_aligned = np.array([[0.0, 0.0], [0.5, 0.0], [1.0, 0.0]], dtype=float)

    # Cas impossible : plus court que la longueur minimale (k=0).
    Q_proj = _projection_orthogonale(Q_base)
    L_min = _polyline_length(Q_proj)

    # Cas "dizaine de points" (11 points -> 10 segments), non aligné.
    # On construit un profil en cloche puis on l'ancre sur y=0 aux extrémités.
    x10 = np.linspace(0.0, 10.0, 11)
    y10 = np.array([0.0, 0.25, 0.55, 0.9, 1.1, 1.2, 1.05, 0.75, 0.45, 0.2, 0.0], dtype=float)
    Q_10 = np.column_stack([x10, y10]).astype(float)
    T_lt, _ = solver.aplatir_polyline(Q_10, 0.6)
    T_gt, _ = solver.aplatir_polyline(Q_10, 1.4)
    assert T_lt is not None and T_gt is not None
    L_10_k_lt_1 = _polyline_length(T_lt)
    L_10_k_gt_1 = _polyline_length(T_gt)

    return [
        DeformerPolylineCase(name="Atteignable_k_gt_1", Q=Q_base, L_target=L_k2),
        DeformerPolylineCase(name="Atteignable_k_lt_1", Q=Q_base, L_target=L_k02),
        DeformerPolylineCase(name="Dizaine_points_k_lt_1", Q=Q_10, L_target=L_10_k_lt_1),
        DeformerPolylineCase(name="Dizaine_points_k_gt_1", Q=Q_10, L_target=L_10_k_gt_1),
        DeformerPolylineCase(name="Impossible_L_trop_court", Q=Q_base, L_target=max(0.0, L_min - 0.2)),
        DeformerPolylineCase(name="Aligne_retourne_None", Q=Q_aligned, L_target=1.0),
    ]


def generate_deformer_polyline_report(output_file: str | Path | None = None) -> Path:
    solver = _make_solver()
    cases = _build_cases(solver)
    n_cases = len(cases)

    fig = make_subplots(
        rows=n_cases,
        cols=1,
        specs=[[{"type": "scatter"}] for _ in range(n_cases)],
        subplot_titles=[case.name for case in cases],
        vertical_spacing=subplot_vertical_spacing(n_cases),
    )

    for idx, case in enumerate(cases):
        row = idx + 1
        col = 1
        Q = np.asarray(case.Q, dtype=float)
        L_target = float(case.L_target)

        Q_proj = _projection_orthogonale(Q)
        L_Q = _polyline_length(Q)
        L_min = _polyline_length(Q_proj)

        R, _expl = solver.deformer_polyline(Q, L_target=L_target, atol=1e-9, rtol=1e-9)
        ok = R is not None

        # Tracer : droite de référence (entre endpoints)
        fig.add_trace(
            go.Scatter(
                x=[Q[0, 0], Q[-1, 0]],
                y=[Q[0, 1], Q[-1, 1]],
                mode="lines",
                name="Droite Q0-Qn",
                legendgroup="Ref",
                showlegend=(idx == 0),
                line=dict(color="#7f7f7f", width=1, dash="dash"),
            ),
            row=row,
            col=col,
        )

        # Q (entrée)
        fig.add_trace(
            go.Scatter(
                x=Q[:, 0],
                y=Q[:, 1],
                mode="lines+markers",
                name="Q (entrée)",
                legendgroup="Q",
                showlegend=(idx == 0),
                line=dict(color="#1f77b4", width=2),
                marker=dict(size=7),
                **cable_polyline_hover_plotly_kwargs(Q[:, 0], Q[:, 1]),
            ),
            row=row,
            col=col,
        )

        # Q_proj (k=0)
        fig.add_trace(
            go.Scatter(
                x=Q_proj[:, 0],
                y=Q_proj[:, 1],
                mode="lines+markers",
                name="Q_proj (k=0)",
                legendgroup="Qproj",
                showlegend=(idx == 0),
                line=dict(color="#ff7f0e", width=2, dash="dot"),
                marker=dict(size=7),
                **cable_polyline_hover_plotly_kwargs(Q_proj[:, 0], Q_proj[:, 1]),
            ),
            row=row,
            col=col,
        )

        # R (solution)
        if ok:
            R = np.asarray(R, dtype=float)
            k_est = _estimate_k_from_Q_Qproj_R(Q, Q_proj, R)
            L_R = _polyline_length(R)
            fig.add_trace(
                go.Scatter(
                    x=R[:, 0],
                    y=R[:, 1],
                    mode="lines+markers",
                    name="R (solution)",
                    legendgroup="R",
                    showlegend=(idx == 0),
                    line=dict(color="#2ca02c", width=3),
                    marker=dict(size=7),
                    **cable_polyline_hover_plotly_kwargs(R[:, 0], R[:, 1]),
                ),
                row=row,
                col=col,
            )
            k_txt = "k≈?" if k_est is None else f"k≈{k_est:.4f}"
            status = f"OK ({k_txt})"
        else:
            R = None
            L_R = float("nan")
            status = "ÉCHEC (R=None)"

        # Axes adaptés aux 3 polylignes affichées
        xs = [Q[:, 0], Q_proj[:, 0]]
        ys = [Q[:, 1], Q_proj[:, 1]]
        if R is not None:
            xs.append(R[:, 0])
            ys.append(R[:, 1])
        xs_all = np.concatenate(xs)
        ys_all = np.concatenate(ys)
        x_rng, y_rng = data_ranges_for_cable_view(xs_all, ys_all, pad_frac=0.08)
        fig.update_xaxes(title_text="x", range=list(x_rng), row=row, col=col)
        fig.update_yaxes(title_text="y", range=list(y_rng), row=row, col=col)

        # Encarts numériques
        stats = (
            f"statut: {status}<br>"
            f"L_target = {L_target:.6g}<br>"
            f"L(Q) = {L_Q:.6g}<br>"
            f"L_min(k=0) = {L_min:.6g}"
        )
        if R is not None:
            stats += f"<br>L(R) = {L_R:.6g}<br>err = {abs(L_R - L_target):.3g}"

        fig.add_annotation(
            xref="paper",
            yref=f"y{idx + 1}" if idx > 0 else "y",
            x=1.05,
            y=y_rng[1] - 0.05 * max(y_rng[1] - y_rng[0], 1.0),
            text=stats,
            showarrow=False,
            align="left",
            bgcolor="rgba(255,255,255,0.9)",
            bordercolor="#999",
            borderwidth=1,
            font=dict(size=9),
        )

    w_px, h_px, margin = figure_layout_square_subplots(n_cases, margin_r=320, margin_t=110)
    fig.update_layout(
        title="Visualisation de deformer_polyline (Q, projection k=0, solution R)",
        template="plotly_white",
        width=w_px,
        height=h_px,
        margin=margin,
        autosize=False,
        hovermode="closest",
    )

    if output_file is None:
        rel = get_visual_default_html_path("visual_deformer_polyline")
        if rel is None:
            raise RuntimeError("Catalogue: entrée visual_deformer_polyline introuvable.")
        output_file = Path(rel)
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_plotly_html_with_generation_line(fig, output_path, include_plotlyjs="inline")
    return output_path


def main() -> None:
    output_path = generate_deformer_polyline_report()
    print("Rapport visuel deformer_polyline genere :", output_path)


if __name__ == "__main__":
    main()

