"""
Intégration half-explicit : pas RK4 à géométrie de câble figée, puis un solveur
et une projection géométrique une seule fois en fin de pas.

Voir docs/dae_cable_rov_spec.md.
"""
from __future__ import annotations

from typing import Any, Callable

import numpy as np

from src.models.state_projection import (
    apply_slack_tension_soft_cap_inplace,
    project_cable_state_inplace,
)


def _rk4_step(
    system: Any,
    t: float,
    y: np.ndarray,
    dt: float,
    u_func: Callable[[float], dict],
) -> np.ndarray:
    """Un pas de Runge–Kutta 4, géométrie de câble figée (pas de solveur dans f)."""
    y = np.asarray(y, dtype=float)
    previous_freeze = bool(getattr(system, "_freeze_cable_geometry", False))
    system._freeze_cable_geometry = True
    try:
        u1 = u_func(t)
        k1 = system.compute_derivatives(t, y, u1)
        u2 = u_func(t + 0.5 * dt)
        k2 = system.compute_derivatives(t + 0.5 * dt, y + 0.5 * dt * k1, u2)
        k3 = system.compute_derivatives(t + 0.5 * dt, y + 0.5 * dt * k2, u2)
        u4 = u_func(t + dt)
        k4 = system.compute_derivatives(t + dt, y + dt * k3, u4)
        return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    finally:
        system._freeze_cable_geometry = previous_freeze


class TabulatedOdeSolution:
    """
    Résultat d'intégration par pas, avec les attributs utilisés par l'interface
    et les tests (``success``, ``message``, ``t``, ``y``, ``sol``, ``nfev``).
    """

    def __init__(self, t, states, *, success: bool = True, message: str = "ok"):
        self.t = np.asarray(t, dtype=float)
        columns = [np.asarray(state, dtype=float) for state in states]
        self.y = np.column_stack(columns) if columns else np.zeros((0, 0))
        self.success = bool(success)
        self.message = str(message)
        self.nfev = 4 * max(int(self.t.size) - 1, 1)
        self.sol = self._interpolate

    def _interpolate(self, t_query):
        scalar = np.isscalar(t_query)
        query = np.atleast_1d(np.asarray(t_query, dtype=float))
        if self.t.size == 1:
            column = np.repeat(self.y[:, 0:1], query.size, axis=1)
        else:
            column = np.vstack(
                [np.interp(query, self.t, self.y[i]) for i in range(self.y.shape[0])]
            )
        if scalar:
            return column[:, 0]
        return column


def integrate_span_with_projection(
    system: Any,
    t_span: tuple[float, float],
    y0: np.ndarray,
    u_func: Callable[[float], dict],
    *,
    n_substeps: int = 4,
    k_tail: int = 10,
    project: bool = True,
) -> np.ndarray:
    """
    Intègre de t_span[0] à t_span[1] par n_substeps pas RK4.

    Après les sous-pas, un seul appel au solveur de câble puis, si project=True,
    une seule projection géométrique (normalisation de longueur). La géométrie
    n'est pas rééchantillonnée à l'intérieur de Runge–Kutta.
    """
    t0, t1 = float(t_span[0]), float(t_span[1])
    y = np.asarray(y0, dtype=float).copy()
    if t1 <= t0:
        return y
    n_substeps = max(1, int(n_substeps))
    dt = (t1 - t0) / n_substeps
    t = t0
    for _ in range(n_substeps):
        y = _rk4_step(system, t, y, dt, u_func)
        t += dt
    if hasattr(system, "refresh_cable_equilibrium_inplace"):
        try:
            system.refresh_cable_equilibrium_inplace(y)
        except Exception:
            pass
    if project:
        project_cable_state_inplace(system, y, t, k_tail=k_tail)
    # Sécurité : projection échouée ou dérive RK sur T → même logique que GUARD slack.
    apply_slack_tension_soft_cap_inplace(system, y, t_sim=t1)
    return y


def integrate_span_with_projection_simple(
    system: Any,
    t_span: tuple[float, float],
    y0: np.ndarray,
    u_func: Callable[[float], dict],
    dt_max: float,
    *,
    n_substeps: int = 4,
    k_tail: int = 10,
) -> np.ndarray:
    """
    Même chose que integrate_span_with_projection ; dt_max conservé pour compatibilité d'API
    (la taille du pas UI est |t1-t0| ; n_substeps contrôle le sous-découpage interne).
    """
    _ = dt_max
    return integrate_span_with_projection(
        system, t_span, y0, u_func, n_substeps=n_substeps, k_tail=k_tail, project=True
    )
