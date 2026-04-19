"""
Intégration half-explicit : pas RK4 avec projection géométrique du câble après chaque sous-pas.

Voir docs/dae_cable_rov_spec.md.
"""
from __future__ import annotations

from typing import Any, Callable

import numpy as np

from src.models.state_projection import project_cable_state_inplace


def _rk4_step(
    system: Any,
    t: float,
    y: np.ndarray,
    dt: float,
    u_func: Callable[[float], dict],
) -> np.ndarray:
    """Un pas de Runge–Kutta 4."""
    y = np.asarray(y, dtype=float)
    u1 = u_func(t)
    k1 = system.compute_derivatives(t, y, u1)
    u2 = u_func(t + 0.5 * dt)
    k2 = system.compute_derivatives(t + 0.5 * dt, y + 0.5 * dt * k1, u2)
    k3 = system.compute_derivatives(t + 0.5 * dt, y + 0.5 * dt * k2, u2)
    u4 = u_func(t + dt)
    k4 = system.compute_derivatives(t + dt, y + dt * k3, u4)
    return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


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

    Après chaque sous-pas, si project=True, ramène le câble sur la variété
    (normalize + tensions) via project_cable_state_inplace.
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
        if project:
            project_cable_state_inplace(system, y, t, k_tail=k_tail)
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
