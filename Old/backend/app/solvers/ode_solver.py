"""ODE solver utilities"""

from scipy.integrate import solve_ivp
import numpy as np


def solve_ode_rk4(derivative_func, t_span, y0, dt, **kwargs):
    """
    Résout une ODE avec Runge-Kutta d'ordre 4
    
    Args:
        derivative_func: Fonction dérivée f(t, y)
        t_span: [t0, tf]
        y0: Condition initiale
        dt: Pas de temps
        **kwargs: Arguments supplémentaires pour solve_ivp
    
    Returns:
        Solution à tf
    """
    sol = solve_ivp(
        derivative_func,
        t_span,
        y0,
        method='RK45',
        dense_output=False,
        rtol=kwargs.get('rtol', 1e-6),
        atol=kwargs.get('atol', 1e-8)
    )
    
    if not sol.success:
        raise RuntimeError(f"Échec intégration ODE: {sol.message}")
    
    return sol.y[:, -1]

