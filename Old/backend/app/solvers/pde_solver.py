"""PDE solver utilities for cable equations"""

import numpy as np
from scipy.optimize import root


def finite_difference_derivative(f, ds, method='central'):
    """
    Calcule la dérivée spatiale avec différences finies
    
    Args:
        f: Array de valeurs
        ds: Pas spatial
        method: 'forward', 'backward', ou 'central'
    
    Returns:
        Dérivée df/ds
    """
    if method == 'forward':
        return np.diff(f) / ds
    elif method == 'backward':
        return np.diff(f[::-1])[::-1] / ds
    else:  # central
        df = np.zeros_like(f)
        df[1:-1] = (f[2:] - f[:-2]) / (2 * ds)
        df[0] = (f[1] - f[0]) / ds
        df[-1] = (f[-1] - f[-2]) / ds
        return df

