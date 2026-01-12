"""Physical constants and force calculations"""

import numpy as np

# Physical constants
G = 9.81  # m/s²
RHO_WATER = 1025.0  # kg/m³ (eau de mer)
MU_WATER = 0.001  # Pa·s (viscosité dynamique de l'eau)

def compute_drag_force_quadratic(C_drag, rho, area, v_rel):
    """
    Calcule la force de traînée quadratique (régime turbulent)
    
    Args:
        C_drag: Coefficient de traînée
        rho: Masse volumique du fluide (kg/m³)
        area: Surface frontale (m²)
        v_rel: Vitesse relative (m/s) - peut être un vecteur
    
    Returns:
        Force de traînée (N)
    """
    v_norm = np.linalg.norm(v_rel) if isinstance(v_rel, np.ndarray) else abs(v_rel)
    if v_norm == 0:
        return 0.0
    
    F_drag = -0.5 * C_drag * rho * area * v_norm * v_rel
    return F_drag

def compute_drag_force_linear(C_drag, mu, d, v_rel):
    """
    Calcule la force de traînée linéaire (régime laminaire)
    
    Args:
        C_drag: Coefficient de traînée
        mu: Viscosité dynamique (Pa·s)
        d: Diamètre caractéristique (m)
        v_rel: Vitesse relative (m/s)
    
    Returns:
        Force de traînée par unité de longueur (N/m)
    """
    F_drag = -C_drag * mu * v_rel * d
    return F_drag

def current_velocity_uniform(y, V_courant):
    """Courant uniforme (constant avec la profondeur)"""
    return V_courant

def current_velocity_linear(y, V_surface, y_max):
    """Courant linéaire décroissant avec la profondeur"""
    if y_max <= 0:
        return V_surface
    return V_surface * (1 - y / y_max) if y < y_max else 0.0

