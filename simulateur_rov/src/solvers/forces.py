"""Calcul des forces sur le câble"""
import numpy as np


def compute_cable_drag_force(v_rel, d, Cx_cable, rho_eau, ds):
    """
    Calcule la force de traînée sur un segment de câble
    
    Parameters:
    -----------
    v_rel : float ou array
        Vitesse relative du fluide par rapport au câble (m/s)
    d : float
        Diamètre du câble (m)
    Cx_cable : float
        Coefficient de traînée du câble
    rho_eau : float
        Masse volumique de l'eau (kg/m³)
    ds : float
        Longueur du segment (m)
    
    Returns:
    --------
    float ou array
        Force de traînée par segment (N)
    """
    return Cx_cable * 0.5 * rho_eau * d * v_rel * np.abs(v_rel) * ds


def compute_cable_apparent_weight(rho_cable, rho_eau, A_cable, g, ds):
    """
    Calcule le poids apparent d'un segment de câble
    
    Parameters:
    -----------
    rho_cable : float
        Masse volumique du câble (kg/m³)
    rho_eau : float
        Masse volumique de l'eau (kg/m³)
    A_cable : float
        Section transversale du câble (m²)
    g : float
        Accélération de la pesanteur (m/s²)
    ds : float
        Longueur du segment (m)
    
    Returns:
    --------
    float
        Poids apparent vers le bas (N)
        Pour un câble plus dense que l'eau (rho_cable > rho_eau), retourne une valeur négative
        car dans le système de coordonnées utilisé, y < 0 signifie "vers le bas"
    """
    # Le poids apparent est dirigé vers le bas, donc négatif dans le système de coordonnées
    # où y < 0 signifie "vers le bas"
    return -(rho_cable - rho_eau) * A_cable * g * ds


def compute_cable_forces(x_cable, y_cable, vx_cable, vy_cable, 
                         environment, params, L):
    """
    Calcule les forces sur tous les segments du câble
    
    Parameters:
    -----------
    x_cable : array
        Positions horizontales des points du câble (m)
    y_cable : array
        Positions verticales des points du câble (m)
    vx_cable : array
        Vitesses horizontales des points du câble (m/s)
    vy_cable : array
        Vitesses verticales des points du câble (m/s)
    environment : Environment
        Objet environnement
    params : dict
        Paramètres du câble (d, rho_cable, Cx_cable)
    L : float
        Longueur actuelle du câble (m)
    
    Returns:
    --------
    tuple (Fx, Fy)
        Forces horizontales et verticales par segment (N)
    """
    N = len(x_cable) - 1
    ds = L / N if N > 0 else 0.1
    
    d = params['d']
    rho_cable = params['rho_cable']
    Cx_cable = params['Cx_cable']
    A_cable = np.pi * (d / 2)**2
    
    # Vitesses du courant à chaque point
    v_courant = getattr(environment, "v_courant_raw", None)
    v_current = np.array([environment.get_current_velocity(y, v_courant) for y in y_cable])
    
    # Vitesses relatives
    vx_rel = v_current - vx_cable
    vy_rel = -vy_cable  # Le courant est horizontal
    
    # Forces par segment (moyenne des deux extrémités)
    Fx = np.zeros(N)
    Fy = np.zeros(N)
    
    for i in range(N):
        # Traînée horizontale
        vx_rel_avg = 0.5 * (vx_rel[i] + vx_rel[i+1])
        Fx[i] = compute_cable_drag_force(vx_rel_avg, d, Cx_cable, 
                                        environment.rho_eau, ds)
        
        # Traînée verticale
        vy_rel_avg = 0.5 * (vy_rel[i] + vy_rel[i+1])
        Fy_drag = compute_cable_drag_force(vy_rel_avg, d, Cx_cable,
                                          environment.rho_eau, ds)
        
        # Poids apparent
        Fy_weight = compute_cable_apparent_weight(rho_cable, environment.rho_eau,
                                                 A_cable, environment.g, ds)
        
        Fy[i] = Fy_weight + Fy_drag
    
    return Fx, Fy

