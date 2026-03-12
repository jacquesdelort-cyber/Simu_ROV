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
    
    Utilise un repère local (U, V) pour chaque segment :
    - U : vecteur tangent unitaire (direction du segment)
    - V : vecteur normal unitaire (perpendiculaire à U)
    - Traînée longitudinale (direction U) avec Cf_cable
    - Traînée perpendiculaire (direction V) avec Cx_cable
    
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
        Paramètres du câble (d, rho_cable, Cx_cable, Cf_cable)
    L : float
        Longueur actuelle du câble (m)
    
    Returns:
    --------
    tuple (Fx, Fy, Fx_longitudinal, Fy_longitudinal, Fx_perpendicular, Fy_perpendicular)
        Forces horizontales et verticales par segment (N)
        - Fx, Fy : Forces totales (longitudinale + perpendiculaire)
        - Fx_longitudinal, Fy_longitudinal : Composantes de la traînée longitudinale
        - Fx_perpendicular, Fy_perpendicular : Composantes de la traînée perpendiculaire
    """
    N = len(x_cable) - 1
    if N <= 0:
        return np.array([]), np.array([]), np.array([]), np.array([]), np.array([]), np.array([])
    
    d = params['d']
    rho_cable = params['rho_cable']
    Cx_cable = params.get('Cx_cable', 1.2)
    Cf_cable = params.get('Cf_cable', 0.04)
    A_cable = np.pi * (d / 2)**2
    
    # Vitesses du courant à chaque point
    v_courant = getattr(environment, "v_courant_raw", None)
    v_current = np.array([environment.get_current_velocity(y, v_courant) for y in y_cable])
    
    # Vitesses relatives (courant - vitesse du câble)
    vx_rel = v_current - vx_cable
    vy_rel = -vy_cable  # Le courant est horizontal
    
    # Forces par segment
    Fx = np.zeros(N)
    Fy = np.zeros(N)
    Fx_longitudinal = np.zeros(N)
    Fy_longitudinal = np.zeros(N)
    Fx_perpendicular = np.zeros(N)
    Fy_perpendicular = np.zeros(N)
    
    for i in range(N):
        # 1. Calculer le vecteur tangent U (direction du segment i -> i+1)
        dx = x_cable[i+1] - x_cable[i]
        dy = y_cable[i+1] - y_cable[i]
        ds_segment = np.sqrt(dx**2 + dy**2)
        
        if ds_segment > 1e-6:
            U_x = dx / ds_segment  # Vecteur tangent unitaire
            U_y = dy / ds_segment
        else:
            # Segment dégénéré : utiliser direction par défaut (verticale vers le bas)
            U_x = 0.0
            U_y = -1.0
            ds_segment = L / N if N > 0 else 0.1
        
        # 2. Calculer le vecteur normal V (perpendiculaire à U, rotation de 90°)
        # V = (-U_y, U_x) pour un repère orthonormé direct (U, V)
        V_x = -U_y
        V_y = U_x
        
        # 3. Vitesse relative moyenne sur le segment
        vx_rel_avg = 0.5 * (vx_rel[i] + vx_rel[i+1])
        vy_rel_avg = 0.5 * (vy_rel[i] + vy_rel[i+1])
        
        # 4. Projeter la vitesse relative dans le repère local (U, V)
        v_longitudinal = vx_rel_avg * U_x + vy_rel_avg * U_y  # Composante le long de U
        v_perpendicular = vx_rel_avg * V_x + vy_rel_avg * V_y  # Composante le long de V
        
        # 5. Calculer les traînées dans le repère local
        # Traînée longitudinale (frottement de surface)
        # Surface de frottement : π * d * ds (surface latérale du cylindre)
        F_longitudinal = Cf_cable * 0.5 * environment.rho_eau * np.pi * d * ds_segment * v_longitudinal * np.abs(v_longitudinal)
        
        # Traînée perpendiculaire (traînée normale)
        # Surface projetée : d * ds
        F_perpendicular = Cx_cable * 0.5 * environment.rho_eau * d * ds_segment * v_perpendicular * np.abs(v_perpendicular)
        
        # 6. Reprojeter dans le repère global (x, y)
        Fx_drag_longitudinal = F_longitudinal * U_x
        Fy_drag_longitudinal = F_longitudinal * U_y
        Fx_drag_perpendicular = F_perpendicular * V_x
        Fy_drag_perpendicular = F_perpendicular * V_y
        
        Fx_drag = Fx_drag_longitudinal + Fx_drag_perpendicular
        Fy_drag = Fy_drag_longitudinal + Fy_drag_perpendicular
        
        # 7. Ajouter le poids apparent
        Fy_weight = compute_cable_apparent_weight(rho_cable, environment.rho_eau,
                                                 A_cable, environment.g, ds_segment)
        
        Fx[i] = Fx_drag
        Fy[i] = Fy_weight + Fy_drag
        Fx_longitudinal[i] = Fx_drag_longitudinal
        Fy_longitudinal[i] = Fy_drag_longitudinal
        Fx_perpendicular[i] = Fx_drag_perpendicular
        Fy_perpendicular[i] = Fy_drag_perpendicular
    
    return Fx, Fy, Fx_longitudinal, Fy_longitudinal, Fx_perpendicular, Fy_perpendicular

