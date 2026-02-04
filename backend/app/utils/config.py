"""Default configuration parameters"""

DEFAULT_PARAMS = {
    # ROV parameters
    "m_ROV": 100.0,  # kg
    "a": 0.5,  # m (largeur)
    "b": 0.8,  # m (longueur)
    "h": 0.6,  # m (hauteur)
    "Cx_ROV": 0.8,  # coefficient de traînée horizontal
    "Cy_ROV": 0.8,  # coefficient de traînée vertical
    
    # Boat parameters
    "m_bateau": 10000.0,  # kg
    
    # Cable parameters
    "d": 0.02,  # m (diamètre)
    "rho_cable": 1800.0,  # kg/m³
    "Cx_cable": 1.2,  # coefficient de traînée
    
    # Initial conditions
    "L_initial": 50.0,  # m
    "x_ROV_initial": 0.0,  # m
    "y_ROV_initial": 10.0,  # m
    "vx_ROV_initial": 0.0,  # m/s
    "vy_ROV_initial": 0.0,  # m/s
    "x_bateau_initial": 0.0,  # m
    "vx_bateau_initial": 0.0,  # m/s
    
    # Numerical parameters
    "N_segments": 50,  # nombre de segments pour discrétisation câble
    "dt": 0.01,  # s (pas de temps)
    "t_max": 100.0,  # s (durée max simulation)
    
    # Environmental parameters
    "rho_eau": 1025.0,  # kg/m³
    "g": 9.81,  # m/s²
    "mu": 0.001,  # Pa·s
    "V_courant_type": "uniform",  # "uniform" or "linear"
    "V_courant_value": 0.5,  # m/s
}

