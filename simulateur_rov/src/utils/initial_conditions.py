"""Conditions initiales"""
import numpy as np


def get_initial_state(system, x_rov=0.0, y_rov=-10.0, vx_rov=0.0, vy_rov=0.0,
                     x_boat=0.0, vx_boat=0.0, L=50.0, use_current_geometry=True):
    """
    Crée l'état initial du système
    
    Parameters:
    -----------
    system : ROVSystem
        Système ROV
    x_rov, y_rov : float
        Position initiale du ROV (m)
    vx_rov, vy_rov : float
        Vitesse initiale du ROV (m/s)
    x_boat, vx_boat : float
        Position et vitesse initiale du bateau (m, m/s)
    L : float
        Longueur initiale du câble (m)
    use_current_geometry : bool
        Si True, utilise un solveur statique itératif qui déforme la géométrie
        du câble sous l'effet du courant.
    
    Returns:
    --------
    array
        Vecteur d'état initial
    """
    # Résoudre la configuration initiale du câble (statique)
    # Passer les paramètres du ROV pour calculer correctement la tension initiale
    rov_m = system.rov.m
    rov_vol = system.rov.V
    if use_current_geometry:
        x_cable, y_cable, T = system.cable.solver.solve_equilibrium_static_with_current(
            x_rov, y_rov, x_boat, L, rov_m=rov_m, rov_vol=rov_vol
        )
    else:
        x_cable, y_cable, T = system.cable.solver.solve_equilibrium_static(
            x_rov, y_rov, x_boat, L, rov_m=rov_m, rov_vol=rov_vol
        )
    
    # Créer le vecteur d'état
    y0 = system.pack_state(
        x_rov, y_rov, vx_rov, vy_rov,
        x_boat, vx_boat,
        x_cable, y_cable, T, L
    )
    
    return y0


def get_initial_state_simple():
    """
    Retourne un état initial simple pour tests
    
    Returns:
    --------
    dict
        État initial simplifié
    """
    return {
        'x_rov': 0.0,
        'y_rov': 10.0,
        'vx_rov': 0.0,
        'vy_rov': 0.0,
        'x_boat': 0.0,
        'vx_boat': 0.0,
        'L': 50.0
    }

