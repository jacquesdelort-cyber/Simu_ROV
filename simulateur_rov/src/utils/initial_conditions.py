"""Conditions initiales."""
import numpy as np


def _select_cable_init_mode(cable_init_mode=None, legacy_init_geometry=False):
    """Retourne le mode d'initialisation câble normalisé."""
    if cable_init_mode is None:
        return "legacy_geometry" if bool(legacy_init_geometry) else "strict_static"
    mode = str(cable_init_mode).strip().lower()
    if mode in {"strict", "strict_static", "static_strict"}:
        return "strict_static"
    if mode in {"legacy", "legacy_geometry", "heuristic"}:
        return "legacy_geometry"
    return "strict_static"


def get_initial_state(system, x_rov=0.0, y_rov=-10.0, vx_rov=0.0, vy_rov=0.0,
                     x_boat=0.0, vx_boat=0.0, L=50.0, use_current_geometry=True,
                     cable_init_mode=None, legacy_init_geometry=False):
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
    cable_init_mode : str | None
        Mode explicite d'initialisation câble:
        - "strict_static" (défaut)
        - "legacy_geometry"
    legacy_init_geometry : bool
        Compatibilité rétroactive. Ignoré si cable_init_mode est fourni.
    
    Returns:
    --------
    array
        Vecteur d'état initial
    """
    # Résoudre la configuration initiale du câble (statique)
    # Passer les paramètres du ROV pour calculer correctement la tension initiale
    rov_m = system.rov.m
    rov_vol = system.rov.V
    mode = _select_cable_init_mode(cable_init_mode, legacy_init_geometry)

    # Les deux modes conservent le solveur statique comme source de vérité pour t=0.
    if use_current_geometry:
        x_cable, y_cable, T = system.cable.solver.solve_equilibrium_static_with_current(
            x_rov, y_rov, x_boat, L, rov_m=rov_m, rov_vol=rov_vol
        )
    else:
        x_cable, y_cable, T = system.cable.solver.solve_equilibrium_static(
            x_rov, y_rov, x_boat, L, rov_m=rov_m, rov_vol=rov_vol
        )
    
    # En mode strict, on fige les extrémités pour éviter toute dérive à l'initialisation.
    if mode == "strict_static":
        x_cable = np.asarray(x_cable, dtype=float).copy()
        y_cable = np.asarray(y_cable, dtype=float).copy()
        x_cable[0], y_cable[0] = float(x_boat), 0.0
        x_cable[-1], y_cable[-1] = float(x_rov), float(y_rov)

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

