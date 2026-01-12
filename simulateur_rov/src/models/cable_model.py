"""Modèle du câble ombilical"""
import numpy as np
from ..solvers.cable_solver import CableSolver


class Cable:
    """Représente le câble ombilical"""
    
    def __init__(self, params, N_segments=50, environment=None):
        """
        Initialise le modèle du câble
        
        Parameters:
        -----------
        params : dict
            Paramètres du câble:
            - d : diamètre (m)
            - rho_cable : masse volumique (kg/m³)
            - Cx_cable : coefficient de traînée
        N_segments : int
            Nombre de segments pour discrétisation
        environment : Environment
            Objet environnement
        """
        self.d = params.get('d', 0.01)
        self.rho_cable = params.get('rho_cable', 1500.0)
        self.Cx_cable = params.get('Cx_cable', 1.2)
        self.A_cable = np.pi * (self.d / 2)**2
        self.N = N_segments
        
        # Solveur
        self.solver = CableSolver(N_segments, params, environment) if environment else None
        self.environment = environment
    
    def set_environment(self, environment):
        """Définit l'environnement pour le solveur"""
        self.environment = environment
        params = {
            'd': self.d,
            'rho_cable': self.rho_cable,
            'Cx_cable': self.Cx_cable
        }
        self.solver = CableSolver(self.N, params, environment)
    
    def solve_equilibrium(self, x_rov, y_rov, vx_rov, vy_rov,
                         x_boat, vx_boat, L, x_cable_prev=None, y_cable_prev=None):
        """
        Résout l'équilibre du câble
        
        Parameters:
        -----------
        x_rov, y_rov : float
            Position du ROV
        vx_rov, vy_rov : float
            Vitesse du ROV
        x_boat, vx_boat : float
            Position et vitesse du bateau
        L : float
            Longueur du câble
        x_cable_prev, y_cable_prev : array, optional
            Configuration précédente
        
        Returns:
        --------
        tuple (x_cable, y_cable, T)
            Configuration du câble
        """
        if self.solver is None:
            raise ValueError("Environment must be set before solving")
        
        if x_cable_prev is None or y_cable_prev is None:
            # Solution statique
            x_cable, y_cable, T = self.solver.solve_equilibrium_static(x_rov, y_rov, x_boat, L)
            self.x_cable_prev = x_cable.copy()
            self.y_cable_prev = y_cable.copy()
            return x_cable, y_cable, T
        else:
            # Solution dynamique
            x_cable, y_cable, T = self.solver.solve_equilibrium_dynamic(
                x_rov, y_rov, vx_rov, vy_rov,
                x_boat, vx_boat, L, x_cable_prev, y_cable_prev
            )
            self.x_cable_prev = x_cable.copy()
            self.y_cable_prev = y_cable.copy()
            return x_cable, y_cable, T

