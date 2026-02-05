"""Boat physical model"""

import numpy as np
from scipy.integrate import solve_ivp


class BoatModel:
    """Modèle physique du bateau"""
    
    def __init__(self, params):
        """
        Initialise le modèle bateau
        
        Args:
            params: Dictionnaire de paramètres
        """
        self.m_bateau = params['m_bateau']
        
        # État
        self.state = None  # [x, vx]
        
    def compute_propulsion_force(self, vx, vx_cmd, k_prop=1000.0):
        """
        Calcule la force de propulsion pour atteindre la vitesse de commande
        
        Args:
            vx: Vitesse actuelle (m/s)
            vx_cmd: Vitesse de commande (m/s)
            k_prop: Gain de propulsion (N·s/m)
        
        Returns:
            Force de propulsion (N)
        """
        error = vx_cmd - vx
        F_prop = k_prop * error
        return F_prop
    
    def derivative(self, t, state, T_L, theta_L, vx_cmd):
        """
        Calcule les dérivées pour solve_ivp
        
        Args:
            t: Temps (s)
            state: [x_bateau, vx_bateau]
            T_L: Tension du câble au niveau bateau (N)
            theta_L: Angle du câble au niveau bateau (rad)
            vx_cmd: Vitesse de commande (m/s)
        
        Returns:
            np.array([dx/dt, dvx/dt])
        """
        x_bateau, vx_bateau = state
        
        # Force de tension du câble (vers l'arrière)
        F_cable_x = -T_L * np.cos(theta_L)
        
        # Force de propulsion
        F_prop = self.compute_propulsion_force(vx_bateau, vx_cmd)
        
        # Dérivées
        dvx_dt = (F_cable_x + F_prop) / self.m_bateau
        dx_dt = vx_bateau
        
        return np.array([dx_dt, dvx_dt])
    
    def integrate_step(self, t, state, dt, T_L, theta_L, vx_cmd):
        """
        Intègre les équations du bateau sur un pas de temps
        
        Args:
            t: Temps actuel (s)
            state: État actuel [x, vx]
            dt: Pas de temps (s)
            T_L: Tension câble (N)
            theta_L: Angle câble (rad)
            vx_cmd: Vitesse commande (m/s)
        
        Returns:
            Nouvel état [x, vx]
        """
        def deriv(t, s):
            return self.derivative(t, s, T_L, theta_L, vx_cmd)
        
        sol = solve_ivp(
            deriv,
            [t, t + dt],
            state,
            method='RK45',
            dense_output=False,
            rtol=1e-6,
            atol=1e-8
        )
        
        if not sol.success:
            raise RuntimeError(f"Échec intégration Bateau: {sol.message}")
        
        return sol.y[:, -1]

