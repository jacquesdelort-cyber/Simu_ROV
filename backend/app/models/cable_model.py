"""Cable physical model"""

import numpy as np
from scipy.optimize import root
from ..utils import physics


class CableModel:
    """Modèle physique du câble avec résolution quasi-statique"""
    
    def __init__(self, params):
        """
        Initialise le modèle câble
        
        Args:
            params: Dictionnaire de paramètres
        """
        self.d = params['d']  # diamètre
        self.rho_cable = params['rho_cable']
        self.Cx_cable = params['Cx_cable']
        self.rho_eau = params.get('rho_eau', physics.RHO_WATER)
        self.g = params.get('g', physics.G)
        
        # Géométrie
        self.A_cable = np.pi * (self.d / 2)**2  # section transversale
        self.N = params.get('N_segments', 50)
        
        # Discrétisation
        self.s = None  # abscisses curvilignes
        self.ds = None  # pas spatial
        
    def discretize(self, L):
        """
        Crée la discrétisation du câble
        
        Args:
            L: Longueur du câble (m)
        """
        self.ds = L / self.N
        self.s = np.linspace(0, L, self.N + 1)
        
    def compute_apparent_weight(self):
        """Calcule le poids apparent par unité de longueur (N/m)"""
        w = (self.rho_cable - self.rho_eau) * self.A_cable * self.g
        return w
    
    def solve_quasi_static(self, x_ROV, y_ROV, x_boat, y_boat, V_courant_func=None):
        """
        Résout les équations d'équilibre du câble (quasi-statique)
        
        Args:
            x_ROV, y_ROV: Position ROV (m)
            x_boat, y_boat: Position bateau (m)
            V_courant_func: Fonction courant(y) -> vitesse (m/s), optionnel
        
        Returns:
            T, theta, x, y pour chaque noeud
        """
        if V_courant_func is None:
            V_courant_func = lambda y: 0.0
        
        L = np.sqrt((x_boat - x_ROV)**2 + (y_boat - y_ROV)**2)
        if L < 1e-6:
            raise ValueError("ROV et bateau trop proches")
        
        self.discretize(L)
        
        # Variables : T[i], theta[i] pour i=0..N
        n_vars = 2 * (self.N + 1)
        
        def residuals(vars):
            """Résidus pour les équations d'équilibre"""
            T = vars[:self.N+1]
            theta = vars[self.N+1:]
            
            # Positions
            x = np.zeros(self.N + 1)
            y = np.zeros(self.N + 1)
            x[0] = x_ROV
            y[0] = y_ROV
            
            # Résidus
            res = np.zeros(n_vars)
            
            # Poids apparent
            w = self.compute_apparent_weight()
            
            # Équations d'équilibre pour chaque segment
            for i in range(self.N):
                # Calculer positions intermédiaires
                x[i+1] = x[i] + np.cos(theta[i]) * self.ds
                y[i+1] = y[i] + np.sin(theta[i]) * self.ds
                
                # Équation horizontale : d(T*cos(theta))/ds = F_drag_x
                # Approximation : on néglige la traînée pour simplifier initialement
                # (peut être ajoutée plus tard)
                res[i] = (T[i+1] * np.cos(theta[i+1]) - 
                         T[i] * np.cos(theta[i])) / self.ds
                
                # Équation verticale : d(T*sin(theta))/ds = w + F_drag_y
                res[i + self.N + 1] = (T[i+1] * np.sin(theta[i+1]) - 
                                      T[i] * np.sin(theta[i])) / self.ds - w
            
            # Conditions aux limites position
            res[self.N] = x[-1] - x_boat
            res[2*self.N + 1] = y[-1] - y_boat
            
            return res
        
        # Estimation initiale (caténaire simple)
        # Tension initiale estimée
        w = self.compute_apparent_weight()
        T_est = w * L / 2  # estimation grossière
        T0 = np.ones(self.N + 1) * max(T_est, 100.0)
        
        # Angle initial (direction ROV -> bateau)
        theta_est = np.arctan2(y_boat - y_ROV, x_boat - x_ROV)
        theta0 = np.linspace(theta_est, 0.0, self.N + 1)
        
        x0 = np.concatenate([T0, theta0])
        
        # Résolution
        sol = root(residuals, x0, method='lm', options={'maxfev': 5000})
        
        if not sol.success:
            # Si échec, essayer avec estimation différente
            T0 = np.ones(self.N + 1) * 200.0
            theta0 = np.linspace(theta_est, 0.0, self.N + 1)
            x0 = np.concatenate([T0, theta0])
            sol = root(residuals, x0, method='lm', options={'maxfev': 10000})
            
            if not sol.success:
                raise ValueError(f"Échec résolution équations câble: {sol.message}")
        
        T = sol.x[:self.N+1]
        theta = sol.x[self.N+1:]
        
        # Recalculer positions finales
        x = np.zeros(self.N + 1)
        y = np.zeros(self.N + 1)
        x[0] = x_ROV
        y[0] = y_ROV
        for i in range(self.N):
            x[i+1] = x[i] + np.cos(theta[i]) * self.ds
            y[i+1] = y[i] + np.sin(theta[i]) * self.ds
        
        return T, theta, x, y
    
    def get_tension_at_ROV(self, T, theta):
        """Retourne tension et angle au niveau ROV (i=0)"""
        return T[0], theta[0]
    
    def get_tension_at_boat(self, T, theta):
        """Retourne tension et angle au niveau bateau (i=N)"""
        return T[-1], theta[-1]

