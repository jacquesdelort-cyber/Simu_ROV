"""ROV physical model"""

import numpy as np
from scipy.integrate import solve_ivp
from ..utils import physics


class ROVModel:
    """Modèle physique du ROV"""
    
    def __init__(self, params):
        """
        Initialise le modèle ROV
        
        Args:
            params: Dictionnaire de paramètres
        """
        self.m_ROV = params['m_ROV']
        self.a = params['a']  # largeur
        self.b = params['b']  # longueur
        self.h = params['h']  # hauteur
        
        # Géométrie
        self.V_ROV = self.a * self.b * self.h  # volume
        self.Sx_ROV = self.a * self.h  # section frontale perpendiculaire à x
        self.Sy_ROV = self.a * self.b  # section frontale perpendiculaire à y
        
        # Coefficients
        self.Cx_ROV = params['Cx_ROV']
        self.Cy_ROV = params['Cy_ROV']
        
        # Paramètres environnementaux
        self.rho_eau = params.get('rho_eau', physics.RHO_WATER)
        self.g = params.get('g', physics.G)
        
        # État
        self.state = None  # [x, y, vx, vy]
        
    def compute_drag_force(self, vx, vy, V_courant_y):
        """
        Calcule la force de traînée
        
        Args:
            vx: Vitesse horizontale ROV (m/s)
            vy: Vitesse verticale ROV (m/s)
            V_courant_y: Vitesse du courant à la profondeur y (m/s)
        
        Returns:
            np.array([F_drag_x, F_drag_y]) en N
        """
        vx_rel = vx - V_courant_y
        vy_rel = vy
        
        F_drag_x = physics.compute_drag_force_quadratic(
            self.Cx_ROV, self.rho_eau, self.Sx_ROV, vx_rel
        )
        F_drag_y = physics.compute_drag_force_quadratic(
            self.Cy_ROV, self.rho_eau, self.Sy_ROV, vy_rel
        )
        
        return np.array([F_drag_x, F_drag_y])
    
    def compute_buoyancy_force(self):
        """Calcule la poussée d'Archimède (vers le haut)"""
        F_buoy = self.rho_eau * self.V_ROV * self.g
        return np.array([0.0, F_buoy])
    
    def compute_gravity_force(self):
        """Calcule le poids (vers le bas)"""
        F_weight = -self.m_ROV * self.g
        return np.array([0.0, F_weight])
    
    def derivative(self, t, state, T0, theta0, F_prop, V_courant_func):
        """
        Calcule les dérivées pour solve_ivp
        
        Args:
            t: Temps (s)
            state: [x_ROV, y_ROV, vx_ROV, vy_ROV]
            T0: Tension du câble au niveau ROV (N)
            theta0: Angle du câble au niveau ROV (rad)
            F_prop: Forces de propulsion [Fx, Fy] (N)
            V_courant_func: Fonction courant(y) -> vitesse (m/s)
        
        Returns:
            np.array([dx/dt, dy/dt, dvx/dt, dvy/dt])
        """
        x_ROV, y_ROV, vx_ROV, vy_ROV = state
        
        # Forces
        V_courant = V_courant_func(y_ROV)
        F_drag = self.compute_drag_force(vx_ROV, vy_ROV, V_courant)
        F_buoy = self.compute_buoyancy_force()
        F_weight = self.compute_gravity_force()
        F_cable = np.array([
            T0 * np.cos(theta0),
            T0 * np.sin(theta0)
        ])
        
        F_total = F_drag + F_buoy + F_weight + F_cable + F_prop
        
        # Dérivées
        dvx_dt = F_total[0] / self.m_ROV
        dvy_dt = F_total[1] / self.m_ROV
        dx_dt = vx_ROV
        dy_dt = vy_ROV
        
        return np.array([dx_dt, dy_dt, dvx_dt, dvy_dt])
    
    def integrate_step(self, t, state, dt, T0, theta0, F_prop, V_courant_func):
        """
        Intègre les équations du ROV sur un pas de temps
        
        Args:
            t: Temps actuel (s)
            state: État actuel [x, y, vx, vy]
            dt: Pas de temps (s)
            T0: Tension câble (N)
            theta0: Angle câble (rad)
            F_prop: Forces propulsion [Fx, Fy] (N)
            V_courant_func: Fonction courant(y)
        
        Returns:
            Nouvel état [x, y, vx, vy]
        """
        def deriv(t, s):
            return self.derivative(t, s, T0, theta0, F_prop, V_courant_func)
        
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
            raise RuntimeError(f"Échec intégration ROV: {sol.message}")
        
        return sol.y[:, -1]

