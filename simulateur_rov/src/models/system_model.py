"""Système complet ROV-Câble-Bateau"""
import numpy as np
from .rov_model import ROV
from .cable_model import Cable
from .boat_model import Boat
from .environment import Environment
from ..solvers.integrator import TimeIntegrator


class ROVSystem:
    """Système complet ROV-Câble-Bateau"""
    
    def __init__(self, params, N_segments=50):
        """
        Initialise le système complet
        
        Parameters:
        -----------
        params : dict
            Dictionnaire contenant:
            - 'rov' : paramètres du ROV
            - 'cable' : paramètres du câble
            - 'boat' : paramètres du bateau
            - 'environment' : paramètres environnementaux
        N_segments : int
            Nombre de segments pour discrétiser le câble
        """
        # Créer les composants
        self.environment = Environment(params.get('environment', {}))
        self.rov = ROV(params.get('rov', {}))
        self.boat = Boat(params.get('boat', {}))
        self.cable = Cable(params.get('cable', {}), N_segments, self.environment)
        
        self.N = N_segments
        
        # Intégrateur
        self.integrator = TimeIntegrator(method='RK45', max_step=0.1)
        
        # État précédent du câble (pour itération)
        self.x_cable_prev = None
        self.y_cable_prev = None
    
    def get_state_size(self):
        """Retourne la taille du vecteur d'état"""
        # ROV: 4 (x, y, vx, vy)
        # Bateau: 2 (x, vx)
        # Câble: (N+1)*2 positions + (N+1) tensions = 3*(N+1)
        # Longueur: 1
        return 6 + 3 * (self.N + 1) + 1
    
    def pack_state(self, x_rov, y_rov, vx_rov, vy_rov,
                   x_boat, vx_boat, x_cable, y_cable, T, L):
        """
        Empaquette l'état dans un vecteur
        
        Returns:
        --------
        array
            Vecteur d'état
        """
        y = np.zeros(self.get_state_size())
        y[0] = x_rov
        y[1] = y_rov
        y[2] = vx_rov
        y[3] = vy_rov
        y[4] = x_boat
        y[5] = vx_boat
        
        idx = 6
        y[idx:idx+self.N+1] = x_cable
        idx += self.N + 1
        y[idx:idx+self.N+1] = y_cable
        idx += self.N + 1
        y[idx:idx+self.N+1] = T
        idx += self.N + 1
        y[idx] = L
        
        return y
    
    def unpack_state(self, y):
        """
        Dépaquette le vecteur d'état
        
        Returns:
        --------
        tuple
            Composantes de l'état
        """
        x_rov = y[0]
        y_rov = y[1]
        vx_rov = y[2]
        vy_rov = y[3]
        x_boat = y[4]
        vx_boat = y[5]
        
        idx = 6
        x_cable = y[idx:idx+self.N+1]
        idx += self.N + 1
        y_cable = y[idx:idx+self.N+1]
        idx += self.N + 1
        T = y[idx:idx+self.N+1]
        idx += self.N + 1
        L = y[idx]
        
        return (x_rov, y_rov, vx_rov, vy_rov, x_boat, vx_boat,
                x_cable, y_cable, T, L)
    
    def compute_derivatives(self, t, y, u):
        """
        Calcule les dérivées du système
        
        Parameters:
        -----------
        t : float
            Temps (s)
        y : array
            Vecteur d'état
        u : dict
            Vecteur de commande:
            - Fx_rov : force horizontale ROV (N)
            - Fy_rov : force verticale ROV (N)
            - vx_boat_cmd : vitesse commande bateau (m/s)
            - dL_dt : variation longueur câble (m/s)
        
        Returns:
        --------
        array
            Dérivées dy/dt
        """
        # Dépaqueter l'état
        (x_rov, y_rov, vx_rov, vy_rov, x_boat, vx_boat,
         x_cable, y_cable, T, L) = self.unpack_state(y)
        
        # Résoudre la configuration du câble
        if self.x_cable_prev is None or self.y_cable_prev is None:
            # Première itération : solution statique
            x_cable_new, y_cable_new, T_new = self.cable.solver.solve_equilibrium_static(
                x_rov, y_rov, x_boat, L
            )
        else:
            # Solution dynamique
            x_cable_new, y_cable_new, T_new = self.cable.solve_equilibrium(
                x_rov, y_rov, vx_rov, vy_rov,
                x_boat, vx_boat, L,
                self.x_cable_prev, self.y_cable_prev
            )
        
        # Mettre à jour pour la prochaine itération
        self.x_cable_prev = x_cable_new.copy()
        self.y_cable_prev = y_cable_new.copy()
        
        # Tension et angle au niveau du ROV
        T0 = T_new[0]
        if len(x_cable_new) > 1:
            dx_cable = x_cable_new[1] - x_cable_new[0]
            dy_cable = y_cable_new[1] - y_cable_new[0]
            ds = np.sqrt(dx_cable**2 + dy_cable**2)
            if ds > 1e-6:
                cos_theta0 = dx_cable / ds
                sin_theta0 = dy_cable / ds
            else:
                cos_theta0 = 1.0
                sin_theta0 = 0.0
        else:
            cos_theta0 = 1.0
            sin_theta0 = 0.0
        
        # Forces sur le ROV
        Fx_drag_rov, Fy_drag_rov = self.rov.compute_drag_force(
            vx_rov, vy_rov, y_rov, self.environment
        )
        F_buoyancy = self.rov.compute_buoyancy_force(self.environment)
        F_weight = self.rov.compute_weight_force(self.environment)
        
        # Équations du ROV
        dvx_rov_dt = (Fx_drag_rov + T0 * cos_theta0 + u['Fx_rov']) / self.rov.m
        dvy_rov_dt = (F_weight - F_buoyancy + Fy_drag_rov + T0 * sin_theta0 + u['Fy_rov']) / self.rov.m
        
        # Forces sur le bateau
        T_L = T_new[-1]
        if len(x_cable_new) > 1:
            dx_cable_L = x_cable_new[-1] - x_cable_new[-2]
            dy_cable_L = y_cable_new[-1] - y_cable_new[-2]
            ds_L = np.sqrt(dx_cable_L**2 + dy_cable_L**2)
            if ds_L > 1e-6:
                cos_theta_L = dx_cable_L / ds_L
            else:
                cos_theta_L = 1.0
        else:
            cos_theta_L = 1.0
        
        F_prop_boat = self.boat.compute_propulsion_force(
            u['vx_boat_cmd'], vx_boat, self.environment
        )
        
        # Équation du bateau
        dvx_boat_dt = (-T_L * cos_theta_L + F_prop_boat) / self.boat.m
        
        # Évolution de la longueur du câble
        dL_dt = u['dL_dt'] 
        
        # Dérivées des positions du câble (vitesses)
        # Simplification : interpolation linéaire des vitesses
        vx_cable = np.linspace(vx_rov, vx_boat, self.N + 1)
        vy_cable = np.linspace(vy_rov, 0.0, self.N + 1)
        
        # Dérivées des tensions : mise à jour dynamique vers les tensions calculées
        # Utiliser un modèle de relaxation pour mettre à jour les tensions
        # Temps de relaxation (s) - ajustable selon les besoins de stabilité
        tau_tension = 0.01  # 10 ms de relaxation pour réponse rapide 
        dT_dt = (T_new - T) / tau_tension
        
        # Assembler les dérivées
        dydt = np.zeros_like(y)
        dydt[0] = vx_rov  # dx_rov/dt
        dydt[1] = vy_rov  # dy_rov/dt
        dydt[2] = dvx_rov_dt
        dydt[3] = dvy_rov_dt
        dydt[4] = vx_boat  # dx_boat/dt
        dydt[5] = dvx_boat_dt
        
        idx = 6
        dydt[idx:idx+self.N+1] = vx_cable  # dx_cable/dt
        idx += self.N + 1
        dydt[idx:idx+self.N+1] = vy_cable  # dy_cable/dt
        idx += self.N + 1
        dydt[idx:idx+self.N+1] = dT_dt  # dT/dt
        idx += self.N + 1
        dydt[idx] = dL_dt  # dL/dt
        
        return dydt
    
    def integrate(self, t_span, y0, u_func, dt_max=0.1):
        """
        Intègre le système sur un intervalle de temps
        
        Parameters:
        -----------
        t_span : tuple
            (t0, tf) intervalle de temps (s)
        y0 : array
            Conditions initiales
        u_func : callable
            Fonction u(t) retournant le vecteur de commande
        dt_max : float
            Pas de temps maximum (s)
        
        Returns:
        --------
        solution
            Objet solution de scipy.integrate.solve_ivp
        """
        self.integrator.max_step = dt_max
        
        def system_ode(t, y):
            u = u_func(t)
            return self.compute_derivatives(t, y, u)
        
        sol = self.integrator.integrate(system_ode, t_span, y0)
        return sol

