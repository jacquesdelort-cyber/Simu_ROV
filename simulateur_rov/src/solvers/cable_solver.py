"""Solveur pour les équations du câble"""
import numpy as np
from scipy.optimize import fsolve


class CableSolver:
    """Résout les équations d'équilibre du câble"""
    
    def __init__(self, N_segments, params, environment):
        """
        Initialise le solveur de câble
        
        Parameters:
        -----------
        N_segments : int
            Nombre de segments pour discrétiser le câble
        params : dict
            Paramètres du câble
        environment : Environment
            Objet environnement
        """
        self.N = N_segments
        self.params = params
        self.environment = environment
        self.d = params['d']
        self.rho_cable = params['rho_cable']
        self.Cx_cable = params['Cx_cable']
        self.A_cable = np.pi * (self.d / 2)**2
    
    def solve_equilibrium_static(self, x_rov, y_rov, x_boat, L):
        """
        Résout l'équilibre statique du câble (caténaire)
        
        Parameters:
        -----------
        x_rov : float
            Position horizontale du ROV (m)
        y_rov : float
            Position verticale du ROV (m)
        x_boat : float
            Position horizontale du bateau (m)
        L : float
            Longueur du câble (m)
        
        Returns:
        --------
        tuple (x_cable, y_cable, T)
            Positions et tensions du câble
        """
        if L <= 0:
            # Câble de longueur nulle
            return np.array([x_rov, x_boat]), np.array([y_rov, 0.0]), np.array([0.0, 0.0])
        
        ds = L / self.N
        
        # Approche simplifiée : interpolation linéaire initiale
        s = np.linspace(0, L, self.N + 1)
        x_cable = np.linspace(x_rov, x_boat, self.N + 1)
        y_cable = np.linspace(y_rov, 0.0, self.N + 1)
        
        # Ajustement pour respecter la longueur L (caténaire simplifiée)
        # Calculer les angles et ajuster
        dx = x_boat - x_rov
        dy = -y_rov  # Profondeur
        
        # Longueur rectiligne
        L_straight = np.sqrt(dx**2 + dy**2)
        
        if L < L_straight:
            # Câble tendu
            x_cable = np.linspace(x_rov, x_boat, self.N + 1)
            y_cable = np.linspace(y_rov, 0.0, self.N + 1)
        else:
            # Câble en caténaire
            # Approximation par une parabole
            for i in range(self.N + 1):
                s_i = s[i] / L
                x_cable[i] = x_rov + dx * s_i
                # Forme caténaire simplifiée (parabole)
                y_cable[i] = y_rov * (1 - s_i**2)
        
        # Tension initiale (approximation)
        # Tension minimale au point le plus bas
        weight_per_unit = (self.rho_cable - self.environment.rho_eau) * self.A_cable * self.environment.g
        T_min = weight_per_unit * L / 10.0  # Estimation
        
        # Distribution de tension le long du câble
        # Plus grande tension aux extrémités
        T = np.zeros(self.N + 1)
        for i in range(self.N + 1):
            # Tension augmente avec la profondeur et la position
            s_i = s[i] / L
            T[i] = T_min + weight_per_unit * L * abs(0.5 - s_i)
        
        return x_cable, y_cable, T
    
    def solve_equilibrium_dynamic(self, x_rov, y_rov, vx_rov, vy_rov,
                                  x_boat, vx_boat, L, x_cable_prev, y_cable_prev):
        """
        Résout l'équilibre dynamique du câble avec forces hydrodynamiques
        
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
        x_cable_prev, y_cable_prev : array
            Configuration précédente du câble
        
        Returns:
        --------
        tuple (x_cable, y_cable, T)
            Nouvelle configuration du câble
        """
        if L <= 0:
            return np.array([x_rov, x_boat]), np.array([y_rov, 0.0]), np.array([0.0, 0.0])
        
        ds = L / self.N
        
        # Initialiser avec la solution statique
        x_cable, y_cable, T = self.solve_equilibrium_static(x_rov, y_rov, x_boat, L)
        
        # Vitesses des points du câble (interpolation linéaire entre ROV et bateau)
        vx_cable = np.linspace(vx_rov, vx_boat, self.N + 1)
        vy_cable = np.linspace(vy_rov, 0.0, self.N + 1)
        
        # Itération pour améliorer la solution en tenant compte des forces
        max_iter = 5
        tolerance = 1e-3
        
        for iteration in range(max_iter):
            x_cable_old = x_cable.copy()
            y_cable_old = y_cable.copy()
            
            # Calculer les forces
            from .forces import compute_cable_forces
            Fx, Fy = compute_cable_forces(x_cable, y_cable, vx_cable, vy_cable,
                                         self.environment, self.params, L)
            
            # Résoudre l'équilibre des forces par segment
            for i in range(1, self.N):
                # Ajuster la position pour équilibrer les forces
                # Approche simplifiée : ajustement proportionnel
                if i > 0:
                    dx_seg = x_cable[i] - x_cable[i-1]
                    dy_seg = y_cable[i] - y_cable[i-1]
                    length_seg = np.sqrt(dx_seg**2 + dy_seg**2)
                    
                    if length_seg > 1e-6:
                        # Normaliser
                        dx_seg /= length_seg
                        dy_seg /= length_seg
                        
                        # Ajuster selon les forces
                        alpha = 0.1  # Coefficient de relaxation
                        x_cable[i] += alpha * Fx[i-1] * dx_seg / (T[i] + 1e-6)
                        y_cable[i] += alpha * Fy[i-1] * dy_seg / (T[i] + 1e-6)
                
                # Assurer la longueur du segment
                if i > 0:
                    dx_seg = x_cable[i] - x_cable[i-1]
                    dy_seg = y_cable[i] - y_cable[i-1]
                    length_seg = np.sqrt(dx_seg**2 + dy_seg**2)
                    if length_seg > 1e-6:
                        scale = ds / length_seg
                        x_cable[i] = x_cable[i-1] + (x_cable[i] - x_cable[i-1]) * scale
                        y_cable[i] = y_cable[i-1] + (y_cable[i] - y_cable[i-1]) * scale
            
            # Forcer les conditions aux limites
            x_cable[0] = x_rov
            y_cable[0] = y_rov
            x_cable[-1] = x_boat
            y_cable[-1] = 0.0
            
            # Vérifier la convergence
            if np.max(np.abs(x_cable - x_cable_old)) < tolerance and \
               np.max(np.abs(y_cable - y_cable_old)) < tolerance:
                break
        
        # Recalculer les tensions
        T = self.compute_tensions(x_cable, y_cable, L, Fx, Fy)
        
        return x_cable, y_cable, T
    
    def compute_tensions(self, x_cable, y_cable, L, Fx, Fy):
        """
        Calcule les tensions le long du câble
        
        Parameters:
        -----------
        x_cable, y_cable : array
            Positions du câble
        L : float
            Longueur du câble
        Fx, Fy : array
            Forces par segment
        
        Returns:
        --------
        array
            Tensions le long du câble
        """
        N = len(x_cable) - 1
        ds = L / N if N > 0 else 0.1
        
        T = np.zeros(N + 1)
        
        # Intégrer les forces depuis le ROV vers le bateau
        T[0] = 100.0  # Tension initiale au ROV
        
        for i in range(N):
            # Direction du segment
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            length = np.sqrt(dx**2 + dy**2)
            
            if length > 1e-6:
                # Composantes directionnelles
                cos_theta = dx / length
                sin_theta = dy / length
                
                # Équilibre des forces
                dT_x = Fx[i] * ds
                dT_y = Fy[i] * ds
                
                # Mettre à jour la tension
                T[i+1] = T[i] + np.sqrt(dT_x**2 + dT_y**2)
            else:
                T[i+1] = T[i]
        
        return T

