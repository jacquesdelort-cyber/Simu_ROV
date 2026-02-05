"""Coupled system solver for ROV-Cable-Boat"""

import numpy as np
from ..models.rov_model import ROVModel
from ..models.cable_model import CableModel
from ..models.boat_model import BoatModel
from ..utils import physics


class CoupledSolver:
    """Solveur pour le système couplé ROV-Câble-Bateau"""
    
    def __init__(self, params):
        """
        Initialise le solveur couplé
        
        Args:
            params: Dictionnaire de paramètres
        """
        self.params = params
        self.dt = params.get('dt', 0.01)
        
        # Modèles
        self.rov = ROVModel(params)
        self.cable = CableModel(params)
        self.boat = BoatModel(params)
        
        # État système
        self.t = 0.0
        self.state = {
            'rov': np.array([
                params.get('x_ROV_initial', 0.0),
                params.get('y_ROV_initial', 10.0),
                params.get('vx_ROV_initial', 0.0),
                params.get('vy_ROV_initial', 0.0)
            ]),
            'boat': np.array([
                params.get('x_bateau_initial', 0.0),
                params.get('vx_bateau_initial', 0.0)
            ]),
            'L': params.get('L_initial', 50.0)
        }
        
        # Commandes
        self.commands = {
            'Fx_ROV': 0.0,
            'Fy_ROV': 0.0,
            'vx_bateau_cmd': 0.0,
            'dL_dt': 0.0
        }
        
        # Fonction courant
        self.V_courant_func = self._create_current_function()
        
    def _create_current_function(self):
        """Crée la fonction de courant selon le type"""
        current_type = self.params.get('V_courant_type', 'uniform')
        current_value = self.params.get('V_courant_value', 0.5)
        
        if current_type == 'uniform':
            return lambda y: current_value
        elif current_type == 'linear':
            y_max = 100.0  # profondeur max
            return lambda y: physics.current_velocity_linear(y, current_value, y_max)
        else:
            return lambda y: 0.0
    
    def set_commands(self, Fx_ROV=0.0, Fy_ROV=0.0, vx_bateau_cmd=0.0, dL_dt=0.0):
        """Met à jour les commandes"""
        self.commands['Fx_ROV'] = Fx_ROV
        self.commands['Fy_ROV'] = Fy_ROV
        self.commands['vx_bateau_cmd'] = vx_bateau_cmd
        self.commands['dL_dt'] = dL_dt
    
    def step(self):
        """
        Effectue un pas de simulation
        
        Returns:
            État du système après le pas
        """
        # Extraire état actuel
        x_ROV = self.state['rov'][0]
        y_ROV = self.state['rov'][1]
        vx_ROV = self.state['rov'][2]
        vy_ROV = self.state['rov'][3]
        
        x_boat = self.state['boat'][0]
        vx_boat = self.state['boat'][1]
        
        L = self.state['L']
        
        # 1. Résoudre équations câble (quasi-statique)
        try:
            T, theta, x_cable, y_cable = self.cable.solve_quasi_static(
                x_ROV, y_ROV, x_boat, 0.0, self.V_courant_func
            )
        except Exception as e:
            print(f"Erreur résolution câble: {e}")
            # En cas d'erreur, utiliser valeurs précédentes ou par défaut
            T = np.ones(self.cable.N + 1) * 100.0
            theta = np.zeros(self.cable.N + 1)
            x_cable = np.linspace(x_ROV, x_boat, self.cable.N + 1)
            y_cable = np.linspace(y_ROV, 0.0, self.cable.N + 1)
        
        # 2. Extraire tensions aux extrémités
        T0, theta0 = self.cable.get_tension_at_ROV(T, theta)
        T_L, theta_L = self.cable.get_tension_at_boat(T, theta)
        
        # 3. Intégrer ODE ROV
        F_prop = np.array([self.commands['Fx_ROV'], self.commands['Fy_ROV']])
        self.state['rov'] = self.rov.integrate_step(
            self.t,
            self.state['rov'],
            self.dt,
            T0,
            theta0,
            F_prop,
            self.V_courant_func
        )
        
        # 4. Intégrer ODE Bateau
        self.state['boat'] = self.boat.integrate_step(
            self.t,
            self.state['boat'],
            self.dt,
            T_L,
            theta_L,
            self.commands['vx_bateau_cmd']
        )
        
        # 5. Mettre à jour longueur câble
        self.state['L'] += self.commands['dL_dt'] * self.dt
        self.state['L'] = max(self.state['L'], 1.0)  # longueur minimale
        
        # 6. Mettre à jour temps
        self.t += self.dt
        
        # 7. Retourner état complet
        return self.get_state()
    
    def get_state(self):
        """
        Retourne l'état actuel du système
        
        Returns:
            Dictionnaire avec état complet
        """
        # Recalculer câble pour avoir les positions
        x_ROV = self.state['rov'][0]
        y_ROV = self.state['rov'][1]
        x_boat = self.state['boat'][0]
        L = self.state['L']
        
        try:
            T, theta, x_cable, y_cable = self.cable.solve_quasi_static(
                x_ROV, y_ROV, x_boat, 0.0, self.V_courant_func
            )
        except:
            # Fallback
            x_cable = np.linspace(x_ROV, x_boat, self.cable.N + 1)
            y_cable = np.linspace(y_ROV, 0.0, self.cable.N + 1)
            T = np.ones(self.cable.N + 1) * 100.0
            theta = np.zeros(self.cable.N + 1)
        
        # Construire noeuds câble
        nodes = []
        for i in range(len(x_cable)):
            nodes.append({
                'x': float(x_cable[i]),
                'y': float(y_cable[i]),
                'T': float(T[i]),
                'theta': float(theta[i])
            })
        
        return {
            't': self.t,
            'rov': {
                'x': float(self.state['rov'][0]),
                'y': float(self.state['rov'][1]),
                'vx': float(self.state['rov'][2]),
                'vy': float(self.state['rov'][3])
            },
            'boat': {
                'x': float(self.state['boat'][0]),
                'vx': float(self.state['boat'][1])
            },
            'cable': {
                'L': float(L),
                'nodes': nodes
            }
        }
    
    def reset(self, params=None):
        """Réinitialise la simulation"""
        if params:
            self.params.update(params)
        
        self.t = 0.0
        self.state = {
            'rov': np.array([
                self.params.get('x_ROV_initial', 0.0),
                self.params.get('y_ROV_initial', 10.0),
                self.params.get('vx_ROV_initial', 0.0),
                self.params.get('vy_ROV_initial', 0.0)
            ]),
            'boat': np.array([
                self.params.get('x_bateau_initial', 0.0),
                self.params.get('vx_bateau_initial', 0.0)
            ]),
            'L': self.params.get('L_initial', 50.0)
        }
        
        self.commands = {
            'Fx_ROV': 0.0,
            'Fy_ROV': 0.0,
            'vx_bateau_cmd': 0.0,
            'dL_dt': 0.0
        }

