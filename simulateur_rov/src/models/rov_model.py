"""Modèle du ROV (Remotely Operated Vehicle)"""
import numpy as np


class ROV:
    """Représente le ROV avec ses caractéristiques physiques"""
    
    def __init__(self, params):
        """
        Initialise le modèle du ROV
        
        Parameters:
        -----------
        params : dict
            Paramètres du ROV:
            - m : masse (kg)
            - a : largeur (m)
            - b : longueur (m)
            - h : hauteur (m)
            - Cx : coefficient de traînée horizontal
            - Cy : coefficient de traînée vertical
        """
        self.m = params.get('m', 100.0)
        self.a = params.get('a', 0.5)
        self.b = params.get('b', 1.0)
        self.h = params.get('h', 0.5)
        
        # Volume et sections
        self.V = self.a * self.b * self.h
        self.Sx = self.a * self.h  # Section frontale perpendiculaire à x
        self.Sy = self.a * self.b  # Section frontale perpendiculaire à y
        
        # Coefficients de traînée
        self.Cx = params.get('Cx', 0.8)
        self.Cy = params.get('Cy', 1.0)
    
    def compute_drag_force(self, vx, vy, y_depth, environment):
        """
        Calcule la force de traînée sur le ROV
        
        Parameters:
        -----------
        vx : float
            Vitesse horizontale du ROV (m/s)
        vy : float
            Vitesse verticale du ROV (m/s)
        y_depth : float
            Profondeur du ROV (m)
        environment : Environment
            Objet environnement contenant les propriétés de l'eau
        
        Returns:
        --------
        tuple (Fx_drag, Fy_drag)
            Forces de traînée horizontale et verticale (N)
        """
        # Vitesse relative horizontale (ROV - courant)
        v_current = environment.get_current_velocity(y_depth)
        vx_rel = vx - v_current
        
        # Force de traînée horizontale
        Fx_drag = -self.Cx * 0.5 * environment.rho_eau * self.Sx * vx_rel * abs(vx_rel)
        
        # Force de traînée verticale
        Fy_drag = -self.Cy * 0.5 * environment.rho_eau * self.Sy * vy * abs(vy)
        
        return Fx_drag, Fy_drag
    
    def compute_buoyancy_force(self, environment):
        """
        Calcule la force de poussée d'Archimède
        
        Parameters:
        -----------
        environment : Environment
            Objet environnement
        
        Returns:
        --------
        float
            Force de poussée d'Archimède vers le haut (N)
        """
        return environment.rho_eau * self.V * environment.g
    
    def compute_weight_force(self, environment):
        """
        Calcule le poids du ROV
        
        Parameters:
        -----------
        environment : Environment
            Objet environnement
        
        Returns:
        --------
        float
            Poids vers le bas (N)
        """
        return self.m * environment.g

