"""Modèle du bateau"""
import numpy as np


class Boat:
    """Représente le navire de surface"""
    
    def __init__(self, params):
        """
        Initialise le modèle du bateau
        
        Parameters:
        -----------
        params : dict
            Paramètres du bateau:
            - m : masse (kg)
            - drag_coefficient : coefficient de résistance à l'avancement
        """
        self.m = params.get('m', 10000.0)
        self.drag_coefficient = params.get('drag_coefficient', 0.5)
    
    def compute_propulsion_force(self, v_cmd, v_current, environment):
        """
        Calcule la force de propulsion nécessaire pour atteindre la vitesse de commande
        
        Parameters:
        -----------
        v_cmd : float
            Vitesse de commande (m/s)
        v_current : float
            Vitesse actuelle (m/s)
        environment : Environment
            Objet environnement
        
        Returns:
        --------
        float
            Force de propulsion (N)
        """
        # Modèle simple : force proportionnelle à la différence de vitesse
        # avec amortissement
        k_prop = 1000.0  # Coefficient de propulsion
        k_drag = self.drag_coefficient * 500.0  # Coefficient de traînée
        
        dv = v_cmd - v_current
        F_prop = k_prop * dv - k_drag * v_current * abs(v_current)
        
        return F_prop

