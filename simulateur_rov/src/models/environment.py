"""Modèle de l'environnement (courant, eau)"""
import numpy as np


class Environment:
    """Représente l'environnement aquatique"""
    
    def __init__(self, params):
        """
        Initialise l'environnement
        
        Parameters:
        -----------
        params : dict
            Paramètres environnementaux:
            - rho_eau : masse volumique de l'eau (kg/m³)
            - g : accélération de la pesanteur (m/s²)
            - mu : viscosité dynamique de l'eau (Pa·s)
            - current_profile : fonction ou array pour le profil de courant
        """
        self.rho_eau = params.get('rho_eau', 1025.0)  # Eau de mer
        self.g = params.get('g', 9.81)
        self.mu = params.get('mu', 1.0e-3)  # Viscosité dynamique
        
        # Profil de courant : peut être une fonction ou un array
        current_profile = params.get('current_profile', None)
        if current_profile is None:
            # Par défaut, courant uniforme nul
            self.current_profile = lambda y: 0.0
        elif callable(current_profile):
            self.current_profile = current_profile
        else:
            # Array avec interpolation
            depths = current_profile.get('depths', [0, 100])
            speeds = current_profile.get('speeds', [0, 0])
            self.current_profile = lambda y: np.interp(y, depths, speeds)
    
    def get_current_velocity(self, y):
        """
        Retourne la vitesse du courant à une profondeur donnée
        
        Parameters:
        -----------
        y : float ou array
            Profondeur (m), positive vers le bas
        
        Returns:
        --------
        float ou array
            Vitesse du courant horizontale (m/s)
        """
        return self.current_profile(y)

