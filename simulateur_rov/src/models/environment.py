"""Modèle de l'environnement (courant, eau)"""
import re
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
        
        # Valeur brute du courant (peut être une chaîne, float, etc.)
        self.v_courant_raw = params.get('v_courant', None)

    def _parse_current_profile_string(self, profile_text):
        """
        Parse une chaîne décrivant un profil de courant.
        Formats acceptés:
        - valeur constante: "0.5"
        - paires profondeur/vitesse: "0:0;10:0.5;50:1.0"
          (les séparateurs sont libres; les nombres sont extraits dans l'ordre)
        """
        if profile_text is None:
            return None
        if isinstance(profile_text, (int, float, np.number)):
            return float(profile_text)
        if not isinstance(profile_text, str):
            return None
        
        text = profile_text.strip()
        if text == "":
            return 0.0
        
        numbers = re.findall(r"[-+]?\d+(?:[.,]\d+)?(?:[eE][-+]?\d+)?", text)
        if not numbers:
            return 0.0
        
        values = [float(num.replace(",", ".")) for num in numbers]
        if len(values) == 1:
            return values[0]
        if len(values) % 2 == 0:
            depths = values[0::2]
            speeds = values[1::2]
            return depths, speeds
        
        return values[0]
    
    def get_current_velocity(self, y, v_courant=None):
        """
        Retourne la vitesse du courant à une profondeur donnée
        
        Parameters:
        -----------
        y : float ou array
            Profondeur (m), positive vers le bas
        v_courant : str|float|None
            Profil de courant sous forme de chaîne ou valeur constante
        
        Returns:
        --------
        float ou array
            Vitesse du courant horizontale (m/s)
        """
        y_array = np.asarray(y)
        depth = np.where(y_array < 0, -y_array, y_array)
        
        if v_courant is None:
            return self.current_profile(depth)
        
        parsed = self._parse_current_profile_string(v_courant)
        if parsed is None:
            return self.current_profile(depth)
        if isinstance(parsed, tuple):
            depths, speeds = parsed
            return np.interp(depth, depths, speeds)
        return float(parsed)

