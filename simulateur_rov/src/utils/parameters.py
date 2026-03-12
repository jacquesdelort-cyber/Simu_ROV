"""Gestion des paramètres du système"""
import json
import os


def get_default_parameters():
    """
    Retourne les paramètres par défaut du système
    
    Returns:
    --------
    dict
        Paramètres par défaut
    """
    return {
        'rov': {
            'm': 100.0,      # Masse du ROV (kg)
            'vol': None,     # Volume du ROV (m³) - None = utiliser a*b*h
            'a': 0.5,        # Largeur (m)
            'b': 1.0,        # Longueur (m)
            'h': 0.5,        # Hauteur (m)
            'Cx': 0.8,       # Coefficient de traînée horizontal
            'Cy': 1.0        # Coefficient de traînée vertical
        },
        'cable': {
            'd': 0.01,       # Diamètre du câble (m)
            'rho_cable': 1500.0,  # Masse volumique (kg/m³)
            'Cx_cable': 1.2,      # Coefficient de traînée perpendiculaire
            'Cf_cable': 0.04      # Coefficient de frottement longitudinal
        },
        'boat': {
            'm': 10000.0,    # Masse du bateau (kg)
            'drag_coefficient': 0.5,
            'dl_dt_min': -1.0,  # Limite de rembobinage (m/s)
            'dl_dt_max': 1.0,   # Limite de débobinage (m/s)
            'gamma_moulinet_min': -0.5,  # Accel min (m/s²)
            'gamma_moulinet_max': 0.5,   # Accel max (m/s²)
        },
        'environment': {
            'rho_eau': 1025.0,  # Masse volumique eau de mer (kg/m³)
            'g': 9.81,          # Accélération pesanteur (m/s²)
            'mu': 1.0e-3,       # Viscosité dynamique (Pa·s)
            'current_profile': None  # Courant uniforme nul par défaut
        }
    }


def load_parameters(filepath):
    """
    Charge les paramètres depuis un fichier JSON
    
    Parameters:
    -----------
    filepath : str
        Chemin vers le fichier JSON
    
    Returns:
    --------
    dict
        Paramètres chargés (fusionnés avec les défauts)
    """
    defaults = get_default_parameters()
    
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            loaded = json.load(f)
        # Fusionner avec les défauts
        return merge_parameters(defaults, loaded)
    else:
        return defaults


def merge_parameters(defaults, loaded):
    """Fusionne récursivement deux dictionnaires de paramètres"""
    result = defaults.copy()
    for key, value in loaded.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = merge_parameters(result[key], value)
        else:
            result[key] = value
    return result


def save_parameters(params, filepath):
    """
    Sauvegarde les paramètres dans un fichier JSON
    
    Parameters:
    -----------
    params : dict
        Paramètres à sauvegarder
    filepath : str
        Chemin vers le fichier de sortie
    """
    with open(filepath, 'w') as f:
        json.dump(params, f, indent=4)

