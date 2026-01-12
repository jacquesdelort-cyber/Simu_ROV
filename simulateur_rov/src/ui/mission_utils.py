"""Fonctions utilitaires pour la gestion des missions (réutilisables pour Dash)"""
from pathlib import Path
import json
from datetime import datetime


def get_missions_directory():
    """Retourne le chemin du répertoire Missions"""
    base_dir = Path(__file__).parent.parent.parent
    missions_dir = base_dir / "Missions"
    missions_dir.mkdir(exist_ok=True)
    return missions_dir


def list_missions():
    """Liste toutes les missions disponibles"""
    missions_dir = get_missions_directory()
    missions = []
    if missions_dir.exists():
        for item in missions_dir.iterdir():
            if item.is_dir():
                missions.append(item.name)
    return sorted(missions)


def create_mission(mission_name):
    """Crée un nouveau répertoire de mission"""
    missions_dir = get_missions_directory()
    mission_dir = missions_dir / mission_name
    mission_dir.mkdir(exist_ok=True)
    return mission_dir


def load_environment_params(mission_dir):
    """Charge les paramètres d'environnement depuis le fichier JSON de la mission"""
    params_file = mission_dir / "Paramètres_environnement.json"
    if params_file.exists():
        try:
            with open(params_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # Extraire les paramètres
            if 'paramètres' in data:
                params = data['paramètres']
                env_params = params.get('environment', {})
                
                # Capturer la valeur de v_courant pour la closure
                v_courant_value = env_params.get('v_courant', 0.0)
                
                # Reconstruire la structure attendue avec la fonction lambda pour current_profile
                loaded_params = {
                    'rov': params.get('rov', {}),
                    'cable': params.get('cable', {}),
                    'boat': params.get('boat', {}),
                    'environment': {
                        'rho_eau': env_params.get('rho_eau', 1025.0),
                        'g': env_params.get('g', 9.81),
                        'mu': env_params.get('mu', 0.001),
                        'current_profile': lambda y, v=v_courant_value: v
                    }
                }
                return loaded_params
        except Exception as e:
            print(f"Erreur lors du chargement des paramètres d'environnement: {e}")
    return None


def load_calc_params(mission_dir):
    """Charge les paramètres de calcul depuis le fichier JSON de la mission"""
    calc_params_file = mission_dir / "Paramètres_calcul.json"
    if calc_params_file.exists():
        try:
            with open(calc_params_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            if 'paramètres_calcul' in data:
                return data['paramètres_calcul']
        except Exception as e:
            print(f"Erreur lors du chargement des paramètres de calcul: {e}")
    return None


def load_init_params(mission_dir):
    """Charge les conditions initiales depuis le fichier JSON de la mission"""
    init_params_file = mission_dir / "Param_init.json"
    if init_params_file.exists():
        try:
            with open(init_params_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            if 'conditions_initiales' in data:
                return data['conditions_initiales']
        except Exception as e:
            print(f"Erreur lors du chargement des conditions initiales: {e}")
    return None
