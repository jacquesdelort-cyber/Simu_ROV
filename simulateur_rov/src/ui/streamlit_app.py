"""Application Streamlit pour le simulateur ROV"""
import streamlit as st
import numpy as np
import plotly.graph_objects as go
from pathlib import Path
import sys
import time as time_module
import os
import json
from datetime import datetime

# Ajouter le chemin src au PYTHONPATH
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.models.system_model import ROVSystem
from src.utils.parameters import get_default_parameters
from src.utils.initial_conditions import get_initial_state
from src.visualization.plotter import (
    create_system_plot, create_position_plot, 
    create_velocity_plot, create_tension_plot
)

# ==================== GESTION DES MISSIONS ====================

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

def get_current_mission_directory():
    """Retourne le répertoire de la mission actuelle"""
    if 'current_mission' in st.session_state and st.session_state['current_mission']:
        missions_dir = get_missions_directory()
        mission_dir = missions_dir / st.session_state['current_mission']
        mission_dir.mkdir(exist_ok=True)
        return mission_dir
    return None

def callback_create_mission(mission_name):
    """Callback pour créer une nouvelle mission"""
    if mission_name and mission_name.strip():
        # Nettoyer le nom de la mission (enlever les caractères invalides)
        clean_name = "".join(c for c in mission_name.strip() if c.isalnum() or c in (' ', '-', '_')).strip()
        if clean_name:
            create_mission(clean_name)
            st.session_state['current_mission'] = clean_name
            st.session_state['mission_created'] = True

def load_environment_params(mission_dir):
    """Charge les paramètres d'environnement depuis le fichier JSON de la mission"""
    params_file = mission_dir / "Paramètres_environnement.json"
    if params_file.exists():
        try:
            with open(params_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # Extraire les paramètres (structure: {'date_sauvegarde': ..., 'mission': ..., 'paramètres': {...}})
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
            st.warning(f"Erreur lors du chargement des paramètres d'environnement: {e}")
    return None

def load_calc_params(mission_dir):
    """Charge les paramètres de calcul depuis le fichier JSON de la mission"""
    calc_params_file = mission_dir / "Paramètres_calcul.json"
    if calc_params_file.exists():
        try:
            with open(calc_params_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # Extraire les paramètres de calcul (structure: {'date_sauvegarde': ..., 'mission': ..., 'paramètres_calcul': {...}})
            if 'paramètres_calcul' in data:
                return data['paramètres_calcul']
        except Exception as e:
            st.warning(f"Erreur lors du chargement des paramètres de calcul: {e}")
    return None

def load_init_params(mission_dir):
    """Charge les conditions initiales depuis le fichier JSON de la mission"""
    init_params_file = mission_dir / "Param_init.json"
    if init_params_file.exists():
        try:
            with open(init_params_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # Extraire les conditions initiales (structure: {'date_sauvegarde': ..., 'mission': ..., 'conditions_initiales': {...}})
            if 'conditions_initiales' in data:
                return data['conditions_initiales']
        except Exception as e:
            st.warning(f"Erreur lors du chargement des conditions initiales: {e}")
    return None

def callback_select_mission():
    """Callback pour sélectionner une mission"""
    selected = st.session_state.get('select_mission')
    if selected and selected != "":
        st.session_state['current_mission'] = selected
        
        # Réinitialiser les flags de chargement
        st.session_state['params_loaded'] = False
        st.session_state['calc_params_loaded'] = False
        st.session_state['init_params_loaded'] = False
        
        # Charger les paramètres d'environnement de la mission
        missions_dir = get_missions_directory()
        mission_dir = missions_dir / selected
        
        if mission_dir.exists():
            # Charger les paramètres d'environnement
            env_params = load_environment_params(mission_dir)
            if env_params is not None:
                st.session_state['params'] = env_params
                
                # Mettre à jour les widgets des paramètres d'environnement
                params = env_params
                # ROV
                if 'rov' in params:
                    rov = params['rov']
                    st.session_state['m_rov'] = float(rov.get('m', 100.0))
                    st.session_state['a_rov'] = float(rov.get('a', 0.5))
                    st.session_state['b_rov'] = float(rov.get('b', 1.0))
                    st.session_state['h_rov'] = float(rov.get('h', 0.5))
                    st.session_state['cx_rov'] = float(rov.get('Cx', 0.8))
                    st.session_state['cy_rov'] = float(rov.get('Cy', 1.0))
                
                # Câble
                if 'cable' in params:
                    cable = params['cable']
                    st.session_state['d_cable'] = float(cable.get('d', 0.01))
                    st.session_state['rho_cable'] = float(cable.get('rho_cable', 1500.0))
                    st.session_state['cx_cable'] = float(cable.get('Cx_cable', 1.2))
                
                # Bateau
                if 'boat' in params:
                    boat = params['boat']
                    st.session_state['m_boat'] = float(boat.get('m', 10000.0))
                
                # Environnement
                if 'environment' in params:
                    env = params['environment']
                    st.session_state['rho_eau'] = float(env.get('rho_eau', 1025.0))
                    st.session_state['g'] = float(env.get('g', 9.81))
                    st.session_state['mu'] = float(env.get('mu', 0.001))
                    # Pour v_courant, extraire la valeur depuis current_profile si c'est une fonction lambda
                    # Sinon, utiliser v_courant directement
                    v_courant_value = env.get('v_courant', 0.0)
                    if v_courant_value is None or callable(v_courant_value):
                        # Si c'est une fonction lambda, on essaie de l'appeler avec 0 pour obtenir la valeur
                        try:
                            v_courant_value = v_courant_value(0.0) if callable(v_courant_value) else 0.0
                        except:
                            v_courant_value = 0.0
                    st.session_state['v_courant'] = float(v_courant_value)
                
                st.session_state['params_loaded'] = True
                st.session_state['params_loaded_mission'] = selected
            
            # Charger les paramètres de calcul
            calc_params = load_calc_params(mission_dir)
            if calc_params is not None:
                # Stocker les paramètres chargés dans session_state
                # Les widgets liront depuis calc_params au lieu de modifier directement st.session_state
                st.session_state['calc_params'] = calc_params
                st.session_state['calc_params_loaded'] = True
                st.session_state['calc_params_loaded_mission'] = selected
            
            # Charger les conditions initiales
            init_params = load_init_params(mission_dir)
            if init_params is not None:
                # Stocker les conditions initiales chargées dans session_state
                # Les widgets liront depuis init_params au lieu de modifier directement st.session_state
                st.session_state['init_params'] = init_params
                st.session_state['init_params_loaded'] = True
                st.session_state['init_params_loaded_mission'] = selected

# ==================== CALLBACKS ====================

def callback_start_simulation():
    """Callback pour démarrer la simulation"""
    st.session_state['start_simulation'] = True

def callback_stop_simulation():
    """Callback pour arrêter la simulation"""
    if 'solution' in st.session_state:
        del st.session_state['solution']
    if 'system' in st.session_state:
        del st.session_state['system']
    if 'simulation_running' in st.session_state:
        st.session_state['simulation_running'] = False
    if 'simulation_paused' in st.session_state:
        del st.session_state['simulation_paused']
    # Les conteneurs ne sont plus utilisés, les graphiques sont recréés à chaque rerun

def callback_pause_resume():
    """Callback pour mettre en pause/reprendre la simulation"""
    if 'simulation_paused' in st.session_state:
        st.session_state['simulation_paused'] = not st.session_state['simulation_paused']
    else:
        st.session_state['simulation_paused'] = True

def callback_restart():
    """Callback pour relancer la simulation"""
    if 'simulation_running' in st.session_state:
        del st.session_state['simulation_running']
    if 'simulation_paused' in st.session_state:
        del st.session_state['simulation_paused']

def callback_save_params(m_rov, a_rov, b_rov, h_rov, cx_rov, cy_rov,
                         d_cable, rho_cable, cx_cable, m_boat,
                         rho_eau, g, mu, v_courant):
    """Callback pour sauvegarder les paramètres"""
    # Créer le dictionnaire des paramètres
    params = {
        'rov': {
            'm': m_rov,
            'a': a_rov,
            'b': b_rov,
            'h': h_rov,
            'Cx': cx_rov,
            'Cy': cy_rov
        },
        'cable': {
            'd': d_cable,
            'rho_cable': rho_cable,
            'Cx_cable': cx_cable
        },
        'boat': {
            'm': m_boat
        },
        'environment': {
            'rho_eau': rho_eau,
            'g': g,
            'mu': mu,
            'v_courant': v_courant  # Sauvegarder la valeur du courant (pas la fonction lambda)
        }
    }
    
    # Mettre à jour le session state
    st.session_state['params'] = {
        'rov': params['rov'],
        'cable': params['cable'],
        'boat': params['boat'],
        'environment': {
            'rho_eau': params['environment']['rho_eau'],
            'g': params['environment']['g'],
            'mu': params['environment']['mu'],
            'current_profile': lambda y: params['environment']['v_courant']  # Recréer la fonction lambda
        }
    }
    
    # Sauvegarder dans le fichier de la mission si une mission est sélectionnée
    mission_dir = get_current_mission_directory()
    if mission_dir:
        params_file = mission_dir / "Paramètres_environnement.json"
        try:
            # Ajouter la date de sauvegarde
            params_with_metadata = {
                'date_sauvegarde': datetime.now().isoformat(),
                'mission': st.session_state.get('current_mission'),
                'paramètres': params
            }
            
            with open(params_file, 'w', encoding='utf-8') as f:
                json.dump(params_with_metadata, f, indent=4, ensure_ascii=False)
            st.session_state['params_saved'] = True
            st.session_state['params_file_saved'] = True
        except Exception as e:
            st.session_state['params_saved'] = True
            st.session_state['params_save_error'] = str(e)
    else:
        # Sauvegarder quand même dans le session state même sans mission
        st.session_state['params_saved'] = True
        st.session_state['params_save_warning'] = "Aucune mission sélectionnée. Les paramètres sont sauvegardés en mémoire uniquement."

def callback_save_calc_params(method, rtol, atol, max_step, steps_per_update,
                                t_final, dt_max, N_segments):
    """Callback pour sauvegarder les paramètres de calcul"""
    # Créer le dictionnaire des paramètres de calcul
    calc_params = {
        'method': method,
        'rtol': rtol,
        'atol': atol,
        'max_step': max_step,
        'steps_per_update': steps_per_update,
        't_final': t_final,
        'dt_max': dt_max,
        'N_segments': N_segments
    }
    
    # Mettre à jour le session state
    st.session_state['calc_params'] = calc_params
    
    # Sauvegarder dans le fichier de la mission si une mission est sélectionnée
    mission_dir = get_current_mission_directory()
    if mission_dir:
        calc_params_file = mission_dir / "Paramètres_calcul.json"
        try:
            # Ajouter la date de sauvegarde
            calc_params_with_metadata = {
                'date_sauvegarde': datetime.now().isoformat(),
                'mission': st.session_state.get('current_mission'),
                'paramètres_calcul': calc_params
            }
            
            with open(calc_params_file, 'w', encoding='utf-8') as f:
                json.dump(calc_params_with_metadata, f, indent=4, ensure_ascii=False)
            st.session_state['calc_params_saved'] = True
            st.session_state['calc_params_file_saved'] = True
        except Exception as e:
            st.session_state['calc_params_saved'] = True
            st.session_state['calc_params_save_error'] = str(e)
    else:
        # Sauvegarder quand même dans le session state même sans mission
        st.session_state['calc_params_saved'] = True
        st.session_state['calc_params_save_warning'] = "Aucune mission sélectionnée. Les paramètres de calcul sont sauvegardés en mémoire uniquement."

def callback_save_init_params(x_rov_init, y_rov_init, L_init, v_courant):
    """Callback pour sauvegarder les conditions initiales"""
    # Créer le dictionnaire des conditions initiales
    init_params = {
        'x_rov_init': x_rov_init,
        'y_rov_init': y_rov_init,
        'L_init': L_init,
        'v_courant': v_courant
    }
    
    # Mettre à jour le session state
    st.session_state['init_params'] = init_params
    
    # Sauvegarder dans le fichier de la mission si une mission est sélectionnée
    mission_dir = get_current_mission_directory()
    if mission_dir:
        init_params_file = mission_dir / "Param_init.json"
        try:
            # Ajouter la date de sauvegarde
            init_params_with_metadata = {
                'date_sauvegarde': datetime.now().isoformat(),
                'mission': st.session_state.get('current_mission'),
                'conditions_initiales': init_params
            }
            
            with open(init_params_file, 'w', encoding='utf-8') as f:
                json.dump(init_params_with_metadata, f, indent=4, ensure_ascii=False)
            st.session_state['init_params_saved'] = True
            st.session_state['init_params_file_saved'] = True
        except Exception as e:
            st.session_state['init_params_saved'] = True
            st.session_state['init_params_save_error'] = str(e)
    else:
        # Sauvegarder quand même dans le session state même sans mission
        st.session_state['init_params_saved'] = True
        st.session_state['init_params_save_warning'] = "Aucune mission sélectionnée. Les conditions initiales sont sauvegardées en mémoire uniquement."

def callback_save_all_config():
    """Callback pour sauvegarder tous les paramètres (environnement, calcul et conditions initiales)"""
    # Récupérer toutes les valeurs depuis session_state
    # Paramètres d'environnement
    m_rov = st.session_state.get('m_rov', 100.0)
    a_rov = st.session_state.get('a_rov', 0.5)
    b_rov = st.session_state.get('b_rov', 1.0)
    h_rov = st.session_state.get('h_rov', 0.5)
    cx_rov = st.session_state.get('cx_rov', 0.8)
    cy_rov = st.session_state.get('cy_rov', 1.0)
    d_cable = st.session_state.get('d_cable', 0.01)
    rho_cable = st.session_state.get('rho_cable', 1500.0)
    cx_cable = st.session_state.get('cx_cable', 1.2)
    m_boat = st.session_state.get('m_boat', 10000.0)
    rho_eau = st.session_state.get('rho_eau', 1025.0)
    g = st.session_state.get('g', 9.81)
    mu = st.session_state.get('mu', 0.001)
    v_courant = st.session_state.get('v_courant', 0.0)
    
    # Paramètres de calcul
    method = st.session_state.get('method', 'RK45')
    rtol = st.session_state.get('rtol', 1e-5)
    atol = st.session_state.get('atol', 1e-7)
    max_step = st.session_state.get('max_step', 0.1)
    steps_per_update = st.session_state.get('steps_per_update', 5)
    t_final = st.session_state.get('t_final', 60.0)
    dt_max = st.session_state.get('dt_max', 0.1)
    N_segments = st.session_state.get('N_segments', 50)
    
    # Conditions initiales
    x_rov_init = st.session_state.get('x_rov_init', 0.0)
    y_rov_init = st.session_state.get('y_rov_init', 10.0)
    L_init = st.session_state.get('L_init', 50.0)
    
    # Appeler les 3 fonctions de sauvegarde
    callback_save_params(m_rov, a_rov, b_rov, h_rov, cx_rov, cy_rov,
                        d_cable, rho_cable, cx_cable, m_boat,
                        rho_eau, g, mu, v_courant)
    
    callback_save_calc_params(method, rtol, atol, max_step, steps_per_update,
                              t_final, dt_max, N_segments)
    
    callback_save_init_params(x_rov_init, y_rov_init, L_init, v_courant)
    
    # Marquer que la sauvegarde globale est terminée
    st.session_state['all_config_saved'] = True

def main():
    """Fonction principale de l'application Streamlit"""
    st.set_page_config(
        page_title="Simulateur ROV",
        page_icon="🌊",
        layout="wide"
    )

    # Onglets principaux
    tab_simu, tab_params, tab_calc, tab_init = st.tabs(["Simulation", "Paramètres d'environnement", "Paramètres calcul", "Conditions initiales"])
    
    # Initialiser les paramètres dans session state
    if 'params' not in st.session_state:
        st.session_state['params'] = get_default_parameters()
    
    # Initialiser les paramètres de calcul dans session state
    if 'calc_params' not in st.session_state:
        st.session_state['calc_params'] = {
            'method': 'RK45',
            'rtol': 1e-5,
            'atol': 1e-7,
            'max_step': 0.1,
            'steps_per_update': 5,
            't_final': 60.0,
            'dt_max': 0.1,
            'N_segments': 50
        }
    
    # Onglet Paramètres
    with tab_params:
        # Créer un tableau en 4 colonnes
        col1, col2, col3, col4 = st.columns(4)
        
        with col1:
            st.subheader("ROV")
            params_rov = st.session_state['params']['rov']
            
            m_rov = st.number_input(
                "Masse du ROV (kg) :",
                value=float(st.session_state.get('m_rov', params_rov.get('m', 100.0))),
                key="m_rov",
                format="%.2f"
            )
            a_rov = st.number_input(
                "Largeur du ROV (m) :",
                value=float(st.session_state.get('a_rov', params_rov.get('a', 0.5))),
                key="a_rov",
                format="%.3f"
            )
            b_rov = st.number_input(
                "Longueur du ROV (m) :",
                value=float(st.session_state.get('b_rov', params_rov.get('b', 1.0))),
                key="b_rov",
                format="%.3f"
            )
            h_rov = st.number_input(
                "Hauteur du ROV (m) :",
                value=float(st.session_state.get('h_rov', params_rov.get('h', 0.5))),
                key="h_rov",
                format="%.3f"
            )
            cx_rov = st.number_input(
                "Cx_ROV :",
                value=float(st.session_state.get('cx_rov', params_rov.get('Cx', 0.8))),
                key="cx_rov",
                format="%.2f"
            )
            cy_rov = st.number_input(
                "Cy_ROV :",
                value=float(st.session_state.get('cy_rov', params_rov.get('Cy', 1.0))),
                key="cy_rov",
                format="%.2f"
            )
        
        with col2:
            st.subheader("Câble")
            params_cable = st.session_state['params']['cable']
            
            d_cable = st.number_input(
                "Diamètre du câble (m) :",
                value=float(st.session_state.get('d_cable', params_cable.get('d', 0.01))),
                key="d_cable",
                format="%.4f"
            )
            rho_cable = st.number_input(
                "Masse volumique du câble (kg/m³) :",
                value=float(st.session_state.get('rho_cable', params_cable.get('rho_cable', 1500.0))),
                key="rho_cable",
                format="%.2f"
            )
            cx_cable = st.number_input(
                "Coefficient de traînée du câble :",
                value=float(st.session_state.get('cx_cable', params_cable.get('Cx_cable', 1.2))),
                key="cx_cable",
                format="%.2f"
            )
        
        with col3:
            st.subheader("Bateau")
            params_boat = st.session_state['params']['boat']
            
            m_boat = st.number_input(
                "Masse du bateau (kg) :",
                value=float(st.session_state.get('m_boat', params_boat.get('m', 10000.0))),
                key="m_boat",
                format="%.2f"
            )
        
        with col4:
            st.subheader("Environnement")
            params_env = st.session_state['params']['environment']
            
            rho_eau = st.number_input(
                "Masse volumique de l'eau (kg/m³) :",
                value=float(st.session_state.get('rho_eau', params_env.get('rho_eau', 1025.0))),
                key="rho_eau",
                format="%.2f"
            )
            g = st.number_input(
                "Accélération de la pesanteur (m/s²) :",
                value=float(st.session_state.get('g', params_env.get('g', 9.81))),
                key="g",
                format="%.2f"
            )
            mu = st.number_input(
                "Viscosité dynamique de l'eau (Pa·s) :",
                value=float(st.session_state.get('mu', params_env.get('mu', 0.001))),
                key="mu",
                format="%.6f"
            )
        
    # Onglet Paramètres calcul
    with tab_calc:
        # st.header("Paramètres de calcul")
        # st.markdown("Configurez les paramètres numériques pour l'intégration des équations différentielles.")
        
        # Récupérer calc_params depuis session_state (peut être chargé depuis une mission)
        loaded_calc_params = st.session_state.get('calc_params', {})
        calc_params = st.session_state.get('calc_params', {
            'method': 'RK45',
            'rtol': 1e-5,
            'atol': 1e-7,
            'max_step': 0.1,
            'steps_per_update': 5,
            't_final': 60.0,
            'dt_max': 0.1,
            'N_segments': 50
        })
        
        col_calc1, col_calc2 = st.columns(2)
        
        with col_calc1:
            st.subheader("Méthode d'intégration")
            
            methods = ['RK45', 'RK23', 'DOP853', 'Radau', 'BDF', 'LSODA']
            # Utiliser la valeur du widget si elle existe, sinon utiliser calc_params chargé
            current_method = st.session_state.get('method', loaded_calc_params.get('method', calc_params.get('method', 'RK45')))
            try:
                method_index = methods.index(current_method)
            except ValueError:
                method_index = 0  # Défaut à RK45 si méthode inconnue
            
            method = st.selectbox(
                "Méthode d'intégration :",
                options=methods,
                index=method_index,
                key="method",
                help="RK45: Runge-Kutta 4/5 (adaptatif, recommandé pour systèmes non-rigides)\n"
                     "DOP853: Runge-Kutta 8/9 (haute précision)\n"
                     "BDF: Backward Differentiation Formula (pour systèmes rigides)\n"
                     "Radau: Méthode implicite (pour systèmes très rigides)"
            )
            
            st.markdown("---")
            st.subheader("Tolérances")
            
            rtol = st.number_input(
                "Tolérance relative (rtol) :",
                value=float(st.session_state.get('rtol', loaded_calc_params.get('rtol', calc_params.get('rtol', 1e-5)))),
                min_value=1e-10,
                max_value=1e-1,
                format="%.2e",
                key="rtol",
                help="Erreur relative tolérée. Plus petit = plus précis mais plus lent."
            )
            
            atol = st.number_input(
                "Tolérance absolue (atol) :",
                value=float(st.session_state.get('atol', loaded_calc_params.get('atol', calc_params.get('atol', 1e-7)))),
                min_value=1e-12,
                max_value=1e-3,
                format="%.2e",
                key="atol",
                help="Erreur absolue tolérée. Plus petit = plus précis mais plus lent."
            )
            
            st.markdown("---")
            st.subheader("Performance")
            
            steps_per_update = st.number_input(
                "Pas de calcul par mise à jour d'affichage :",
                value=int(st.session_state.get('steps_per_update', loaded_calc_params.get('steps_per_update', calc_params.get('steps_per_update', 5)))),
                min_value=1,
                max_value=50,
                step=1,
                key="steps_per_update",
                help=(
                    "Nombre de pas de calcul effectués avant de mettre à jour l'affichage. "
                    "Augmenter pour améliorer les performances mais réduire la réactivité."
                )
            )
        
        # Aide sur les paramètres

        c1_1, c1_2 = st.columns(2)
        with c1_1:
            st.markdown("---")
            with st.expander("ℹ️ Aide sur les paramètres"):
                    st.markdown("""
                    **Méthodes d'intégration :**
                    - **RK45** (Recommandé) : Méthode adaptative Runge-Kutta d'ordre 4/5, efficace pour la plupart des systèmes
                    - **RK23** : Version d'ordre inférieur, plus rapide mais moins précise
                    - **DOP853** : Méthode d'ordre 8/9, très précise mais plus lente
                    - **Radau** : Méthode implicite, adaptée aux systèmes rigides
                    - **BDF** : Backward Differentiation Formula, pour systèmes rigides
                    - **LSODA** : Sélectionne automatiquement entre méthode explicite et implicite
                    
                    **Tolérances :**
                    - Des tolérances plus petites donnent des résultats plus précis mais ralentissent le calcul
                    - Valeurs recommandées : rtol = 1e-5, atol = 1e-7
                    - Pour des calculs rapides : rtol = 1e-3, atol = 1e-5
                    - Pour haute précision : rtol = 1e-7, atol = 1e-9
                    
                    **Performance :**
                    - Augmenter "Pas de calcul par mise à jour" améliore les performances mais réduit la fréquence d'affichage
                    - Valeur recommandée : 5
                    """)
            with c1_2:
                    st.markdown("---")
                    with st.expander("ℹ️ Aide sur les paramètres de simulation et d'intégration"):
                        st.markdown("""
                        **Paramètres de simulation :**
                        - **Temps final** : Durée totale de la simulation en secondes. Détermine jusqu'à quel moment la simulation sera exécutée.
                        - **Pas de temps max** : Pas de temps maximum pour l'intégration. Un pas plus petit donne une meilleure précision mais ralentit le calcul. Recommandé : 0.1 s pour la plupart des cas.
                        - **Nombre de segments du câble** : Nombre de segments utilisés pour discrétiser le câble. Plus de segments = meilleure précision mais calcul plus lent. Recommandé : 50 segments pour un bon compromis.
                        
                        **Paramètres d'intégration :**
                        - **Pas de temps maximum intégrateur** : Pas de temps maximum autorisé par l'intégrateur numérique. Ce paramètre limite la taille des pas que l'intégrateur peut prendre. Il sera automatiquement limité par le "Pas de temps max" de la simulation. Recommandé : 0.1 s ou égal au pas de temps max.
                        
                        **Conseils d'optimisation :**
                        - Pour des simulations rapides : Augmenter le pas de temps max (0.2-0.5 s) et réduire le nombre de segments (30-40)
                        - Pour haute précision : Réduire le pas de temps max (0.01-0.05 s) et augmenter le nombre de segments (70-100)
                        - Le temps final doit être adapté à la durée réelle de la mission à simuler
                        """)    

        with col_calc2:
            st.subheader("Paramètres de simulation")
            
            t_final = st.number_input(
                "Temps final (s) :",
                value=float(st.session_state.get('t_final', loaded_calc_params.get('t_final', calc_params.get('t_final', 60.0)))),
                min_value=10.0,
                max_value=300.0,
                step=1.0,
                format="%.1f",
                key="t_final",
                help="Durée totale de la simulation en secondes."
            )
            
            dt_max = st.number_input(
                "Pas de temps max (s) :",
                value=float(st.session_state.get('dt_max', loaded_calc_params.get('dt_max', calc_params.get('dt_max', 0.1)))),
                min_value=0.01,
                max_value=1.0,
                step=0.01,
                format="%.3f",
                key="dt_max",
                help="Pas de temps maximum pour l'intégration. Plus petit = plus précis mais plus lent."
            )
            
            N_segments = st.number_input(
                "Nombre de segments du câble :",
                value=int(st.session_state.get('N_segments', loaded_calc_params.get('N_segments', calc_params.get('N_segments', 50)))),
                min_value=10,
                max_value=100,
                step=1,
                key="N_segments",
                help="Nombre de segments pour la discrétisation du câble. Plus grand = plus précis mais plus lent."
            )
            
            st.markdown("---")
            st.subheader("Paramètres d'intégration")
            
            max_step = st.number_input(
                "Pas de temps maximum intégrateur (s) :",
                value=float(st.session_state.get('max_step', loaded_calc_params.get('max_step', calc_params.get('max_step', 0.1)))),
                min_value=0.001,
                max_value=10.0,
                step=0.01,
                format="%.3f",
                key="max_step",
                help="Pas de temps maximum autorisé par l'intégrateur. Sera limité par dt_max de la simulation."
            )

    # Onglet Conditions initiales
    with tab_init:
        col_init_1, col_init_2 = st.columns(2)
        with col_init_1:
            st.subheader("Conditions initiales")

            # Récupérer init_params depuis session_state (peut être chargé depuis une mission)
            loaded_init_params = st.session_state.get('init_params', {})
            
            # Utiliser session_state si disponible, sinon utiliser les valeurs chargées, sinon valeurs par défaut
            x_rov_init = st.number_input(
                "Position horizontale ROV (m)", 
                0.0, 100.0, 
                float(st.session_state.get('x_rov_init', loaded_init_params.get('x_rov_init', 0.0))), 
                1.0, 
                key="x_rov_init"
            )
            y_rov_init = st.number_input(
                "Profondeur ROV (m)", 
                0.0, 200.0, 
                float(st.session_state.get('y_rov_init', loaded_init_params.get('y_rov_init', 10.0))), 
                1.0, 
                key="y_rov_init"
            )
            st.markdown("---")
            st.subheader("Câble")
            L_init = st.number_input(
                "Longueur câble (m)", 
                10.0, 500.0, 
                float(st.session_state.get('L_init', loaded_init_params.get('L_init', 50.0))), 
                1.0, 
                key="L_init"
            )
            
            st.markdown("---")
            st.subheader("Environnement")
            
            # Récupérer les paramètres d'environnement pour la valeur par défaut
            params_env = st.session_state.get('params', {}).get('environment', {})
            v_courant = st.number_input(
                "Vitesse du courant (m/s) :",
                value=float(st.session_state.get('v_courant', loaded_init_params.get('v_courant', 0.0))),
                key="v_courant",
                format="%.3f"
            )
        
    # Onglet Simulation
    with tab_simu:
        # Initialiser la mission actuelle si nécessaire
        if 'current_mission' not in st.session_state:
            st.session_state['current_mission'] = None
        
        # Récupérer les conditions initiales depuis session_state
        x_rov_init = st.session_state.get('x_rov_init', 0.0)
        y_rov_init = st.session_state.get('y_rov_init', 10.0)
        L_init = st.session_state.get('L_init', 50.0)
        
        # Paramètres de simulation et commandes
        col_sim_1, col_sim_2, col_sim_3, col_sim_4, col_sim_5 = st.columns([1, 1, 4, 1, 1])
        
        with col_sim_1:
            # Conteneur pour le nom de la mission
            mission_container = st.container()
            with mission_container:
                if st.session_state.get('current_mission'):
                    st.markdown(f"### 🎯 Mission: **{st.session_state['current_mission']}**")
                else:
                    st.info("ℹ️ Aucune mission sélectionnée")
                    st.markdown("---")
            
            # Gestion des missions
            with st.expander("📁 Gérer les missions", expanded=False):
                # Liste des missions existantes
                missions = list_missions()
                if missions:
                    # Déterminer l'index initial
                    current_mission = st.session_state.get('current_mission')
                    if current_mission and current_mission in missions:
                        initial_index = missions.index(current_mission) + 1
                    else:
                        initial_index = 0
                    
                    st.selectbox(
                        "Sélectionner une mission existante",
                        options=[""] + missions,
                        key="select_mission",
                        index=initial_index,
                        on_change=callback_select_mission
                    )
                
                # Créer une nouvelle mission
                new_mission_name = st.text_input(
                    "Nom de la nouvelle mission",
                    key="new_mission_name",
                    placeholder="Ex: Mission_2024_01_15"
                )
                if st.button("➕ Créer une nouvelle mission", 
                            on_click=callback_create_mission,
                            args=(new_mission_name,)):
                    pass
                          # Boutons de contrôle avec callbacks
                st.markdown("---")
                
                # Bouton pour sauvegarder toute la configuration
                if st.button("💾 Sauvegarder config", type="primary", width='stretch',
                            on_click=callback_save_all_config):
                    pass
                # Afficher le message de confirmation
                if st.session_state.get('mission_created', False):
                    st.success(f"Mission '{st.session_state.get('current_mission')}' créée avec succès!")
                    st.session_state['mission_created'] = False
                
                # Afficher les messages de chargement des paramètres
                if st.session_state.get('params_loaded', False):
                    mission_name = st.session_state.get('params_loaded_mission', 'mission')
                    st.success(f"✓ Paramètres d'environnement chargés depuis la mission '{mission_name}'")
                    st.session_state['params_loaded'] = False
                
                if st.session_state.get('calc_params_loaded', False):
                    mission_name = st.session_state.get('calc_params_loaded_mission', 'mission')
                    st.success(f"✓ Paramètres de calcul chargés depuis la mission '{mission_name}'")
                    st.session_state['calc_params_loaded'] = False
                
                if st.session_state.get('init_params_loaded', False):
                    mission_name = st.session_state.get('init_params_loaded_mission', 'mission')
                    st.success(f"✓ Conditions initiales chargées depuis la mission '{mission_name}'")
                    st.session_state['init_params_loaded'] = False
                            # Afficher les messages de sauvegarde globale
                if st.session_state.get('all_config_saved', False):
                    # Vérifier les résultats de chaque sauvegarde
                    messages = []
                    errors = []
                    warnings = []
                    
                    if st.session_state.get('params_file_saved', False):
                        mission_name = st.session_state.get('current_mission', 'mission')
                        messages.append(f"✓ Paramètres d'environnement sauvegardés dans la mission '{mission_name}'")
                        st.session_state['params_file_saved'] = False
                    elif st.session_state.get('params_save_error'):
                        errors.append(f"✗ Erreur paramètres d'environnement: {st.session_state['params_save_error']}")
                        st.session_state['params_save_error'] = None
                    elif st.session_state.get('params_save_warning'):
                        warnings.append(st.session_state['params_save_warning'])
                        st.session_state['params_save_warning'] = None
                    
                    if st.session_state.get('calc_params_file_saved', False):
                        mission_name = st.session_state.get('current_mission', 'mission')
                        messages.append(f"✓ Paramètres de calcul sauvegardés dans la mission '{mission_name}'")
                        st.session_state['calc_params_file_saved'] = False
                    elif st.session_state.get('calc_params_save_error'):
                        errors.append(f"✗ Erreur paramètres de calcul: {st.session_state['calc_params_save_error']}")
                        st.session_state['calc_params_save_error'] = None
                    elif st.session_state.get('calc_params_save_warning'):
                        warnings.append(st.session_state['calc_params_save_warning'])
                        st.session_state['calc_params_save_warning'] = None
                    
                    if st.session_state.get('init_params_file_saved', False):
                        mission_name = st.session_state.get('current_mission', 'mission')
                        messages.append(f"✓ Conditions initiales sauvegardées dans la mission '{mission_name}'")
                        st.session_state['init_params_file_saved'] = False
                    elif st.session_state.get('init_params_save_error'):
                        errors.append(f"✗ Erreur conditions initiales: {st.session_state['init_params_save_error']}")
                        st.session_state['init_params_save_error'] = None
                    elif st.session_state.get('init_params_save_warning'):
                        warnings.append(st.session_state['init_params_save_warning'])
                        st.session_state['init_params_save_warning'] = None
                    
                    # Afficher les messages
                    if messages:
                        for msg in messages:
                            st.success(msg)
                    if warnings:
                        for warn in warnings:
                            st.warning(warn)
                    if errors:
                        for err in errors:
                            st.error(err)
                    
                    if not messages and not warnings and not errors:
                        mission_name = st.session_state.get('current_mission', 'mission')
                        if mission_name:
                            st.success(f"Configuration sauvegardée avec succès dans la mission '{mission_name}'!")
                        else:
                            st.warning("Aucune mission sélectionnée. La configuration est sauvegardée en mémoire uniquement.")
                    
                    st.session_state['all_config_saved'] = False

            st.markdown("---")
            st.subheader("Commandes")
            
            st.markdown("**ROV**")
            fx_rov = st.slider("Force horizontale (N)", -100.0, 100.0, 0.0, 1.0)
            fy_rov = st.slider("Force verticale (N)", -100.0, 100.0, 0.0, 1.0)
            
            st.markdown("**Bateau**")
            vx_bateau_cmd = st.slider("Vitesse commande (m/s)", -2.0, 2.0, 0.0, 0.1)
            
            st.markdown("**Câble**")
            dL_dt = st.slider("Vitesse déroulement (m/s)", -1.0, 1.0, 0.0, 0.1)
            


        # Zone de visualisation - toujours visible
        st.markdown("---")
        #st.subheader("Visualisation")
        
        # Initialiser les données de visualisation dans session state
        if 'simulation_data' not in st.session_state:
            st.session_state['simulation_data'] = {
                'time': [],
                'x_rov': [],
                'y_rov': [],
                'vx_rov': [],
                'vy_rov': [],
                'x_boat': [],
                'L': [],
                'T0': [],
                'T_boat': [],
                'T_max': [],
                'x_cable_curr': None,
                'y_cable_curr': None
            }
        
        # Les graphiques sont recréés à chaque rerun pour se rafraîchir automatiquement
        
        # Zone de visualisation et résultats
        # Vérifier si la simulation doit être démarrée (via callback)
        if st.session_state.get('start_simulation', False):
            st.session_state['start_simulation'] = False
            run_simulation = True
        else:
            run_simulation = False
        
        if run_simulation or 'simulation_running' in st.session_state:
            if run_simulation:
                # Récupérer les paramètres de calcul
                calc_params = st.session_state.get('calc_params', {
                    'method': 'RK45',
                    'rtol': 1e-5,
                    'atol': 1e-7,
                    'max_step': 0.1,
                    'steps_per_update': 5,
                    't_final': 60.0,
                    'dt_max': 0.1,
                    'N_segments': 50
                })
                
                # Initialiser la simulation
                params = st.session_state['params']
                N_segments = int(calc_params.get('N_segments', 50))
                system = ROVSystem(params, N_segments=N_segments)
                
                # Conditions initiales
                y0 = get_initial_state(
                    system,
                    x_rov=x_rov_init,
                    y_rov=y_rov_init,
                    x_boat=x_rov_init,
                    L=L_init
                )
                
                # Réinitialiser les données
                st.session_state['simulation_data'] = {
                    'time': [],
                    'x_rov': [],
                    'y_rov': [],
                    'vx_rov': [],
                    'vy_rov': [],
                    'x_boat': [],
                    'L': [],
                    'T0': [],
                    'T_boat': [],
                    'T_max': [],
                    'x_cable_curr': None,
                    'y_cable_curr': None
                }
                
                # Stocker l'état initial
                st.session_state['system'] = system
                st.session_state['y_current'] = y0
                st.session_state['t_current'] = 0.0
                st.session_state['simulation_running'] = True
                st.session_state['simulation_paused'] = False
                import time as time_module
                st.session_state['last_display_update_time'] = time_module.time()  # Temps réel de la dernière mise à jour d'affichage
                
                # Paramètres de simulation - utiliser les valeurs directement sans modifier st.session_state pour les widgets
                # Les valeurs t_final et dt_max viennent des widgets, on les lit depuis st.session_state ou calc_params
                t_final_sim = st.session_state.get('t_final', calc_params.get('t_final', 60.0))
                dt_max_sim = st.session_state.get('dt_max', calc_params.get('dt_max', 0.1))
                st.session_state['sim_t_final'] = t_final_sim
                st.session_state['sim_dt_max'] = dt_max_sim
                st.session_state['fx_rov'] = fx_rov
                st.session_state['fy_rov'] = fy_rov
                st.session_state['vx_bateau_cmd'] = vx_bateau_cmd
                st.session_state['dL_dt'] = dL_dt
            
            # Fonction de commande
            def u_func(t):
                return {
                    'Fx_rov': st.session_state.get('fx_rov', fx_rov),
                    'Fy_rov': st.session_state.get('fy_rov', fy_rov),
                    'vx_boat_cmd': st.session_state.get('vx_bateau_cmd', vx_bateau_cmd),
                    'dL_dt': st.session_state.get('dL_dt', dL_dt)
                }
            
            # Simulation en temps réel
            if ('simulation_running' in st.session_state and st.session_state['simulation_running'] and
                'system' in st.session_state and 'y_current' in st.session_state and 't_current' in st.session_state):
                system = st.session_state['system']
                y_current = st.session_state['y_current']
                t_current = st.session_state['t_current']
                
                # Faire plusieurs pas de simulation avant de mettre à jour l'affichage
                t_final_sim = st.session_state.get('sim_t_final', st.session_state.get('t_final', 60.0))
                if not st.session_state.get('simulation_paused', False) and t_current < t_final_sim:
                    from scipy.integrate import solve_ivp
                    import time
                    
                    # Calculer un pas de temps
                    dt_max_sim = st.session_state.get('sim_dt_max', st.session_state.get('dt_max', 0.1))
                    t_final_sim = st.session_state.get('sim_t_final', st.session_state.get('t_final', 60.0))
                    dt = min(dt_max_sim, t_final_sim - t_current)
                    
                    # Intégrer sur un petit intervalle
                    def system_ode(t, y):
                        u = u_func(t)
                        return system.compute_derivatives(t, y, u)
                    
                    try:
                        # Récupérer les paramètres de calcul
                        calc_params = st.session_state.get('calc_params', {
                            'method': 'RK45',
                            'rtol': 1e-5,
                            'atol': 1e-7,
                            'max_step': 0.1,
                            'steps_per_update': 5,
                            't_final': 60.0,
                            'dt_max': 0.1,
                            'N_segments': 50
                        })
                        
                        import time as time_module
                        last_display_time = st.session_state.get('last_display_update_time', time_module.time())
                        current_real_time = time_module.time()
                        time_since_last_display = current_real_time - last_display_time
                        
                        # Faire plusieurs pas avant de mettre à jour l'affichage (pour performance)
                        steps_per_update = calc_params.get('steps_per_update', 5)
                        for _ in range(steps_per_update):
                            if t_current >= t_final_sim:
                                break
                            
                            sol_step = solve_ivp(
                                fun=system_ode,
                                t_span=[t_current, min(t_current + dt, t_final_sim)],
                                y0=y_current,
                                method=calc_params.get('method', 'RK45'),
                                max_step=min(dt, calc_params.get('max_step', 0.1)),
                                dense_output=False,
                                rtol=calc_params.get('rtol', 1e-5),
                                atol=calc_params.get('atol', 1e-7)
                            )
                            
                            if sol_step.success and len(sol_step.t) > 1:
                                # Mettre à jour l'état
                                t_current = sol_step.t[-1]
                                y_current = sol_step.y[:, -1]
                                
                                # Extraire les données
                                (x_rov_val, y_rov_val, vx_rov_val, vy_rov_val,
                                 x_boat_val, _, x_cable, y_cable, T, L_val) = system.unpack_state(y_current)
                                
                                # Ajouter aux données
                                data = st.session_state['simulation_data']
                                data['time'].append(t_current)
                                data['x_rov'].append(x_rov_val)
                                data['y_rov'].append(y_rov_val)
                                data['vx_rov'].append(vx_rov_val)
                                data['vy_rov'].append(vy_rov_val)
                                data['x_boat'].append(x_boat_val)
                                data['L'].append(L_val)
                                data['T0'].append(T[0] if len(T) > 0 else 0.0)
                                data['T_boat'].append(T[-1] if len(T) > 0 else 0.0)
                                data['T_max'].append(np.max(T) if len(T) > 0 else 0.0)
                                data['x_cable_curr'] = x_cable
                                data['y_cable_curr'] = y_cable
                        
                        # Mettre à jour l'état dans session state
                        st.session_state['t_current'] = t_current
                        st.session_state['y_current'] = y_current
                        
                        # Vérifier si la simulation est terminée
                        if t_current >= t_final_sim:
                            st.session_state['simulation_running'] = False
                        else:
                            # Mettre à jour le temps de dernière mise à jour d'affichage
                            st.session_state['last_display_update_time'] = time_module.time()
                            # Marquer qu'un rafraîchissement est nécessaire (st.rerun() sera appelé à la fin)
                            st.session_state['needs_refresh'] = True
                    except Exception as e:
                        st.error(f"Erreur lors de la simulation: {e}")
                        st.session_state['simulation_running'] = False
            elif 'simulation_running' in st.session_state and st.session_state['simulation_running']:
                # Si simulation_running est True mais que les clés nécessaires manquent, arrêter la simulation
                st.session_state['simulation_running'] = False
                if 'system' not in st.session_state:
                    st.warning("État de simulation invalide. La simulation a été arrêtée.")
            
            # Afficher les graphiques avec les données actuelles
            data = st.session_state['simulation_data']
            
            # Obtenir le temps actuel de la simulation (depuis session state ou données)
            t_final = st.session_state.get('sim_t_final', st.session_state.get('t_final', 60.0))
            if 't_current' in st.session_state:
                current_time_display = st.session_state['t_current']
            elif len(data['time']) > 0:
                current_time_display = data['time'][-1]
            else:
                current_time_display = 0.0
            
            # Initialiser les variables pour les indicateurs (valeurs par défaut)
            if len(data['time']) > 0 and data['x_cable_curr'] is not None:
                x_cable_curr = data['x_cable_curr']
                y_cable_curr = data['y_cable_curr']
                x_rov_curr = data['x_rov'][-1]
                y_rov_curr = data['y_rov'][-1]
                x_boat_curr = data['x_boat'][-1]
                L_curr = data['L'][-1]
                T_curr_val = data['T0'][-1]
                T_boat_curr_val = data['T_boat'][-1]
                T_max_curr_val = data['T_max'][-1]
                vx_rov_curr = data['vx_rov'][-1]
            else:
                # Données par défaut si pas encore disponible
                x_cable_curr = np.array([x_rov_init, x_rov_init])
                y_cable_curr = np.array([y_rov_init, 0.0])
                x_rov_curr = x_rov_init
                y_rov_curr = y_rov_init
                x_boat_curr = x_rov_init
                L_curr = L_init
                T_curr_val = 0.0
                T_boat_curr_val = 0.0
                T_max_curr_val = 0.0
                vx_rov_curr = 0.0
            
            # Afficher le temps écoulé et le graphique dans col_sim_3 - toujours visible pendant la simulation
            with col_sim_3:
                # Afficher le temps écoulé - rafraîchi à chaque seconde
                # Le champ s'affiche dès que la simulation est lancée ou en cours
                st.metric(
                    "Temps écoulé 1",
                    f"{current_time_display:.2f} s",
                    delta=f"{t_final - current_time_display:.2f} s restants" if current_time_display < t_final else "Simulation terminée"
                )
                
                if len(data['time']) > 0:
                    # Vue du système dans col_sim_3 - rafraîchi régulièrement pendant la simulation
                    # Recréer le graphique à chaque rerun pour qu'il se rafraîchisse
                    fig_system = create_system_plot(
                        x_rov_curr, y_rov_curr, 
                        x_cable_curr, y_cable_curr, 
                        x_boat_curr, L_curr,
                        title=f"t = {current_time_display:.2f} s / {t_final:.2f} s"
                    )
                    # Utiliser une clé dynamique basée sur le temps pour forcer le rafraîchissement
                    st.plotly_chart(fig_system, width='stretch', key=f"system_view_realtime_{current_time_display:.2f}")

            # Indicateurs de synthèses
            with col_sim_5:
                st.subheader("Synthèse")
                # Recréer les métriques à chaque rerun pour qu'elles se rafraîchissent
                # Ajouter un delta pour forcer le rafraîchissement (comme pour "Temps écoulé 1")
                st.metric(
                    "Temps écoulé 2", 
                    f"{current_time_display:.2f} s",
                    delta=f"{t_final - current_time_display:.2f} s restants" if current_time_display < t_final else "Simulation terminée"
                )
                st.metric("Profondeur ROV", f"{y_rov_curr:.2f} m")
                st.metric("Vitesse horizontale ROV", f"{vx_rov_curr:.2f} m/s")
                st.metric("Tension câble ROV", f"{T_curr_val:.1f} N")
                st.metric("Tension câble bateau", f"{T_boat_curr_val:.1f} N")
                st.metric("Tension max câble", f"{T_max_curr_val:.1f} N")
                st.metric("Longueur câble", f"{L_curr:.2f} m")
                
                # Graphiques temps réel - recréer à chaque rerun pour qu'ils se rafraîchissent
                # Créer les colonnes avant le if pour qu'elles soient disponibles dans les deux cas
                col_graphs1, col_graphs2 = st.columns(2)
                
                if len(data['time']) > 1 and False:
                    with col_graphs1:
                        st.markdown("**Graphiques Temps Réel**")
                        fig_pos = create_position_plot(np.array(data['time']), np.array(data['x_rov']), np.array(data['y_rov']))
                        # Utiliser une clé dynamique basée sur le nombre de points pour forcer le rafraîchissement
                        st.plotly_chart(fig_pos, width='stretch', key=f"position_realtime_{len(data['time'])}")
                    
                    with col_graphs2:
                        st.markdown("**Vitesses**")
                        fig_vel = create_velocity_plot(np.array(data['time']), np.array(data['vx_rov']), np.array(data['vy_rov']))
                        # Utiliser une clé dynamique basée sur le nombre de points pour forcer le rafraîchissement
                        st.plotly_chart(fig_vel, width='stretch', key=f"velocity_realtime_{len(data['time'])}")
                else:
                    # Afficher des graphiques vides au début
                    with col_graphs1:
                        st.info("Les graphiques apparaîtront pendant la simulation...")
                    with col_graphs2:
                        st.info("Les graphiques de vitesse apparaîtront pendant la simulation...")
        
        else:
            # État initial - afficher des graphiques vides ou état initial
            data = st.session_state['simulation_data']
            
            # Afficher l'état initial si disponible
            if len(data['time']) == 0:
                # Vue initiale du système dans col_sim_3
                x_cable_init = np.linspace(x_rov_init, x_rov_init, 11)
                y_cable_init = np.linspace(y_rov_init, 0.0, 11)
                
                fig_system_init = create_system_plot(
                    x_rov_init, y_rov_init, 
                    x_cable_init, y_cable_init, 
                    x_rov_init, L_init,
                    title="État initial - Cliquez sur 'Démarrer simulation' pour commencer"
                )
                with col_sim_3:
                    st.plotly_chart(fig_system_init, width='stretch', key="system_view_init")
                
                # Indicateurs initiaux dans col_sim_5
                with col_sim_5:
                    st.subheader("Synthèse")
                    st.metric("Profondeur ROV", f"{y_rov_init:.2f} m")
                    st.metric("Vitesse horizontale", "0.00 m/s")
                    st.metric("Tension câble", "0.0 N")
                    st.metric("Longueur câble", f"{L_init:.2f} m")
                
                # Graphiques vides
                col_graphs1, col_graphs2 = st.columns(2)
                with col_graphs1:
                    st.info("👈 Configurez les paramètres et cliquez sur 'Démarrer simulation' pour voir les visualisations en temps réel.")
                with col_graphs2:
                    st.info("Les graphiques de vitesse apparaîtront pendant la simulation...")
        
        # Contrôles de simulation (bouton pause/reprendre) avec callbacks
        col_ctrl1, col_ctrl2, col_ctrl3, col_ctrl4, col_ctrl5 = st.columns(5)
        with col_ctrl1:
            run_simulation = st.button("▶ Démarrer simulation", type="primary", 
                                       width='stretch', on_click=callback_start_simulation)
        if 'simulation_running' in st.session_state:
            with col_ctrl2:
                st.button("⏹ Arrêter", width='stretch', on_click=callback_stop_simulation)
            with col_ctrl3:
                if st.session_state['simulation_running']:
                    pause_label = "⏸ Pause" if not st.session_state.get('simulation_paused', False) else "▶ Reprendre"
                    st.button(pause_label, on_click=callback_pause_resume)
                else:
                    st.button("🔄 Relancer", on_click=callback_restart)
            with col_ctrl4:
                st.button("⏹ Arrêter", on_click=callback_stop_simulation)
            with col_ctrl5:
                if st.button("📊 Export CSV"):
                    if len(st.session_state['simulation_data']['time']) > 0:
                        import pandas as pd
                        data = st.session_state['simulation_data']
                        df = pd.DataFrame({
                            'time': data['time'],
                            'x_rov': data['x_rov'],
                            'y_rov': data['y_rov'],
                            'vx_rov': data['vx_rov'],
                            'vy_rov': data['vy_rov'],
                            'x_boat': data['x_boat'],
                            'L': data['L'],
                            'T0': data['T0']
                        })
                        csv = df.to_csv(index=False)
                        st.download_button(
                            label="Télécharger CSV",
                            data=csv,
                            file_name="simulation_rov.csv",
                            mime="text/csv",
                            key="download_csv"
                        )
        
        # Graphiques détaillés en bas (si simulation terminée)
        if 'simulation_running' in st.session_state and not st.session_state['simulation_running'] and len(st.session_state['simulation_data']['time']) > 0:
            st.markdown("---")
            st.subheader("Analyse Détaillée")
            tab1, tab2, tab3 = st.tabs(["Positions", "Vitesses", "Tension"])
            
            data = st.session_state['simulation_data']
            
            with tab1:
                fig_pos_detailed = create_position_plot(np.array(data['time']), np.array(data['x_rov']), np.array(data['y_rov']))
                st.plotly_chart(fig_pos_detailed, width='stretch', key="position_detailed")
            
            with tab2:
                fig_vel_detailed = create_velocity_plot(np.array(data['time']), np.array(data['vx_rov']), np.array(data['vy_rov']))
                st.plotly_chart(fig_vel_detailed, width='stretch', key="velocity_detailed")
            
            with tab3:
                fig_tension = create_tension_plot(np.array(data['time']), np.array(data['T0']))
                st.plotly_chart(fig_tension, width='stretch', key="tension_detailed")
            
            if 'solution' in st.session_state:
                solution = st.session_state['solution']
                system = st.session_state['system']
                
                # Extraire les données
                n_points = len(solution.t)
                time = solution.t
                
                x_rov = np.zeros(n_points)
                y_rov = np.zeros(n_points)
                vx_rov = np.zeros(n_points)
                vy_rov = np.zeros(n_points)
                x_boat = np.zeros(n_points)
                L = np.zeros(n_points)
                T0 = np.zeros(n_points)  # Tension au ROV
                
                for i, t in enumerate(solution.t):
                    y = solution.sol(t)
                    (x_rov[i], y_rov[i], vx_rov[i], vy_rov[i],
                     x_boat[i], _, x_cable, y_cable, T, L[i]) = system.unpack_state(y)
                    T0[i] = T[0] if len(T) > 0 else 0.0
                
                # Sélecteur de temps pour visualisation
                st.markdown("---")
                st.subheader("Visualisation")
                col_time, col_play = st.columns([3, 1])
                with col_time:
                    time_idx = st.slider(
                        "Temps (s)", 
                        0, n_points - 1, n_points - 1,
                        format="%.2f",
                        key="time_slider"
                    )
                    current_time = time[time_idx]
                with col_play:
                    auto_play = st.checkbox("Lecture auto", value=False)
                
                # Obtenir l'état au temps sélectionné
                y_current = solution.sol(current_time)
                (x_rov_curr, y_rov_curr, vx_rov_curr, vy_rov_curr,
                 x_boat_curr, _, x_cable_curr, y_cable_curr, T_curr, L_curr) = system.unpack_state(y_current)
                
                # Colonnes principales : Visualisation 2D + Graphiques
                col_viz, col_graphs = st.columns([2, 1])
                
                with col_viz:
                    st.markdown("**Vue du Système**")
                    fig_system = create_system_plot(
                        x_rov_curr, y_rov_curr, 
                        x_cable_curr, y_cable_curr, 
                        x_boat_curr, L_curr,
                        title=f"t = {current_time:.2f} s"
                    )
                    st.plotly_chart(fig_system, width='stretch', key="system_view")
                    
                    # Indicateurs en temps réel
                    col_ind1, col_ind2, col_ind3, col_ind4 = st.columns(4)
                    with col_ind1:
                        st.metric("Profondeur ROV", f"{y_rov_curr:.2f} m")
                    with col_ind2:
                        st.metric("Vitesse horizontale", f"{vx_rov_curr:.2f} m/s")
                    with col_ind3:
                        st.metric("Tension câble", f"{T_curr[0]:.1f} N")
                    with col_ind4:
                        st.metric("Longueur câble", f"{L_curr:.2f} m")
                
                with col_graphs:
                    st.markdown("**Graphiques Temps Réel**")
                    
                    # Position
                    fig_pos = create_position_plot(time, x_rov, y_rov)
                    st.plotly_chart(fig_pos, width='stretch', key="position_realtime")
                    
                    # Vitesse
                    fig_vel = create_velocity_plot(time, vx_rov, vy_rov)
                    st.plotly_chart(fig_vel, width='stretch', key="velocity_realtime")
                
                # Graphiques détaillés en bas
                st.markdown("---")
                st.subheader("Analyse Détaillée")
                tab1, tab2, tab3 = st.tabs(["Positions", "Vitesses", "Tension"])
                
                with tab1:
                    fig_pos_detailed = create_position_plot(time, x_rov, y_rov)
                    st.plotly_chart(fig_pos_detailed, width='stretch', key="position_detailed")
                
                with tab2:
                    fig_vel_detailed = create_velocity_plot(time, vx_rov, vy_rov)
                    st.plotly_chart(fig_vel_detailed, width='stretch', key="velocity_detailed")
                
                with tab3:
                    fig_tension = create_tension_plot(time, T0)
                    st.plotly_chart(fig_tension, width='stretch', key="tension_detailed")
                
                # Export des données
                st.markdown("---")
                st.subheader("Export")
                if st.button("Exporter CSV"):
                    import pandas as pd
                    df = pd.DataFrame({
                        'time': time,
                        'x_rov': x_rov,
                        'y_rov': y_rov,
                        'vx_rov': vx_rov,
                        'vy_rov': vy_rov,
                        'x_boat': x_boat,
                        'L': L,
                        'T0': T0
                    })
                    csv = df.to_csv(index=False)
                    st.download_button(
                        label="Télécharger CSV",
                        data=csv,
                        file_name="simulation_rov.csv",
                        mime="text/csv"
                    )
        else:
            # Message d'accueil
            st.info("👈 Configurez les paramètres dans l'onglet 'Paramètres', puis utilisez les contrôles ci-dessus pour lancer une simulation.")
            
            st.markdown("""
                       
            **Pour démarrer une simulation :**
            1. Allez dans l'onglet "Paramètres" pour configurer les paramètres physiques
            2. Revenez dans l'onglet "Simulation"
            3. Ajustez les paramètres de simulation et les commandes
            4. Cliquez sur "▶ Démarrer simulation"
            """)
    
    # Rafraîchissement contrôlé : seulement si nécessaire (simulation en cours)
    if st.session_state.get('needs_refresh', False) and st.session_state.get('simulation_running', False):
        st.session_state['needs_refresh'] = False
        st.rerun()

if __name__ == "__main__":
    main()
