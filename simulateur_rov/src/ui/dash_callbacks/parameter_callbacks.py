"""Callbacks pour les paramètres"""
from dash import Input, Output, State, html, callback_context, no_update
import dash_bootstrap_components as dbc
from datetime import datetime
import json
from pathlib import Path
from ..mission_utils import get_missions_directory


def get_current_mission_directory(mission_data):
    """Retourne le répertoire de la mission actuelle"""
    if mission_data and mission_data.get('current'):
        missions_dir = get_missions_directory()
        mission_dir = missions_dir / mission_data['current']
        mission_dir.mkdir(exist_ok=True)
        return mission_dir
    return None


def register_parameter_callbacks(app):
    """Enregistre les callbacks pour les paramètres"""
    
    # Callbacks pour mettre à jour les inputs depuis les stores (chargement de mission)
    
    # Callbacks pour les paramètres d'environnement
    @app.callback(
        [Output('m_rov', 'value'),
         Output('a_rov', 'value'),
         Output('b_rov', 'value'),
         Output('h_rov', 'value'),
         Output('cx_rov', 'value'),
         Output('cy_rov', 'value'),
         Output('d_cable', 'value'),
         Output('rho_cable', 'value'),
         Output('cx_cable', 'value'),
         Output('m_boat', 'value'),
         Output('rho_eau', 'value'),
         Output('g', 'value'),
         Output('mu', 'value')],
        Input('parameters-store', 'data'),
        prevent_initial_call=True
    )
    def update_params_inputs(params_data):
        """Met à jour les inputs de paramètres depuis le store"""
        print(f"DEBUG update_params_inputs appelé avec: {params_data}")
        
        # Ne pas mettre à jour si le store est vide ou None
        if not params_data:
            print("DEBUG: params_data est vide ou None")
            return [no_update] * 13
        
        # Vérifier si le store contient des données valides
        if not isinstance(params_data, dict) or not any(key in params_data for key in ['rov', 'cable', 'boat', 'environment']):
            print(f"DEBUG: params_data ne contient pas les clés attendues. Clés présentes: {list(params_data.keys()) if isinstance(params_data, dict) else 'N/A'}")
            return [no_update] * 13
        
        try:
            # Extraire les valeurs
            rov = params_data.get('rov', {})
            cable = params_data.get('cable', {})
            boat = params_data.get('boat', {})
            env = params_data.get('environment', {})
            
            print(f"DEBUG: Mise à jour des paramètres - ROV: {rov}, Cable: {cable}, Boat: {boat}, Env: {env}")
            
            return [
                float(rov.get('m', 100.0)) if rov.get('m') is not None else 100.0,
                float(rov.get('a', 0.5)) if rov.get('a') is not None else 0.5,
                float(rov.get('b', 1.0)) if rov.get('b') is not None else 1.0,
                float(rov.get('h', 0.5)) if rov.get('h') is not None else 0.5,
                float(rov.get('Cx', 0.8)) if rov.get('Cx') is not None else 0.8,
                float(rov.get('Cy', 1.0)) if rov.get('Cy') is not None else 1.0,
                float(cable.get('d', 0.01)) if cable.get('d') is not None else 0.01,
                float(cable.get('rho_cable', 1500.0)) if cable.get('rho_cable') is not None else 1500.0,
                float(cable.get('Cx_cable', 1.2)) if cable.get('Cx_cable') is not None else 1.2,
                float(boat.get('m', 10000.0)) if boat.get('m') is not None else 10000.0,
                float(env.get('rho_eau', 1025.0)) if env.get('rho_eau') is not None else 1025.0,
                float(env.get('g', 9.81)) if env.get('g') is not None else 9.81,
                float(env.get('mu', 0.001)) if env.get('mu') is not None else 0.001,
            ]
        except Exception as e:
            print(f"Erreur dans update_params_inputs: {e}")
            import traceback
            traceback.print_exc()
            return [no_update] * 13
    
    # Callbacks pour les paramètres de calcul
    @app.callback(
        [Output('method', 'value'),
         Output('rtol', 'value'),
         Output('atol', 'value'),
         Output('max_step', 'value'),
         Output('steps_per_update', 'value'),
         Output('t_final', 'value'),
         Output('dt_max', 'value'),
         Output('N_segments', 'value')],
        Input('calc-params-store', 'data'),
        prevent_initial_call=True
    )
    def update_calc_params_inputs(calc_params_data):
        """Met à jour les inputs de paramètres de calcul depuis le store"""
        # Ne pas mettre à jour si le store est vide ou None
        if not calc_params_data:
            return [no_update] * 8
        
        # Vérifier si le store contient des données valides (au moins une clé)
        if not isinstance(calc_params_data, dict) or len(calc_params_data) == 0:
            return [no_update] * 8
        
        try:
            print(f"DEBUG: Mise à jour des paramètres de calcul: {calc_params_data}")
            return [
                calc_params_data.get('method', 'RK45'),
                float(calc_params_data.get('rtol', 1e-5)),
                float(calc_params_data.get('atol', 1e-7)),
                float(calc_params_data.get('max_step', 0.1)),
                int(calc_params_data.get('steps_per_update', 5)),
                float(calc_params_data.get('t_final', 60.0)),
                float(calc_params_data.get('dt_max', 0.1)),
                int(calc_params_data.get('N_segments', 50)),
            ]
        except Exception as e:
            print(f"Erreur dans update_calc_params_inputs: {e}")
            import traceback
            traceback.print_exc()
            return [no_update] * 8
    
    # Callbacks pour les conditions initiales
    @app.callback(
        [Output('x_rov_init', 'value'),
         Output('y_rov_init', 'value'),
         Output('L_init', 'value'),
         Output('v_courant', 'value')],
        Input('init-params-store', 'data'),
        prevent_initial_call=True
    )
    def update_init_params_inputs(init_params_data):
        """Met à jour les inputs de conditions initiales depuis le store"""
        # Ne pas mettre à jour si le store est vide ou None
        if not init_params_data:
            return [no_update] * 4
        
        # Vérifier si le store contient des données valides (au moins une clé)
        if not isinstance(init_params_data, dict) or len(init_params_data) == 0:
            return [no_update] * 4
        
        try:
            print(f"DEBUG: Mise à jour des conditions initiales: {init_params_data}")
            return [
                float(init_params_data.get('x_rov_init', 0.0)),
                float(init_params_data.get('y_rov_init', 10.0)),
                float(init_params_data.get('L_init', 50.0)),
                float(init_params_data.get('v_courant', 0.0)),
            ]
        except Exception as e:
            print(f"Erreur dans update_init_params_inputs: {e}")
            import traceback
            traceback.print_exc()
            return [no_update] * 4
    
    # Callback pour sauvegarder les paramètres d'environnement
    @app.callback(
        [Output('parameters-store', 'data', allow_duplicate=True),
         Output('save-params-message', 'children')],
        Input('save-params-button', 'n_clicks'),
        [State('m_rov', 'value'),
         State('a_rov', 'value'),
         State('b_rov', 'value'),
         State('h_rov', 'value'),
         State('cx_rov', 'value'),
         State('cy_rov', 'value'),
         State('d_cable', 'value'),
         State('rho_cable', 'value'),
         State('cx_cable', 'value'),
         State('m_boat', 'value'),
         State('rho_eau', 'value'),
         State('g', 'value'),
         State('mu', 'value'),
         State('mission-store', 'data')],
        prevent_initial_call=True
    )
    def save_params(n_clicks, m_rov, a_rov, b_rov, h_rov, cx_rov, cy_rov,
                   d_cable, rho_cable, cx_cable, m_boat, rho_eau, g, mu, mission_data):
        """Sauvegarde les paramètres d'environnement"""
        
        # Créer le dictionnaire des paramètres
        params = {
            'rov': {
                'm': m_rov or 100.0,
                'a': a_rov or 0.5,
                'b': b_rov or 1.0,
                'h': h_rov or 0.5,
                'Cx': cx_rov or 0.8,
                'Cy': cy_rov or 1.0
            },
            'cable': {
                'd': d_cable or 0.01,
                'rho_cable': rho_cable or 1500.0,
                'Cx_cable': cx_cable or 1.2
            },
            'boat': {
                'm': m_boat or 10000.0
            },
            'environment': {
                'rho_eau': rho_eau or 1025.0,
                'g': g or 9.81,
                'mu': mu or 0.001,
            }
        }
        
        # Sauvegarder dans le fichier de la mission si une mission est sélectionnée
        mission_dir = get_current_mission_directory(mission_data)
        message = None
        
        if mission_dir:
            params_file = mission_dir / "Paramètres_environnement.json"
            try:
                # Ajouter la date de sauvegarde
                params_with_metadata = {
                    'date_sauvegarde': datetime.now().isoformat(),
                    'mission': mission_data.get('current'),
                    'paramètres': params
                }
                
                with open(params_file, 'w', encoding='utf-8') as f:
                    json.dump(params_with_metadata, f, indent=4, ensure_ascii=False)
                
                message = dbc.Alert(
                    f"✓ Paramètres d'environnement sauvegardés dans la mission '{mission_data.get('current')}'",
                    color="success"
                )
            except Exception as e:
                message = dbc.Alert(
                    f"✗ Erreur lors de la sauvegarde: {e}",
                    color="danger"
                )
        else:
            message = dbc.Alert(
                "⚠️ Aucune mission sélectionnée. Les paramètres sont sauvegardés en mémoire uniquement.",
                color="warning"
            )
        
        return params, message
    
    # Callback pour sauvegarder les paramètres de calcul
    @app.callback(
        [Output('calc-params-store', 'data', allow_duplicate=True),
         Output('save-calc-params-message', 'children')],
        Input('save-calc-params-button', 'n_clicks'),
        [State('method', 'value'),
         State('rtol', 'value'),
         State('atol', 'value'),
         State('max_step', 'value'),
         State('steps_per_update', 'value'),
         State('t_final', 'value'),
         State('dt_max', 'value'),
         State('N_segments', 'value'),
         State('mission-store', 'data')],
        prevent_initial_call=True
    )
    def save_calc_params(n_clicks, method, rtol, atol, max_step, steps_per_update,
                        t_final, dt_max, N_segments, mission_data):
        """Sauvegarde les paramètres de calcul"""
        
        # Créer le dictionnaire des paramètres de calcul
        calc_params = {
            'method': method or 'RK45',
            'rtol': rtol or 1e-5,
            'atol': atol or 1e-7,
            'max_step': max_step or 0.1,
            'steps_per_update': steps_per_update or 5,
            't_final': t_final or 60.0,
            'dt_max': dt_max or 0.1,
            'N_segments': N_segments or 50
        }
        
        # Sauvegarder dans le fichier de la mission si une mission est sélectionnée
        mission_dir = get_current_mission_directory(mission_data)
        message = None
        
        if mission_dir:
            calc_params_file = mission_dir / "Paramètres_calcul.json"
            try:
                # Ajouter la date de sauvegarde
                calc_params_with_metadata = {
                    'date_sauvegarde': datetime.now().isoformat(),
                    'mission': mission_data.get('current'),
                    'paramètres_calcul': calc_params
                }
                
                with open(calc_params_file, 'w', encoding='utf-8') as f:
                    json.dump(calc_params_with_metadata, f, indent=4, ensure_ascii=False)
                
                message = dbc.Alert(
                    f"✓ Paramètres de calcul sauvegardés dans la mission '{mission_data.get('current')}'",
                    color="success"
                )
            except Exception as e:
                message = dbc.Alert(
                    f"✗ Erreur lors de la sauvegarde: {e}",
                    color="danger"
                )
        else:
            message = dbc.Alert(
                "⚠️ Aucune mission sélectionnée. Les paramètres de calcul sont sauvegardés en mémoire uniquement.",
                color="warning"
            )
        
        return calc_params, message
    
    # Callback pour sauvegarder les conditions initiales
    @app.callback(
        [Output('init-params-store', 'data', allow_duplicate=True),
         Output('save-init-params-message', 'children')],
        Input('save-init-params-button', 'n_clicks'),
        [State('x_rov_init', 'value'),
         State('y_rov_init', 'value'),
         State('L_init', 'value'),
         State('v_courant', 'value'),
         State('mission-store', 'data')],
        prevent_initial_call=True
    )
    def save_init_params(n_clicks, x_rov_init, y_rov_init, L_init, v_courant, mission_data):
        """Sauvegarde les conditions initiales"""
        
        # Créer le dictionnaire des conditions initiales
        init_params = {
            'x_rov_init': x_rov_init or 0.0,
            'y_rov_init': y_rov_init or 10.0,
            'L_init': L_init or 50.0,
            'v_courant': v_courant or 0.0
        }
        
        # Sauvegarder dans le fichier de la mission si une mission est sélectionnée
        mission_dir = get_current_mission_directory(mission_data)
        message = None
        
        if mission_dir:
            init_params_file = mission_dir / "Param_init.json"
            try:
                # Ajouter la date de sauvegarde
                init_params_with_metadata = {
                    'date_sauvegarde': datetime.now().isoformat(),
                    'mission': mission_data.get('current'),
                    'conditions_initiales': init_params
                }
                
                with open(init_params_file, 'w', encoding='utf-8') as f:
                    json.dump(init_params_with_metadata, f, indent=4, ensure_ascii=False)
                
                message = dbc.Alert(
                    f"✓ Conditions initiales sauvegardées dans la mission '{mission_data.get('current')}'",
                    color="success"
                )
            except Exception as e:
                message = dbc.Alert(
                    f"✗ Erreur lors de la sauvegarde: {e}",
                    color="danger"
                )
        else:
            message = dbc.Alert(
                "⚠️ Aucune mission sélectionnée. Les conditions initiales sont sauvegardées en mémoire uniquement.",
                color="warning"
            )
        
        return init_params, message
