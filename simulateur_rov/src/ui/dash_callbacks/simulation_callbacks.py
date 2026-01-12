"""Callbacks pour les contrôles de simulation"""
from dash import Input, Output, State, html, callback_context, no_update
import dash_bootstrap_components as dbc
from copy import deepcopy
import numpy as np
from .simulation_cache import _simulation_cache

# Variable pour stocker l'état précédent des contrôles
_previous_controls_state = None


def register_simulation_callbacks(app):
    """Enregistre les callbacks pour les contrôles de simulation"""
    
    # Callback pour mettre à jour l'affichage des boutons selon l'état de la simulation
    # OPTIMISÉ : Ne modifie pas les intervals pour éviter les boucles
    @app.callback(
        [Output('start-simulation-button', 'disabled'),
         Output('stop-simulation-button', 'disabled'),
         Output('pause-simulation-button', 'disabled'),
         Output('restart-simulation-button', 'disabled'),
         Output('export-csv-button', 'disabled'),
         Output('pause-simulation-button', 'children'),
         Output('simulation-status-message', 'children')],
        Input('simulation-state', 'data'),
        prevent_initial_call=True
    )
    def update_simulation_controls(simulation_state):
        """Met à jour l'état des boutons selon l'état de la simulation"""
        try:
            if not simulation_state:
                return True, True, True, True, True, "⏸ Pause", None
            
            running = simulation_state.get('running', False)
            paused = simulation_state.get('paused', False)
            data_dict = simulation_state.get('data', {})
            has_data = data_dict.get('time', []) if isinstance(data_dict, dict) else []
            
            # Désactiver le bouton Démarrer si la simulation est en cours
            start_disabled = running
            
            # Désactiver les autres boutons si la simulation n'est pas en cours
            stop_disabled = not running
            pause_disabled = not running
            restart_disabled = not running
            export_disabled = len(has_data) == 0
            
            # Texte du bouton Pause
            pause_text = "▶ Reprendre" if paused else "⏸ Pause"
            
            # Message d'état
            status_message = None
            if running:
                if paused:
                    status_message = dbc.Alert("⏸ Simulation en pause", color="warning", className="mb-0")
                else:
                    status_message = dbc.Alert("▶ Simulation en cours...", color="success", className="mb-0")
            
            return (start_disabled, stop_disabled, pause_disabled, 
                    restart_disabled, export_disabled, pause_text, status_message)
        except Exception as e:
            print(f"Erreur dans update_simulation_controls: {e}")
            import traceback
            traceback.print_exc()
            # Retourner des valeurs par défaut en cas d'erreur
            return False, True, True, True, True, "⏸ Pause", None, True, True
    
    # Callback pour démarrer la simulation
    @app.callback(
        [Output('simulation-state', 'data', allow_duplicate=True),
         Output('simulation-interval', 'disabled', allow_duplicate=True),
         Output('metrics-interval', 'disabled', allow_duplicate=True)],
        Input('start-simulation-button', 'n_clicks'),
        [State('simulation-state', 'data'),
         State('parameters-store', 'data'),
         State('calc-params-store', 'data'),
         State('init-params-store', 'data')],
        prevent_initial_call=True
    )
    def start_simulation(n_clicks, simulation_state, parameters, calc_params, init_params):
        """Démarre la simulation"""
        if not n_clicks:
            return no_update, no_update, no_update
        
        try:
            from src.models.system_model import ROVSystem
            from src.utils.initial_conditions import get_initial_state
            
            # Vérifier que les paramètres sont disponibles
            if not parameters:
                print("Erreur: Aucun paramètre d'environnement chargé")
                return no_update, no_update, no_update, no_update
            
            # Créer une copie de l'état
            new_state = deepcopy(simulation_state) if simulation_state else {}
            
            # Initialiser le système ROV
            N_segments = int(calc_params.get('N_segments', 50) if calc_params else 50)
            system = ROVSystem(parameters, N_segments=N_segments)
            
            # Conditions initiales
            x_rov_init = init_params.get('x_rov_init', 0.0) if init_params else 0.0
            y_rov_init = init_params.get('y_rov_init', 10.0) if init_params else 10.0
            L_init = init_params.get('L_init', 50.0) if init_params else 50.0
            
            y0 = get_initial_state(
                system,
                x_rov=x_rov_init,
                y_rov=y_rov_init,
                x_boat=x_rov_init,
                L=L_init
            )
            
            # Stocker le système dans le cache (non sérialisable)
            # Utiliser un ID unique basé sur le temps et un compteur
            import time
            cache_key = f'system_{int(time.time() * 1000000)}'
            _simulation_cache[cache_key] = {
                'system': system,
                'y_current': y0.copy(),
                't_current': 0.0
            }
            
            # Initialiser l'état de la simulation
            new_state['running'] = True
            new_state['paused'] = False
            new_state['current_time'] = 0.0
            new_state['t_current'] = 0.0
            new_state['cache_key'] = cache_key
            new_state['t_final'] = calc_params.get('t_final', 60.0) if calc_params else 60.0
            new_state['dt_max'] = calc_params.get('dt_max', 0.1) if calc_params else 0.1
            
            # Réinitialiser les données
            new_state['data'] = {
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
                'y_cable_curr': None,
            }
            
            # Démarrer le thread de simulation
            from .data_callbacks import register_data_callbacks
            if hasattr(register_data_callbacks, 'start_simulation_thread'):
                register_data_callbacks.start_simulation_thread(
                    cache_key,
                    calc_params,
                    new_state['t_final'],
                    new_state['dt_max']
                )
            
            # Activer simulation-interval et metrics-interval
            print(f"[DEBUG] start_simulation: Activation des intervals pour cache_key={new_state.get('cache_key')}")
            return new_state, False, False  # Activer les deux intervals
        except Exception as e:
            print(f"Erreur dans start_simulation: {e}")
            import traceback
            traceback.print_exc()
            return no_update, no_update, no_update
    
    # Callback pour arrêter la simulation
    @app.callback(
        [Output('simulation-state', 'data', allow_duplicate=True),
         Output('simulation-interval', 'disabled', allow_duplicate=True),
         Output('metrics-interval', 'disabled', allow_duplicate=True)],
        Input('stop-simulation-button', 'n_clicks'),
        State('simulation-state', 'data'),
        prevent_initial_call=True
    )
    def stop_simulation(n_clicks, simulation_state):
        """Arrête la simulation"""
        if not n_clicks:
            return no_update, no_update, no_update
        
        try:
            # Créer une copie de l'état
            new_state = deepcopy(simulation_state) if simulation_state else {}
            
            # Arrêter la simulation
            new_state['running'] = False
            new_state['paused'] = False
            
            # Arrêter le thread de simulation
            cache_key = new_state.get('cache_key')
            if cache_key:
                from .data_callbacks import register_data_callbacks
                if hasattr(register_data_callbacks, 'stop_simulation_thread'):
                    register_data_callbacks.stop_simulation_thread(cache_key)
            
            return new_state, True  # Désactiver simulation-interval
        except Exception as e:
            print(f"Erreur dans stop_simulation: {e}")
            import traceback
            traceback.print_exc()
            return no_update, no_update, no_update
    
    # Callback pour mettre en pause/reprendre la simulation
    @app.callback(
        Output('simulation-state', 'data', allow_duplicate=True),
        Input('pause-simulation-button', 'n_clicks'),
        State('simulation-state', 'data'),
        prevent_initial_call=True
    )
    def pause_resume_simulation(n_clicks, simulation_state):
        """Met en pause ou reprend la simulation"""
        if not n_clicks:
            return no_update
        
        try:
            # Créer une copie de l'état
            new_state = deepcopy(simulation_state) if simulation_state else {}
            
            # Basculer l'état de pause
            if 'paused' in new_state:
                new_state['paused'] = not new_state['paused']
            else:
                new_state['paused'] = True
            
            # Mettre à jour le cache pour informer le thread
            cache_key = new_state.get('cache_key')
            if cache_key and cache_key in _simulation_cache:
                _simulation_cache[cache_key]['paused'] = new_state['paused']
            
            return new_state
        except Exception as e:
            print(f"Erreur dans pause_resume_simulation: {e}")
            import traceback
            traceback.print_exc()
            return no_update
    
    # Callback pour relancer la simulation
    @app.callback(
        [Output('simulation-state', 'data', allow_duplicate=True),
         Output('simulation-interval', 'disabled', allow_duplicate=True),
         Output('metrics-interval', 'disabled', allow_duplicate=True)],
        Input('restart-simulation-button', 'n_clicks'),
        State('simulation-state', 'data'),
        prevent_initial_call=True
    )
    def restart_simulation(n_clicks, simulation_state):
        """Relance la simulation"""
        if not n_clicks:
            return no_update, no_update, no_update
        
        try:
            # Créer une copie de l'état
            new_state = deepcopy(simulation_state) if simulation_state else {}
            
            # Réinitialiser l'état de la simulation
            new_state['running'] = False
            new_state['paused'] = False
            
            # Arrêter le thread de simulation
            cache_key = new_state.get('cache_key')
            if cache_key:
                from .data_callbacks import register_data_callbacks
                if hasattr(register_data_callbacks, 'stop_simulation_thread'):
                    register_data_callbacks.stop_simulation_thread(cache_key)
            
            # Réinitialiser les données
            new_state['data'] = {
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
                'y_cable_curr': None,
            }
            
            return new_state, True, True  # Désactiver les deux intervals (ils seront réactivés par start_simulation)
        except Exception as e:
            print(f"Erreur dans restart_simulation: {e}")
            import traceback
            traceback.print_exc()
            return no_update, no_update, no_update
    
    # Callback pour exporter en CSV (à implémenter plus tard)
    @app.callback(
        Output('export-csv-button', 'children', allow_duplicate=True),
        Input('export-csv-button', 'n_clicks'),
        State('simulation-state', 'data'),
        prevent_initial_call=True
    )
    def export_csv(n_clicks, simulation_state):
        """Exporte les données de simulation en CSV"""
        if not n_clicks:
            return no_update
        
        # TODO: Implémenter l'export CSV
        # Pour l'instant, on retourne juste le texte du bouton
        return "📊 Export CSV"
