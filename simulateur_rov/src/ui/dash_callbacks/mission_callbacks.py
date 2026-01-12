"""Callbacks pour la gestion des missions"""
from dash import Input, Output, State, callback_context, no_update, html
import dash_bootstrap_components as dbc
from ..mission_utils import create_mission, load_environment_params, load_calc_params, load_init_params, get_missions_directory
from pathlib import Path
import copy


def register_mission_callbacks(app):
    """Enregistre les callbacks pour la gestion des missions"""
    
    # Callback pour afficher la mission actuelle
    @app.callback(
        Output('mission-display', 'children'),
        Input('mission-store', 'data'),
        prevent_initial_call=True
    )
    def update_mission_display(mission_data):
        """Met à jour l'affichage de la mission actuelle"""
        current_mission = mission_data.get('current') if mission_data else None
        
        if current_mission:
            return dbc.Alert([
                html.H5("🎯 Mission:", className="mb-0"),
                html.Strong(current_mission)
            ], color="info", className="mb-3")
        else:
            return dbc.Alert([
                html.Strong("ℹ️ Aucune mission sélectionnée")
            ], color="secondary", className="mb-3")
    
    # Callback pour ouvrir/fermer le collapse des missions
    @app.callback(
        Output('mission-collapse', 'is_open'),
        Input('toggle-mission-collapse', 'n_clicks'),
        State('mission-collapse', 'is_open'),
        prevent_initial_call=True
    )
    def toggle_mission_collapse(n_clicks, is_open):
        """Ouvre/ferme le collapse de gestion des missions"""
        if n_clicks:
            return not is_open
        return is_open
    
    # Callback pour sélectionner une mission
    @app.callback(
        [Output('mission-store', 'data', allow_duplicate=True),
         Output('parameters-store', 'data', allow_duplicate=True),
         Output('calc-params-store', 'data', allow_duplicate=True),
         Output('init-params-store', 'data', allow_duplicate=True),
         Output('mission-messages', 'children', allow_duplicate=True)],
        Input('select-mission-dropdown', 'value'),
        State('mission-store', 'data'),
        prevent_initial_call=True
    )
    def select_mission(selected_mission, mission_data):
        """Sélectionne une mission et charge ses paramètres"""
        try:
            ctx = callback_context
            if not ctx.triggered:
                return no_update, no_update, no_update, no_update, no_update
            
            if not selected_mission or selected_mission == "":
                return mission_data if mission_data else {'current': None}, no_update, no_update, no_update, no_update
            
            # Mettre à jour la mission actuelle (créer une copie pour éviter les problèmes de référence)
            new_mission_data = copy.deepcopy(mission_data) if mission_data else {}
            new_mission_data['current'] = selected_mission
            
            # Charger les paramètres de la mission
            missions_dir = get_missions_directory()
            mission_dir = missions_dir / selected_mission
            
            messages = []
            loaded_params = None
            loaded_calc_params = None
            loaded_init_params = None
            
            if mission_dir.exists():
                # Charger les paramètres d'environnement
                try:
                    env_params = load_environment_params(mission_dir)
                    if env_params:
                        # Pour Dash, nous devons enlever la fonction lambda qui ne peut pas être sérialisée
                        # Extraire v_courant depuis current_profile si c'est une fonction
                        env_dict = env_params.get('environment', {})
                        if 'current_profile' in env_dict and callable(env_dict['current_profile']):
                            try:
                                v_courant_value = env_dict['current_profile'](0.0)
                            except:
                                v_courant_value = 0.0
                            # Recréer le dictionnaire sans la fonction lambda
                            loaded_params = {
                                'rov': env_params.get('rov', {}),
                                'cable': env_params.get('cable', {}),
                                'boat': env_params.get('boat', {}),
                                'environment': {
                                    'rho_eau': env_dict.get('rho_eau', 1025.0),
                                    'g': env_dict.get('g', 9.81),
                                    'mu': env_dict.get('mu', 0.001),
                                    'v_courant': v_courant_value
                                }
                            }
                        else:
                            loaded_params = env_params
                        
                        print(f"DEBUG: Paramètres d'environnement chargés (pour Dash): {loaded_params}")
                        messages.append(dbc.Alert(
                            f"✓ Paramètres d'environnement chargés depuis la mission '{selected_mission}'",
                            color="success",
                            className="mt-2"
                        ))
                except Exception as e:
                    print(f"Erreur lors du chargement des paramètres d'environnement: {e}")
                    import traceback
                    traceback.print_exc()
                
                # Charger les paramètres de calcul
                try:
                    calc_params = load_calc_params(mission_dir)
                    if calc_params:
                        loaded_calc_params = calc_params
                        print(f"DEBUG: Paramètres de calcul chargés: {calc_params}")
                        messages.append(dbc.Alert(
                            f"✓ Paramètres de calcul chargés depuis la mission '{selected_mission}'",
                            color="success",
                            className="mt-2"
                        ))
                except Exception as e:
                    print(f"Erreur lors du chargement des paramètres de calcul: {e}")
                    import traceback
                    traceback.print_exc()
                
                # Charger les conditions initiales
                try:
                    init_params = load_init_params(mission_dir)
                    if init_params:
                        loaded_init_params = init_params
                        print(f"DEBUG: Conditions initiales chargées: {init_params}")
                        messages.append(dbc.Alert(
                            f"✓ Conditions initiales chargées depuis la mission '{selected_mission}'",
                            color="success",
                            className="mt-2"
                        ))
                except Exception as e:
                    print(f"Erreur lors du chargement des conditions initiales: {e}")
                    import traceback
                    traceback.print_exc()
            
            message_output = html.Div(messages) if messages else html.Div()
            
            return (new_mission_data,
                    loaded_params if loaded_params else no_update,
                    loaded_calc_params if loaded_calc_params else no_update,
                    loaded_init_params if loaded_init_params else no_update,
                    message_output)
        except Exception as e:
            print(f"Erreur dans select_mission: {e}")
            import traceback
            traceback.print_exc()
            return no_update, no_update, no_update, no_update, html.Div([
                dbc.Alert(f"Erreur lors du chargement de la mission: {e}", color="danger")
            ])
    
    # Callback pour créer une nouvelle mission
    @app.callback(
        [Output('mission-store', 'data', allow_duplicate=True),
         Output('mission-messages', 'children', allow_duplicate=True),
         Output('new-mission-name', 'value')],
        Input('create-mission-button', 'n_clicks'),
        State('new-mission-name', 'value'),
        State('mission-store', 'data'),
        prevent_initial_call=True
    )
    def create_mission_callback(n_clicks, new_mission_name, mission_data):
        """Crée une nouvelle mission"""
        if not new_mission_name or not new_mission_name.strip():
            return no_update, dbc.Alert("⚠️ Veuillez entrer un nom de mission", color="warning"), no_update
        
        # Nettoyer le nom de la mission (enlever les caractères invalides)
        clean_name = "".join(c for c in new_mission_name.strip() if c.isalnum() or c in (' ', '-', '_')).strip()
        
        if not clean_name:
            return no_update, dbc.Alert("⚠️ Nom de mission invalide", color="warning"), no_update
        
        try:
            # Créer la mission
            create_mission(clean_name)
            
            # Mettre à jour la mission actuelle
            mission_data = mission_data or {}
            mission_data['current'] = clean_name
            
            message = dbc.Alert(
                f"✓ Mission '{clean_name}' créée avec succès!",
                color="success",
                className="mt-2"
            )
            
            # Réinitialiser le champ de saisie
            return mission_data, message, ""
        
        except Exception as e:
            error_message = dbc.Alert(
                f"✗ Erreur lors de la création de la mission: {e}",
                color="danger",
                className="mt-2"
            )
            return no_update, error_message, no_update
