"""Callbacks pour les données et visualisations"""
from dash import Input, Output, State, html, callback_context, no_update
import dash_bootstrap_components as dbc
import numpy as np
from copy import deepcopy
from .simulation_cache import _simulation_cache
import threading
import time


def register_data_callbacks(app):
    """Enregistre les callbacks pour les données et visualisations"""
    
    # Définition de la fonction de simulation (utilisée par les deux types de callbacks)
    def run_simulation_step_impl(n_intervals, simulation_state, calc_params):
        """Exécute un pas de simulation (optimisé pour éviter les timeouts)"""
        try:
            # Vérifier que le callback a été déclenché par l'interval
            ctx = callback_context
            if not ctx.triggered:
                return no_update
            
            # Vérifier que la simulation est en cours
            if not simulation_state or not simulation_state.get('running'):
                return no_update
            
            if simulation_state.get('paused', False):
                return no_update
            
            cache_key = simulation_state.get('cache_key')
            if not cache_key or cache_key not in _simulation_cache:
                return no_update
            
            cache = _simulation_cache[cache_key]
            system = cache['system']
            y_current = cache['y_current']
            t_current = cache['t_current']
            
            t_final = simulation_state.get('t_final', 60.0)
            dt_max = simulation_state.get('dt_max', 0.1)
            
            if t_current >= t_final:
                # Simulation terminée
                new_state = deepcopy(simulation_state)
                new_state['running'] = False
                if cache_key in _simulation_cache:
                    del _simulation_cache[cache_key]
                return new_state
            
            from scipy.integrate import solve_ivp
            
            # Fonction de commande (simplifiée pour l'instant)
            def u_func(t):
                return {
                    'Fx_rov': 0.0,
                    'Fy_rov': 0.0,
                    'vx_boat_cmd': 0.0,
                    'dL_dt': 0.0
                }
            
            # Fonction ODE
            def system_ode(t, y):
                u = u_func(t)
                return system.compute_derivatives(t, y, u)
            
            # Paramètres de calcul
            method = calc_params.get('method', 'RK45') if calc_params else 'RK45'
            rtol = calc_params.get('rtol', 1e-5) if calc_params else 1e-5
            atol = calc_params.get('atol', 1e-7) if calc_params else 1e-7
            max_step = calc_params.get('max_step', 0.1) if calc_params else 0.1
            # Avec Background Callbacks, on peut faire plusieurs pas sans timeout
            steps_per_update = calc_params.get('steps_per_update', 5) if calc_params else 5
            
            # Calculer un pas de temps
            dt = min(dt_max, t_final - t_current)
            
            # Faire plusieurs pas avant de mettre à jour l'affichage
            new_data = deepcopy(simulation_state.get('data', {}))
            
            for _ in range(steps_per_update):
                if t_current >= t_final:
                    break
                
                sol_step = solve_ivp(
                    fun=system_ode,
                    t_span=[t_current, min(t_current + dt, t_final)],
                    y0=y_current,
                    method=method,
                    max_step=min(dt, max_step),
                    dense_output=False,
                    rtol=rtol,
                    atol=atol
                )
                
                if sol_step.success and len(sol_step.t) > 1:
                    # Mettre à jour l'état
                    t_current = sol_step.t[-1]
                    y_current = sol_step.y[:, -1]
                    
                    # Extraire les données
                    (x_rov_val, y_rov_val, vx_rov_val, vy_rov_val,
                     x_boat_val, _, x_cable, y_cable, T, L_val) = system.unpack_state(y_current)
                    
                    # Ajouter aux données
                    new_data['time'].append(t_current)
                    new_data['x_rov'].append(float(x_rov_val))
                    new_data['y_rov'].append(float(y_rov_val))
                    new_data['vx_rov'].append(float(vx_rov_val))
                    new_data['vy_rov'].append(float(vy_rov_val))
                    new_data['x_boat'].append(float(x_boat_val))
                    new_data['L'].append(float(L_val))
                    new_data['T0'].append(float(T[0]) if len(T) > 0 else 0.0)
                    new_data['T_boat'].append(float(T[-1]) if len(T) > 0 else 0.0)
                    new_data['T_max'].append(float(np.max(T)) if len(T) > 0 else 0.0)
                    new_data['x_cable_curr'] = x_cable.tolist() if isinstance(x_cable, np.ndarray) else x_cable
                    new_data['y_cable_curr'] = y_cable.tolist() if isinstance(y_cable, np.ndarray) else y_cable
                else:
                    # Si solve_ivp a échoué, arrêter la simulation
                    print(f"Erreur dans solve_ivp: {sol_step.message if hasattr(sol_step, 'message') else 'Échec'}")
                    new_state = deepcopy(simulation_state)
                    new_state['running'] = False
                    if cache_key in _simulation_cache:
                        del _simulation_cache[cache_key]
                    return new_state
            
            # Mettre à jour le cache
            cache['y_current'] = y_current
            cache['t_current'] = t_current
            
            # Mettre à jour l'état
            new_state = deepcopy(simulation_state)
            new_state['data'] = new_data
            new_state['t_current'] = t_current
            new_state['current_time'] = t_current
            
            # Vérifier si la simulation est terminée
            if t_current >= t_final:
                new_state['running'] = False
                if cache_key in _simulation_cache:
                    del _simulation_cache[cache_key]
            
            return new_state
        except Exception as e:
            print(f"Erreur dans run_simulation_step: {e}")
            import traceback
            traceback.print_exc()
            # En cas d'erreur, arrêter la simulation proprement
            try:
                if simulation_state and simulation_state.get('running'):
                    new_state = deepcopy(simulation_state)
                    new_state['running'] = False
                    cache_key = simulation_state.get('cache_key')
                    if cache_key and cache_key in _simulation_cache:
                        del _simulation_cache[cache_key]
                    return new_state
            except:
                pass
            return no_update
    
    # Thread pour exécuter la simulation en arrière-plan
    _simulation_threads = {}  # Dictionnaire pour stocker les threads par cache_key
    _simulation_thread_lock = threading.Lock()
    
    # Verrouillages séparés pour éviter les appels simultanés aux callbacks
    _simulation_callback_lock = threading.Lock()  # Pour update_simulation_from_thread
    _metrics_callback_lock = threading.Lock()  # Pour update_real_time_metrics
    
    # Limite du nombre de points de données à conserver (pour éviter la surcharge mémoire)
    MAX_DATA_POINTS = 5  # Garder seulement les 5 derniers points (très réduit pour éviter les timeouts)
    
    # Compteur pour déclencher les mises à jour explicitement depuis le thread
    _update_counter = {}  # Dictionnaire cache_key -> compteur d'itérations
    _update_trigger = {}  # Dictionnaire cache_key -> Event pour signaler une mise à jour nécessaire
    
    def run_simulation_thread(cache_key, calc_params, t_final, dt_max):
        """Thread qui exécute la simulation en arrière-plan"""
        try:
            if cache_key not in _simulation_cache:
                return
            
            cache = _simulation_cache[cache_key]
            system = cache['system']
            y_current = cache['y_current'].copy()
            t_current = cache['t_current']
            
            from scipy.integrate import solve_ivp
            
            # Fonction de commande (simplifiée pour l'instant)
            def u_func(t):
                return {
                    'Fx_rov': 0.0,
                    'Fy_rov': 0.0,
                    'vx_boat_cmd': 0.0,
                    'dL_dt': 0.0
                }
            
            # Fonction ODE
            def system_ode(t, y):
                u = u_func(t)
                return system.compute_derivatives(t, y, u)
            
            # Paramètres de calcul
            method = calc_params.get('method', 'RK45') if calc_params else 'RK45'
            rtol = calc_params.get('rtol', 1e-5) if calc_params else 1e-5
            atol = calc_params.get('atol', 1e-7) if calc_params else 1e-7
            max_step = calc_params.get('max_step', 0.1) if calc_params else 0.1
            steps_per_update = calc_params.get('steps_per_update', 5) if calc_params else 5
            
            # Nombre d'itérations solve_ivp avant de déclencher une mise à jour
            ITERATIONS_PER_UPDATE = 30  # Mettre à jour toutes les 30 itérations (réduit encore plus la fréquence)
            
            # Initialiser le compteur d'itérations
            iteration_count = 0
            
            # Initialiser le compteur d'itérations
            iteration_count = 0
            
            # Boucle de simulation
            while t_current < t_final:
                # Vérifier si le thread doit s'arrêter (cache_key supprimé ou simulation arrêtée)
                if cache_key not in _simulation_cache:
                    break
                
                # Vérifier si la simulation est en pause (via le cache)
                if cache.get('paused', False):
                    time.sleep(0.1)  # Attendre si en pause
                    continue
                
                # Calculer un pas de temps
                dt = min(dt_max, t_final - t_current)
                
                # Faire plusieurs pas avant de mettre à jour
                for _ in range(steps_per_update):
                    if t_current >= t_final:
                        break
                    
                    if cache_key not in _simulation_cache:
                        break
                    
                    sol_step = solve_ivp(
                        fun=system_ode,
                        t_span=[t_current, min(t_current + dt, t_final)],
                        y0=y_current,
                        method=method,
                        max_step=min(dt, max_step),
                        dense_output=False,
                        rtol=rtol,
                        atol=atol
                    )
                    
                    if sol_step.success and len(sol_step.t) > 1:
                        t_current = sol_step.t[-1]
                        y_current = sol_step.y[:, -1].copy()
                        
                        # Extraire les données
                        (x_rov_val, y_rov_val, vx_rov_val, vy_rov_val,
                         x_boat_val, _, x_cable, y_cable, T, L_val) = system.unpack_state(y_current)
                        
                        # Mettre à jour le cache avec les nouvelles données (thread-safe)
                        with _simulation_thread_lock:
                            if cache_key in _simulation_cache:
                                cache['y_current'] = y_current.copy()
                                cache['t_current'] = t_current
                                
                                # Incrémenter le compteur d'itérations
                                iteration_count += 1
                                
                                # Toutes les N itérations, préparer les données et signaler une mise à jour
                                if iteration_count >= ITERATIONS_PER_UPDATE:
                                    cache['last_data'] = {
                                        'time': t_current,
                                        'x_rov': float(x_rov_val),
                                        'y_rov': float(y_rov_val),
                                        'vx_rov': float(vx_rov_val),
                                        'vy_rov': float(vy_rov_val),
                                        'x_boat': float(x_boat_val),
                                        'L': float(L_val),
                                        'T0': float(T[0]) if len(T) > 0 else 0.0,
                                        'T_boat': float(T[-1]) if len(T) > 0 else 0.0,
                                        'T_max': float(np.max(T)) if len(T) > 0 else 0.0,
                                        # Ne pas stocker les données du câble (trop volumineuses - 50+ points par segment)
                                        # 'x_cable_curr': x_cable.tolist() if isinstance(x_cable, np.ndarray) else x_cable,
                                        # 'y_cable_curr': y_cable.tolist() if isinstance(y_cable, np.ndarray) else y_cable,
                                    }
                                    iteration_count = 0
                                    # Signaler qu'une mise à jour est nécessaire
                                    if cache_key in _update_trigger:
                                        _update_trigger[cache_key].set()
                                        print(f"[DEBUG] Thread: Mise à jour signalée pour {cache_key}, t_current={t_current:.2f}")
                    else:
                        print(f"Erreur dans solve_ivp: {sol_step.message if hasattr(sol_step, 'message') else 'Échec'}")
                        # Arrêter le thread en cas d'erreur
                        with _simulation_thread_lock:
                            if cache_key in _simulation_cache:
                                cache['error'] = True
                        break
                
                # Petite pause pour ne pas surcharger le CPU
                time.sleep(0.01)
            
            # Nettoyer le thread du dictionnaire et les triggers
            with _simulation_thread_lock:
                if cache_key in _simulation_threads:
                    del _simulation_threads[cache_key]
                if cache_key in _update_counter:
                    del _update_counter[cache_key]
                if cache_key in _update_trigger:
                    del _update_trigger[cache_key]
                    
        except Exception as e:
            print(f"Erreur dans run_simulation_thread: {e}")
            import traceback
            traceback.print_exc()
            # Marquer l'erreur dans le cache
            with _simulation_thread_lock:
                if cache_key in _simulation_cache:
                    _simulation_cache[cache_key]['error'] = True
                if cache_key in _simulation_threads:
                    del _simulation_threads[cache_key]
    
    # Fonction pour démarrer le thread de simulation
    def start_simulation_thread(cache_key, calc_params, t_final, dt_max):
        """Démarre le thread de simulation"""
        with _simulation_thread_lock:
            # Arrêter le thread existant s'il y en a un
            if cache_key in _simulation_threads:
                thread = _simulation_threads[cache_key]
                if thread.is_alive():
                    # Le thread s'arrêtera naturellement quand cache_key sera supprimé
                    pass
            
            # Initialiser le trigger et le compteur pour ce thread
            _update_counter[cache_key] = 0
            _update_trigger[cache_key] = threading.Event()
            
            # Créer et démarrer un nouveau thread
            thread = threading.Thread(
                target=run_simulation_thread,
                args=(cache_key, calc_params, t_final, dt_max),
                daemon=True  # Thread daemon pour qu'il s'arrête avec l'application
            )
            thread.start()
            _simulation_threads[cache_key] = thread
            print(f"✓ Thread de simulation démarré pour {cache_key}")
    
    # Fonction pour arrêter le thread de simulation
    def stop_simulation_thread(cache_key):
        """Arrête le thread de simulation"""
        with _simulation_thread_lock:
            if cache_key in _simulation_threads:
                # Le thread s'arrêtera naturellement quand cache_key sera supprimé
                if cache_key in _simulation_cache:
                    del _simulation_cache[cache_key]
                del _simulation_threads[cache_key]
                print(f"✓ Thread de simulation arrêté pour {cache_key}")
    
    # Exporter les fonctions pour utilisation dans simulation_callbacks
    register_data_callbacks.start_simulation_thread = start_simulation_thread
    register_data_callbacks.stop_simulation_thread = stop_simulation_thread
    
    # Callback pour mettre à jour l'état depuis le thread
    # Optimisé pour éviter les timeouts : vérifie le trigger avant de mettre à jour
    @app.callback(
        [Output('simulation-state', 'data', allow_duplicate=True),
         Output('simulation-interval', 'disabled', allow_duplicate=True),
         Output('metrics-interval', 'disabled', allow_duplicate=True)],
        Input('simulation-interval', 'n_intervals'),
        State('simulation-state', 'data'),
        prevent_initial_call=True
    )
    def update_simulation_from_thread(n_intervals, simulation_state):
        """Met à jour l'état de la simulation depuis le thread"""
        # Log pour débogage - TOUJOURS afficher pour voir si le callback est appelé
        print(f"[DEBUG] update_simulation_from_thread appelé (n_intervals={n_intervals}, running={simulation_state.get('running') if simulation_state else None})")
        
        # Protection contre les appels simultanés (non-bloquant)
        if not _simulation_callback_lock.acquire(blocking=False):
            print("[DEBUG] update_simulation_from_thread: Verrou non acquis, retour no_update")
            return no_update, no_update, no_update  # Ignorer si un autre callback est déjà en cours
        
        try:
            if not simulation_state or not simulation_state.get('running'):
                print("[DEBUG] update_simulation_from_thread: Simulation non running, désactivation des intervals")
                # Désactiver les intervals si la simulation n'est plus en cours
                return no_update, True, True
            
            cache_key = simulation_state.get('cache_key')
            print(f"[DEBUG] update_simulation_from_thread: cache_key={cache_key}")
            
            if not cache_key or cache_key not in _simulation_cache:
                print(f"[DEBUG] update_simulation_from_thread: Cache key {cache_key} non trouvé")
                return no_update, True, True  # Désactiver les intervals
            
            cache = _simulation_cache[cache_key]
            
            # Vérifier s'il y a une erreur
            if cache.get('error', False):
                print("[DEBUG] update_simulation_from_thread: Erreur détectée dans le cache")
                new_state = simulation_state.copy()  # Copie superficielle (plus rapide)
                new_state['running'] = False
                stop_simulation_thread(cache_key)
                return new_state
            
            # Vérifier si le thread a signalé une mise à jour
            if cache_key not in _update_trigger:
                # Le trigger a été supprimé, probablement parce que la simulation est terminée
                # Vérifier si la simulation est terminée et arrêter proprement
                t_final = simulation_state.get('t_final', 60.0)
                t_current = cache.get('t_current', 0.0)
                if t_current >= t_final:
                    print(f"[DEBUG] update_simulation_from_thread: Simulation terminée (t_current={t_current:.2f} >= t_final={t_final})")
                    new_state = simulation_state.copy()
                    new_state['running'] = False
                    # Arrêter le thread proprement
                    stop_simulation_thread(cache_key)
                    return new_state, True, True  # Désactiver les intervals
                # Sinon, le trigger n'existe pas encore ou a été supprimé prématurément
                return no_update, no_update, no_update
            
            # Vérifier si une mise à jour a été signalée (non-bloquant)
            trigger_set = _update_trigger[cache_key].is_set()
            print(f"[DEBUG] update_simulation_from_thread: Trigger activé = {trigger_set}, last_data présent = {'last_data' in cache}")
            
            if not trigger_set:
                return no_update, no_update, no_update
            
            # Réinitialiser le trigger
            _update_trigger[cache_key].clear()
            
            # Vérifier si de nouvelles données sont disponibles
            if 'last_data' not in cache:
                print("[DEBUG] update_simulation_from_thread: Pas de last_data dans le cache")
                return no_update, no_update, no_update
            
            # Récupérer les dernières données du thread (thread-safe)
            with _simulation_thread_lock:
                if 'last_data' not in cache:
                    print("[DEBUG] update_simulation_from_thread: Pas de last_data après lock")
                    return no_update, no_update, no_update
                last_data = cache.pop('last_data')
                t_current = cache['t_current']
                print(f"[DEBUG] update_simulation_from_thread: Données récupérées, t_current={t_current:.2f}")
            
            # Mettre à jour l'état de manière ultra-simplifiée
            # Ne stocker que les données essentielles (pas les données du câble pour réduire la taille)
            existing_data = simulation_state.get('data', {})
            
            # Créer les nouvelles listes de manière optimisée (une seule opération par liste)
            # Limiter drastiquement les données pour éviter les timeouts
            new_data = {
                'time': (existing_data.get('time', []) + [last_data['time']])[-MAX_DATA_POINTS:],
                'x_rov': (existing_data.get('x_rov', []) + [last_data['x_rov']])[-MAX_DATA_POINTS:],
                'y_rov': (existing_data.get('y_rov', []) + [last_data['y_rov']])[-MAX_DATA_POINTS:],
                'vx_rov': (existing_data.get('vx_rov', []) + [last_data['vx_rov']])[-MAX_DATA_POINTS:],
                'vy_rov': (existing_data.get('vy_rov', []) + [last_data['vy_rov']])[-MAX_DATA_POINTS:],
                'x_boat': (existing_data.get('x_boat', []) + [last_data['x_boat']])[-MAX_DATA_POINTS:],
                'L': (existing_data.get('L', []) + [last_data['L']])[-MAX_DATA_POINTS:],
                'T0': (existing_data.get('T0', []) + [last_data['T0']])[-MAX_DATA_POINTS:],
                'T_boat': (existing_data.get('T_boat', []) + [last_data['T_boat']])[-MAX_DATA_POINTS:],
                'T_max': (existing_data.get('T_max', []) + [last_data['T_max']])[-MAX_DATA_POINTS:],
                # Ne pas stocker les données du câble (trop volumineuses) - peut être réactivé plus tard si nécessaire
                # 'x_cable_curr': last_data.get('x_cable_curr'),
                # 'y_cable_curr': last_data.get('y_cable_curr'),
            }
            
            # Créer le nouvel état avec seulement les champs modifiés (évite la copie complète)
            new_state = dict(simulation_state)  # Copie superficielle rapide
            new_state.update({
                'data': new_data,
                't_current': t_current,
                'current_time': t_current,
            })
            
            # Vérifier si la simulation est terminée
            t_final = simulation_state.get('t_final', 60.0)
            intervals_disabled = False
            if t_current >= t_final:
                print("[DEBUG] update_simulation_from_thread: Simulation terminée")
                new_state['running'] = False
                stop_simulation_thread(cache_key)
                intervals_disabled = True  # Désactiver les intervals quand la simulation est terminée
            
            print(f"[DEBUG] update_simulation_from_thread: État mis à jour avec succès, t_current={t_current:.2f}")
            return new_state, intervals_disabled, intervals_disabled
        except Exception as e:
            print(f"Erreur dans update_simulation_from_thread: {e}")
            import traceback
            traceback.print_exc()
            return no_update, no_update, no_update
        finally:
            # Libérer le verrou dans tous les cas
            try:
                _simulation_callback_lock.release()
            except:
                pass  # Ignorer si le verrou n'était pas acquis
    
    # Callback pour mettre à jour les métriques temps réel
    # Utilise un intervalle séparé avec une fréquence très faible pour éviter les timeouts
    @app.callback(
        [Output('time-display', 'children'),
         Output('rov-position', 'children'),
         Output('cable-length', 'children'),
         Output('tension-max', 'children')],
        Input('metrics-interval', 'n_intervals'),
        State('simulation-state', 'data'),
        prevent_initial_call=True
    )
    def update_real_time_metrics(n_intervals, simulation_state):
        """Met à jour les métriques temps réel quand les données changent"""
        try:
            # Si pas de simulation en cours, afficher des valeurs par défaut
            if not simulation_state or not simulation_state.get('running'):
                return (
                    dbc.Alert("Temps: 0.00 s", color="info", className="mb-2 p-2"),
                    dbc.Alert("ROV: (0.0, 0.0) m", color="info", className="mb-2 p-2"),
                    dbc.Alert("Câble: 0.0 m", color="info", className="mb-2 p-2"),
                    dbc.Alert("Tension max: 0.0 N", color="info", className="mb-2 p-2"),
                )
            
            data = simulation_state.get('data', {})
            t_current = simulation_state.get('t_current', 0.0)
            
            # Extraire les dernières valeurs
            time_list = data.get('time', [])
            x_rov_list = data.get('x_rov', [])
            y_rov_list = data.get('y_rov', [])
            L_list = data.get('L', [])
            T_max_list = data.get('T_max', [])
            
            # Valeurs actuelles
            current_time = time_list[-1] if time_list else t_current
            x_rov = x_rov_list[-1] if x_rov_list else 0.0
            y_rov = y_rov_list[-1] if y_rov_list else 0.0
            L = L_list[-1] if L_list else 0.0
            T_max = T_max_list[-1] if T_max_list else 0.0
            
            print(f"[DEBUG] update_real_time_metrics: t={current_time:.2f}, ROV=({x_rov:.2f}, {y_rov:.2f}), L={L:.2f}, T_max={T_max:.1f}")
            
            return (
                dbc.Alert(f"Temps: {current_time:.2f} s", color="primary", className="mb-2 p-2"),
                dbc.Alert(f"ROV: ({x_rov:.2f}, {y_rov:.2f}) m", color="primary", className="mb-2 p-2"),
                dbc.Alert(f"Câble: {L:.2f} m", color="primary", className="mb-2 p-2"),
                dbc.Alert(f"Tension max: {T_max:.1f} N", color="primary", className="mb-2 p-2"),
            )
        except Exception as e:
            print(f"Erreur dans update_real_time_metrics: {e}")
            import traceback
            traceback.print_exc()
            return (
                dbc.Alert("Erreur", color="danger", className="mb-2 p-2"),
                dbc.Alert("Erreur", color="danger", className="mb-2 p-2"),
                dbc.Alert("Erreur", color="danger", className="mb-2 p-2"),
                dbc.Alert("Erreur", color="danger", className="mb-2 p-2"),
            )
    
    # Callback pour mettre à jour le graphique système
    # Réactivé avec un intervalle séparé pour éviter la surcharge
    @app.callback(
        Output('system-plot', 'figure', allow_duplicate=True),
        Input('metrics-interval', 'n_intervals'),
        State('simulation-state', 'data'),
        prevent_initial_call=True
    )
    def update_system_plot(n_intervals, simulation_state):
        """Met à jour le graphique système"""
        try:
            # Si pas de simulation en cours ou pas de données, créer un graphique vide
            if not simulation_state or not simulation_state.get('running') or not simulation_state.get('data'):
                import plotly.graph_objects as go
                fig = go.Figure()
                fig.update_layout(
                    title="Vue du système",
                    xaxis_title="X (m)",
                    yaxis_title="Y (m)",
                    template="plotly_white",
                    height=500,
                )
                return fig
            
            from src.visualization.plotter import create_system_plot
            
            data = simulation_state.get('data', {})
            
            # Extraire les dernières valeurs
            x_rov_list = data.get('x_rov', [])
            y_rov_list = data.get('y_rov', [])
            x_boat_list = data.get('x_boat', [])
            L_list = data.get('L', [])
            # Note: x_cable_curr et y_cable_curr ne sont plus stockés pour réduire la charge réseau
            # On utilise des arrays vides pour le câble (peut être amélioré plus tard si nécessaire)
            x_cable = None
            y_cable = None
            
            if not x_rov_list or not y_rov_list:
                # Pas encore de données
                import plotly.graph_objects as go
                fig = go.Figure()
                fig.update_layout(
                    title="Vue du système",
                    xaxis_title="X (m)",
                    yaxis_title="Y (m)",
                    template="plotly_white",
                    height=500,
                )
                return fig
            
            # Utiliser les dernières valeurs
            x_rov_curr = x_rov_list[-1]
            y_rov_curr = y_rov_list[-1]
            x_boat_curr = x_boat_list[-1] if x_boat_list else 0.0
            L_curr = L_list[-1] if L_list else 0.0
            
            # Si pas de données de câble, créer des arrays vides
            # Le graphique affichera quand même le ROV et le bateau avec une ligne simple pour le câble
            if x_cable is None or y_cable is None:
                x_cable = np.array([])
                y_cable = np.array([])
            
            # Créer le graphique avec la fonction existante
            fig = create_system_plot(
                x_rov_curr, y_rov_curr,
                x_cable, y_cable,
                x_boat_curr, L_curr,
                title=f"Vue du système - t = {simulation_state.get('t_current', 0.0):.2f} s"
            )
            
            return fig
        except Exception as e:
            print(f"Erreur dans update_system_plot: {e}")
            import traceback
            traceback.print_exc()
            import plotly.graph_objects as go
            fig = go.Figure()
            fig.update_layout(title="Erreur de chargement", template="plotly_white")
            return fig
