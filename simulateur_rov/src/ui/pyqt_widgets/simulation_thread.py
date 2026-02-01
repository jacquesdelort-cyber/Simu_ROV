"""
Thread pour exécuter la simulation en arrière-plan
"""
from PyQt6.QtCore import QThread, pyqtSignal
import numpy as np
import time
import sys
import os
from src.utils.logger import trace_print

# Ajouter le répertoire parent au path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))


class SimulationThread(QThread):
    """Thread pour exécuter la simulation sans bloquer l'interface"""
    
    # Signaux pour communication avec l'interface
    simulation_updated = pyqtSignal(dict)  # Émet les données mises à jour
    simulation_finished = pyqtSignal()  # Émis quand la simulation est terminée
    error_occurred = pyqtSignal(str)  # Émis en cas d'erreur
    
    def __init__(self, parameters, calc_params, init_params, simulation_state,
                 fx_rov=0.0, fy_rov=0.0, vx_boat_cmd=0.0, dl_dt=0.0,
                 sc_fx_rov: str = "", sc_fy_rov: str = "",
                 sc_v_bateau: str = "", sc_v_moulinet: str = ""):
        super().__init__()
        self.parameters = parameters
        self.calc_params = calc_params
        self.init_params = init_params
        self.simulation_state = simulation_state
        
        # Variables de commande
        self.fx_rov = fx_rov
        self.fy_rov = fy_rov
        self.vx_boat_cmd = vx_boat_cmd
        self.dl_dt = dl_dt
        self.sc_fx_rov = sc_fx_rov
        self.sc_fy_rov = sc_fy_rov
        self.sc_v_bateau = sc_v_bateau
        self.sc_v_moulinet = sc_v_moulinet
        
        self._stop_requested = False
        self._paused = False
    
    def stop(self):
        """Demande l'arrêt de la simulation"""
        self._stop_requested = True
    
    def pause(self):
        """Met en pause la simulation"""
        self._paused = True
    
    def resume(self):
        """Reprend la simulation"""
        self._paused = False
    
    def run(self):
        """Exécute la simulation"""
        try:
            from src.models.system_model import ROVSystem
            from src.utils.initial_conditions import get_initial_state
            from src.utils.scenario_utils import commande_scenario
            from src.utils import scenario_utils
            
            # Réutiliser l'état initial calculé lors de l'initialisation si disponible
            cached_system = None
            cached_y0 = None
            if isinstance(self.simulation_state, dict):
                cached_system = self.simulation_state.get('system')
                cached_y0 = self.simulation_state.get('y_current')
            if cached_system is not None and cached_y0 is not None:
                system = cached_system
                y0 = np.array(cached_y0, copy=True)
            else:
                # Créer le système ROV
                N_segments = int(self.calc_params.get('N_segments', 50))
                system = ROVSystem(self.parameters, N_segments=N_segments)
                
                # Profil de courant (chaîne) depuis les conditions initiales
                v_courant = self.init_params.get('v_courant', "0.0")
                system.environment.v_courant_raw = v_courant
                
                # Conditions initiales
                x_rov_init = self.init_params.get('x_rov_init', 0.0)
                y_rov_init = self.init_params.get('y_rov_init', -10.0)  # Profondeur négative
                x_boat_init = self.init_params.get('x_boat_init', 0.0)
                L_init = self.init_params.get('L_init', 50.0)
                
                y0 = get_initial_state(
                    system,
                    x_rov=x_rov_init,
                    y_rov=y_rov_init,
                    x_boat=x_boat_init,
                    L=L_init,
                    use_current_geometry=bool(self.init_params.get('use_current_geometry', True))
                )
            
            # Paramètres de calcul
            t_final = self.calc_params.get('t_final', 60.0)
            dt_max = self.calc_params.get('dt_max', 0.1)
            steps_per_update = int(self.calc_params.get('steps_per_update', 5))
            
            # Fonction de commande (lit les valeurs depuis simulation_state à chaque appel)
            # Cela permet de mettre à jour les commandes pendant la simulation
            def u_func(t):
                # Lire les valeurs depuis simulation_state si disponible, sinon utiliser les valeurs initiales
                # simulation_state est un dictionnaire
                if isinstance(self.simulation_state, dict):
                    fx_rov = self.simulation_state.get('fx_rov', self.fx_rov)
                    fy_rov = self.simulation_state.get('fy_rov', self.fy_rov)
                    vx_boat_cmd = self.simulation_state.get('vx_boat_cmd', self.vx_boat_cmd)
                    dl_dt = self.simulation_state.get('dl_dt', self.dl_dt)
                else:
                    # Fallback sur les valeurs initiales si simulation_state n'est pas un dict
                    fx_rov = self.fx_rov
                    fy_rov = self.fy_rov
                    vx_boat_cmd = self.vx_boat_cmd
                    dl_dt = self.dl_dt
                
                return {
                    'Fx_rov': fx_rov,
                    'Fy_rov': fy_rov,
                    'vx_boat_cmd': vx_boat_cmd,
                    'dL_dt': dl_dt
                }
            
            # Simulation avec pas adaptatifs
            t_current = 0.0
            y_current = y0.copy()
            dt = min(dt_max, t_final / 100.0)  # Pas initial
            dt_min = float(self.calc_params.get('dt_min', 1e-4))
            
            # Données accumulées
            data = {
                'time': [],
                'x_rov': [],
                'y_rov': [],
                'vx_rov': [],
                'vy_rov': [],
                'x_boat': [],
                'vx_boat': [],
                'L': [],
                'T0': [],
                'T_rov': [],
                'T_boat': [],
                'T_max': [],
                'Fx_drag': [],
                'Fy_drag': [],
                'F_apparent_weight': [],
                'Fx_total': [],
                'Fy_total': [],
                'Fx_traction_boat': [],
                'Fy_traction_boat': [],
                'F_prop_boat': [],
                'Fx_total_boat': [],
                'Fy_total_boat': [],
                'dl_dt_cmd': [],
                'x_cable_curr': None,
                'y_cable_curr': None,
            }
            
            step_count = 0
            
            while t_current < t_final and not self._stop_requested:
                # Attendre si en pause
                while self._paused and not self._stop_requested:
                    time.sleep(0.1)
                    # Vérifier l'état de pause depuis le state partagé
                    if hasattr(self.simulation_state, 'get'):
                        self._paused = self.simulation_state.get('paused', False)
                
                if self._stop_requested:
                    break
                
                fx_rov_cmd = None
                fy_rov_cmd = None
                vx_boat_cmd = None
                dl_dt_cmd = None
                dl_dt_mode = "scen"
                if isinstance(self.simulation_state, dict):
                    dl_dt_mode = self.simulation_state.get('dl_dt_mode', 'scen')

                if self.sc_fx_rov or self.sc_fy_rov or self.sc_v_bateau or self.sc_v_moulinet or dl_dt_mode == "auto":
                    try:
                        (_, y_rov_current, _, _, _, _, _, _, T_current, L_current) = system.unpack_state(y_current)
                        if self.sc_fx_rov:
                            fx_rov_cmd = commande_scenario(
                                "Fx_rov",
                                self.sc_fx_rov,
                                t_current,
                                L_current,
                                y_rov_current,
                            )
                        if self.sc_fy_rov:
                            fy_rov_cmd = commande_scenario(
                                "Fy_rov",
                                self.sc_fy_rov,
                                t_current,
                                L_current,
                                y_rov_current,
                            )
                        if self.sc_v_bateau:
                            vx_boat_cmd = commande_scenario(
                                "Vx_bateau",
                                self.sc_v_bateau,
                                t_current,
                                L_current,
                                y_rov_current,
                            )
                        if dl_dt_mode == "auto":
                            auto_index = 1
                            if isinstance(self.calc_params, dict):
                                try:
                                    auto_index = int(self.calc_params.get("auto_L", 1))
                                except Exception:
                                    auto_index = 1
                            if auto_index < 1:
                                auto_index = 1
                            auto_func_name = f"auto_L_{auto_index}"
                            auto_func = getattr(scenario_utils, auto_func_name, None)
                            if auto_func is None:
                                trace_print(
                                    8,
                                    f"[AUTO dL/dt] Fonction {auto_func_name} introuvable, fallback auto_L_1"
                                )
                                auto_func = scenario_utils.auto_L_1
                            if t_current == 0 and hasattr(auto_func, "_history"):
                                auto_func._history = []
                            trupt = None
                            try:
                                trupt = float(self.parameters.get("cable", {}).get("tension_rupture", 50.0))
                            except Exception:
                                trupt = 50.0
                            tcible = trupt / 2.0
                            t_boat = None
                            if T_current is not None and len(T_current) > 0:
                                t_boat = float(T_current[0])
                            dl_dt_cmd = auto_func(
                                t_current,
                                y_rov_current,
                                L_current,
                                float(self.simulation_state.get('fx_rov', self.fx_rov)) if isinstance(self.simulation_state, dict) else self.fx_rov,
                                float(self.simulation_state.get('fy_rov', self.fy_rov)) if isinstance(self.simulation_state, dict) else self.fy_rov,
                                t_boat,
                                Trupt=trupt,
                                Tcible=tcible,
                            )
                            trace_print(
                                1,
                                f"[AUTO dL/dt] t={t_current:.2f} L={L_current:.2f} "
                                f"T_boat={t_boat if t_boat is not None else 'None'} "
                                f"Tcible={tcible:.3f} "
                                f"cmd={dl_dt_cmd:.4f}"
                            )
                        elif self.sc_v_moulinet:
                            dl_dt_cmd = commande_scenario(
                                "dL_dt",
                                self.sc_v_moulinet,
                                t_current,
                                L_current,
                                y_rov_current,
                            )
                    except Exception as e:
                        trace_print(8, f"Erreur commande_scenario: {e}")
                        fx_rov_cmd = None
                        fy_rov_cmd = None
                        vx_boat_cmd = None
                        dl_dt_cmd = None

                if isinstance(self.simulation_state, dict):
                    if fx_rov_cmd is not None:
                        self.simulation_state['fx_rov'] = float(fx_rov_cmd)
                        self.simulation_state['fx_rov_source'] = "scenario"
                    if fy_rov_cmd is not None:
                        self.simulation_state['fy_rov'] = float(fy_rov_cmd)
                        self.simulation_state['fy_rov_source'] = "scenario"
                    if vx_boat_cmd is not None:
                        self.simulation_state['vx_boat_cmd'] = float(vx_boat_cmd)
                        self.simulation_state['vx_boat_cmd_source'] = "scenario"
                    if dl_dt_cmd is not None:
                        self.simulation_state['dl_dt'] = float(dl_dt_cmd)
                        self.simulation_state['dl_dt_source'] = "auto" if dl_dt_mode == "auto" else "scenario"

                # Calculer le pas suivant
                t_next = min(t_current + dt, t_final)
                
                # Intégrer un pas (avec réduction adaptative si nécessaire)
                try:
                    # Vérifications optimisées : seulement tous les N pas et seulement les valeurs critiques
                    # Vérifier seulement les premières valeurs (ROV) au lieu de tout le vecteur
                    if step_count % 10 == 0:  # Vérifier seulement tous les 10 pas
                        if (not np.isfinite(y_current[0]) or not np.isfinite(y_current[1]) or 
                            not np.isfinite(y_current[2]) or not np.isfinite(y_current[3])):
                            self.error_occurred.emit(f"État invalide (NaN/Inf) à t={t_current:.2f} s")
                            break
                    
                    solution = None
                    success = False
                    retry_count = 0
                    while retry_count < 5 and not success:
                        # Utiliser la méthode d'intégration du système
                        solution = system.integrate(
                            [t_current, t_next],
                            y_current,
                            u_func,
                            dt_max=dt_max
                        )
                        
                        if solution.success:
                            success = True
                            break
                        
                        # Réduire le pas si l'intégration échoue
                        dt = max(dt / 2.0, dt_min)
                        t_next = min(t_current + dt, t_final)
                        retry_count += 1
                    
                    if not success:
                        msg = solution.message if solution is not None and hasattr(solution, 'message') else "Échec"
                        self.error_occurred.emit(
                            f"Échec de l'intégration à t={t_current:.2f} s (dt={dt:.6f}): {msg}"
                        )
                        break
                    
                    # Mettre à jour l'état
                    y_current = solution.sol(t_next)
                    
                    # Vérification optimisée : seulement tous les N pas et seulement les valeurs critiques
                    if step_count % 10 == 0:  # Vérifier seulement tous les 10 pas
                        if (not np.isfinite(y_current[0]) or not np.isfinite(y_current[1]) or 
                            not np.isfinite(y_current[2]) or not np.isfinite(y_current[3])):
                            self.error_occurred.emit(f"État invalide (NaN/Inf) après intégration à t={t_next:.2f} s")
                            break
                    
                    t_current = t_next
                    
                    # Décoder l'état
                    (x_rov, y_rov, vx_rov, vy_rov,
                     x_boat, vx_boat, x_cable, y_cable, T, L) = system.unpack_state(y_current)
                    
                    # Pour l'affichage, privilégier la géométrie d'équilibre calculée par le solveur
                    x_cable_display = x_cable
                    y_cable_display = y_cable
                    if getattr(system, 'x_cable_prev', None) is not None and getattr(system, 'y_cable_prev', None) is not None:
                        if len(system.x_cable_prev) == len(x_cable) and len(system.y_cable_prev) == len(y_cable):
                            x_cable_display = system.x_cable_prev
                            y_cable_display = system.y_cable_prev
                    
                    # Garantir l'ordre bateau -> ROV pour les directions/tractions affichées
                    if x_cable_display is not None and y_cable_display is not None and len(x_cable_display) > 1:
                        dist_first_to_boat = np.sqrt((x_cable_display[0] - x_boat)**2 + (y_cable_display[0] - 0.0)**2)
                        dist_first_to_rov = np.sqrt((x_cable_display[0] - x_rov)**2 + (y_cable_display[0] - y_rov)**2)
                        if dist_first_to_rov < dist_first_to_boat:
                            x_cable_display = np.flip(x_cable_display)
                            y_cable_display = np.flip(y_cable_display)
                    
                    # Stocker les données
                    data['time'].append(t_current)
                    data['x_rov'].append(float(x_rov))
                    data['y_rov'].append(float(y_rov))
                    data['vx_rov'].append(float(vx_rov))
                    data['vy_rov'].append(float(vy_rov))
                    data['x_boat'].append(float(x_boat))
                    data['vx_boat'].append(float(vx_boat))
                    data['L'].append(float(L))
                    if isinstance(self.simulation_state, dict):
                        data['dl_dt_cmd'].append(float(self.simulation_state.get('dl_dt', self.dl_dt)))
                    
                    # Tensions
                    if T is not None and len(T) > 0:
                        T_array = np.asarray(T)
                        # CORRECTION: Après correction de _compute_catenary_tensions :
                        # T[0] = tension au bateau, T[-1] = tension au ROV
                        T_rov = float(T_array[-1]) if len(T_array) > 0 else 0.0  # Tension au ROV = T[-1]
                        T_boat = float(T_array[0]) if len(T_array) > 0 else 0.0  # Tension au bateau = T[0]
                        T_max = float(np.max(T_array)) if len(T_array) > 0 else 0.0
                        data['T0'].append(T_rov)  # Garder T0 pour compatibilité
                        data['T_rov'].append(T_rov)
                        data['T_max'].append(T_max)
                        data['T_boat'].append(T_boat)
                    else:
                        data['T0'].append(0.0)
                        data['T_rov'].append(0.0)
                        data['T_max'].append(0.0)
                        data['T_boat'].append(0.0)
                    
                    # Calculer les forces sur le ROV
                    # CORRECTION: Après correction de _compute_catenary_tensions :
                    # Les positions sont ordonnées : index 0 = bateau, index -1 = ROV
                    # Calculer le vecteur unitaire Urov au niveau du ROV
                    angle_rov_deg = 0.0
                    Urov_x = 0.0
                    Urov_y = 1.0  # Par défaut
                    
                    if x_cable_display is not None and y_cable_display is not None and len(x_cable_display) > 1:
                        # Urov = direction du point précédent vers le ROV (index -1)
                        dx_rov = x_cable_display[-1] - x_cable_display[-2]
                        dy_rov = y_cable_display[-1] - y_cable_display[-2]
                        ds_rov = np.sqrt(dx_rov**2 + dy_rov**2)
                        
                        if ds_rov > 1e-6:
                            Urov_x = dx_rov / ds_rov
                            Urov_y = dy_rov / ds_rov
                            # Angle avec la verticale (qui pointe vers le bas, donc y négatif)
                            angle_rov_rad = np.arctan2(dx_rov, -dy_rov)
                            angle_rov_deg = np.degrees(angle_rov_rad)
                    
                    # Forces de traînée
                    Fx_drag_rov, Fy_drag_rov = system.rov.compute_drag_force(
                        vx_rov, vy_rov, y_rov, system.environment
                    )
                    
                    # Poussée d'Archimède et poids
                    F_buoyancy = system.rov.compute_buoyancy_force(system.environment)
                    F_weight = system.rov.compute_weight_force(system.environment)
                    
                    # Poids apparent (poids - poussée d'Archimède)
                    F_apparent_weight = F_weight - F_buoyancy
                    
                    # Forces de traction du câble sur le ROV
                    # Force exercée PAR le câble SUR le ROV = -T_rov * Urov (opposée à Urov)
                    T_rov_val = T_rov if T is not None and len(T) > 0 else 0.0
                    Fx_traction = -T_rov_val * Urov_x
                    Fy_traction = -T_rov_val * Urov_y
                    
                    # Commandes (pour l'instant nulles, mais on peut les récupérer de u_func)
                    u_current = u_func(t_current)
                    Fx_cmd = u_current.get('Fx_rov', 0.0)
                    Fy_cmd = u_current.get('Fy_rov', 0.0)
                    
                    # Somme des forces appliquées au ROV
                    Fx_total = Fx_drag_rov + Fx_traction + Fx_cmd
                    Fy_total = Fy_drag_rov + Fy_traction + F_apparent_weight + Fy_cmd
                    
                    # Stocker les forces ROV
                    data.setdefault('Fx_drag', []).append(float(Fx_drag_rov))
                    data.setdefault('Fy_drag', []).append(float(Fy_drag_rov))
                    data.setdefault('Fx_traction', []).append(float(Fx_traction))
                    data.setdefault('Fy_traction', []).append(float(Fy_traction))
                    data.setdefault('F_apparent_weight', []).append(float(F_apparent_weight))
                    data.setdefault('Fx_total', []).append(float(Fx_total))
                    data.setdefault('Fy_total', []).append(float(Fy_total))
                    
                    # Calculer les forces sur le bateau
                    # Tension du câble au niveau du bateau
                    T_boat_val = T_boat if T is not None and len(T) > 0 else 0.0
                    
                    # CORRECTION: Après correction de _compute_catenary_tensions :
                    # Les positions sont ordonnées : index 0 = bateau, index -1 = ROV
                    # Calculer le vecteur unitaire Ubateau au niveau du bateau
                    angle_boat_deg = 0.0
                    Ubateau_x = 0.0
                    Ubateau_y = -1.0  # Par défaut
                    
                    if x_cable_display is not None and y_cable_display is not None and len(x_cable_display) > 1:
                        # Ubateau = direction du bateau (index 0) vers le point suivant (index 1)
                        dx_bateau = x_cable_display[1] - x_cable_display[0]
                        dy_bateau = y_cable_display[1] - y_cable_display[0]
                        ds_bateau = np.sqrt(dx_bateau**2 + dy_bateau**2)
                        
                        if ds_bateau > 1e-6:
                            Ubateau_x = dx_bateau / ds_bateau
                            Ubateau_y = dy_bateau / ds_bateau
                            # Angle avec la verticale (qui pointe vers le bas, donc y négatif)
                            angle_boat_rad = np.arctan2(dx_bateau, -dy_bateau)
                            angle_boat_deg = np.degrees(angle_boat_rad)
                    
                    # Force de tension du câble sur le bateau
                    # Force exercée PAR le câble SUR le bateau = -T_bateau * Ubateau
                    # (négatif car le câble tire le bateau dans la direction opposée à Ubateau)
                    Fx_traction_boat = -T_boat_val * Ubateau_x
                    Fy_traction_boat = -T_boat_val * Ubateau_y
                    
                    # Force de propulsion du bateau
                    u_current = u_func(t_current)
                    vx_boat_cmd = u_current.get('vx_boat_cmd', 0.0)
                    F_prop_boat = system.boat.compute_propulsion_force(
                        vx_boat_cmd, vx_boat, system.environment
                    )
                    
                    # Somme des forces appliquées au bateau
                    # L'effet de la tension du câble est négligeable
                    # Le bateau évolue uniquement selon la commande de vitesse
                    Fx_total_boat = F_prop_boat
                    Fy_total_boat = 0.0  # Pas de force verticale
                    
                    # Stocker les forces bateau
                    data.setdefault('Fx_traction_boat', []).append(float(Fx_traction_boat))
                    data.setdefault('Fy_traction_boat', []).append(float(Fy_traction_boat))
                    data.setdefault('F_prop_boat', []).append(float(F_prop_boat))
                    data.setdefault('Fx_total_boat', []).append(float(Fx_total_boat))
                    data.setdefault('Fy_total_boat', []).append(float(Fy_total_boat))
                    
                    # Stocker les angles du câble avec la verticale (après calcul de angle_boat_deg)
                    data.setdefault('angle_rov', []).append(float(angle_rov_deg))
                    data.setdefault('angle_boat', []).append(float(angle_boat_deg))
                    
                    # Câble actuel
                    if x_cable_display is not None and y_cable_display is not None:
                        data['x_cable_curr'] = x_cable_display.tolist() if hasattr(x_cable_display, 'tolist') else list(x_cable_display)
                        data['y_cable_curr'] = y_cable_display.tolist() if hasattr(y_cable_display, 'tolist') else list(y_cable_display)
                    
                    # Tensions du câble actuel
                    if T is not None and len(T) > 0:
                        data['T_cable_curr'] = T.tolist() if hasattr(T, 'tolist') else list(T)
                    else:
                        data['T_cable_curr'] = []
                    
                    step_count += 1
                    
                    # Émettre les données toutes les N étapes
                    if step_count % steps_per_update == 0:
                        # Debug: longueurs de segments réellement émises à l'UI
                        try:
                            x_dbg = data.get('x_cable_curr')
                            y_dbg = data.get('y_cable_curr')
                            if x_dbg is not None and y_dbg is not None and len(x_dbg) > 1:
                                ds_min = None
                                ds_max = None
                                for i in range(1, len(x_dbg)):
                                    dx = x_dbg[i] - x_dbg[i-1]
                                    dy = y_dbg[i] - y_dbg[i-1]
                                    ds = float(np.sqrt(dx**2 + dy**2))
                                    ds_min = ds if ds_min is None else min(ds_min, ds)
                                    ds_max = ds if ds_max is None else max(ds_max, ds)
                                if ds_min is not None and ds_max is not None:
                                    trace_print(
                                        1,
                                        f"[DEBUG] Emit ds min/max (n={len(x_dbg)-1}): "
                                        f"{ds_min:.6f} ; {ds_max:.6f} à t={t_current:.2f}s",
                                        flush=True
                                    )
                        except Exception:
                            pass

                        update_data = {
                            **data,
                            'current_time': t_current
                        }
                        if fx_rov_cmd is not None:
                            update_data['fx_rov_cmd'] = float(fx_rov_cmd)
                            update_data['fx_rov_cmd_source'] = "scenario"
                        self.simulation_updated.emit(update_data)
                        
                        # Petit délai pour ne pas surcharger l'interface
                        time.sleep(0.01)
                    
                except Exception as e:
                    self.error_occurred.emit(f"Erreur lors de l'intégration: {str(e)}")
                    import traceback
                    traceback.print_exc()
                    break
            
            # Émettre les données finales
            if not self._stop_requested:
                update_data = {
                    **data,
                    'current_time': t_current
                }
                if fx_rov_cmd is not None:
                    update_data['fx_rov_cmd'] = float(fx_rov_cmd)
                    update_data['fx_rov_cmd_source'] = "scenario"
                self.simulation_updated.emit(update_data)
                self.simulation_finished.emit()
            
        except Exception as e:
            self.error_occurred.emit(f"Erreur dans la simulation: {str(e)}")
            import traceback
            traceback.print_exc()
