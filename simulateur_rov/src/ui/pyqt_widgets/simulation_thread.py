"""
Thread pour exécuter la simulation en arrière-plan
"""
from PyQt6.QtCore import QThread, pyqtSignal
import numpy as np
import time
import sys
import os
import copy
import re
import json
import concurrent.futures
from src.utils.logger import trace_print, set_trace_file, close_trace_file, get_trace_level

_ANSI_RESET = "\033[0m"
_ANSI_RED = "\033[91m"
_ANSI_GREEN = "\033[92m"
_ANSI_YELLOW = "\033[93m"
_ANSI_BLUE = "\033[94m"
_ANSI_MAGENTA = "\033[95m"
_ANSI_CYAN = "\033[96m"
_ANSI_WHITE = "\033[97m"
_ANSI_GRAY = "\033[90m"
_ANSI_LIGHT_GRAY = "\033[37m"
_ANSI_LIGHT_RED = "\033[91m"

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
                 sc_v_bateau: str = "", sc_v_moulinet: str = "",
                 mission_name: str | None = None):
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
        self.mission_name = mission_name
        
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
        trace_print(9, f"\n{_ANSI_YELLOW}[DEBUG] simulation_thread: RUN :  TRACE_LEVEL ={get_trace_level()}{_ANSI_RESET}")
        try:
            from src.models.system_model import ROVSystem
            from src.utils.initial_conditions import get_initial_state
            from src.utils.scenario_utils import commande_scenario
            from src.utils import scenario_utils
            from src.ui.mission_utils import get_missions_directory
            from datetime import datetime
            import csv
            from src.solvers.forces import compute_cable_forces, compute_cable_apparent_weight
            
            # Réutiliser l'état initial calculé lors de l'initialisation si disponible
            cached_system = None
            cached_y0 = None
            if isinstance(self.simulation_state, dict):
                cached_system = self.simulation_state.get('system')
                cached_y0 = self.simulation_state.get('y_current')
            if cached_system is not None and cached_y0 is not None:
                system = cached_system
                y0 = np.array(cached_y0, copy=True)
                try:
                    alpha_val = float(self.calc_params.get('straight_blend_alpha', 1.0))
                    if getattr(system, 'cable', None) is not None and getattr(system.cable, 'solver', None) is not None:
                        system.cable.solver.params['straight_blend_alpha'] = alpha_val
                except Exception:
                    pass
            else:
                # Créer le système ROV
                N_segments = int(self.calc_params.get('N_segments', 50))
                params = copy.deepcopy(self.parameters)
                params.setdefault('cable', {})
                params['cable']['straight_blend_alpha'] = float(
                    self.calc_params.get('straight_blend_alpha', 1.0)
                )
                system = ROVSystem(params, N_segments=N_segments)
                
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
            
            if self.mission_name and not getattr(system.environment, "mission_name", None):
                system.environment.mission_name = str(self.mission_name)

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
            
            # Préparer le fichier de messages de trace
            trace_message_path = None
            try:
                mission_name = getattr(system.environment, "mission_name", None) or self.mission_name
                if mission_name and str(mission_name).strip():
                    mission_dir = get_missions_directory() / str(mission_name)
                    mission_dir.mkdir(parents=True, exist_ok=True)
                    # Déplacer les anciens fichiers de trace/messages/analyse
                    trace_old_dir = mission_dir / "trace_old"
                    patterns = ["*trace.csv", "*mess.txt", "*analyse.MD"]
                    to_move = []
                    for pattern in patterns:
                        to_move.extend(list(mission_dir.glob(pattern)))
                    if to_move:
                        trace_old_dir.mkdir(parents=True, exist_ok=True)
                        for src_path in to_move:
                            if src_path.is_file():
                                dst_path = trace_old_dir / src_path.name
                                if dst_path.exists():
                                    stem = dst_path.stem
                                    suffix = dst_path.suffix
                                    k = 1
                                    while dst_path.exists():
                                        dst_path = trace_old_dir / f"{stem}_{k}{suffix}"
                                        k += 1
                                src_path.replace(dst_path)

                    root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))

                    # Déterminer le répertoire Téléchargements de l'utilisateur
                    user_home = os.path.expanduser("~")
                    downloads_candidates = [
                        os.path.join(user_home, "Téléchargements"),
                        os.path.join(user_home, "Downloads"),
                    ]
                    downloads_dir = None
                    for candidate in downloads_candidates:
                        if os.path.isdir(candidate):
                            downloads_dir = candidate
                            break
                    if downloads_dir is None:
                        # Si aucun des dossiers n'existe, on crée le premier candidat ("Téléchargements")
                        downloads_dir = downloads_candidates[0]
                        try:
                            os.makedirs(downloads_dir, exist_ok=True)
                        except Exception:
                            # En dernier recours, retomber sur le répertoire racine du projet
                            downloads_dir = root_dir

                    # Fichier de messages de trace dans Téléchargements
                    trace_message_path = os.path.join(downloads_dir, "trace_print.txt")
                    try:
                        with open(trace_message_path, "w", encoding="utf-8") as f:
                            f.write("")
                    except Exception:
                        pass
                    set_trace_file(trace_message_path)
            except Exception as e:
                trace_print(8, f"[TRACE] Erreur initialisation: {e}")

            # Simulation avec pas adaptatifs
            t_current = 0.0
            y_current = y0.copy()

            # Normaliser et recoller en douceur la géométrie du câble à l'initialisation
            # pour éviter un segment final anormalement long (ex : mission M103).
            try:
                (x_rov_init, y_rov_init, vx_rov_init, vy_rov_init,
                 x_boat_init, vx_boat_init, x_cable_init, y_cable_init, T_init, L_init) = system.unpack_state(y_current)
                if (
                    x_cable_init is not None
                    and y_cable_init is not None
                    and len(x_cable_init) > 1
                    and L_init is not None
                    and float(L_init) > 1e-6
                ):
                    # Normaliser la longueur du câble à L_init avec recollement doux
                    system.cable.solver._last_sim_time = float(t_current)
                    x_corr, y_corr, straight_mode, _ = system.cable.solver._normalize_cable_length(
                        np.asarray(x_cable_init, dtype=float),
                        np.asarray(y_cable_init, dtype=float),
                        float(L_init),
                        x_boat=float(x_boat_init),
                        y_boat=0.0,
                        x_rov=float(x_rov_init),
                        y_rov=float(y_rov_init),
                        k_tail=10,
                        t=float(t_current),
                    )

                    # Réinjecter la géométrie corrigée dans l'état initial
                    idx_x_cable = 6
                    idx_y_cable = idx_x_cable + system.N + 1
                    y_current[idx_x_cable:idx_y_cable] = x_corr
                    y_current[idx_y_cable:idx_y_cable + system.N + 1] = y_corr
                    # Aligner le ROV sur l'extrémité câble si la branche corde > L a été utilisée
                    if straight_mode:
                        y_current[0] = float(x_corr[-1])
                        y_current[1] = float(y_corr[-1])

                    # Mettre à jour les buffers précédents du système
                    system.x_cable_prev = np.asarray(x_corr, dtype=float).copy()
                    system.y_cable_prev = np.asarray(y_corr, dtype=float).copy()
            except Exception as e:
                trace_print(9, f"[INIT] Échec renormalisation câble initial: {e}")

            dt = min(dt_max, t_final / 100.0)  # Pas initial
            dt_min = float(self.calc_params.get('dt_min', 1e-4))
            max_step_wall_time = float(self.calc_params.get('max_step_wall_time', 1.0))
            max_step_nfev = int(self.calc_params.get('max_step_nfev', 2000))
            ode_timeout_s = float(self.calc_params.get('ode_timeout_s', 2.0))
            ode_fallback_mode = False
            ode_executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
            
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
                'L_seg': [],
                'D_straight': [],
                'T_rov': [],
                'T_boat': [],
                'T_max': [],
                'Fx_drag_rov': [],
                'Fy_drag_rov': [],
                'Fy_rov_app_w': [],
                'Fx_rov_total': [],
                'Fy_rov_total': [],
                'Fx_traction_boat': [],
                'Fy_traction_boat': [],
                'Fx_traction_rov': [],
                'Fy_traction_rov': [],
                'F_prop_boat': [],
                'Fx_total_boat': [],
                'Fy_total_boat': [],
                'Fx_drag_cable': [],
                'Fy_drag_cable': [],
                'Fy_cable_app_w': [],
                'Fx_drag_cable_longitudinal': [],
                'Fy_drag_cable_longitudinal': [],
                'Fx_drag_cable_perpendicular': [],
                'Fy_drag_cable_perpendicular': [],
                'Fx_cmd_rov': [],
                'Fy_cmd_rov': [],
                'dl_dt_cmd': [],
                'dl_dt_auto_explain': [],
                'x_cable_curr': None,
                'y_cable_curr': None,
            }
            
            # Écrire la liste des clés du dictionnaire data dans dic.txt avec descriptions
            root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
            dic_file_path = os.path.join(root_dir, "dic.txt")
            
            # Dictionnaire des descriptions pour chaque clé
            data_descriptions = {
                'time': 'Temps de simulation (s)',
                'x_rov': 'Position horizontale du ROV (m)',
                'y_rov': 'Position verticale (profondeur) du ROV (m, négatif vers le bas)',
                'vx_rov': 'Vitesse horizontale du ROV (m/s)',
                'vy_rov': 'Vitesse verticale du ROV (m/s)',
                'x_boat': 'Position horizontale du bateau (m)',
                'vx_boat': 'Vitesse horizontale du bateau (m/s)',
                'L': 'Longueur du câble déployée (m)',
                'L_seg': 'Somme des longueurs des segments du câble (m)',
                'D_straight': 'Distance en ligne droite entre le bateau et le ROV (m)',
                'T_rov': 'Tension du câble au niveau du ROV (N)',
                'T_boat': 'Tension du câble au niveau du bateau (N)',
                'T_max': 'Tension maximale le long du câble (N)',
                'Fx_drag_rov': 'Force de traînée horizontale sur le ROV (N)',
                'Fy_drag_rov': 'Force de traînée verticale sur le ROV (N, positif vers le haut/surface, négatif vers le bas/fond)',
                'Fy_rov_app_w': 'Poids apparent du ROV (N, positif vers le haut/surface si flottabilité positive, négatif vers le bas/fond si flottabilité négative)',
                'Fx_rov_total': 'Force horizontale totale sur le ROV (N)',
                'Fy_rov_total': 'Force verticale totale sur le ROV (N, positif vers le haut/surface, négatif vers le bas/fond)',
                'Fx_traction_boat': 'Force de traction horizontale du câble sur le bateau (N)',
                'Fy_traction_boat': 'Force de traction verticale du câble sur le bateau (N)',
                'Fx_traction_rov': 'Force de traction horizontale du câble sur le ROV (N)',
                'Fy_traction_rov': 'Force de traction verticale du câble sur le ROV (N, positif vers le haut/surface, négatif vers le bas/fond)',
                'F_prop_boat': 'Force de propulsion du bateau (N)',
                'Fx_total_boat': 'Force horizontale totale sur le bateau (N)',
                'Fy_total_boat': 'Force verticale totale sur le bateau (N)',
                'Fx_drag_cable': 'Composante horizontale de la force de traînée exercée sur le câble (N)',
                'Fy_drag_cable': 'Composante verticale de la force de traînée exercée sur le câble (N)',
                'Fy_cable_app_w': 'Poids apparent du câble (N, positif vers le bas)',
                'Fx_drag_cable_longitudinal': 'Composante horizontale de la traînée longitudinale (frottement) sur le câble (N)',
                'Fy_drag_cable_longitudinal': 'Composante verticale de la traînée longitudinale (frottement) sur le câble (N)',
                'Fx_drag_cable_perpendicular': 'Composante horizontale de la traînée perpendiculaire (normale) sur le câble (N)',
                'Fy_drag_cable_perpendicular': 'Composante verticale de la traînée perpendiculaire (normale) sur le câble (N)',
                'Fx_cmd_rov': 'Composante horizontale de la force de propulsion du ROV résultant des commandes envoyées par le pilote du ROV (N)',
                'Fy_cmd_rov': 'Composante verticale de la force de propulsion du ROV résultant des commandes envoyées par le pilote du ROV (N, positif vers le haut/surface, négatif vers le bas/fond)',
                'dl_dt_cmd': 'Commande de vitesse de déroulement du câble (m/s)',
                'dl_dt_auto_explain': 'Explication de la commande automatique dL/dt',
                'x_cable_curr': 'Positions horizontales actuelles des points du câble (liste)',
                'y_cable_curr': 'Positions verticales actuelles des points du câble (liste)',
                'cable_mode': 'Mode du câble: "catenary" (caténaire) ou "straight" (tendu)',
                'scenario_triggers': 'Déclencheurs de scénario activés à cet instant (liste)',
                'F_buoyancy_net': 'Flottabilité nette du ROV (N, positif si flottant)',
                'cable_drag': 'Forces de traînée du câble (tuple: Fx, Fy) (N)',
                'angle_rov': 'Angle du câble avec la verticale au niveau du ROV (degrés)',
                'angle_boat': 'Angle du câble avec la verticale au niveau du bateau (degrés)',
                'T_cable_curr': 'Tensions actuelles le long du câble (liste) (N)',
                'fx_rov_cmd': 'Commande de force horizontale sur le ROV (N)',
                'fx_rov_cmd_source': 'Source de la commande fx_rov: "scenario" ou autre',
            }
            
            try:
                with open(dic_file_path, "w", encoding="utf-8") as f:
                    f.write("Liste des cles du dictionnaire data:\n")
                    f.write("=" * 80 + "\n\n")
                    f.write(f"Total: {len(data.keys())} cles\n\n")
                    
                    # Trouver la longueur maximale des clés pour l'alignement
                    max_key_len = max(len(key) for key in data.keys())
                    
                    for i, key in enumerate(sorted(data.keys()), 1):
                        description = data_descriptions.get(key, "Description non disponible")
                        f.write(f"{i:3d}. {key:<{max_key_len}} : {description}\n")
                    
                    f.write("\n" + "=" * 80 + "\n")
                    f.write("Genere automatiquement par simulation_thread.py\n")
            except Exception as e:
                trace_print(8, f"[DIC.TXT] Erreur écriture: {e}")

            def check_cable_invariants(context=None):
                """
                Vérifie les contraintes physiques sur le câble :
                - cohérence longueur L vs somme des segments L_seg,
                - slack (L - D_straight),
                - recollement des extrémités (bateau et ROV).

                En cas de violation, trace un message détaillé au niveau 9.
                """
                try:
                    ctx = context or {}
                    t_chk = ctx.get('t')
                    phase = ctx.get('phase', 'iteration')
                    step = ctx.get('step')
                    dl_dt_cmd_val = ctx.get('dl_dt_cmd')
                    dl_dt_explain = ctx.get('dl_dt_explain')
                    cable_mode_val = ctx.get('cable_mode')
                    triggers = ctx.get('triggers')

                    # Si des métriques explicites sont fournies dans le contexte (celles
                    # calculées en fin d'itération pour l'UI), les utiliser de
                    # préférence pour garantir la cohérence UI / invariants.
                    x_cable_ctx = ctx.get('x_cable')
                    y_cable_ctx = ctx.get('y_cable')
                    L_ctx = ctx.get('L')
                    L_seg_ctx = ctx.get('L_seg')
                    D_straight_ctx = ctx.get('D_straight')
                    x_boat_ctx = ctx.get('x_boat')
                    y_boat_ctx = ctx.get('y_boat', 0.0)
                    x_rov_ctx = ctx.get('x_rov')
                    y_rov_ctx = ctx.get('y_rov')
                    T_ctx = ctx.get('T')

                    # Utiliser T du contexte si disponible, sinon None
                    T_c = None
                    if T_ctx is not None:
                        try:
                            T_array = np.asarray(T_ctx, dtype=float)
                            # Vérifier que le tableau n'est pas vide
                            if len(T_array) > 0:
                                T_c = T_array
                            else:
                                # Debug : T_ctx est vide
                                try:
                                    trace_print(9, f"[CABLE INVARIANTS] DEBUG: T_ctx est vide ou None")
                                except:
                                    pass
                        except Exception as e:
                            T_c = None
                            # Debug : erreur lors de la conversion de T_ctx
                            try:
                                trace_print(9, f"[CABLE INVARIANTS] DEBUG: Erreur conversion T_ctx: {e}, type={type(T_ctx)}")
                            except:
                                pass
                    else:
                        # Debug : T_ctx n'est pas dans le contexte
                        try:
                            trace_print(9, f"[CABLE INVARIANTS] DEBUG: T_ctx n'est pas dans le contexte")
                        except:
                            pass

                    if (
                        x_cable_ctx is not None
                        and y_cable_ctx is not None
                        and len(x_cable_ctx) >= 2
                        and L_ctx is not None
                        and x_boat_ctx is not None
                        and x_rov_ctx is not None
                        and y_rov_ctx is not None
                    ):
                        # Utiliser directement la géométrie et les longueurs passées par le caller
                        x_cable_c = np.asarray(x_cable_ctx, dtype=float)
                        y_cable_c = np.asarray(y_cable_ctx, dtype=float)
                        L_c = float(L_ctx)

                        # Somme des longueurs des segments + stats élémentaires
                        ds_list = []
                        if L_seg_ctx is not None:
                            # On fait confiance à la valeur fournie pour L_seg
                            L_seg = float(L_seg_ctx)
                        else:
                            L_seg = 0.0
                            for i in range(len(x_cable_c) - 1):
                                dx = float(x_cable_c[i + 1] - x_cable_c[i])
                                dy = float(y_cable_c[i + 1] - y_cable_c[i])
                                ds = float(np.hypot(dx, dy))
                                L_seg += ds
                                ds_list.append(ds)

                        if not ds_list:
                            # Si ds_list est vide mais L_seg_ctx fourni, reconstruire ds_list
                            for i in range(len(x_cable_c) - 1):
                                dx = float(x_cable_c[i + 1] - x_cable_c[i])
                                dy = float(y_cable_c[i + 1] - y_cable_c[i])
                                ds = float(np.hypot(dx, dy))
                                ds_list.append(ds)
                        
                        # Ajouter les segments aux extrémités (bateau-premier point et avant-dernier point-ROV)
                        # pour avoir une vue complète de tous les segments violant la contrainte
                        if len(x_cable_c) > 0:
                            # Segment bateau -> premier point du câble
                            dx_boat_to_first = float(x_cable_c[0] - x_boat_ctx)
                            dy_boat_to_first = float(y_cable_c[0] - y_boat_ctx)
                            ds_boat_to_first = float(np.hypot(dx_boat_to_first, dy_boat_to_first))
                            # Insérer au début de ds_list
                            ds_list.insert(0, ds_boat_to_first)
                            
                            # Segment dernier point du câble -> ROV
                            dx_last_to_rov = float(x_rov_ctx - x_cable_c[-1])
                            dy_last_to_rov = float(y_rov_ctx - y_cable_c[-1])
                            ds_last_to_rov = float(np.hypot(dx_last_to_rov, dy_last_to_rov))
                            # Ajouter à la fin de ds_list
                            ds_list.append(ds_last_to_rov)

                        # Distance en ligne droite et slack
                        if D_straight_ctx is not None:
                            D_straight = float(D_straight_ctx)
                        else:
                            dx_straight = float(x_rov_ctx - x_boat_ctx)
                            dy_straight = float(y_rov_ctx - y_boat_ctx)
                            D_straight = float(np.hypot(dx_straight, dy_straight))
                        slack = float(L_c - D_straight)

                        # Recollement des extrémités (bateau / ROV)
                        dist_boat = float(
                            np.hypot(
                                float(x_cable_c[0] - x_boat_ctx),
                                float(y_cable_c[0] - y_boat_ctx),
                            )
                        )
                        dist_rov = float(
                            np.hypot(
                                float(x_cable_c[-1] - x_rov_ctx),
                                float(y_cable_c[-1] - y_rov_ctx),
                            )
                        )
                        # Stocker les coordonnées du bateau et du ROV pour le message
                        x_boat_final = float(x_boat_ctx)
                        y_boat_final = float(y_boat_ctx)
                        x_rov_final = float(x_rov_ctx)
                        y_rov_final = float(y_rov_ctx)
                    else:
                        # Fallback : reconstruire entièrement les métriques à partir de y_vec
                        y_vec = ctx.get('y')
                        if y_vec is None:
                            return

                        (x_rov_c, y_rov_c, vx_rov_c, vy_rov_c,
                         x_boat_c, vx_boat_c, x_cable_c, y_cable_c, T_c, L_c) = system.unpack_state(y_vec)

                        if x_cable_c is None or y_cable_c is None or len(x_cable_c) < 2:
                            return

                        # Somme des longueurs des segments + stats élémentaires
                        L_seg = 0.0
                        ds_list = []
                        for i in range(len(x_cable_c) - 1):
                            dx = float(x_cable_c[i + 1] - x_cable_c[i])
                            dy = float(y_cable_c[i + 1] - y_cable_c[i])
                            ds = float(np.hypot(dx, dy))
                            L_seg += ds
                            ds_list.append(ds)

                        # Distance en ligne droite et slack
                        dx_straight = float(x_rov_c - x_boat_c)
                        dy_straight = float(y_rov_c - 0.0)
                        D_straight = float(np.hypot(dx_straight, dy_straight))
                        slack = float(L_c - D_straight)

                        # Recollement des extrémités
                        dist_boat = float(np.hypot(float(x_cable_c[0] - x_boat_c),
                                                   float(y_cable_c[0] - 0.0)))
                        dist_rov = float(np.hypot(float(x_cable_c[-1] - x_rov_c),
                                                  float(y_cable_c[-1] - y_rov_c)))
                        
                        # Ajouter les segments aux extrémités (bateau-premier point et avant-dernier point-ROV)
                        # pour avoir une vue complète de tous les segments violant la contrainte
                        if len(x_cable_c) > 0:
                            # Segment bateau -> premier point du câble
                            dx_boat_to_first = float(x_cable_c[0] - x_boat_c)
                            dy_boat_to_first = float(y_cable_c[0] - 0.0)
                            ds_boat_to_first = float(np.hypot(dx_boat_to_first, dy_boat_to_first))
                            # Insérer au début de ds_list
                            ds_list.insert(0, ds_boat_to_first)
                            
                            # Segment dernier point du câble -> ROV
                            dx_last_to_rov = float(x_rov_c - x_cable_c[-1])
                            dy_last_to_rov = float(y_rov_c - y_cable_c[-1])
                            ds_last_to_rov = float(np.hypot(dx_last_to_rov, dy_last_to_rov))
                            # Ajouter à la fin de ds_list
                            ds_list.append(ds_last_to_rov)
                        
                        # Stocker les coordonnées du bateau et du ROV pour le message
                        x_boat_final = float(x_boat_c)
                        y_boat_final = 0.0
                        x_rov_final = float(x_rov_c)
                        y_rov_final = float(y_rov_c)

                    tol_L_rel = 1e-3
                    tol_recol = 1e-3
                    tol_slack_neg = 1e-3

                    violations = []
                    details = {}
                    
                    # Stocker dist_rov dans details pour cohérence avec l'affichage (après initialisation de details)
                    # dist_rov est défini dans les deux branches (if/else) ci-dessus
                    if '_debug_ds_cable_ROV' not in details:
                        details['_debug_ds_cable_ROV'] = dist_rov

                    if float(L_c) > 1e-6:
                        L_c_val = float(L_c)
                        rel_err = abs(L_seg - L_c_val) / L_c_val
                        if rel_err > tol_L_rel:
                            violations.append(f"L_mismatch(rel={rel_err:.3e})")
                        # Détails sur la distribution des longueurs de segments
                        if ds_list:
                            ds_min = float(np.min(ds_list))
                            ds_max = float(np.max(ds_list))
                            ds_mean = float(np.mean(ds_list))
                            details["ds_min"] = ds_min
                            details["ds_max"] = ds_max
                            details["ds_mean"] = ds_mean
                            # ds_first et ds_last sont les segments internes (pas les extrémités)
                            # Si ds_list contient les segments aux extrémités, ils sont en première et dernière position
                            if len(ds_list) >= 3:
                                # ds_list[0] = bateau-premier point, ds_list[1] = premier segment interne
                                # ds_list[-2] = dernier segment interne, ds_list[-1] = dernier point-ROV
                                details["ds_first"] = float(ds_list[1]) if len(ds_list) > 1 else float(ds_list[0])
                                details["ds_last"] = float(ds_list[-2]) if len(ds_list) > 1 else float(ds_list[-1])
                            else:
                                details["ds_first"] = float(ds_list[0])
                                details["ds_last"] = float(ds_list[-1])
                            # Coordonnées des points du câble
                            try:
                                if len(x_cable_c) >= 1:
                                    # Point 0 du câble (premier point)
                                    details["x0"] = float(x_cable_c[0])
                                    details["y0"] = float(y_cable_c[0])
                                if len(x_cable_c) >= 3:
                                    # Points 1 et N-1 du câble
                                    details["x1"] = float(x_cable_c[1])
                                    details["y1"] = float(y_cable_c[1])
                                    details["xNm1"] = float(x_cable_c[-2])
                                    details["yNm1"] = float(y_cable_c[-2])
                                if len(x_cable_c) >= 1:
                                    # Point N du câble (dernier point)
                                    details["xN"] = float(x_cable_c[-1])
                                    details["yN"] = float(y_cable_c[-1])
                            except Exception:
                                pass

                            # Contrôle local : aucun segment ne doit être trop long
                            # par rapport à la longueur moyenne ds_target = L_c / N.
                            # N_seg est le nombre de segments internes du câble (sans les extrémités)
                            # ds_list contient maintenant N_seg + 2 segments (internes + extrémités)
                            N_seg = len(x_cable_c) - 1 if len(x_cable_c) > 1 else 0
                            if N_seg > 0:
                                # Utiliser L_c_val (longueur commandée) pour ds_target
                                # ds_target est basé sur le nombre de segments internes
                                ds_target = L_c_val / N_seg
                                ratio = ds_max / ds_target if ds_target > 0.0 else 0.0

                                # Traçage systématique pour diagnostic (même sans violation)
                                try:
                                    t_dbg = t_chk if t_chk is not None else -1.0
                                except Exception:
                                    pass

                                # Alerter dès que ds_max / ds dépasse le seuil k_max
                                k_max = 2.0
                                if ds_target > 0.0 and ds_max > k_max * ds_target:
                                    i_max = int(np.argmax(ds_list))
                                    violations.append(
                                        f"segment_too_long(i={i_max},ratio={ratio:.3f})"
                                    )

                                    # Compter le nombre de segments qui violent la contrainte
                                    # Utiliser le même seuil k_max * ds_target que pour la détection
                                    seuil = k_max * ds_target
                                    # Compter explicitement pour debug
                                    nb_seg_ko_list = [i for i, ds in enumerate(ds_list) if ds > seuil]
                                    nb_seg_ko = len(nb_seg_ko_list)
                                    # Vérifier aussi les segments aux extrémités pour debug
                                    if len(ds_list) > 0:
                                        details["_debug_ds_bat_cable"] = float(ds_list[0])
                                        # Ne pas écraser _debug_ds_cable_ROV si déjà défini (pour cohérence avec dist_rov)
                                        if "_debug_ds_cable_ROV" not in details:
                                            details["_debug_ds_cable_ROV"] = float(ds_list[-1])
                                        details["_debug_seuil"] = float(seuil)
                                        details["_debug_ds_target"] = float(ds_target)
                                        details["_debug_N_seg"] = N_seg
                                        details["_debug_ds_list_len"] = len(ds_list)
                                        if nb_seg_ko > 0:
                                            details["_debug_indices_ko"] = ",".join(str(i) for i in nb_seg_ko_list[:10])  # Limiter à 10 pour éviter un message trop long
                                    details["ds_max_ratio"] = float(ratio)
                                    details["i_max_segment"] = i_max
                                    details["nb_seg_ko"] = nb_seg_ko

                    if dist_boat > tol_recol:
                        violations.append(f"recollement_bateau(dist={dist_boat:.3e})")

                    if dist_rov > tol_recol:
                        violations.append(f"recollement_rov(dist={dist_rov:.3e})")

                    if slack < -tol_slack_neg:
                        violations.append(f"slack_neg(slack={slack:.3e})")

                    # Contrôle : aucun point du câble ne doit être au-dessus de la surface (y > 0)
                    # y < 0 représente la profondeur sous la surface, y = 0 est la surface
                    points_above_surface = []
                    if y_cable_c is not None and len(y_cable_c) > 0:
                        for i, y_val in enumerate(y_cable_c):
                            if y_val > 0.0:  # Point au-dessus de la surface
                                points_above_surface.append((i, float(y_val)))
                    
                    if points_above_surface:
                        # Trouver le point le plus haut
                        max_y_point = max(points_above_surface, key=lambda x: x[1])
                        violations.append(f"point_above_surface(i={max_y_point[0]},y={max_y_point[1]:.3e},nb_points={len(points_above_surface)})")
                        # Stocker les détails sur les points au-dessus de la surface
                        details["points_above_surface"] = len(points_above_surface)
                        details["max_y_above_surface"] = max_y_point[1]
                        details["i_max_y_above_surface"] = max_y_point[0]

                    # Stocker les coordonnées du bateau (P0) et du ROV (PN) dans details
                    try:
                        details["x_boat"] = x_boat_final
                        details["y_boat"] = y_boat_final
                        details["x_rov"] = x_rov_final
                        details["y_rov"] = y_rov_final
                    except Exception:
                        pass
                    
                    # Stocker les informations sur les violations de chaque distance pour l'affichage coloré
                    try:
                        # Codes ANSI pour les couleurs
                        ANSI_RED = "\033[91m"  # Rouge
                        ANSI_GREEN = "\033[92m"  # Vert
                        ANSI_RESET = "\033[0m"  # Reset
                        
                        # Distance bateau -> P0
                        ds_bat_cable = details.get("_debug_ds_bat_cable", 0.0)
                        seuil_val = details.get("_debug_seuil", 0.0)
                        dist_boat_violation = dist_boat > tol_recol or (seuil_val > 0.0 and ds_bat_cable > seuil_val)
                        details["_dist_bat_P0"] = ds_bat_cable
                        details["_dist_bat_P0_violation"] = dist_boat_violation
                        
                        # Distance P0 -> P1
                        ds_first_val = details.get("ds_first", 0.0)
                        dist_P0_P1_violation = seuil_val > 0.0 and ds_first_val > seuil_val
                        details["_dist_P0_P1"] = ds_first_val
                        details["_dist_P0_P1_violation"] = dist_P0_P1_violation
                        
                        # Distance PNm1 -> PN
                        ds_last_val = details.get("ds_last", 0.0)
                        dist_PNm1_PN_violation = seuil_val > 0.0 and ds_last_val > seuil_val
                        details["_dist_PNm1_PN"] = ds_last_val
                        details["_dist_PNm1_PN_violation"] = dist_PNm1_PN_violation
                        
                        # Distance PN -> ROV
                        # Utiliser dist_rov directement pour garantir la cohérence avec le message principal
                        # ds_cable_ROV peut être différent de dist_rov si calculé à un moment différent
                        ds_cable_ROV = details.get("_debug_ds_cable_ROV", dist_rov)
                        # Utiliser dist_rov pour l'affichage pour garantir la cohérence
                        details["_dist_PN_ROV"] = dist_rov
                        dist_PN_ROV_violation = dist_rov > tol_recol or (seuil_val > 0.0 and ds_cable_ROV > seuil_val)
                        details["_dist_PN_ROV_violation"] = dist_PN_ROV_violation
                        
                        # Stocker les codes ANSI pour utilisation dans le message
                        details["_ANSI_RED"] = ANSI_RED
                        details["_ANSI_GREEN"] = ANSI_GREEN
                        details["_ANSI_RESET"] = ANSI_RESET
                    except Exception:
                        pass

                    if not violations:
                        return
                    
                    # Fonction locale pour formater la ligne de coordonnées avec distances colorées
                    def format_coordinates_line(det):
                        ANSI_RED = det.get("_ANSI_RED", "")
                        ANSI_GREEN = det.get("_ANSI_GREEN", "")
                        ANSI_RESET = det.get("_ANSI_RESET", "")
                        
                        line = f"\nBateau=({det.get('x_boat', 0.0):.3f},{det.get('y_boat', 0.0):.3f})"
                        
                        # Distance bateau -> P0
                        dist_bat_P0 = det.get("_dist_bat_P0", 0.0)
                        color_bat_P0 = ANSI_RED if det.get("_dist_bat_P0_violation", False) else ANSI_GREEN
                        line += f"  {color_bat_P0}{dist_bat_P0:.3f}{ANSI_RESET}"
                        line += f" P0=({det.get('x0', 0.0):.3f},{det.get('y0', 0.0):.3f})"
                        
                        # Distance P0 -> P1
                        dist_P0_P1 = det.get("_dist_P0_P1", 0.0)
                        color_P0_P1 = ANSI_RED if det.get("_dist_P0_P1_violation", False) else ANSI_GREEN
                        line += f" {color_P0_P1}{dist_P0_P1:.3f}{ANSI_RESET}"
                        line += f" P1=({det.get('x1', 0.0):.3f},{det.get('y1', 0.0):.3f})"
                        
                        line += f" PNm1=({det.get('xNm1', 0.0):.3f},{det.get('yNm1', 0.0):.3f})"
                        
                        # Distance PNm1 -> PN
                        dist_PNm1_PN = det.get("_dist_PNm1_PN", 0.0)
                        color_PNm1_PN = ANSI_RED if det.get("_dist_PNm1_PN_violation", False) else ANSI_GREEN
                        line += f" {color_PNm1_PN}{dist_PNm1_PN:.3f}{ANSI_RESET}"
                        line += f" PN=({det.get('xN', 0.0):.3f},{det.get('yN', 0.0):.3f})"
                        
                        # Distance PN -> ROV
                        dist_PN_ROV = det.get("_dist_PN_ROV", 0.0)
                        color_PN_ROV = ANSI_RED if det.get("_dist_PN_ROV_violation", False) else ANSI_GREEN
                        line += f" {color_PN_ROV}{dist_PN_ROV:.3f}{ANSI_RESET}"
                        line += f" ROV=({det.get('x_rov', 0.0):.3f},{det.get('y_rov', 0.0):.3f})"
                        
                        return line
                    
                    mission_name = getattr(system.environment, "mission_name", None)
                    dl_dt_str = (
                        f"{float(dl_dt_cmd_val):.6f}"
                        if isinstance(dl_dt_cmd_val, (int, float))
                        else "None"
                    )
                    T_boat_val = float(T_c[0]) if T_c is not None and len(T_c) > 0 else 0.0
                    T_rov_val = float(T_c[-1]) if T_c is not None and len(T_c) > 0 else 0.0

                    mission_name = getattr(system.environment, "mission_name", None)
                    
                    base_prefix = "\n[CABLE INVARIANTS] VIOLATION -> "
                    if mission_name is not None:
                        base_prefix += f"mission={mission_name} "
                    if t_chk is not None:
                        base_prefix += f"phase={phase} t={t_chk:.3f} "
                    else:
                        base_prefix += f"phase={phase} "

                    if step is not None:
                        base_prefix += f"step={step} "

                    # Formater les violations avec les libellés en rouge
                    ANSI_RED = "\033[91m"
                    ANSI_RESET = "\033[0m"
                    violations_formatted = []
                    for violation in violations:
                        # Extraire le nom de la violation (avant la parenthèse) et les paramètres
                        if '(' in violation:
                            violation_name = violation.split('(')[0]
                            violation_params = '(' + violation.split('(', 1)[1]
                            violations_formatted.append(f"{ANSI_RED}{violation_name}{ANSI_RESET}{violation_params}")
                        else:
                            violations_formatted.append(f"{ANSI_RED}{violation}{ANSI_RESET}")
                    
                    message = (
                        base_prefix
                        + f"\nviolations={','.join(violations_formatted)}\n"
                        + f"L={float(L_c):.2f} L_seg={L_seg:.2f} "
                        + f"D_straight={D_straight:.2f} slack={slack:.2f} "
                        + f"dist_boat={dist_boat:.3e} dist_rov={dist_rov:.3e} "
                        + f"T_boat={T_boat_val:.2f} T_rov={T_rov_val:.2f} "
                        + (f"mode={cable_mode_val} " if cable_mode_val is not None else "")
                        + f"dL_dt_cmd={dl_dt_str} "
                        + (
                            f"dL_dt_explain={dl_dt_explain} "
                            if dl_dt_explain not in (None, "")
                            else ""
                        )
                        + (
                            f"triggers={triggers} " if triggers not in (None, []) else ""
                        )
                        + (
                            f" ds_mean={details.get('ds_mean', 0.0):.6f}"
                            f" ds_min={details.get('ds_min', 0.0):.6f}"
                            f" ds_max={details.get('ds_max', 0.0):.6f}"
                            f" ds_first={details.get('ds_first', 0.0):.6f}"
                            f" ds_last={details.get('ds_last', 0.0):.6f}"
                            if details
                            else ""
                        )
                        + (
                            f" ds_max_ratio={details.get('ds_max_ratio', 0.0):.3f}"
                            f" i_max_segment={details.get('i_max_segment', -1):d}"
                            f" nb_seg_ko={details.get('nb_seg_ko', 0):d}"
                            if "ds_max_ratio" in details and "i_max_segment" in details 
                            else ""
                        )
                        + (
                            f"\nDEBUG: ds_target={details.get('_debug_ds_target', 0.0):.6f}"
                            f" seuil={details.get('_debug_seuil', 0.0):.6f}"
                            f" ds_bat_cable={details.get('_debug_ds_bat_cable', 0.0):.6f}"
                            f" ds_cable_ROV={details.get('_debug_ds_cable_ROV', 0.0):.6f}"
                            f" N_seg={details.get('_debug_N_seg', 0):d}"
                            f" ds_list_len={details.get('_debug_ds_list_len', 0):d}"
                            + (f" indices_ko=[{details.get('_debug_indices_ko', '')}]" if "_debug_indices_ko" in details else "")
                            if "_debug_ds_target" in details
                            else ""
                        )
                        + (
                            format_coordinates_line(details)
                            if "x0" in details and "x1" in details and "xNm1" in details and "xN" in details and "x_boat" in details and "x_rov" in details
                            else ""
                        )
                    )
                    trace_print(9, message)
                except Exception as e:
                    trace_print(9, f"\n[CABLE INVARIANTS] Erreur verification: {e}")

            # Initialiser le fichier Excel pour sauvegarder data dans Téléchargements
            data_excel_path = None
            data_excel_wb = None
            data_excel_ws = None
            data_excel_headers = None
            data_excel_row = 2  # Ligne 1 = en-têtes
            
            try:
                # Utiliser la même logique que pour "Analyse missions.xlsm"
                user_home = os.path.expanduser("~")
                downloads_candidates = [
                    os.path.join(user_home, "Téléchargements"),
                    os.path.join(user_home, "Downloads"),
                ]
                downloads_dir = None
                for candidate in downloads_candidates:
                    if os.path.isdir(candidate):
                        downloads_dir = candidate
                        break
                if downloads_dir is None:
                    downloads_dir = downloads_candidates[0]
                    os.makedirs(downloads_dir, exist_ok=True)
                
                data_excel_path = os.path.join(downloads_dir, "trace_dic.xlsx")
                
                from openpyxl import Workbook
                data_excel_wb = Workbook()
                data_excel_ws = data_excel_wb.active
                data_excel_ws.title = "Data"
                
                # Créer les en-têtes (toutes les clés du dictionnaire)
                data_excel_headers = sorted(data.keys())
                for col_idx, header in enumerate(data_excel_headers, 1):
                    data_excel_ws.cell(row=1, column=col_idx, value=header)
                
                data_excel_wb.save(data_excel_path)
                trace_print(1, f"[DATA EXCEL] Fichier initialise: {data_excel_path}")
            except Exception as e:
                trace_print(8, f"[DATA EXCEL] Erreur initialisation: {e}")
                data_excel_path = None
                data_excel_wb = None
                data_excel_ws = None
            
            def extract_data_row(data):
                """Extrait les valeurs actuelles du dictionnaire data pour une ligne Excel."""
                row_values = {}
                for key in sorted(data.keys()):
                    value = data[key]
                    if isinstance(value, list) and len(value) > 0:
                        # Prendre la dernière valeur de la liste
                        row_values[key] = value[-1]
                    elif value is None:
                        row_values[key] = None
                    elif isinstance(value, (list, tuple)) and len(value) == 0:
                        row_values[key] = None
                    elif isinstance(value, (list, tuple)) and len(value) > 0:
                        # Pour les listes/tuples complexes, convertir en JSON string
                        try:
                            row_values[key] = json.dumps(value)
                        except:
                            row_values[key] = str(value)
                    else:
                        row_values[key] = value
                return row_values
            
            def trim_data(data, max_size=10000):
                """Limite la taille des listes dans data à max_size points en gardant les N derniers points."""
                for key, value in data.items():
                    if isinstance(value, list) and len(value) > max_size:
                        # Garder les N derniers points
                        data[key] = value[-max_size:]
            
            step_count = 0
            fx_rov_cmd = None  # Initialiser pour l'émission finale
            step_triggers = []
            # Limite par défaut pour le nombre de points envoyés à l'UI
            plot_points_limit = int(self.calc_params.get('plot_points_limit', 1000))

            # Vérifier les invariants du câble à la fin de la phase d'initialisation
            # Calculer les mêmes données qu'en fin d'itération pour avoir un contexte complet
            try:
                (x_rov_init, y_rov_init, vx_rov_init, vy_rov_init,
                 x_boat_init, vx_boat_init, x_cable_init, y_cable_init, T_init, L_init) = system.unpack_state(y_current)
                
                # Préparer x_cable_display et y_cable_display comme en fin d'itération
                x_cable_display_init = np.asarray(x_cable_init, dtype=float).copy() if x_cable_init is not None else None
                y_cable_display_init = np.asarray(y_cable_init, dtype=float).copy() if y_cable_init is not None else None
                
                # Garantir l'ordre bateau -> ROV (index 0 = bateau, index -1 = ROV)
                # Le solveur peut parfois retourner les points dans l'ordre inverse dans certains cas d'erreur
                if x_cable_display_init is not None and y_cable_display_init is not None and len(x_cable_display_init) > 1:
                    dist_first_to_boat = np.sqrt((x_cable_display_init[0] - x_boat_init)**2 + (y_cable_display_init[0] - 0.0)**2)
                    dist_first_to_rov = np.sqrt((x_cable_display_init[0] - x_rov_init)**2 + (y_cable_display_init[0] - y_rov_init)**2)
                    dist_last_to_boat = np.sqrt((x_cable_display_init[-1] - x_boat_init)**2 + (y_cable_display_init[-1] - 0.0)**2)
                    dist_last_to_rov = np.sqrt((x_cable_display_init[-1] - x_rov_init)**2 + (y_cable_display_init[-1] - y_rov_init)**2)
                    # Inverser si le premier point est plus proche du ROV ET le dernier point est plus proche du bateau
                    # NOTE: Après correction des fallbacks dans cable_solver.py, cette inversion ne devrait plus être nécessaire
                    # Si elle se produit, c'est un cas anormal qui doit être signalé
                    if dist_first_to_rov < dist_first_to_boat and dist_last_to_boat < dist_last_to_rov:
                        trace_print(5, f"[CABLE ORDER] ⚠️  Inversion nécessaire en phase d'initialisation: "
                            f"le solveur a retourné les points dans l'ordre ROV->bateau au lieu de bateau->ROV. "
                            f"AVANT inversion: P0=({x_cable_display_init[0]:.3f},{y_cable_display_init[0]:.3f}) "
                            f"PN=({x_cable_display_init[-1]:.3f},{y_cable_display_init[-1]:.3f}) "
                            f"dist_P0_to_boat={dist_first_to_boat:.3f} dist_P0_to_rov={dist_first_to_rov:.3f} "
                            f"dist_PN_to_boat={dist_last_to_boat:.3f} dist_PN_to_rov={dist_last_to_rov:.3f}")
                        x_cable_display_init = np.flip(x_cable_display_init)
                        y_cable_display_init = np.flip(y_cable_display_init)
                        # Inverser aussi les tensions si disponibles
                        if T_init is not None and len(T_init) == len(x_cable_display_init):
                            T_init = np.flip(T_init)
                        trace_print(5, f"[CABLE ORDER] Après inversion: P0=({x_cable_display_init[0]:.3f},{y_cable_display_init[0]:.3f}) "
                            f"PN=({x_cable_display_init[-1]:.3f},{y_cable_display_init[-1]:.3f})")
                
                # Calculer L_seg
                L_seg_init = 0.0
                if x_cable_display_init is not None and y_cable_display_init is not None and len(x_cable_display_init) > 1:
                    for i in range(len(x_cable_display_init) - 1):
                        dx_seg = x_cable_display_init[i + 1] - x_cable_display_init[i]
                        dy_seg = y_cable_display_init[i + 1] - y_cable_display_init[i]
                        L_seg_init += float(np.hypot(dx_seg, dy_seg))
                
                # Calculer D_straight
                dx_straight_init = x_rov_init - x_boat_init
                dy_straight_init = y_rov_init - 0.0
                D_straight_init = float(np.hypot(dx_straight_init, dy_straight_init))
                
                # Construire le contexte complet comme en fin d'itération
                check_cable_invariants({
                    'phase': 'initialisation',
                    't': t_current,
                    'step': 0,
                    'y': y_current,
                    'x_cable': x_cable_display_init,
                    'y_cable': y_cable_display_init,
                    'L': L_init,
                    'L_seg': L_seg_init,
                    'D_straight': D_straight_init,
                    'x_boat': x_boat_init,
                    'y_boat': 0.0,
                    'x_rov': x_rov_init,
                    'y_rov': y_rov_init,
                    'T': T_init,
                    'dl_dt_cmd': None,
                    'dl_dt_explain': None,
                    'cable_mode': None,
                    'triggers': [],
                })
            except Exception as e:
                # En cas d'erreur, utiliser le contexte minimal (fallback)
                trace_print(8, f"[CABLE INVARIANTS] Erreur préparation contexte initialisation: {e}")
                check_cable_invariants({
                    'phase': 'initialisation',
                    't': t_current,
                    'step': 0,
                    'y': y_current,
                })

            def _trace_non_finite_rov(stage_name, y_vec, t_val, step_val):
                """Trace compacte si une composante ROV de l'état n'est pas finie."""
                try:
                    xr, yr, vxr, vyr = float(y_vec[0]), float(y_vec[1]), float(y_vec[2]), float(y_vec[3])
                    if all(np.isfinite(v) for v in (xr, yr, vxr, vyr)):
                        return
                    trace_print(
                        1,
                        f"[NON_FINITE] stage={stage_name} t={t_val:.2f} step={step_val} "
                        f"x_rov={xr} y_rov={yr} vx_rov={vxr} vy_rov={vyr}"
                    )
                except Exception as e_nf:
                    trace_print(1, f"[NON_FINITE] stage={stage_name} t={t_val:.2f} step={step_val} inspect_error={e_nf}")
            
            while t_current < t_final and not self._stop_requested:
                trace_print(8, f"\n{_ANSI_YELLOW}[DEBUG] simulation_thread: LOOP : t_current={t_current:.2f} {_ANSI_RESET} ")
                # Attendre si en pause
                while self._paused and not self._stop_requested:
                    time.sleep(0.1)
                    # Vérifier l'état de pause depuis le state partagé
                    if hasattr(self.simulation_state, 'get'):
                        self._paused = self.simulation_state.get('paused', False)
                
                if self._stop_requested:
                    break
                
                # Calculer le pas suivant AVANT les calculs de commandes pour que auto_L_7
                # puisse afficher le temps auquel la commande sera appliquée
                t_next = min(t_current + dt, t_final)
                dt_used = t_next - t_current
                try:
                    system.cable.solver._last_sim_time = float(t_next)
                except Exception:
                    pass

                fx_rov_cmd = None
                fy_rov_cmd = None
                vx_boat_cmd = None
                dl_dt_cmd = None
                dl_dt_auto_explain = ""
                dl_dt_mode = "scen"
                if isinstance(self.simulation_state, dict):
                    dl_dt_mode = self.simulation_state.get('dl_dt_mode', 'scen')

                _trace_non_finite_rov("pre_command", y_current, t_current, step_count)
                if self.sc_fx_rov or self.sc_fy_rov or self.sc_v_bateau or self.sc_v_moulinet or dl_dt_mode == "auto":
                    try:
                        (x_rov_current, y_rov_current, _, _, x_boat_current, _, _, _, T_current, L_current) = system.unpack_state(y_current)
                        # DEBUG: Traçage de L_current avant intégration
                        trace_print(7, f"[DEBUG L] t={t_current:.2f} AVANT intégration: L_current={L_current:.6f} m")
                        t_boat = None
                        if T_current is not None and len(T_current) > 0:
                            t_boat = float(T_current[0])
                        step_triggers = []
                        if self.sc_fx_rov:
                            fx_rov_cmd = commande_scenario(
                                "Fx_rov",
                                self.sc_fx_rov,
                                t_current,
                                L_current,
                                y_rov_current,
                                x_rov_current,
                                t_boat,
                                step_triggers,
                            )
                        if self.sc_fy_rov:
                            fy_rov_cmd = commande_scenario(
                                "Fy_rov",
                                self.sc_fy_rov,
                                t_current,
                                L_current,
                                y_rov_current,
                                x_rov_current,
                                t_boat,
                                step_triggers,
                            )
                        if self.sc_v_bateau:
                            vx_boat_cmd = commande_scenario(
                                "Vx_bateau",
                                self.sc_v_bateau,
                                t_current,
                                L_current,
                                y_rov_current,
                                x_rov_current,
                                t_boat,
                                step_triggers,
                            )
                        if dl_dt_mode == "auto":
                            auto_L_selector = self.calc_params.get("auto_L", "auto_L_1") if isinstance(self.calc_params, dict) else "auto_L_1"
                            auto_func = scenario_utils.get_auto_L_function(auto_L_selector)
                            if auto_func is None:
                                trace_print(8, f"[AUTO dL/dt] Fonction {auto_L_selector!r} introuvable, fallback auto_L_1")
                                auto_func = scenario_utils.get_auto_L_function("auto_L_1") or scenario_utils.auto_L_1
                            if t_current == 0 and hasattr(auto_func, "_history"):
                                auto_func._history = []
                            trupt = None
                            try:
                                trupt = float(self.parameters.get("cable", {}).get("tension_rupture", 50.0))
                            except Exception:
                                trupt = 50.0
                            tcible = None
                            try:
                                tcible = self.calc_params.get("Tcible", None)
                            except Exception:
                                tcible = None
                            if tcible is None or tcible == 0:
                                tcible = trupt / 2.0
                            gamma_moulinet_max = None
                            gamma_moulinet_min = None
                            gamma_moulinet_max = None
                            try:
                                boat_params = self.parameters.get("boat", {})
                                gamma_moulinet_min = boat_params.get("gamma_moulinet_min", -0.5)
                                gamma_moulinet_max = boat_params.get("gamma_moulinet_max", 0.5)
                            except Exception:
                                gamma_moulinet_min = -0.5
                                gamma_moulinet_max = 0.5
                            dl_dt_min = None
                            dl_dt_max = None
                            try:
                                boat_params = self.parameters.get("boat", {})
                                dl_dt_min = boat_params.get("dl_dt_min", -1.0)
                                dl_dt_max = boat_params.get("dl_dt_max", 1.0)
                            except Exception:
                                dl_dt_min = -1.0
                                dl_dt_max = 1.0
                            t_boat = None
                            if T_current is not None and len(T_current) > 0:
                                t_boat = float(T_current[0])
                                auto_result = auto_func(
                                    t_current,
                                    y_rov_current,
                                    L_current,
                                    float(self.simulation_state.get('fx_rov', self.fx_rov)) if isinstance(self.simulation_state, dict) else self.fx_rov,
                                    float(self.simulation_state.get('fy_rov', self.fy_rov)) if isinstance(self.simulation_state, dict) else self.fy_rov,
                                    t_boat,
                                    float(x_rov_current),
                                    float(x_boat_current),
                                    Trupt=trupt,
                                    Tcible=tcible,
                                    Gamma_moulinet_min=gamma_moulinet_min,
                                    Gamma_moulinet_max=gamma_moulinet_max,
                                    dl_dt_min=dl_dt_min,
                                    dl_dt_max=dl_dt_max,
                                    data=data,
                                )
                            if isinstance(auto_result, tuple) and len(auto_result) >= 2:
                                dl_dt_cmd = auto_result[0]
                                dl_dt_auto_explain = str(auto_result[1])
                            else:
                                dl_dt_cmd = auto_result
                            # DEBUG: Traçage de dL_dt calculé
                            trace_print(7, f"[DEBUG L] t={t_current:.2f} dL_dt_cmd={dl_dt_cmd:.6f} m/s "
                                f"(L_current={L_current:.6f} m pour calcul)"
                            )
                            trace_print(1, f"[AUTO dL/dt] t={t_current:.2f} L={L_current:.2f} "
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
                                x_rov_current,
                                t_boat,
                                step_triggers,
                            )
                    except Exception as e:
                        trace_print(9, f"{_ANSI_RED}Erreur commande_scenario: {e}{_ANSI_RESET}")
                        fx_rov_cmd = None
                        fy_rov_cmd = None
                        vx_boat_cmd = None
                        dl_dt_cmd = None

                # Éviter une longueur de câble négative
                if dl_dt_cmd is not None:
                    try:
                        L_min = 1e-6
                        if dt > 0:
                            min_dl_dt = (L_min - float(L_current)) / float(dt)
                            if float(L_current) <= L_min and dl_dt_cmd < 0:
                                dl_dt_cmd = 0.0
                            elif dl_dt_cmd < min_dl_dt:
                                trace_print(8, f"[AUTO dL/dt] Clamp dL/dt {dl_dt_cmd:.6f} -> {min_dl_dt:.6f} "
                                    f"pour éviter L<0 (L={L_current:.6f}, dt={dt:.6f})"
                                )
                                dl_dt_cmd = min_dl_dt
                    except Exception:
                        pass

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

                # Fin de mission via événement de scénario
                if hasattr(commande_scenario, "_mission_end") and commande_scenario._mission_end:
                    trace_print(1, "[SCEN] Fin de mission déclenchée par evt x.")
                    if isinstance(self.simulation_state, dict):
                        self.simulation_state["mission_ended"] = True
                        self.simulation_state["mission_end_time"] = getattr(
                            commande_scenario, "_mission_end_time", t_current
                        )
                    self._stop_requested = True
                    break

                # t_next et dt_used ont déjà été calculés au début de l'itération
                # Intégrer un pas (avec réduction adaptative si nécessaire)
                try:
                    trace_print(7, f"[DEBUG] simulation_thread: INTÉGRATION : step_count={step_count}")
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
                    used_fallback = False
                    while retry_count < 5 and not success:
                        if ode_fallback_mode:
                            dy_dt = system.compute_derivatives(
                                t_current, y_current, u_func(t_current)
                            )
                            y_current = y_current + (t_next - t_current) * dy_dt
                            used_fallback = True
                            success = True
                            break

                        # Utiliser la méthode d'intégration du système (avec timeout)
                        step_start = time.perf_counter()
                        future = ode_executor.submit(
                            system.integrate,
                            [t_current, t_next],
                            y_current,
                            u_func,
                            dt_max
                        )
                        try:
                            solution = future.result(timeout=ode_timeout_s)
                        except concurrent.futures.TimeoutError:
                            trace_print(1, f"[WARN] Timeout ODE à t={t_current:.2f} "
                                f"(dt={dt:.6f}, >{ode_timeout_s:.2f}s) -> fallback Euler"
                            )
                            ode_fallback_mode = True
                            if ode_executor is not None:
                                ode_executor.shutdown(wait=False, cancel_futures=True)
                                ode_executor = None
                            dy_dt = system.compute_derivatives(
                                t_current, y_current, u_func(t_current)
                            )
                            y_current = y_current + (t_next - t_current) * dy_dt
                            used_fallback = True
                            success = True
                            break

                        step_elapsed = time.perf_counter() - step_start
                        nfev = getattr(solution, "nfev", None)
                        slow_step = (step_elapsed > max_step_wall_time) or (
                            nfev is not None and nfev > max_step_nfev
                        )
                        if solution.success and slow_step:
                            trace_print(1, f"[WARN] Pas ODE lent à t={t_current:.2f} "
                                f"(dt={dt:.6f}, nfev={nfev}, {step_elapsed:.3f}s) -> fallback Euler"
                            )
                            dy_dt = system.compute_derivatives(
                                t_current, y_current, u_func(t_current)
                            )
                            y_current = y_current + (t_next - t_current) * dy_dt
                            used_fallback = True
                            success = True
                            break
                        
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
                    if not used_fallback:
                        if getattr(solution, "sol", None) is not None:
                            y_current = solution.sol(t_next)
                        else:
                            y_current = solution.y[:, -1]
                    _trace_non_finite_rov("post_integrate", y_current, t_current, step_count)

                    # Sécuriser les tensions (éviter valeurs négatives ou déraisonnables)
                    try:
                        t_rupt = float(self.parameters.get("cable", {}).get("tension_rupture", 50.0))
                    except Exception:
                        t_rupt = 50.0
                    try:
                        t_max_clip = float(self.calc_params.get("T_max_clip", t_rupt * 3.0))
                    except Exception:
                        t_max_clip = t_rupt * 3.0
                    idx_t = 6 + 2 * (system.N + 1)
                    if len(y_current) > idx_t:
                        t_slice = y_current[idx_t:idx_t + system.N + 1]
                        t_slice = np.clip(t_slice, 0.0, t_max_clip)
                        y_current[idx_t:idx_t + system.N + 1] = t_slice
                    
                    # Vérification optimisée : seulement tous les N pas et seulement les valeurs critiques
                    if step_count % 10 == 0:  # Vérifier seulement tous les 10 pas
                        if (not np.isfinite(y_current[0]) or not np.isfinite(y_current[1]) or 
                            not np.isfinite(y_current[2]) or not np.isfinite(y_current[3])):
                            self.error_occurred.emit(f"État invalide (NaN/Inf) après intégration à t={t_next:.2f} s")
                            break
                    
                    # NOTE: t_current n'est PAS incrémenté ici. Il sera incrémenté à la toute fin de l'itération
                    # pour que toutes les opérations (auto_L_7, intégration, vérification invariants) utilisent
                    # le même temps dans une même itération
                    
                    # Décoder l'état
                    (x_rov, y_rov, vx_rov, vy_rov,
                     x_boat, vx_boat, x_cable, y_cable, T, L) = system.unpack_state(y_current)

                    # Projection finale du câble: imposer recollement/L_target sur l'état de fin de pas,
                    # puis redistribuer les tensions sur cette géométrie finale.
                    try:
                        if (
                            x_cable is not None and y_cable is not None
                            and len(x_cable) > 1 and L is not None and float(L) > 1e-9
                        ):
                            x_proj, y_proj, straight_mode, _ = system.cable.solver._normalize_cable_length(
                                np.asarray(x_cable, dtype=float),
                                np.asarray(y_cable, dtype=float),
                                float(L),
                                x_boat=float(x_boat),
                                y_boat=0.0,
                                x_rov=float(x_rov),
                                y_rov=float(y_rov),
                                k_tail=10,
                                t=float(t_next),
                            )

                            x_proj = np.asarray(x_proj, dtype=float)
                            y_proj = np.asarray(y_proj, dtype=float)
                            x_proj[0] = float(x_boat)
                            y_proj[0] = 0.0
                            if straight_mode:
                                x_rov = float(x_proj[-1])
                                y_rov = float(y_proj[-1])
                            else:
                                x_proj[-1] = float(x_rov)
                                y_proj[-1] = float(y_rov)

                            weight_per_unit = (
                                (system.cable.rho_cable - system.environment.rho_eau)
                                * system.cable.A_cable
                                * system.environment.g
                            )
                            T_proj = system.cable.solver._compute_catenary_tensions(
                                x_proj,
                                y_proj,
                                weight_per_unit,
                                rov_m=system.rov.m,
                                rov_vol=system.rov.V,
                            )
                            T_proj = np.asarray(T_proj, dtype=float)

                            idx_x_cable = 6
                            idx_y_cable = idx_x_cable + system.N + 1
                            idx_t = idx_y_cable + system.N + 1
                            y_current[0] = float(x_rov)
                            y_current[1] = float(y_rov)
                            y_current[idx_x_cable:idx_y_cable] = x_proj
                            y_current[idx_y_cable:idx_t] = y_proj
                            y_current[idx_t:idx_t + system.N + 1] = T_proj

                            x_cable = x_proj
                            y_cable = y_proj
                            T = T_proj
                            _trace_non_finite_rov("post_projection", y_current, t_current, step_count)
                    except Exception as e:
                        trace_print(9, f"[FINAL PROJECTION] Échec projection finale câble: {e}")

                    # DEBUG: Traçage de L après intégration et comparaison avec L_current
                    # Récupérer la commande dL_dt qui a été utilisée pour cette intégration
                    # Note: u_func est défini dans la portée de run(), donc on peut l'utiliser
                    try:
                        u_at_t = u_func(t_current) if callable(u_func) else None
                        dL_dt_used = u_at_t.get('dL_dt') if u_at_t and isinstance(u_at_t, dict) else None
                    except:
                        dL_dt_used = None
                    
                    if hasattr(self, '_prev_L_current') and hasattr(self, '_prev_dt'):
                        # Utiliser la commande dL_dt qui a été réellement utilisée pour cette intégration
                        if dL_dt_used is not None:
                            delta_L_expected = dL_dt_used * self._prev_dt
                        else:
                            delta_L_expected = self._prev_dl_dt_cmd * self._prev_dt if self._prev_dl_dt_cmd is not None and self._prev_dt > 0 else 0.0
                        
                        delta_L_actual = L - self._prev_L_current
                        if abs(delta_L_actual - delta_L_expected) > 0.01:  # Seuil de 1 cm
                            trace_print(8, f"[DEBUG L] t={t_current:.2f} APRÈS intégration: "
                                f"L={L:.6f} m, L_prev={self._prev_L_current:.6f} m, "
                                f"delta_L_actual={delta_L_actual:.6f} m, "
                                f"delta_L_expected={delta_L_expected:.6f} m "
                                f"(dL_dt_used={dL_dt_used:.6f} m/s si disponible, "
                                f"dL_dt_cmd_prev={self._prev_dl_dt_cmd:.6f} m/s, dt={self._prev_dt:.6f} s), "
                                f"ÉCART={abs(delta_L_actual - delta_L_expected):.6f} m"
                            )
                    # Stocker les valeurs pour la prochaine itération
                    self._prev_L_current = L_current if 'L_current' in locals() else L
                    self._prev_dl_dt_cmd = dl_dt_cmd if 'dl_dt_cmd' in locals() else None
                    self._prev_dt = dt_used if 'dt_used' in locals() else None
                    
                    # IMPORTANT : Ne PAS recalculer le ROV si L < distance droite
                    # La longueur L est fixe (déterminée par la somme des dL/dt)
                    # Le slack doit être >= 0, et c'est la tension au ROV qui doit être ajustée
                    # pour contraindre le mouvement et respecter slack >= 0
                    dx_straight = x_rov - x_boat
                    dy_straight = y_rov - 0.0
                    L_straight = float(np.hypot(dx_straight, dy_straight))
                    slack = L - L_straight
                    cable_length_constrained = False
                    if slack < 0.0:
                        # Le slack est négatif, le câble est tendu
                        # La tension au ROV sera ajustée dans system_model pour forcer slack >= 0
                        cable_length_constrained = True

                    # Utiliser une vitesse cohérente avec les positions (pour traînée + affichage)
                    prev_x = data.get('x_rov', [])[-1] if data.get('x_rov') else None
                    prev_y = data.get('y_rov', [])[-1] if data.get('y_rov') else None
                    prev_t = data.get('time', [])[-1] if data.get('time') else None
                    if prev_x is not None and prev_y is not None and prev_t is not None:
                        dt_local = t_current - prev_t
                        if dt_local > 0:
                            vx_rov = (x_rov - prev_x) / dt_local
                            vy_rov = (y_rov - prev_y) / dt_local
                        else:
                            vx_rov = 0.0
                            vy_rov = 0.0
                    
                    # Pour l'affichage, privilégier directement la géométrie d'équilibre
                    # calculée par le solveur (qui garantit déjà la normalisation et le
                    # recollement aux extrémités). On travaille sur des copies pour ne
                    # jamais modifier l'état dynamique y_current ici.
                    x_cable_display = np.asarray(x_cable, dtype=float).copy()
                    y_cable_display = np.asarray(y_cable, dtype=float).copy()
                    
                    # DEBUG: Vérifier la forme du câble avant affichage
                    if x_cable_display is not None and y_cable_display is not None and len(x_cable_display) > 2:
                        # Calculer la courbure approximative (écart par rapport à une ligne droite)
                        x_start, y_start = x_cable_display[0], y_cable_display[0]
                        x_end, y_end = x_cable_display[-1], y_cable_display[-1]
                        # Distance perpendiculaire maximale d'un point à la ligne droite
                        max_deviation = 0.0
                        i_max_deviation = -1
                        deviations = []
                        if abs(x_end - x_start) > 1e-6 or abs(y_end - y_start) > 1e-6:
                            # Vecteur directeur de la ligne droite
                            dx_line = x_end - x_start
                            dy_line = y_end - y_start
                            line_length = np.sqrt(dx_line**2 + dy_line**2)
                            for i in range(1, len(x_cable_display) - 1):
                                # Vecteur du point de départ au point courant
                                dx_point = x_cable_display[i] - x_start
                                dy_point = y_cable_display[i] - y_start
                                # Projection sur la ligne droite
                                t = (dx_point * dx_line + dy_point * dy_line) / (line_length**2 + 1e-9)
                                # Point projeté
                                x_proj = x_start + t * dx_line
                                y_proj = y_start + t * dy_line
                                # Distance perpendiculaire
                                deviation = np.sqrt((x_cable_display[i] - x_proj)**2 + (y_cable_display[i] - y_proj)**2)
                                deviations.append(deviation)
                                if deviation > max_deviation:
                                    max_deviation = deviation
                                    i_max_deviation = i
                        
                        # Calculer le slack et la déviation théorique attendue
                        dx_straight = x_end - x_start
                        dy_straight = y_end - y_start
                        D_straight = np.sqrt(dx_straight**2 + dy_straight**2)
                        L_seg_calc = 0.0
                        for i in range(len(x_cable_display) - 1):
                            dx_seg = x_cable_display[i + 1] - x_cable_display[i]
                            dy_seg = y_cable_display[i + 1] - y_cable_display[i]
                            L_seg_calc += np.sqrt(dx_seg**2 + dy_seg**2)
                        slack_calc = L_seg_calc - D_straight
                        
                        # Estimation théorique de la déviation maximale pour une caténaire simple
                        # Pour une caténaire avec slack s et distance horizontale d, la déviation max ≈ s/2 * sqrt(s/d)
                        # (approximation grossière, mais donne un ordre de grandeur)
                        if D_straight > 1e-6 and slack_calc > 0:
                            # Estimation très simplifiée : pour une caténaire symétrique, déviation max ≈ slack * sqrt(slack / D_straight) / 2
                            theoretical_max_deviation = slack_calc * np.sqrt(slack_calc / D_straight) / 2.0
                        else:
                            theoretical_max_deviation = 0.0
                        
                        if max_deviation < 0.01:  # Si la déviation maximale est très petite, c'est une ligne droite
                            trace_print(9, f"[DEBUG CABLE SHAPE] ⚠️  Câble semble être une ligne droite à t={t_current:.3f}: "
                                f"max_deviation={max_deviation:.6f} m, P0=({x_start:.3f},{y_start:.3f}), "
                                f"PN=({x_end:.3f},{y_end:.3f}), slack={slack_calc:.2f} m, "
                                f"théorique_attendu≈{theoretical_max_deviation:.2f} m")
                        else:
                            # Afficher plus de détails si la déviation est beaucoup plus faible que théorique
                            ratio = theoretical_max_deviation / max_deviation if max_deviation > 1e-6 else 0.0
                            if ratio > 2.0:
                                trace_print(7, f"[DEBUG CABLE SHAPE] ⚠️  Câble courbe mais déviation trop faible à t={t_current:.3f}: "
                                    f"max_deviation={max_deviation:.6f} m (théorique≈{theoretical_max_deviation:.2f} m, "
                                    f"ratio={ratio:.1f}x), slack={slack_calc:.2f} m, D_straight={D_straight:.2f} m, "
                                    f"L_seg={L_seg_calc:.2f} m, i_max={i_max_deviation}, "
                                    f"P_max=({x_cable_display[i_max_deviation]:.3f},{y_cable_display[i_max_deviation]:.3f})")
                            else:
                                trace_print(7, f"[DEBUG CABLE SHAPE] ✓ Câble a une forme courbe à t={t_current:.3f}: "
                                    f"max_deviation={max_deviation:.6f} m (théorique≈{theoretical_max_deviation:.2f} m), "
                                    f"slack={slack_calc:.2f} m")
                    
                    # Garantir l'ordre bateau -> ROV (index 0 = bateau, index -1 = ROV)
                    # Le solveur peut parfois retourner les points dans l'ordre inverse dans certains cas d'erreur
                    if x_cable_display is not None and y_cable_display is not None and len(x_cable_display) > 1:
                        dist_first_to_boat = np.sqrt((x_cable_display[0] - x_boat)**2 + (y_cable_display[0] - 0.0)**2)
                        dist_first_to_rov = np.sqrt((x_cable_display[0] - x_rov)**2 + (y_cable_display[0] - y_rov)**2)
                        dist_last_to_boat = np.sqrt((x_cable_display[-1] - x_boat)**2 + (y_cable_display[-1] - 0.0)**2)
                        dist_last_to_rov = np.sqrt((x_cable_display[-1] - x_rov)**2 + (y_cable_display[-1] - y_rov)**2)
                        # Inverser si le premier point est plus proche du ROV ET le dernier point est plus proche du bateau
                        # NOTE: Après correction des fallbacks dans cable_solver.py, cette inversion ne devrait plus être nécessaire
                        # Si elle se produit, c'est un cas anormal qui doit être signalé
                        should_flip = (dist_first_to_rov < dist_first_to_boat and dist_last_to_boat < dist_last_to_rov)
                        if should_flip:
                            # Avertissement: le solveur a retourné les points dans le mauvais ordre
                            # Cela ne devrait plus se produire après correction des fallbacks
                            trace_print(5, f"[CABLE ORDER] ⚠️  Inversion nécessaire à t={t_current:.3f} step={step_count}: "
                                f"le solveur a retourné les points dans l'ordre ROV->bateau au lieu de bateau->ROV. "
                                f"AVANT inversion: P0=({x_cable_display[0]:.3f},{y_cable_display[0]:.3f}) "
                                f"PN=({x_cable_display[-1]:.3f},{y_cable_display[-1]:.3f}) "
                                f"dist_P0_to_boat={dist_first_to_boat:.3f} dist_P0_to_rov={dist_first_to_rov:.3f} "
                                f"dist_PN_to_boat={dist_last_to_boat:.3f} dist_PN_to_rov={dist_last_to_rov:.3f}")
                            # np.flip() retourne une vue, donc on doit créer une copie pour éviter les problèmes de référence
                            x_cable_display = np.flip(x_cable_display).copy()
                            y_cable_display = np.flip(y_cable_display).copy()
                            # Inverser aussi les tensions si disponibles
                            if T is not None and len(T) == len(x_cable_display):
                                T = np.flip(T).copy()
                            trace_print(5, f"[CABLE ORDER] Après inversion: P0=({x_cable_display[0]:.3f},{y_cable_display[0]:.3f}) "
                                f"PN=({x_cable_display[-1]:.3f},{y_cable_display[-1]:.3f})")
                    
                    # Déterminer le mode câble
                    cable_mode = "catenary"
                    dx_straight = x_rov - x_boat
                    dy_straight = y_rov - 0.0
                    L_straight = float(np.hypot(dx_straight, dy_straight))
                    if L_straight > 1e-9 and L < L_straight * 1.000001:
                        cable_mode = "straight"

                    # Calculer L_seg (somme des longueurs des segments du câble)
                    L_seg = 0.0
                    if x_cable_display is not None and y_cable_display is not None and len(x_cable_display) > 1:
                        for i in range(len(x_cable_display) - 1):
                            dx_seg = x_cable_display[i + 1] - x_cable_display[i]
                            dy_seg = y_cable_display[i + 1] - y_cable_display[i]
                            L_seg += float(np.hypot(dx_seg, dy_seg))
                    
                    # Calculer D_straight (distance en ligne droite entre bateau et ROV)
                    # IMPORTANT : Ne PAS forcer D_straight = L même si le câble est tendu
                    # Le ROV n'est plus recalculé, donc D_straight peut être > L
                    # Le slack = L - D_straight peut être négatif, et c'est la tension qui doit
                    # contraindre le mouvement pour ramener slack >= 0
                    dx_straight = x_rov - x_boat
                    dy_straight = y_rov - 0.0
                    D_straight = float(np.hypot(dx_straight, dy_straight))
                    
                    # Stocker les données (ces valeurs serviront aussi bien à l'UI qu'aux invariants)
                    data['time'].append(t_current)
                    data['x_rov'].append(float(x_rov))
                    data['y_rov'].append(float(y_rov))
                    data['vx_rov'].append(float(vx_rov))
                    data['vy_rov'].append(float(vy_rov))
                    data['x_boat'].append(float(x_boat))
                    data['vx_boat'].append(float(vx_boat))
                    data['L'].append(float(L))
                    data['L_seg'].append(float(L_seg))
                    data['D_straight'].append(float(D_straight))
                    # DEBUG: Vérifier cohérence entre L stocké et L_step
                    if len(data['L']) > 0:
                        L_stored = data['L'][-1]
                        if abs(L_stored - L) > 1e-6:
                            trace_print(8, f"[DEBUG L] t={t_current:.2f} INCOHÉRENCE: "
                                f"L={L:.6f} m, L_stored={L_stored:.6f} m, "
                                f"ÉCART={abs(L_stored - L):.6f} m"
                            )
                    # Utiliser dl_dt_cmd directement pour garantir la cohérence avec le contexte partagé
                    if 'dl_dt_cmd' in locals() and dl_dt_cmd is not None:
                        data['dl_dt_cmd'].append(float(dl_dt_cmd))
                    elif isinstance(self.simulation_state, dict):
                        data['dl_dt_cmd'].append(float(self.simulation_state.get('dl_dt', self.dl_dt)))
                    else:
                        data['dl_dt_cmd'].append(0.0)
                    if dl_dt_mode == "auto":
                        data['dl_dt_auto_explain'].append(dl_dt_auto_explain)
                    else:
                        data['dl_dt_auto_explain'].append("")
                    data.setdefault('cable_mode', []).append(cable_mode)
                    data.setdefault('scenario_triggers', []).append(step_triggers)
                    
                    # Calculer les tensions AVANT de construire update_data pour garantir
                    # que les valeurs dans update_data correspondent aux valeurs actuelles
                    if T is not None and len(T) > 0:
                        T_array = np.asarray(T)
                        # CORRECTION: Après correction de _compute_catenary_tensions :
                        # T[0] = tension au bateau, T[-1] = tension au ROV
                        T_rov = float(T_array[-1]) if len(T_array) > 0 else 0.0  # Tension au ROV = T[-1]
                        T_boat = float(T_array[0]) if len(T_array) > 0 else 0.0  # Tension au bateau = T[0]
                        T_max = float(np.max(T_array)) if len(T_array) > 0 else 0.0
                        data['T_rov'].append(T_rov)
                        data['T_max'].append(T_max)
                        data['T_boat'].append(T_boat)
                    else:
                        T_rov = 0.0
                        T_boat = 0.0
                        T_max = 0.0
                        data['T_rov'].append(0.0)
                        data['T_max'].append(0.0)
                        data['T_boat'].append(0.0)
                    
                    # Calculer les forces sur le ROV AVANT de construire le contexte partagé
                    # pour garantir que toutes les données sont cohérentes
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
                    
                    # Poids apparent : positif si flottabilité positive (vers le haut/surface)
                    # Convention : F_apparent_weight = F_buoyancy - F_weight
                    # - Positif si flottabilité positive (ROV plus léger que l'eau, force vers le haut)
                    # - Négatif si flottabilité négative (ROV plus lourd que l'eau, force vers le bas)
                    F_apparent_weight = F_buoyancy - F_weight
                    # Flottabilité nette affichée (positive si flottabilité positive)
                    F_buoyancy_net = F_apparent_weight
                    
                    # Forces de traction du câble sur le ROV
                    # Urov pointe du point précédent vers le ROV (direction du câble vers le ROV)
                    # Pour avoir la force exercée PAR le câble SUR le ROV vers le haut (positif),
                    # on utilise T_rov * (-Urov_y) car Urov_y pointe vers le bas si le ROV est plus profond
                    # Nouvelle convention : positif = vers le haut (surface)
                    # T_rov est déjà calculé et défini plus haut
                    Fx_traction = -T_rov * Urov_x  # Horizontal : signe inchangé
                    Fy_traction = T_rov * (-Urov_y)  # Vertical : inversé pour positif = vers le haut
                    
                    # Commandes (pour l'instant nulles, mais on peut les récupérer de u_func)
                    u_current = u_func(t_current)
                    Fx_cmd = u_current.get('Fx_rov', 0.0)
                    Fy_cmd = u_current.get('Fy_rov', 0.0)
                    
                    # Somme des forces appliquées au ROV
                    # Convention : toutes les forces verticales sont positives vers le haut (surface)
                    Fx_total = Fx_drag_rov + Fx_traction + Fx_cmd
                    Fy_total = F_apparent_weight + Fy_drag_rov + Fy_traction + Fy_cmd
                    
                    # Stocker les forces ROV
                    data.setdefault('Fx_drag_rov', []).append(float(Fx_drag_rov))
                    data.setdefault('Fy_drag_rov', []).append(float(Fy_drag_rov))
                    data.setdefault('Fx_traction_rov', []).append(float(Fx_traction))
                    data.setdefault('Fy_traction_rov', []).append(float(Fy_traction))
                    data.setdefault('Fx_cmd_rov', []).append(float(Fx_cmd))
                    data.setdefault('Fy_cmd_rov', []).append(float(Fy_cmd))
                    data.setdefault('Fy_rov_app_w', []).append(float(F_apparent_weight))
                    data.setdefault('F_buoyancy_net', []).append(float(F_buoyancy_net))
                    data.setdefault('Fx_rov_total', []).append(float(Fx_total))
                    data.setdefault('Fy_rov_total', []).append(float(Fy_total))
                    
                    # Construire le contexte partagé entre check_cable_invariants et le rafraîchissement UI
                    shared_context = {
                        'phase': 'iteration',
                        't': t_current,
                        'step': step_count,
                        'y': y_current,
                        'x_cable': x_cable_display,
                        'y_cable': y_cable_display,
                        'L': L,
                        'L_seg': L_seg,
                        'D_straight': D_straight,
                        'x_boat': x_boat,
                        'y_boat': 0.0,
                        'x_rov': x_rov,
                        'y_rov': y_rov,
                        'T': T,
                        'dl_dt_cmd': dl_dt_cmd,
                        'dl_dt_explain': dl_dt_auto_explain,
                        'cable_mode': cable_mode,
                        'triggers': step_triggers,
                    }
                    
                    # Mettre à jour data['x_cable_curr'] et data['y_cable_curr'] avec les données du contexte partagé
                    # pour garantir la cohérence entre les données stockées et celles utilisées par l'UI
                    if x_cable_display is not None and y_cable_display is not None:
                        x_cable_list = x_cable_display.tolist() if hasattr(x_cable_display, 'tolist') else list(x_cable_display)
                        y_cable_list = y_cable_display.tolist() if hasattr(y_cable_display, 'tolist') else list(y_cable_display)
                        data['x_cable_curr'] = x_cable_list
                        data['y_cable_curr'] = y_cable_list
                        data['cable_point_indices'] = list(range(len(x_cable_list)))
                    
                    # Tensions du câble actuel
                    if T is not None and len(T) > 0:
                        data['T_cable_curr'] = T.tolist() if hasattr(T, 'tolist') else list(T)
                    else:
                        data['T_cable_curr'] = []
                    
                    # Fonction locale pour construire et émettre update_data à partir du contexte
                    def build_and_emit_ui_update(ctx, data_dict, plot_limit, fx_cmd=None):
                        """Construit update_data à partir du contexte partagé et émet vers l'UI"""
                        t_ctx = ctx.get('t', 0.0)
                        L_ctx = ctx.get('L', 0.0)
                        x_cable_ctx = ctx.get('x_cable')
                        y_cable_ctx = ctx.get('y_cable')
                        T_ctx = ctx.get('T')
                        
                        # Construire update_data
                        ui_update = {
                            'current_time': t_ctx,
                        }
                        # Utiliser L_ctx du contexte partagé pour garantir la cohérence avec les invariants
                        ui_update['L_step'] = float(L_ctx)
                        
                        # Ajouter les dernières valeurs pour les métriques et graphiques
                        for key, value in data_dict.items():
                            if isinstance(value, list) and len(value) > 0:
                                # Pour les graphiques, garder seulement les N derniers points
                                if key in ['time', 'x_rov', 'y_rov', 'vx_rov', 'vy_rov', 'x_boat', 'vx_boat',
                                          'L', 'L_seg', 'D_straight', 'T_rov', 'T_boat', 'T_max',
                                          'Fx_drag_rov', 'Fy_drag_rov', 'Fx_rov_total', 'Fy_rov_total',
                                          'Fx_traction_boat', 'Fy_traction_boat', 'Fx_traction_rov', 'Fy_traction_rov',
                                          'F_prop_boat', 'Fx_total_boat', 'Fy_total_boat',
                                          'Fx_drag_cable', 'Fy_drag_cable', 'Fy_cable_app_w',
                                          'Fx_drag_cable_longitudinal', 'Fy_drag_cable_longitudinal',
                                          'Fx_drag_cable_perpendicular', 'Fy_drag_cable_perpendicular',
                                          'Fx_cmd_rov', 'Fy_cmd_rov', 'dl_dt_cmd', 'dl_dt_auto_explain',
                                          'Fy_rov_app_w', 'F_buoyancy_net', 'angle_rov', 'angle_boat',
                                          'cable_mode', 'scenario_triggers']:
                                    if len(value) > plot_limit:
                                        ui_update[key] = value[-plot_limit:]
                                    else:
                                        ui_update[key] = value
                                else:
                                    # Pour les autres listes, garder toutes les valeurs
                                    ui_update[key] = value
                            elif key in ['x_cable_curr', 'y_cable_curr', 'T_cable_curr']:
                                # Ne pas utiliser data['x_cable_curr'] ici, on utilisera les données du contexte plus tard
                                # pour garantir la cohérence avec les invariants
                                pass
                            else:
                                # Autres valeurs (scalaires, None, etc.)
                                ui_update[key] = value
                        
                        # Utiliser dl_dt_cmd du contexte partagé pour garantir la cohérence avec les invariants
                        dl_dt_cmd_ctx = ctx.get('dl_dt_cmd')
                        if dl_dt_cmd_ctx is not None and 'dl_dt_cmd' in ui_update:
                            # Remplacer la dernière valeur par celle du contexte partagé
                            if isinstance(ui_update['dl_dt_cmd'], list) and len(ui_update['dl_dt_cmd']) > 0:
                                ui_update['dl_dt_cmd'][-1] = float(dl_dt_cmd_ctx)
                        
                        # Ajouter les données du câble actuelles depuis le contexte (toujours utiliser le contexte pour garantir la cohérence)
                        if x_cable_ctx is not None and y_cable_ctx is not None:
                            x_cable_list = x_cable_ctx.tolist() if hasattr(x_cable_ctx, 'tolist') else list(x_cable_ctx)
                            y_cable_list = y_cable_ctx.tolist() if hasattr(y_cable_ctx, 'tolist') else list(y_cable_ctx)
                            ui_update['x_cable_curr'] = x_cable_list
                            ui_update['y_cable_curr'] = y_cable_list
                            ui_update['cable_point_indices'] = list(range(len(x_cable_list)))
                        else:
                            # Fallback : utiliser data['x_cable_curr'] si le contexte n'est pas disponible
                            if 'x_cable_curr' in data_dict and 'y_cable_curr' in data_dict:
                                ui_update['x_cable_curr'] = list(data_dict['x_cable_curr']) if isinstance(data_dict['x_cable_curr'], list) else data_dict['x_cable_curr']
                                ui_update['y_cable_curr'] = list(data_dict['y_cable_curr']) if isinstance(data_dict['y_cable_curr'], list) else data_dict['y_cable_curr']
                                if 'cable_point_indices' in data_dict:
                                    ui_update['cable_point_indices'] = list(data_dict['cable_point_indices']) if isinstance(data_dict['cable_point_indices'], list) else data_dict['cable_point_indices']
                        
                        if T_ctx is not None and len(T_ctx) > 0:
                            ui_update['T_cable_curr'] = T_ctx.tolist() if hasattr(T_ctx, 'tolist') else list(T_ctx)
                        else:
                            # Fallback : utiliser data['T_cable_curr'] si le contexte n'est pas disponible
                            if 'T_cable_curr' in data_dict:
                                ui_update['T_cable_curr'] = list(data_dict['T_cable_curr']) if isinstance(data_dict['T_cable_curr'], list) else data_dict['T_cable_curr']
                            else:
                                ui_update['T_cable_curr'] = []
                        
                        # Ajouter fx_rov_cmd si fourni
                        if fx_cmd is not None:
                            ui_update['fx_rov_cmd'] = float(fx_cmd)
                            ui_update['fx_rov_cmd_source'] = "scenario"
                        
                        # Émettre vers l'UI
                        self.simulation_updated.emit(ui_update)
                    
                    # Vérifier les invariants câble en fin d'itération, avec le même
                    # snapshot que celui utilisé pour remplir data / update_data.
                    # IMPORTANT : Appeler APRÈS toutes les corrections de tension et
                    # JUSTE AVANT l'émission vers l'UI pour garantir la cohérence.
                    check_cable_invariants(shared_context)
                    
                    # Émettre les données vers l'IHM APRÈS la vérification des invariants,
                    # en utilisant exactement le même contexte pour garantir la cohérence.
                    # Ne rafraîchir l'UI que toutes les N itérations pour éviter de surcharger l'interface
                    if step_count % steps_per_update == 0:
                        build_and_emit_ui_update(shared_context, data, plot_points_limit, fx_rov_cmd)
                        # Petit délai pour ne pas surcharger l'interface
                        time.sleep(0.01)

                    # Traînée câble (vectorielle, sans poids apparent)
                    try:
                        from src.solvers.forces import compute_cable_forces, compute_cable_apparent_weight
                        if x_cable_display is not None and y_cable_display is not None and len(x_cable_display) > 1:
                            x_cable_arr = np.asarray(x_cable_display)
                            y_cable_arr = np.asarray(y_cable_display)
                            vx_cable = np.linspace(vx_rov, vx_boat, len(x_cable_arr))
                            vy_cable = np.linspace(vy_rov, 0.0, len(x_cable_arr))
                            params_cable = {
                                'd': system.cable.d,
                                'rho_cable': system.cable.rho_cable,
                                'Cx_cable': system.cable.Cx_cable,
                                'Cf_cable': system.cable.Cf_cable
                            }
                            Fx_segments, Fy_segments, Fx_long_seg, Fy_long_seg, Fx_perp_seg, Fy_perp_seg = compute_cable_forces(
                                x_cable_arr, y_cable_arr, vx_cable, vy_cable, system.environment, params_cable, L
                            )
                            F_drag_x = float(np.sum(Fx_segments)) if len(Fx_segments) > 0 else 0.0
                            N_segments = max(len(x_cable_arr) - 1, 1)
                            ds = L / N_segments if N_segments > 0 else L
                            Fy_weight_seg = compute_cable_apparent_weight(
                                system.cable.rho_cable,
                                system.environment.rho_eau,
                                system.cable.A_cable,
                                system.environment.g,
                                ds
                            )
                            F_weight_total = Fy_weight_seg * N_segments
                            F_drag_y = (float(np.sum(Fy_segments)) - F_weight_total) if len(Fy_segments) > 0 else 0.0
                            
                            # Traînées décomposées
                            F_drag_long_x = float(np.sum(Fx_long_seg)) if len(Fx_long_seg) > 0 else 0.0
                            F_drag_long_y = float(np.sum(Fy_long_seg)) if len(Fy_long_seg) > 0 else 0.0
                            F_drag_perp_x = float(np.sum(Fx_perp_seg)) if len(Fx_perp_seg) > 0 else 0.0
                            F_drag_perp_y = float(np.sum(Fy_perp_seg)) if len(Fy_perp_seg) > 0 else 0.0
                            
                            data.setdefault('cable_drag', []).append((F_drag_x, F_drag_y))
                            data.setdefault('Fx_drag_cable', []).append(float(F_drag_x))
                            data.setdefault('Fy_drag_cable', []).append(float(F_drag_y))
                            data.setdefault('Fy_cable_app_w', []).append(float(F_weight_total))
                            data.setdefault('Fx_drag_cable_longitudinal', []).append(float(F_drag_long_x))
                            data.setdefault('Fy_drag_cable_longitudinal', []).append(float(F_drag_long_y))
                            data.setdefault('Fx_drag_cable_perpendicular', []).append(float(F_drag_perp_x))
                            data.setdefault('Fy_drag_cable_perpendicular', []).append(float(F_drag_perp_y))
                        else:
                            data.setdefault('cable_drag', []).append((0.0, 0.0))
                            data.setdefault('Fx_drag_cable', []).append(0.0)
                            data.setdefault('Fy_drag_cable', []).append(0.0)
                            data.setdefault('Fy_cable_app_w', []).append(0.0)
                            data.setdefault('Fx_drag_cable_longitudinal', []).append(0.0)
                            data.setdefault('Fy_drag_cable_longitudinal', []).append(0.0)
                            data.setdefault('Fx_drag_cable_perpendicular', []).append(0.0)
                            data.setdefault('Fy_drag_cable_perpendicular', []).append(0.0)
                    except Exception:
                        data.setdefault('cable_drag', []).append((0.0, 0.0))
                        data.setdefault('Fx_drag_cable', []).append(0.0)
                        data.setdefault('Fy_drag_cable', []).append(0.0)
                        data.setdefault('Fy_cable_app_w', []).append(0.0)
                        data.setdefault('Fx_drag_cable_longitudinal', []).append(0.0)
                        data.setdefault('Fy_drag_cable_longitudinal', []).append(0.0)
                        data.setdefault('Fx_drag_cable_perpendicular', []).append(0.0)
                        data.setdefault('Fy_drag_cable_perpendicular', []).append(0.0)
                    
                    # Calculer les forces sur le bateau
                    # Tension du câble au niveau du bateau
                    # T_boat est déjà calculé et défini plus haut
                    
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
                    # T_boat est déjà calculé et défini plus haut
                    Fx_traction_boat = -T_boat * Ubateau_x
                    Fy_traction_boat = -T_boat * Ubateau_y
                    
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
                    
                    # Note: data['x_cable_curr'], data['y_cable_curr'], et data['T_cable_curr'] 
                    # sont maintenant mis à jour juste après la construction de shared_context
                    # pour garantir la cohérence avec les données utilisées par l'UI et les invariants

                    # Sauvegarder la ligne dans le fichier Excel data (seulement toutes les N itérations)
                    excel_write_interval = int(self.calc_params.get('excel_write_interval', 50))
                    if data_excel_ws is not None and data_excel_headers is not None and step_count % excel_write_interval == 0:
                        try:
                            row_values = extract_data_row(data)
                            for col_idx, header in enumerate(data_excel_headers, 1):
                                value = row_values.get(header)
                                data_excel_ws.cell(row=data_excel_row, column=col_idx, value=value)
                            
                            data_excel_row += 1
                            
                            # Sauvegarder périodiquement (toutes les 50 itérations)
                            if step_count % 50 == 0 and data_excel_path:
                                data_excel_wb.save(data_excel_path)
                        except Exception as e:
                            trace_print(8, f"[DATA EXCEL] Erreur écriture ligne {data_excel_row}: {e}")
                    
                    step_count += 1
                    
                    # Incrémenter le temps à la toute fin de l'itération pour que toutes les opérations
                    # (auto_L_7, intégration, vérification invariants) de l'itération en cours
                    # utilisent le même temps, et que la prochaine itération utilise le nouveau temps
                    t_current = t_next
                    
                    # Limiter la taille des données accumulées (toutes les 1000 itérations)
                    max_data_size = int(self.calc_params.get('max_data_size', 10000))
                    if step_count % 1000 == 0:
                        trim_data(data, max_data_size)
                    
                except Exception as e:
                    self.error_occurred.emit(f"Erreur lors de l'intégration: {str(e)}")
                    import traceback
                    traceback.print_exc()
                    break
            
            # Émettre les données finales (optimisées)
            if not self._stop_requested:
                plot_points_limit = int(self.calc_params.get('plot_points_limit', 1000))
                
                update_data = {
                    'current_time': t_current,
                    'L_step': float(L),
                }
                
                # Ajouter les dernières valeurs pour les métriques
                for key, value in data.items():
                    if isinstance(value, list) and len(value) > 0:
                        # Pour les graphiques, garder seulement les N derniers points
                        if key in ['time', 'x_rov', 'y_rov', 'vx_rov', 'vy_rov', 'x_boat', 'vx_boat',
                                  'L', 'L_seg', 'D_straight', 'T_rov', 'T_boat', 'T_max',
                                  'Fx_drag_rov', 'Fy_drag_rov', 'Fx_rov_total', 'Fy_rov_total',
                                  'Fx_traction_boat', 'Fy_traction_boat', 'Fx_traction_rov', 'Fy_traction_rov',
                                  'F_prop_boat', 'Fx_total_boat', 'Fy_total_boat',
                                  'Fx_drag_cable', 'Fy_drag_cable', 'Fy_cable_app_w',
                                  'Fx_drag_cable_longitudinal', 'Fy_drag_cable_longitudinal',
                                  'Fx_drag_cable_perpendicular', 'Fy_drag_cable_perpendicular',
                                  'Fx_cmd_rov', 'Fy_cmd_rov', 'dl_dt_cmd', 'dl_dt_auto_explain',
                                  'Fy_rov_app_w', 'F_buoyancy_net', 'angle_rov', 'angle_boat',
                                  'cable_mode', 'scenario_triggers']:
                            # Limiter à plot_points_limit points pour les graphiques
                            if len(value) > plot_points_limit:
                                update_data[key] = value[-plot_points_limit:]
                            else:
                                update_data[key] = value
                        else:
                            # Pour les autres listes, garder toutes les valeurs
                            update_data[key] = value
                    elif key in ['x_cable_curr', 'y_cable_curr', 'T_cable_curr']:
                        # Les données du câble actuelles sont toujours incluses
                        update_data[key] = value
                    else:
                        # Autres valeurs (scalaires, None, etc.)
                        update_data[key] = value
                
                if fx_rov_cmd is not None:
                    update_data['fx_rov_cmd'] = float(fx_rov_cmd)
                    update_data['fx_rov_cmd_source'] = "scenario"
                self.simulation_updated.emit(update_data)
                self.simulation_finished.emit()

            # Sauvegarder le fichier Excel data final
            if data_excel_wb is not None and data_excel_path:
                try:
                    data_excel_wb.save(data_excel_path)
                    trace_print(1, f"[DATA EXCEL] Fichier sauvegarde: {data_excel_path}")
                except Exception as e:
                    trace_print(8, f"[DATA EXCEL] Erreur sauvegarde finale: {e}")

            close_trace_file()
            
        except Exception as e:
            self.error_occurred.emit(f"Erreur dans la simulation: {str(e)}")
            import traceback
            traceback.print_exc()
            if 'trace_handle' in locals() and trace_handle is not None:
                trace_handle.close()
            close_trace_file()