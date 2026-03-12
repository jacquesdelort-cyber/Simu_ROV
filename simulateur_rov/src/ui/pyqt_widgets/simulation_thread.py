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
from src.utils.logger import trace_print, set_trace_file, close_trace_file

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

                    tol_L_rel = 1e-3
                    tol_recol = 1e-3
                    tol_slack_neg = 1e-3

                    violations = []
                    details = {}

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
                            details["ds_first"] = ds_list[0]
                            details["ds_last"] = ds_list[-1]

                            # Contrôle local : aucun segment ne doit être trop long
                            # par rapport à la longueur moyenne ds_target = L_c / N.
                            N_seg = len(ds_list)
                            if N_seg > 0:
                                ds_target = L_c_val / N_seg
                                k_max = 2.0
                                if ds_target > 0.0 and ds_max > k_max * ds_target:
                                    i_max = int(np.argmax(ds_list))
                                    ratio = ds_max / ds_target
                                    violations.append(
                                        f"segment_too_long(i={i_max},ratio={ratio:.3f})"
                                    )
                                    details["ds_max_ratio"] = float(ratio)
                                    details["i_max_segment"] = i_max

                    if dist_boat > tol_recol:
                        violations.append(f"recollement_bateau(dist={dist_boat:.3e})")
                    if dist_rov > tol_recol:
                        violations.append(f"recollement_rov(dist={dist_rov:.3e})")

                    if slack < -tol_slack_neg:
                        violations.append(f"slack_neg(slack={slack:.3e})")

                    if not violations:
                        return

                    mission_name = getattr(system.environment, "mission_name", None)
                    dl_dt_str = (
                        f"{float(dl_dt_cmd_val):.6f}"
                        if isinstance(dl_dt_cmd_val, (int, float))
                        else "None"
                    )
                    T_boat_val = float(T_c[0]) if T_c is not None and len(T_c) > 0 else 0.0
                    T_rov_val = float(T_c[-1]) if T_c is not None and len(T_c) > 0 else 0.0

                    base_prefix = "[CABLE INVARIANTS] VIOLATION "
                    if t_chk is not None:
                        base_prefix += f"phase={phase} t={t_chk:.3f} "
                    else:
                        base_prefix += f"phase={phase} "

                    if step is not None:
                        base_prefix += f"step={step} "

                    message = (
                        base_prefix
                        + f"violations={','.join(violations)} "
                        + f"L={float(L_c):.6f} L_seg={L_seg:.6f} "
                        + f"D_straight={D_straight:.6f} slack={slack:.6f} "
                        + f"dist_boat={dist_boat:.3e} dist_rov={dist_rov:.3e} "
                        + f"T_boat={T_boat_val:.3f} T_rov={T_rov_val:.3f} "
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
                            f"mission={mission_name}" if mission_name is not None else ""
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
                            if "ds_max_ratio" in details and "i_max_segment" in details
                            else ""
                        )
                    )

                    trace_print(9, message)
                except Exception as e:
                    trace_print(8, f"[CABLE INVARIANTS] Erreur verification: {e}")
            
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

            # Vérifier les invariants du câble à la fin de la phase d'initialisation
            check_cable_invariants({
                'phase': 'initialisation',
                't': t_current,
                'step': 0,
                'y': y_current,
            })
            
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
                dl_dt_auto_explain = ""
                dl_dt_mode = "scen"
                if isinstance(self.simulation_state, dict):
                    dl_dt_mode = self.simulation_state.get('dl_dt_mode', 'scen')

                if self.sc_fx_rov or self.sc_fy_rov or self.sc_v_bateau or self.sc_v_moulinet or dl_dt_mode == "auto":
                    try:
                        (x_rov_current, y_rov_current, _, _, x_boat_current, _, _, _, T_current, L_current) = system.unpack_state(y_current)
                        # DEBUG: Traçage de L_current avant intégration
                        trace_print(5, f"[DEBUG L] t={t_current:.2f} AVANT intégration: L_current={L_current:.6f} m")
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
                            trace_print(8, f"[DEBUG L] t={t_current:.2f} dL_dt_cmd={dl_dt_cmd:.6f} m/s "
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
                        trace_print(8, f"Erreur commande_scenario: {e}")
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

                # Mise à jour de l'interface graphique juste après l'appel à auto_L_7 (ou autres commandes)
                # Utiliser les données de l'itération précédente (déjà dans data)
                # pour correspondre aux valeurs utilisées dans auto_L_7
                steps_per_update = int(self.calc_params.get('steps_per_update', 5))
                if step_count % steps_per_update == 0 and len(data.get('time', [])) > 0:
                    # Préparer les données optimisées pour l'UI
                    plot_points_limit = int(self.calc_params.get('plot_points_limit', 1000))
                    
                    # Utiliser L_current (avant intégration) pour correspondre aux traces dans auto_L_7
                    update_data = {
                        'current_time': t_current,
                        'L_step': float(L_current),  # Utiliser L_current (avant intégration)
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
                    
                    # Petit délai pour ne pas surcharger l'interface
                    time.sleep(0.01)

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

                # Calculer le pas suivant
                t_next = min(t_current + dt, t_final)
                # Stocker dt pour le débogage
                dt_used = t_next - t_current
                
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
                    
                    t_current = t_next
                    
                    # Décoder l'état
                    (x_rov, y_rov, vx_rov, vy_rov,
                     x_boat, vx_boat, x_cable, y_cable, T, L) = system.unpack_state(y_current)

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
                    
                    # Pour l'affichage, privilégier la géométrie d'équilibre calculée par le solveur.
                    # On travaille sur des copies, puis on resynchronise explicitement l'état
                    # de l'itération pour garantir le recollement du point N avec le ROV.
                    x_cable_display = np.asarray(x_cable, dtype=float).copy()
                    y_cable_display = np.asarray(y_cable, dtype=float).copy()
                    if getattr(system, 'x_cable_prev', None) is not None and getattr(system, 'y_cable_prev', None) is not None:
                        if len(system.x_cable_prev) == len(x_cable) and len(system.y_cable_prev) == len(y_cable):
                            x_cable_display = np.asarray(system.x_cable_prev, dtype=float).copy()
                            y_cable_display = np.asarray(system.y_cable_prev, dtype=float).copy()
                    
                    # Garantir l'ordre bateau -> ROV pour les directions/tractions affichées
                    if x_cable_display is not None and y_cable_display is not None and len(x_cable_display) > 1:
                        dist_first_to_boat = np.sqrt((x_cable_display[0] - x_boat)**2 + (y_cable_display[0] - 0.0)**2)
                        dist_first_to_rov = np.sqrt((x_cable_display[0] - x_rov)**2 + (y_cable_display[0] - y_rov)**2)
                        if dist_first_to_rov < dist_first_to_boat:
                            x_cable_display = np.flip(x_cable_display)
                            y_cable_display = np.flip(y_cable_display)
                    
                    # IMPORTANT : Ne PAS forcer une ligne droite quand le câble est tendu
                    # Le ROV n'est plus recalculé, donc le câble peut avoir une forme de caténaire
                    # même si slack < 0. La tension au ROV sera ajustée pour ramener slack >= 0
                    
                    # Normaliser systématiquement l'affichage par rapport à L
                    # pour garantir que la géométrie utilisée par l'IHM et par
                    # check_cable_invariants reste cohérente avec la longueur scalaire L.
                    if x_cable_display is not None and y_cable_display is not None and len(x_cable_display) > 1:
                        try:
                            x_cable_display, y_cable_display = system.cable.solver._normalize_cable_length(
                                x_cable_display, y_cable_display, L
                            )
                        except Exception:
                            pass
                        
                        # CORRECTION : Toujours forcer les extrémités à correspondre exactement
                        if len(x_cable_display) > 0:
                            x_cable_display[0] = x_boat
                            y_cable_display[0] = 0.0
                            x_cable_display[-1] = x_rov
                            y_cable_display[-1] = y_rov
                            
                            # RENORMALISER après forçage des extrémités pour garantir
                            # que tous les segments conservent la même longueur ds_target = L / N
                            # Le forçage des extrémités peut modifier la longueur du premier
                            # et/ou du dernier segment, donc on renormalise pour préserver
                            # l'uniformité des segments tout en respectant les contraintes d'extrémités
                            try:
                                x_cable_display, y_cable_display = system.cable.solver._normalize_cable_length(
                                    x_cable_display, y_cable_display, L
                                )
                                # Réappliquer le forçage des extrémités après normalisation
                                # (la normalisation devrait déjà les préserver, mais on s'assure)
                                x_cable_display[0] = x_boat
                                y_cable_display[0] = 0.0
                                x_cable_display[-1] = x_rov
                                y_cable_display[-1] = y_rov
                            except Exception as e:
                                trace_print(8, f"[WARN] Échec renormalisation après forçage extrémités: {e}")

                        # Réinjecter la géométrie corrigée dans l'état de l'itération.
                        # Sans cette resynchronisation, l'IHM peut parfois afficher une
                        # géométrie dont le dernier point n'est pas exactement recollé au ROV.
                        idx_x_cable = 6
                        idx_y_cable = idx_x_cable + system.N + 1
                        y_current[idx_x_cable:idx_y_cable] = x_cable_display
                        y_current[idx_y_cable:idx_y_cable + system.N + 1] = y_cable_display
                        system.x_cable_prev = np.asarray(x_cable_display, dtype=float).copy()
                        system.y_cable_prev = np.asarray(y_cable_display, dtype=float).copy()

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

                    # Vérifier les invariants câble en fin d'itération
                    check_cable_invariants({
                        'phase': 'iteration',
                        't': t_current,
                        'step': step_count,
                        'y': y_current,
                        'dl_dt_cmd': dl_dt_cmd,
                        'dl_dt_explain': dl_dt_auto_explain,
                        'cable_mode': cable_mode,
                        'triggers': step_triggers,
                    })

                    # Stocker les données
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
                    if isinstance(self.simulation_state, dict):
                        data['dl_dt_cmd'].append(float(self.simulation_state.get('dl_dt', self.dl_dt)))
                        if dl_dt_mode == "auto":
                            data['dl_dt_auto_explain'].append(dl_dt_auto_explain)
                        else:
                            data['dl_dt_auto_explain'].append("")
                    data.setdefault('cable_mode', []).append(cable_mode)
                    data.setdefault('scenario_triggers', []).append(step_triggers)
                    
                    # Tensions
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
                    T_rov_val = T_rov if T is not None and len(T) > 0 else 0.0
                    Fx_traction = -T_rov_val * Urov_x  # Horizontal : signe inchangé
                    Fy_traction = T_rov_val * (-Urov_y)  # Vertical : inversé pour positif = vers le haut
                    
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
                        x_cable_list = x_cable_display.tolist() if hasattr(x_cable_display, 'tolist') else list(x_cable_display)
                        y_cable_list = y_cable_display.tolist() if hasattr(y_cable_display, 'tolist') else list(y_cable_display)
                        data['x_cable_curr'] = x_cable_list
                        data['y_cable_curr'] = y_cable_list
                        # Indices globaux des points du câble (ordre bateau -> ROV)
                        data['cable_point_indices'] = list(range(len(x_cable_list)))
                    
                    # Tensions du câble actuel
                    if T is not None and len(T) > 0:
                        data['T_cable_curr'] = T.tolist() if hasattr(T, 'tolist') else list(T)
                    else:
                        data['T_cable_curr'] = []

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