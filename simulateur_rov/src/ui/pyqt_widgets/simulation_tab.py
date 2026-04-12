"""
Onglet de simulation avec contrôles et visualisations
"""
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                             QLabel, QGroupBox, QScrollArea, QGridLayout,
                             QMessageBox, QProgressBar, QDoubleSpinBox, QSpinBox, QToolButton, QApplication,
                             QTabWidget, QSizePolicy, QTextEdit)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QPoint
from PyQt6.QtGui import QFont, QCursor
import sys
import os

# Ajouter le répertoire parent au path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

from .plotly_widget import PlotlyWidget
from .simulation_thread import SimulationThread
from src.utils.logger import trace_print, set_trace_level
from src.ui.mission_utils import get_missions_directory
from datetime import datetime
import html
import csv


class CustomToolTip(QLabel):
    """Tooltip personnalisé avec fond couleur sable et texte gris"""
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setWindowFlags(Qt.WindowType.ToolTip | Qt.WindowType.FramelessWindowHint)
        self.setStyleSheet("""
            QLabel {
                background-color: #f4e4bc;
                color: #555;
                border: 1px solid #d4c4a4;
                padding: 5px;
                border-radius: 3px;
                font-size: 10pt;
            }
        """)
        self.setWordWrap(True)
        self.setMaximumWidth(300)
        self.adjustSize()
        self.hide()


class HelpButton(QToolButton):
    """Bouton d'aide avec tooltip personnalisé"""
    def __init__(self, tooltip_text, parent=None):
        super().__init__(parent)
        self.setText("?")
        self.setStyleSheet("font-size: 11pt; padding: 2px; color: #666; font-weight: normal; border: 1px solid #666; border-radius: 10px; background-color: transparent;")
        self.setFixedSize(18, 18)
        self.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.tooltip = CustomToolTip(tooltip_text, self)
    
    def enterEvent(self, event):
        """Affiche le tooltip au survol"""
        pos = self.mapToGlobal(QPoint(self.width() + 5, 0))
        self.tooltip.move(pos)
        self.tooltip.show()
        super().enterEvent(event)
    
    def leaveEvent(self, event):
        """Cache le tooltip quand la souris quitte"""
        self.tooltip.hide()
        super().leaveEvent(event)


class SimulationTab(QWidget):
    """Onglet de simulation"""
    
    # Signaux pour communication avec le thread
    simulation_started = pyqtSignal()
    simulation_stopped = pyqtSignal()
    simulation_updated = pyqtSignal(dict)  # Émet l'état de la simulation
    
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.simulation_thread = None
        self.update_timer = None
        self.last_plot_update_time = None  # Pour limiter les rafraîchissements du graphique
        self.current_x_range = None  # Plage actuelle de l'axe X [x_min, x_max]
        self.current_y_range = None  # Plage actuelle de l'axe Y [y_min, y_max]
        self.current_s_range = None  # Plage actuelle de l'axe S (abscisse curviligne) [s_min, s_max]
        self.current_T_range = None  # Plage actuelle de l'axe T (tension) [T_min, T_max]
        self._fx_rov_scenario_update = False
        self._fy_rov_scenario_update = False
        self._vx_boat_scenario_update = False
        self._dl_dt_scenario_update = False
        self._dl_dt_auto_update = False
        self._prev_cable_time = None
        self._prev_cable_x = None
        self._prev_cable_y = None
        self._prev_cable_drag = None
        self._mission_end_latched = False
        self._scenario_last_triggers = {}
        self._scenario_raw_text = {}
        self._scenario_trigger_index = 0
        
        self.init_ui()
        
        # Mettre à jour l'affichage de la mission après l'initialisation
        QTimer.singleShot(0, self.update_mission_display)
        
        # Initialiser le système automatiquement au démarrage
        QTimer.singleShot(100, self.initialize_system)
        
        # Timer pour mettre à jour l'affichage
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(200)  # Mise à jour toutes les 200ms
    
    def init_ui(self):
        """Initialise l'interface de l'onglet"""
        layout = QHBoxLayout(self)
        layout.setSpacing(10)
        
        # Colonne gauche : Contrôles
        left_panel = self.create_control_panel()
        layout.addWidget(left_panel, 1)  # 1/4 de l'espace
        
        # Colonne centrale : Visualisation principale
        center_panel = self.create_visualization_panel()
        layout.addWidget(center_panel, 2)  # 1/2 de l'espace
        
        # Colonne droite : Métriques + Profil câble
        right_panel = self.create_right_panel()
        layout.addWidget(right_panel, 1)  # 1/4 de l'espace
    
    def create_control_panel(self):
        """Crée le panneau de contrôles"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Groupe : Mission en cours
        mission_group = QGroupBox("Mission")
        mission_layout = QHBoxLayout(mission_group)
        
        # Champ d'affichage de la mission
        self.mission_label = QLabel("Aucune mission sélectionnée")
        self.mission_label.setStyleSheet("padding: 5px; background-color: #f8f9fa; border: 1px solid #dee2e6; border-radius: 3px;")
        mission_layout.addWidget(self.mission_label)
        
        # Emoji d'aide
        mission_help = HelpButton("Vous pouvez sélectionner ou créer une mission dans l'onglet \"Paramètres\".\n"
                                 "Chaque mission possède son propre répertoire avec ses paramètres.", self)
        mission_layout.addWidget(mission_help, 0)
        
        layout.addWidget(mission_group)
        
        # Groupe : Variables de commande
        commands_group = QGroupBox("Variables de commande")
        commands_layout = QGridLayout(commands_group)
        
        # Force horizontale ROV
        commands_layout.addWidget(QLabel("<b>Fx ROV (N):</b>"), 0, 0)
        self.fx_rov_input = QDoubleSpinBox()
        self.fx_rov_input.setRange(-500.0, 500.0)
        self.fx_rov_input.setSingleStep(1.0)
        self.fx_rov_input.setDecimals(2)
        self.fx_rov_input.setValue(0.0)
        self.fx_rov_input.setSuffix(" N")
        # Connecter le signal pour mettre à jour simulation_state
        self.fx_rov_input.valueChanged.connect(self.update_command_in_state)
        commands_layout.addWidget(self.fx_rov_input, 0, 1)
        
        # Force verticale ROV
        commands_layout.addWidget(QLabel("<b>Fy ROV (N):</b>"), 1, 0)
        self.fy_rov_input = QDoubleSpinBox()
        self.fy_rov_input.setRange(-500.0, 500.0)
        self.fy_rov_input.setSingleStep(1.0)
        self.fy_rov_input.setDecimals(2)
        self.fy_rov_input.setValue(0.0)
        self.fy_rov_input.setSuffix(" N")
        # Connecter le signal pour mettre à jour simulation_state
        self.fy_rov_input.valueChanged.connect(self.update_command_in_state)
        commands_layout.addWidget(self.fy_rov_input, 1, 1)
        
        # Vitesse commande bateau
        commands_layout.addWidget(QLabel("<b>vx Bateau (m/s):</b>"), 2, 0)
        self.vx_boat_cmd_input = QDoubleSpinBox()
        self.vx_boat_cmd_input.setRange(-2.0, 2.0)
        self.vx_boat_cmd_input.setSingleStep(0.1)
        self.vx_boat_cmd_input.setDecimals(2)
        self.vx_boat_cmd_input.setValue(0.0)
        self.vx_boat_cmd_input.setSuffix(" m/s")
        # Connecter le signal pour mettre à jour simulation_state
        self.vx_boat_cmd_input.valueChanged.connect(self.update_command_in_state)
        commands_layout.addWidget(self.vx_boat_cmd_input, 2, 1)
        
        # Vitesse déroulement câble
        commands_layout.addWidget(QLabel("<b>dL/dt (m/s):</b>"), 3, 0)
        self.dl_dt_input = QDoubleSpinBox()
        # La plage sera mise à jour dynamiquement via update_dl_dt_range()
        self.dl_dt_input.setRange(-1.0, 1.0)  # Valeur par défaut
        self.dl_dt_input.setSingleStep(0.1)
        self.dl_dt_input.setDecimals(2)
        self.dl_dt_input.setValue(0.0)
        self.dl_dt_input.setSuffix(" m/s")
        # Connecter le signal pour mettre à jour simulation_state
        self.dl_dt_input.valueChanged.connect(self.update_command_in_state)
        commands_layout.addWidget(self.dl_dt_input, 3, 1)

        self.dl_dt_mode_btn = QPushButton("Auto")
        self.dl_dt_mode_btn.setCheckable(True)
        self.dl_dt_mode_btn.setChecked(True)
        self.dl_dt_mode_btn.toggled.connect(self.on_dl_dt_mode_toggled)
        commands_layout.addWidget(self.dl_dt_mode_btn, 3, 2)
        state = self.main_window.get_simulation_state()
        state['dl_dt_mode'] = "auto"
        self.main_window.update_simulation_state(state)
        
        layout.addWidget(commands_group)
        
        # Groupe : Commandes de simulation
        controls_group = QGroupBox("Commandes de simulation")
        controls_layout = QVBoxLayout(controls_group)
        
        # Démarrer simulation
        start_row = QHBoxLayout()
        self.btn_start = QPushButton("▶ Démarrer simulation")
        self.btn_start.setStyleSheet("background-color: #28a745; color: white; font-weight: bold; padding: 8px;")
        self.btn_start.clicked.connect(self.start_simulation)
        start_row.addWidget(self.btn_start)
        start_help = HelpButton("Démarre une nouvelle simulation avec les paramètres et variables de commande actuels.\n"
                               "La simulation s'exécute en arrière-plan et peut être mise en pause ou arrêtée.", self)
        start_row.addWidget(start_help, 0)
        controls_layout.addLayout(start_row)
        
        # Pause
        pause_row = QHBoxLayout()
        self.btn_pause = QPushButton("⏸ Pause")
        self.btn_pause.setStyleSheet("background-color: #ffc107; color: black; padding: 8px;")
        self.btn_pause.setEnabled(False)
        self.btn_pause.clicked.connect(self.pause_resume_simulation)
        pause_row.addWidget(self.btn_pause)
        pause_help = HelpButton("Met en pause ou reprend la simulation en cours.\n"
                               "En pause, vous pouvez examiner les résultats sans que la simulation continue.", self)
        pause_row.addWidget(pause_help, 0)
        controls_layout.addLayout(pause_row)
        
        # Réinitialiser
        restart_row = QHBoxLayout()
        self.btn_restart = QPushButton("🔄 Réinitialiser")
        self.btn_restart.setStyleSheet("background-color: #17a2b8; color: white; padding: 8px;")
        self.btn_restart.setEnabled(True)
        self.btn_restart.clicked.connect(self.restart_simulation)
        restart_row.addWidget(self.btn_restart)
        restart_help = HelpButton("Réinitialise la simulation.\n"
                                 "Les données précédentes sont effacées et les commandes sont remises à 0.", self)
        restart_row.addWidget(restart_help, 0)
        controls_layout.addLayout(restart_row)
        
        # Analyse mission
        export_row = QHBoxLayout()
        self.btn_export = QPushButton("📊 Analyse mission")
        self.btn_export.setStyleSheet("background-color: #d4edda; color: #155724; padding: 8px;")
        self.btn_export.setEnabled(True)
        self.btn_export.clicked.connect(self.analyse_mission)
        export_row.addWidget(self.btn_export)
        export_help = HelpButton(
            "Analyse les fichiers trace de la mission et génère un rapport Markdown.\n"
            "Disponible uniquement après la fin de la simulation.",
            self,
        )
        export_row.addWidget(export_help, 0)
        controls_layout.addLayout(export_row)

        # Rafraîchir graphiques
        refresh_row = QHBoxLayout()
        self.btn_refresh_graphs = QPushButton("🔄 Rafraîchir graphiques")
        self.btn_refresh_graphs.setStyleSheet("background-color: #6c757d; color: white; padding: 8px;")
        self.btn_refresh_graphs.setEnabled(False)
        self.btn_refresh_graphs.clicked.connect(self.refresh_graphs)
        refresh_row.addWidget(self.btn_refresh_graphs)
        refresh_help = HelpButton("Force le rafraîchissement de tous les graphiques.\n"
                                  "Utile si un graphique ne se met pas à jour.", self)
        refresh_row.addWidget(refresh_help, 0)
        controls_layout.addLayout(refresh_row)

        # Niveau de trace
        trace_row = QHBoxLayout()
        trace_label = QLabel("Niveau trace")
        self.trace_level_input = QSpinBox()
        self.trace_level_input.setRange(0, 10)
        self.trace_level_input.setSingleStep(1)
        self.trace_level_input.setValue(9)
        set_trace_level(9)
        self.trace_level_input.valueChanged.connect(lambda v: set_trace_level(int(v)))
        trace_row.addWidget(trace_label)
        trace_row.addWidget(self.trace_level_input)
        trace_help = HelpButton("Définit le niveau global de trace.\n"
                                "Un message s'affiche si son niveau >= niveau trace.", self)
        trace_row.addWidget(trace_help, 0)
        controls_layout.addLayout(trace_row)
        
        layout.addWidget(controls_group)

        # Groupe : Scénarios
        scenarios_group = QGroupBox("Scénarios")
        scenarios_layout = QGridLayout(scenarios_group)
        scenarios_layout.setHorizontalSpacing(8)
        scenarios_layout.setVerticalSpacing(6)

        def _make_scenario_field(multiline: bool) -> QTextEdit:
            field = QTextEdit()
            field.setReadOnly(True)
            field.setAcceptRichText(True)
            field.setStyleSheet("background-color: #ffffff; color: #000000;")
            if multiline:
                field.setLineWrapMode(QTextEdit.LineWrapMode.WidgetWidth)
                field.setFixedHeight(66)
            else:
                field.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
                field.setFixedHeight(28)
                field.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
            field.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
            return field

        scenarios_layout.addWidget(QLabel("Rov Fx :"), 0, 0)
        self.scenario_fx_text = _make_scenario_field(multiline=True)
        scenarios_layout.addWidget(self.scenario_fx_text, 0, 1)

        scenarios_layout.addWidget(QLabel("Rov Fy:"), 1, 0)
        self.scenario_fy_text = _make_scenario_field(multiline=True)
        scenarios_layout.addWidget(self.scenario_fy_text, 1, 1)

        scenarios_layout.addWidget(QLabel("Bateau:"), 2, 0)
        self.scenario_boat_text = _make_scenario_field(multiline=False)
        scenarios_layout.addWidget(self.scenario_boat_text, 2, 1)

        scenarios_layout.addWidget(QLabel("Moulinet:"), 3, 0)
        self.scenario_moulinet_text = _make_scenario_field(multiline=False)
        scenarios_layout.addWidget(self.scenario_moulinet_text, 3, 1)

        layout.addWidget(scenarios_group)
        
        # Groupe : Statut
        status_group = QGroupBox("Statut")
        status_layout = QVBoxLayout(status_group)
        
        self.status_label = QLabel("Simulation arrêtée")
        self.status_label.setStyleSheet("padding: 8px; background-color: #f8f9fa; border: 1px solid #dee2e6;")
        status_layout.addWidget(self.status_label)
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        status_layout.addWidget(self.progress_bar)
        
        layout.addWidget(status_group)
        
        layout.addStretch()
        
        return panel
    
    def update_mission_display(self):
        """Met à jour l'affichage du nom de la mission"""
        if hasattr(self, 'mission_label'):
            current_mission = self.main_window.mission_data.get('current')
            # Vérifier si current_mission existe et n'est pas vide
            if current_mission and str(current_mission).strip():
                self.mission_label.setText(str(current_mission))
            else:
                self.mission_label.setText("Aucune mission sélectionnée")
    
    def update_dl_dt_range(self):
        """Met à jour la plage du widget dl_dt_input en fonction des paramètres chargés"""
        try:
            boat_params = self.main_window.parameters.get('boat', {})
            dl_dt_min = boat_params.get('dl_dt_min', -1.0)
            dl_dt_max = boat_params.get('dl_dt_max', 1.0)
            
            # S'assurer que min < max
            if dl_dt_min > dl_dt_max:
                dl_dt_min, dl_dt_max = dl_dt_max, dl_dt_min
            
            # Mettre à jour la plage du widget
            current_value = self.dl_dt_input.value()
            self.dl_dt_input.setRange(dl_dt_min, dl_dt_max)
            
            # S'assurer que la valeur actuelle reste dans la nouvelle plage
            if current_value < dl_dt_min:
                self.dl_dt_input.setValue(dl_dt_min)
            elif current_value > dl_dt_max:
                self.dl_dt_input.setValue(dl_dt_max)
        except Exception:
            # En cas d'erreur, utiliser les valeurs par défaut
            self.dl_dt_input.setRange(-1.0, 1.0)
    
    def initialize_system(self):
        """Initialise le système ROV et met à jour les métriques temps réel"""
        try:
            import numpy as np  # Import numpy au début de la fonction
            import copy
            from src.models.state_projection import reconcile_straight_mode_after_normalize
            
            # Debug: Afficher le nom de la mission utilisée
            current_mission = self.main_window.mission_data.get('current') if hasattr(self.main_window, 'mission_data') else None
            if current_mission:
                trace_print(1, f"[DEBUG] Initialisation du système avec la mission: '{current_mission}'")
            else:
                trace_print(1, "[DEBUG] Initialisation du système sans mission spécifiée (paramètres par défaut)")
            
            # Vérifier que les paramètres sont chargés
            if self.main_window.parameters is None:
                # Charger les paramètres par défaut
                from src.utils.parameters import get_default_parameters
                self.main_window.parameters = get_default_parameters()
            
            # Mettre à jour la plage de dl_dt_input en fonction des paramètres
            self.update_dl_dt_range()
            
            from src.models.system_model import ROVSystem
            from src.utils.initial_conditions import get_initial_state
            from src.utils.cable_init_buoyant import (
                build_buoyant_cable_polyline,
                is_buoyant_cable,
            )
            
            # Créer le système ROV
            N_segments = int(self.main_window.calc_params.get('N_segments', 50))
            params = copy.deepcopy(self.main_window.parameters)
            params.setdefault('cable', {})
            params['cable']['straight_blend_alpha'] = float(
                self.main_window.calc_params.get('straight_blend_alpha', 1.0)
            )
            system = ROVSystem(params, N_segments=N_segments)
            
            # Profil de courant (chaîne) depuis les conditions initiales
            v_courant = self.main_window.init_params.get('v_courant', "0.0")
            system.environment.v_courant_raw = v_courant
            # Stocker le nom de mission pour les traces
            if current_mission:
                system.environment.mission_name = str(current_mission)
            
            # Conditions initiales
            x_rov_init = self.main_window.init_params.get('x_rov_init', 0.0)
            y_rov_init = self.main_window.init_params.get('y_rov_init', -10.0)  # Profondeur négative
            x_boat_init = self.main_window.init_params.get('x_boat_init', 0.0)
            L_init = self.main_window.init_params.get('L_init', 50.0)
            vx_boat_init = 0.0  # Vitesse initiale du bateau
            
            # Calculer l'état initial
            y0 = get_initial_state(
                system,
                x_rov=x_rov_init,
                y_rov=y_rov_init,
                x_boat=x_boat_init,
                L=L_init,
                use_current_geometry=bool(self.main_window.init_params.get('use_current_geometry', True))
            )
            
            # Dépaqueter l'état initial
            (x_rov, y_rov, vx_rov, vy_rov, x_boat, vx_boat,
             x_cable, y_cable, T, L) = system.unpack_state(y0)

            def _build_length_exact_polyline_case1_or_straight(xb, yb, xr, yr, L_target, n_segments):
                """Construit la géométrie exacte pour le cas 1 (surface+vertical) ou cas 3 (straight)."""
                dx = float(xr - xb)
                dy = float(yr - yb)
                d = float(np.hypot(dx, dy))
                n_pts = max(int(n_segments), 1) + 1
                if d <= 1e-12 or abs(L_target - d) <= 1e-9:
                    # Cas 3 (ou dégénéré): câble rectiligne.
                    x_line = np.linspace(float(xb), float(xr), n_pts)
                    y_line = np.linspace(float(yb), float(yr), n_pts)
                    y_line = np.clip(y_line, None, 0.0)
                    y_line[0] = float(yb)
                    y_line[-1] = float(yr)
                    return x_line, y_line

                # Fallback privilégié :
                # 1) partie flottante en surface (y=0) avec éventuel aller-retour (slack),
                # 2) tronçon vertical au droit du ROV jusqu'à y_rov.
                y_depth = abs(float(yr - yb))
                L_top = float(L_target) - y_depth
                dx_abs = abs(dx)
                manh_tol = max(1e-9, 1e-12 * max(abs(float(L_target)), 1.0))
                extra_sv = L_top - dx_abs
                # L = |dx| + |dy| (Manhattan géométrique) : un seul angle droit
                # (xb,yb)--(xr,yb)--(xr,yr), sans slack horizontal supplémentaire.
                if extra_sv < -manh_tol:
                    return None, None
                if extra_sv <= manh_tol:
                    if y_depth <= 1e-12 or dx_abs <= 1e-12:
                        return None, None
                    L_h = dx_abs
                    L_v = y_depth
                    Ltot = L_h + L_v
                    # Deux segments rectilignes avec nœud commun au coin (xr, yb) : longueur
                    # totale exacte Ltot = L_h + L_v pour len = n1 + n2 - 1 == n_pts.
                    n1 = max(2, int(round((n_pts - 1) * L_h / Ltot)) + 1)
                    n1 = min(n1, n_pts)
                    n2 = n_pts - n1 + 1
                    if n2 < 2:
                        n2 = 2
                        n1 = n_pts - n2 + 1
                        n1 = max(2, n1)
                    x_first = np.linspace(float(xb), float(xr), n1)
                    y_first = np.full(n1, float(yb), dtype=float)
                    x_second = np.full(n2, float(xr), dtype=float)
                    y_second = np.linspace(float(yb), float(yr), n2)
                    x_new = np.concatenate((x_first, x_second[1:]))
                    y_new = np.concatenate((y_first, y_second[1:]))
                    y_new = np.clip(y_new, None, 0.0)
                    y_new[0] = float(yb)
                    y_new[-1] = float(yr)
                    return x_new, y_new

                # extra_sv > manh_tol : longueur horizontale « surface » au-delà du minimum |dx|.
                sign = 1.0 if dx >= 0.0 else -1.0
                extra = L_top - dx_abs
                x_turn = float(xr) + sign * (0.5 * extra)

                # Longueurs des 3 tronçons: surface (boat->turn), surface (turn->x_rov), vertical.
                L1 = abs(x_turn - float(xb))
                L2 = abs(x_turn - float(xr))
                L3 = y_depth
                Ltot = L1 + L2 + L3

                s_vals = np.linspace(0.0, Ltot, n_pts)
                x_new = np.empty(n_pts, dtype=float)
                y_new = np.empty(n_pts, dtype=float)
                for i, s in enumerate(s_vals):
                    if s <= L1:
                        a = s / max(L1, 1e-12)
                        x_new[i] = (1.0 - a) * float(xb) + a * x_turn
                        y_new[i] = 0.0
                    elif s <= (L1 + L2):
                        a = (s - L1) / max(L2, 1e-12)
                        x_new[i] = (1.0 - a) * x_turn + a * float(xr)
                        y_new[i] = 0.0
                    else:
                        a = (s - L1 - L2) / max(L3, 1e-12)
                        x_new[i] = float(xr)
                        y_new[i] = (1.0 - a) * 0.0 + a * float(yr)

                y_new = np.clip(y_new, None, 0.0)
                y_new[0] = float(yb)
                y_new[-1] = float(yr)
                return x_new, y_new

            def _build_case2_catenary_chain(system, xb, yb, xr, yr, L_target):
                """
                Cas 2 sans courant : chaînette physique bateau -> ROV via ``CableSolver._solve_catenary``.

                Les pentes discrètes sur le premier / dernier segment peuvent être de mauvais
                estimateurs de la tangente (maillage fin près des extrémités) ; on journalise
                des pentes lissées sur ~1 % de la longueur d'arc pour comparaison.
                """
                w_pa = (
                    (system.cable.rho_cable - system.environment.rho_eau)
                    * system.cable.A_cable
                    * system.environment.g
                )
                try:
                    x_cat, y_cat = system.cable.solver._solve_catenary(
                        float(xr), float(yr), float(xb), float(L_target), float(w_pa)
                    )
                except Exception:
                    return None, None
                x_cat = np.asarray(x_cat, dtype=float).copy()
                y_cat = np.asarray(y_cat, dtype=float).copy()
                if len(x_cat) < 3:
                    return None, None
                # Extrémités exactes
                x_cat[0], y_cat[0] = float(xb), float(yb)
                x_cat[-1], y_cat[-1] = float(xr), float(yr)
                y_cat = np.clip(y_cat, None, 0.0)
                ds = np.hypot(np.diff(x_cat), np.diff(y_cat))
                s = np.concatenate(([0.0], np.cumsum(ds)))
                L_seg = float(s[-1])
                tol_L = max(0.02, 1e-4 * float(L_target))
                if abs(L_seg - float(L_target)) > tol_L:
                    trace_print(
                        5,
                        f"[INIT] Chaînette cas2: longueur L_seg={L_seg:.3f} vs L={L_target:.3f}, repli heuristique"
                    )
                    return None, None

                def _slope_over_arc_frac(from_start: bool, frac: float = 0.01):
                    if len(x_cat) < 3:
                        return None
                    target = float(frac) * L_seg
                    if from_start:
                        idx = int(np.searchsorted(s, target, side="left"))
                        idx = max(1, min(idx, len(x_cat) - 1))
                        dx = float(x_cat[idx] - x_cat[0])
                        dy = float(y_cat[idx] - y_cat[0])
                    else:
                        # Corde couvrant ~target (m) d'arc depuis le ROV vers le bateau.
                        idx = int(np.searchsorted(s, L_seg - target, side="right")) - 1
                        idx = max(0, min(idx, len(x_cat) - 2))
                        dx = float(x_cat[-1] - x_cat[idx])
                        dy = float(y_cat[-1] - y_cat[idx])
                    if abs(dx) < 1e-12:
                        return None
                    return abs(dy / dx)

                sb = _slope_over_arc_frac(True)
                sr = _slope_over_arc_frac(False)
                trace_print(
                    5,
                    f"[INIT] Chaînette cas2: L_seg={L_seg:.3f} "
                    f"|pente~bateau(1% arc)|={sb} |pente~ROV(1% arc)|={sr}"
                )
                return x_cat, y_cat

            def _build_length_exact_polyline_case2_curve(xb, yb, xr, yr, L_target, n_segments):
                """Cas 2 sans courant: courbe type chaînette approchée + éventuel tronçon surface."""
                n_pts = max(int(n_segments), 1) + 1
                xb_f = float(xb)
                yb_f = float(yb)
                xr_f = float(xr)
                yr_f = float(yr)
                L_target_f = float(L_target)
                if abs(yb_f) > 1e-9:
                    return None, None

                def _arc_points(x_start, q, n_dense=900, sag_amp=0.0):
                    t = np.linspace(0.0, 1.0, int(n_dense))
                    x = float(x_start) + (xr_f - float(x_start)) * t
                    y = yr_f * np.power(t, float(q))
                    # Terme de flèche (vers le bas) nul aux extrémités et à pente nulle
                    # aux extrémités, pour ne pas perturber les tangentes bateau/ROV.
                    bell = (t * t) * ((1.0 - t) * (1.0 - t))
                    if sag_amp > 0.0:
                        y = y - float(sag_amp) * bell

                    # Forcer l'arc à rester du côté "bas" de la corde bateau->ROV:
                    # y doit être <= y_ligne (rappel convention profondeur: y < 0).
                    y_line = yb_f + (yr_f - yb_f) * t
                    delta_up = float(np.max(y - y_line))
                    if delta_up > 0.0:
                        bell_max = max(float(np.max(bell)), 1e-12)
                        sag_auto = (delta_up + 1e-6) / bell_max
                        y = y - sag_auto * bell
                    y = np.clip(y, None, 0.0)
                    return x, y

                def _polyline_len(x, y):
                    return float(np.sum(np.hypot(np.diff(x), np.diff(y))))

                dx_line = xr_f - xb_f
                if abs(dx_line) < 1e-12:
                    return None, None

                # Seuil "pas trop long": arc depuis le bateau avec tangente horizontale en surface (q=2).
                x_boat_arc, y_boat_arc = _arc_points(xb_f, q=2.0, n_dense=700)
                L_arc_q2_from_boat = _polyline_len(x_boat_arc, y_boat_arc)

                # Sous-cas A: L assez grand -> segment horizontal surface + arc avec tangente horizontale en H.
                if L_target_f >= L_arc_q2_from_boat - 1e-6:
                    def _total_len_alpha(alpha):
                        x_h = xb_f + alpha * dx_line
                        x_arc, y_arc = _arc_points(x_h, q=2.0, n_dense=700)
                        return abs(x_h - xb_f) + _polyline_len(x_arc, y_arc), x_h, x_arc, y_arc

                    a_lo, a_hi = 0.0, 1.0
                    f_lo, _, _, _ = _total_len_alpha(a_lo)
                    f_hi, _, _, _ = _total_len_alpha(a_hi)
                    if not (f_lo - 1e-6 <= L_target_f <= f_hi + 1e-6):
                        return None, None

                    x_h_best = xb_f
                    x_arc_best, y_arc_best = x_boat_arc, y_boat_arc
                    for _ in range(60):
                        a_mid = 0.5 * (a_lo + a_hi)
                        f_mid, x_h_mid, x_arc_mid, y_arc_mid = _total_len_alpha(a_mid)
                        x_h_best = x_h_mid
                        x_arc_best, y_arc_best = x_arc_mid, y_arc_mid
                        if abs(f_mid - L_target_f) <= 1e-6:
                            break
                        if f_mid < L_target_f:
                            a_lo = a_mid
                        else:
                            a_hi = a_mid

                    # Rééchantillonnage curviligne global (surface puis arc) pour obtenir N+1 points.
                    x_surface = np.linspace(xb_f, x_h_best, 120)
                    y_surface = np.zeros_like(x_surface)
                    x_combo = np.concatenate((x_surface[:-1], x_arc_best))
                    y_combo = np.concatenate((y_surface[:-1], y_arc_best))
                else:
                    # Sous-cas B: arc arrive au bateau, tangente au bateau libre.
                    # On impose q>1 pour garantir une pente plus forte au ROV qu'au bateau.
                    # Ajout d'une flèche contrôlée pour éviter la courbure "vers le haut".
                    sag_case_b = 0.35 * abs(yr_f)

                    def _len_q(q_val):
                        x_arc, y_arc = _arc_points(xb_f, q=q_val, n_dense=900, sag_amp=sag_case_b)
                        return _polyline_len(x_arc, y_arc), x_arc, y_arc

                    q_eps = 1e-3
                    q_lo, q_hi = 1.0 + q_eps, 3.0
                    f_lo, x_lo_arc, y_lo_arc = _len_q(q_lo)
                    f_hi, x_hi_arc, y_hi_arc = _len_q(q_hi)
                    if not (f_lo - 1e-6 <= L_target_f <= f_hi + 1e-6):
                        return None, None

                    x_combo, y_combo = x_lo_arc, y_lo_arc
                    for _ in range(60):
                        q_mid = 0.5 * (q_lo + q_hi)
                        f_mid, x_mid_arc, y_mid_arc = _len_q(q_mid)
                        x_combo, y_combo = x_mid_arc, y_mid_arc
                        if abs(f_mid - L_target_f) <= 1e-6:
                            break
                        if f_mid < L_target_f:
                            q_lo = q_mid
                        else:
                            q_hi = q_mid

                ds_combo = np.hypot(np.diff(x_combo), np.diff(y_combo))
                s_combo = np.concatenate(([0.0], np.cumsum(ds_combo)))
                if float(s_combo[-1]) <= 1e-12:
                    return None, None
                s_target = np.linspace(0.0, L_target_f, n_pts)
                s_target = np.clip(s_target, 0.0, float(s_combo[-1]))
                x_new = np.interp(s_target, s_combo, x_combo)
                y_new = np.interp(s_target, s_combo, y_combo)
                y_new = np.clip(y_new, None, 0.0)
                x_new[0], y_new[0] = xb_f, yb_f
                x_new[-1], y_new[-1] = xr_f, yr_f

                # Vérification explicite:
                # 1) signe de pente au ROV identique à la droite bateau-ROV
                # 2) |pente tangente| au ROV > |pente tangente| au bateau.
                if len(x_new) >= 3:
                    dx_tan = float(x_new[-1] - x_new[-2])
                    if dx_tan * dx_line <= 0.0:
                        return None, None
                    dx_boat = float(x_new[1] - x_new[0])
                    dy_boat = float(y_new[1] - y_new[0])
                    dx_rov = float(x_new[-1] - x_new[-2])
                    dy_rov = float(y_new[-1] - y_new[-2])
                    if abs(dx_boat) < 1e-12 or abs(dx_rov) < 1e-12:
                        return None, None
                    slope_boat = dy_boat / dx_boat
                    slope_rov = dy_rov / dx_rov
                    if abs(slope_rov) <= abs(slope_boat):
                        return None, None
                return x_new, y_new

            # Contrainte physique forte : L (commande/état) doit rester égale à la
            # longueur géométrique réelle du câble (somme des segments), même à t=0.
            if x_cable is not None and y_cable is not None and len(x_cable) > 1:
                try:
                    x_cable_np = np.asarray(x_cable, dtype=float)
                    y_cable_np = np.asarray(y_cable, dtype=float)
                    x_corr, y_corr, straight_mode, _ = system.cable.solver._normalize_cable_length(
                        x_cable_np,
                        y_cable_np,
                        float(L),
                        x_boat=float(x_boat),
                        y_boat=0.0,
                        x_rov=float(x_rov),
                        y_rov=float(y_rov),
                        k_tail=10,
                        t=0.0,
                    )
                    x_corr, y_corr, x_rov, y_rov = reconcile_straight_mode_after_normalize(
                        x_corr,
                        y_corr,
                        straight_mode,
                        float(x_rov),
                        float(y_rov),
                        float(x_boat),
                        0.0,
                    )
                    x_cable = np.asarray(x_corr, dtype=float)
                    y_cable = np.asarray(y_corr, dtype=float)
                except Exception as e_norm:
                    trace_print(8, f"[INIT] Échec normalisation géométrie câble: {e_norm}")

                # Vérifier la cohérence longueur géométrique <-> longueur commandée.
                # Si la normalisation n'atteint pas la cible, appliquer un fallback robuste
                # qui garantit L_seg == L à tolérance numérique.
                ds_after_norm = np.hypot(np.diff(np.asarray(x_cable, dtype=float)),
                                         np.diff(np.asarray(y_cable, dtype=float)))
                L_seg_after_norm = float(np.sum(ds_after_norm))
                # Appliquer le modèle géométrique d'initialisation sans courant même
                # si L_seg est déjà proche de L, pour garantir la forme voulue des cas 1/2/3.
                no_current = False
                try:
                    env = system.environment
                    v0 = float(np.asarray(env.get_current_velocity(0.0), dtype=float).reshape(-1)[0])
                    vr = float(np.asarray(env.get_current_velocity(float(y_rov)), dtype=float).reshape(-1)[0])
                    no_current = max(abs(v0), abs(vr)) <= 1e-9
                except Exception:
                    no_current = False

                if no_current:
                    L_cmd = float(L)
                    dx_abs = abs(float(x_rov - x_boat))
                    dy_abs = abs(float(y_rov - 0.0))
                    L_straight = float(np.hypot(float(x_rov - x_boat), float(y_rov - 0.0)))
                    L_surface_vertical = dx_abs + dy_abs
                    tol = 1e-9

                    # Cas 4: impossible physiquement.
                    if L_cmd < L_straight - tol:
                        raise ValueError(
                            f"[INIT] Configuration mission impossible: L={L_cmd:.3f} < L_straight={L_straight:.3f}"
                        )

                    # Cas 1 et 3: géométrie déterministe exacte.
                    if L_cmd >= L_surface_vertical - tol or abs(L_cmd - L_straight) <= tol:
                        x_fix, y_fix = _build_length_exact_polyline_case1_or_straight(
                            x_boat, 0.0, x_rov, y_rov, L_cmd, system.N
                        )
                        if x_fix is not None and y_fix is not None:
                            x_cable = np.asarray(x_fix, dtype=float)
                            y_cable = np.asarray(y_fix, dtype=float)
                            trace_print(
                                5,
                                f"[INIT] Sans courant cas 1/3 appliqué: "
                                f"L_cmd={L_cmd:.3f} L_seg_before={L_seg_after_norm:.3f} "
                                f"L_seg_after={float(np.sum(np.hypot(np.diff(x_cable), np.diff(y_cable)))):.3f}"
                            )
                    else:
                        # Cas 2 : câble flottant (spec chaînette z=-y) puis solveur / heuristique.
                        used = ""
                        x_fix, y_fix = None, None
                        if is_buoyant_cable(system):
                            try:
                                x_fix, y_fix = build_buoyant_cable_polyline(
                                    float(x_boat),
                                    0.0,
                                    float(x_rov),
                                    float(y_rov),
                                    L_cmd,
                                    system.N,
                                    slack_side="auto",
                                )
                                if x_fix is not None and y_fix is not None:
                                    used = "buoyant_spec"
                            except Exception as e_buoy:
                                trace_print(
                                    5,
                                    f"[INIT] Chaînette flottante (spec analytique): {e_buoy}",
                                )
                                x_fix, y_fix = None, None
                        if x_fix is None:
                            x_fix, y_fix = _build_case2_catenary_chain(
                                system, x_boat, 0.0, x_rov, y_rov, L_cmd
                            )
                            if x_fix is None or y_fix is None:
                                x_fix, y_fix = _build_length_exact_polyline_case2_curve(
                                    x_boat, 0.0, x_rov, y_rov, L_cmd, system.N
                                )
                                used = used or "heuristique"
                            else:
                                used = used or "chaînette"
                        if x_fix is not None and y_fix is not None:
                            x_cable = np.asarray(x_fix, dtype=float)
                            y_cable = np.asarray(y_fix, dtype=float)
                            if used == "buoyant_spec":
                                try:
                                    xc, yc, sm, _ = (
                                        system.cable.solver._normalize_cable_length(
                                            x_cable,
                                            y_cable,
                                            L_cmd,
                                            x_boat=float(x_boat),
                                            y_boat=0.0,
                                            x_rov=float(x_rov),
                                            y_rov=float(y_rov),
                                            k_tail=10,
                                            t=0.0,
                                        )
                                    )
                                    x_cable = np.asarray(xc, dtype=float)
                                    y_cable = np.asarray(yc, dtype=float)
                                    if sm:
                                        x_rov = float(x_cable[-1])
                                        y_rov = float(y_cable[-1])
                                except Exception as e_pn:
                                    trace_print(
                                        8,
                                        f"[INIT] Post-normalisation câble flottant: {e_pn}",
                                    )
                            trace_print(
                                5,
                                f"[INIT] Sans courant cas 2 ({used or 'chaînette'}): "
                                f"L_cmd={L_cmd:.3f} L_seg_before={L_seg_after_norm:.3f} "
                                f"L_seg_after={float(np.sum(np.hypot(np.diff(x_cable), np.diff(y_cable)))):.3f}"
                            )
                        else:
                            trace_print(
                                5,
                                f"[INIT] Sans courant cas 2 impossible: "
                                f"L_cmd={L_cmd:.3f} L_seg={L_seg_after_norm:.3f}"
                            )
                elif abs(L_seg_after_norm - float(L)) > 1e-2:
                    trace_print(
                        5,
                        f"[INIT] Dérive longueur conservée (courant non nul): "
                        f"L_cmd={float(L):.3f} L_seg={L_seg_after_norm:.3f}"
                    )

                # Synchroniser explicitement l'état packé avec la géométrie corrigée.
                idx_x_cable = 6
                idx_y_cable = idx_x_cable + system.N + 1
                idx_t_cable = idx_y_cable + system.N + 1
                y0[idx_x_cable:idx_y_cable] = x_cable
                y0[idx_y_cable:idx_t_cable] = y_cable
                y0[0] = float(x_rov)
                y0[1] = float(y_rov)
            
            
            # IMPORTANT: Dans solve_equilibrium_static, le câble va du ROV (index 0) au bateau (index -1)
            # Mais dans system_model.compute_derivatives, après résolution, les conditions aux limites
            # sont inversées : y_cable_new[0] = 0.0 (bateau) et y_cable_new[-1] = y_rov (ROV)
            # Pour l'initialisation, on utilise directement le résultat de solve_equilibrium_static,
            # donc l'ordre est : index 0 = ROV, index -1 = bateau
            
            # Calculer les forces initiales
            Fx_drag_rov, Fy_drag_rov = system.rov.compute_drag_force(
                vx_rov, vy_rov, y_rov, system.environment
            )
            F_buoyancy = system.rov.compute_buoyancy_force(system.environment)
            F_weight = system.rov.compute_weight_force(system.environment)
            # Poids apparent : positif si flottabilité positive (vers le haut/surface)
            # Convention : F_apparent_weight = F_buoyancy - F_weight
            F_apparent_weight = F_buoyancy - F_weight
            F_buoyancy_net = F_apparent_weight
            
            # Tension au niveau du ROV et du bateau
            # CORRECTION: Après correction de _compute_catenary_tensions :
            # Les tensions suivent l'ordre des positions : T[0] = tension au bateau, T[-1] = tension au ROV
            T_rov = T[-1] if len(T) > 0 else 0.0  # Tension au ROV = T[-1]
            T_boat = T[0] if len(T) > 0 else 0.0  # Tension au bateau = T[0]
            T_max = max(T) if len(T) > 0 else 0.0
            
            # Calculer le vecteur unitaire Urov au niveau du ROV
            # CORRECTION: Après correction de _compute_catenary_tensions :
            # Les positions sont ordonnées : index 0 = bateau, index -1 = ROV
            # Urov = direction du point précédent vers le ROV (direction du câble vers le ROV)
            if len(x_cable) > 1:
                # Direction du point précédent vers le ROV (index -1)
                dx_rov = x_cable[-1] - x_cable[-2]
                dy_rov = y_cable[-1] - y_cable[-2]
                ds_rov = np.sqrt(dx_rov**2 + dy_rov**2)
                if ds_rov > 1e-6:
                    Urov_x = dx_rov / ds_rov
                    Urov_y = dy_rov / ds_rov
                else:
                    Urov_x = 0.0
                    Urov_y = 1.0  # Direction verticale par défaut
            else:
                Urov_x = 0.0
                Urov_y = 1.0
            
            # Forces de traction du câble sur le ROV
            # Nouvelle convention : positif = vers le haut (surface)
            # Urov pointe du point précédent vers le ROV, donc vers le bas si le ROV est plus profond
            # Pour avoir la force vers le haut, on utilise T_rov * (-Urov_y)
            Fx_traction = -T_rov * Urov_x  # Horizontal : signe inchangé
            Fy_traction = T_rov * (-Urov_y)  # Vertical : inversé pour positif = vers le haut
            
            # Variables de commande (initialisées à 0)
            Fx_cmd = 0.0
            Fy_cmd = 0.0
            
            # Somme des forces ROV
            # Convention : toutes les forces verticales sont positives vers le haut (surface)
            Fx_total = Fx_drag_rov + Fx_traction + Fx_cmd
            Fy_total = F_apparent_weight + Fy_drag_rov + Fy_traction + Fy_cmd
            
            # Calculer le vecteur unitaire Ubateau au niveau du bateau
            # CORRECTION: Après correction de _compute_catenary_tensions :
            # Les positions sont ordonnées : index 0 = bateau, index -1 = ROV
            # Ubateau = direction du bateau vers le point suivant (direction du câble depuis le bateau)
            if len(x_cable) > 1:
                # Direction depuis le bateau (index 0) vers le point suivant (index 1)
                dx_bateau = x_cable[1] - x_cable[0]
                dy_bateau = y_cable[1] - y_cable[0]
                ds_bateau = np.sqrt(dx_bateau**2 + dy_bateau**2)
                if ds_bateau > 1e-6:
                    Ubateau_x = dx_bateau / ds_bateau
                    Ubateau_y = dy_bateau / ds_bateau
                else:
                    Ubateau_x = 0.0
                    Ubateau_y = -1.0  # Direction verticale par défaut
            else:
                Ubateau_x = 0.0
                Ubateau_y = -1.0
            
            # Force de tension du câble sur le bateau
            # Force exercée PAR le câble SUR le bateau = -T_bateau * Ubateau
            # (négatif car le câble tire le bateau dans la direction opposée à Ubateau)
            Fx_traction_boat = -T_boat * Ubateau_x
            Fy_traction_boat = -T_boat * Ubateau_y
            
            # Force de propulsion du bateau (initialement nulle)
            vx_boat_cmd_init = 0.0
            F_prop_boat = system.boat.compute_propulsion_force(
                vx_boat_cmd_init, vx_boat, system.environment
            )
            
            # Somme des forces appliquées au bateau
            # L'effet de la tension du câble est négligeable
            # Le bateau évolue uniquement selon la commande de vitesse
            Fx_total_boat = F_prop_boat
            Fy_total_boat = 0.0  # Pas de force verticale
            
            # Calculer les forces de traînée du câble à l'état initial (vitesses nulles)
            from src.solvers.forces import compute_cable_forces, compute_cable_apparent_weight
            vx_cable_init = np.zeros(len(x_cable))
            vy_cable_init = np.zeros(len(x_cable))
            params_cable = {
                'd': system.cable.d,
                'rho_cable': system.cable.rho_cable,
                'Cx_cable': system.cable.Cx_cable,
                'Cf_cable': system.cable.Cf_cable
            }
            Fx_segments, Fy_segments, Fx_long_seg, Fy_long_seg, Fx_perp_seg, Fy_perp_seg = compute_cable_forces(
                x_cable, y_cable, vx_cable_init, vy_cable_init, system.environment, params_cable, L
            )
            F_drag_cable_x = float(np.sum(Fx_segments)) if len(Fx_segments) > 0 else 0.0
            N_segments = max(len(x_cable) - 1, 1)
            ds = L / N_segments if N_segments > 0 else L
            Fy_weight_seg = compute_cable_apparent_weight(
                system.cable.rho_cable,
                system.environment.rho_eau,
                system.cable.A_cable,
                system.environment.g,
                ds
            )
            F_weight_total = Fy_weight_seg * N_segments
            F_drag_cable_y = (float(np.sum(Fy_segments)) - F_weight_total) if len(Fy_segments) > 0 else 0.0
            
            # Traînées décomposées
            F_drag_long_x = float(np.sum(Fx_long_seg)) if len(Fx_long_seg) > 0 else 0.0
            F_drag_long_y = float(np.sum(Fy_long_seg)) if len(Fy_long_seg) > 0 else 0.0
            F_drag_perp_x = float(np.sum(Fx_perp_seg)) if len(Fx_perp_seg) > 0 else 0.0
            F_drag_perp_y = float(np.sum(Fy_perp_seg)) if len(Fy_perp_seg) > 0 else 0.0
            
            # Mettre à jour l'état de simulation avec les données initiales
            state = self.main_window.get_simulation_state()
            state['data'] = {
                'time': [0.0],
                'x_rov': [x_rov],
                'y_rov': [y_rov],
                'vx_rov': [vx_rov],
                'vy_rov': [vy_rov],
                'x_boat': [x_boat],
                'vx_boat': [vx_boat],
                'L': [L],
                'T_rov': [T_rov],
                'T_boat': [T_boat],
                'T_max': [T_max],
                'Fx_drag_rov': [Fx_drag_rov],
                'Fy_drag_rov': [Fy_drag_rov],
                'Fx_traction_rov': [Fx_traction],
                'Fy_traction_rov': [Fy_traction],
                'Fy_rov_app_w': [F_apparent_weight],
                'F_buoyancy_net': [F_buoyancy_net],
                'Fx_rov_total': [Fx_total],
                'Fy_rov_total': [Fy_total],
                'Fx_traction_boat': [Fx_traction_boat],
                'Fy_traction_boat': [Fy_traction_boat],
                'F_prop_boat': [F_prop_boat],
                'Fx_total_boat': [Fx_total_boat],
                'Fy_total_boat': [Fy_total_boat],
                'x_cable_curr': x_cable.tolist() if hasattr(x_cable, 'tolist') else list(x_cable),
                'y_cable_curr': y_cable.tolist() if hasattr(y_cable, 'tolist') else list(y_cable),
                'T_cable_curr': T.tolist() if hasattr(T, 'tolist') else list(T),
                'cable_drag': [(F_drag_cable_x, F_drag_cable_y)],
                'Fx_drag_cable': [F_drag_cable_x],
                'Fy_drag_cable': [F_drag_cable_y],
                'Fx_drag_cable_longitudinal': [F_drag_long_x],
                'Fy_drag_cable_longitudinal': [F_drag_long_y],
                'Fx_drag_cable_perpendicular': [F_drag_perp_x],
                'Fy_drag_cable_perpendicular': [F_drag_perp_y],
            }
            state['current_time'] = 0.0
            state['system'] = system
            state['y_current'] = y0
            # Mettre la simulation en pause après l'initialisation pour permettre l'analyse
            state['running'] = False
            state['paused'] = False
            state['mission_ended'] = False
            if 'mission_end_time' in state:
                state.pop('mission_end_time', None)
            self._mission_end_latched = False
            self._scenario_trigger_index = 0
            self.main_window.update_simulation_state(state)

            self._set_scenario_texts()
            
            # Mettre à jour les boutons pour refléter l'état initialisé mais en pause
            if hasattr(self, 'btn_start'):
                self.btn_start.setEnabled(True)
                self.btn_pause.setEnabled(False)
                self.btn_restart.setEnabled(True)
                self._update_refresh_button_state()
            if hasattr(self, 'status_label'):
                self.status_label.setText("Lancer la simulation")
                self.status_label.setStyleSheet("padding: 8px; background-color: #f8f9fa; border: 1px solid #dee2e6;")
            if hasattr(self, 'progress_bar'):
                self.progress_bar.setValue(0)
            
            # Mettre à jour l'affichage immédiatement (forcer le rafraîchissement du graphique)
            self.last_plot_update_time = None  # Forcer le rafraîchissement du graphique
            # Réinitialiser les plages pour recalculer l'échelle à l'initialisation
            self.current_s_range = None
            self.current_T_range = None
            
            # Mettre à jour le graphique "Tension" avec les données initiales
            if len(x_cable) > 0 and len(y_cable) > 0 and len(T) > 0:
                try:
                    import numpy as np
                    from src.visualization.plotter import create_tension_curvilinear_plot
                    
                    x_cable_arr = np.asarray(x_cable)
                    y_cable_arr = np.asarray(y_cable)
                    T_cable_arr = np.asarray(T)
                    
                    # IMPORTANT: Après correction de _compute_catenary_tensions :
                    # Les tensions T retournées par le solveur suivent l'ordre des positions :
                    # T[0] = tension au bateau, T[-1] = tension au ROV
                    # (cohérent avec x_cable[0] = bateau, x_cable[-1] = ROV)
                    
                    # Vérifier l'ordre du câble et le réorganiser si nécessaire
                    dist_first_to_boat = np.sqrt((x_cable_arr[0] - x_boat)**2 + (y_cable_arr[0] - 0.0)**2)
                    dist_first_to_rov = np.sqrt((x_cable_arr[0] - x_rov)**2 + (y_cable_arr[0] - y_rov)**2)
                    
                    if dist_first_to_rov < dist_first_to_boat:
                        # Le câble est dans l'ordre inverse : ROV en premier, bateau en dernier
                        # Il faut inverser les positions ET les tensions pour maintenir la cohérence
                        x_cable_arr = np.flip(x_cable_arr)
                        y_cable_arr = np.flip(y_cable_arr)
                        T_cable_arr = np.flip(T_cable_arr)
                        # Après inversion : T_cable_arr[0] = ancienne T[-1] = tension au bateau ✓
                        #                  T_cable_arr[-1] = ancienne T[0] = tension au ROV ✓
                    # Sinon, le câble est déjà dans le bon ordre et les tensions aussi, pas besoin d'inverser
                    
                    # S'assurer que le premier point est exactement au bateau et le dernier au ROV
                    x_cable_arr[0] = x_boat
                    y_cable_arr[0] = 0.0
                    x_cable_arr[-1] = x_rov
                    y_cable_arr[-1] = y_rov
                    
                    # Calculer l'abscisse curviligne s en cumulant les distances
                    s_curvilinear = np.zeros(len(x_cable_arr))
                    for i in range(1, len(x_cable_arr)):
                        dx = x_cable_arr[i] - x_cable_arr[i-1]
                        dy = y_cable_arr[i] - y_cable_arr[i-1]
                        ds = np.sqrt(dx**2 + dy**2)
                        s_curvilinear[i] = s_curvilinear[i-1] + ds
                    
                    # Si les tensions sont définies par segment (N) au lieu de nœud (N+1),
                    # interpoler sur les nœuds pour garantir l'affichage.
                    if len(T_cable_arr) == len(s_curvilinear) - 1:
                        s_seg = 0.5 * (s_curvilinear[:-1] + s_curvilinear[1:])
                        T_cable_arr = np.interp(
                            s_curvilinear, s_seg, T_cable_arr,
                            left=T_cable_arr[0], right=T_cable_arr[-1]
                        )
                    
                    # Calculer les composantes Tx et Ty de la tension
                    Tx_cable = np.zeros(len(x_cable_arr))
                    Ty_cable = np.zeros(len(y_cable_arr))
                    
                    for i in range(len(x_cable_arr)):
                        if i == 0:
                            # Premier point (bateau) : utiliser le segment suivant
                            if len(x_cable_arr) > 1:
                                dx = x_cable_arr[1] - x_cable_arr[0]
                                dy = y_cable_arr[1] - y_cable_arr[0]
                            else:
                                dx, dy = 0, 0
                        elif i == len(x_cable_arr) - 1:
                            # Dernier point (ROV) : utiliser le segment précédent
                            dx = x_cable_arr[i] - x_cable_arr[i-1]
                            dy = y_cable_arr[i] - y_cable_arr[i-1]
                        else:
                            # Point intermédiaire : moyenne des deux segments adjacents
                            dx = (x_cable_arr[i+1] - x_cable_arr[i-1]) / 2.0
                            dy = (y_cable_arr[i+1] - y_cable_arr[i-1]) / 2.0
                        
                        ds = np.sqrt(dx**2 + dy**2)
                        if ds > 1e-6:
                            cos_theta = dx / ds
                            sin_theta = dy / ds
                            Tx_cable[i] = T_cable_arr[i] * cos_theta
                            Ty_cable[i] = T_cable_arr[i] * sin_theta
                        else:
                            Tx_cable[i] = 0
                            Ty_cable[i] = 0
                    
                    # Créer le graphique Tension
                    fig_tension = create_tension_curvilinear_plot(
                        s_curvilinear.tolist(), T_cable_arr.tolist(), "Tension",
                        s_range=self.current_s_range, T_range=self.current_T_range,
                        Tx=Tx_cable.tolist(), Ty=Ty_cable.tolist(),
                        x_cable=x_cable_arr.tolist(), y_cable=y_cable_arr.tolist()
                    )
                    self.plotly_widget_tension.update_figure(fig_tension)
                    
                    # Mettre à jour les plages stockées
                    if fig_tension.layout.xaxis.range is not None:
                        s_min, s_max = fig_tension.layout.xaxis.range
                        s_min = np.floor(s_min / 10.0) * 10.0
                        s_max = np.ceil(s_max / 10.0) * 10.0
                        self.current_s_range = [s_min, s_max]
                    if fig_tension.layout.yaxis.range is not None:
                        T_min, T_max = fig_tension.layout.yaxis.range
                        T_min = np.floor(T_min / 10.0) * 10.0
                        T_max = np.ceil(T_max / 10.0) * 10.0
                        self.current_T_range = [T_min, T_max]
                except Exception as e:
                    trace_print(8, f"Erreur lors de la mise à jour du graphique Tension lors de l'initialisation: {e}")
                    import traceback
                    traceback.print_exc()
            
            self.update_display()
            
        except Exception as e:
            trace_print(8, f"Erreur lors de l'initialisation du système: {e}")
            import traceback
            traceback.print_exc()
    
    def create_visualization_panel(self):
        """Crée le panneau de visualisation"""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Onglets en haut de la colonne centrale
        self.top_tabs = QTabWidget()

        # Onglet 1 (haut) : Commande dL/dt
        self.tab_dl_dt_top = QWidget()
        tab_dl_dt_top_layout = QVBoxLayout(self.tab_dl_dt_top)
        tab_dl_dt_top_layout.setContentsMargins(0, 0, 0, 0)
        self.plotly_widget_dl_dt = PlotlyWidget()
        tab_dl_dt_top_layout.addWidget(self.plotly_widget_dl_dt)
        from src.visualization.plotter import create_dl_dt_plot
        fig_dl_dt = create_dl_dt_plot([], [], "Commande dL/dt", None)
        self.plotly_widget_dl_dt.update_figure(fig_dl_dt)
        self.top_tabs.addTab(self.tab_dl_dt_top, "Commande dL/dt")

        # Onglet 2 (haut) : Slack / L  (%)
        self.tab_slack = QWidget()
        tab_slack_layout = QVBoxLayout(self.tab_slack)
        tab_slack_layout.setContentsMargins(0, 0, 0, 0)

        # Widget Plotly pour le graphique "Slack / L  (%)"
        self.plotly_widget_slack = PlotlyWidget()
        tab_slack_layout.addWidget(self.plotly_widget_slack)

        # Graphique initial "Slack / L  (%)"
        from src.visualization.plotter import create_slack_plot
        fig_slack = create_slack_plot([], [], [], [], "Slack / L  (%)")
        self.plotly_widget_slack.update_figure(fig_slack)

        self.top_tabs.addTab(self.tab_slack, "Slack / L  (%)")

        # Onglet 3 (haut) : Répartition slack
        self.tab_slack_dist = QWidget()
        tab_slack_dist_layout = QVBoxLayout(self.tab_slack_dist)
        tab_slack_dist_layout.setContentsMargins(0, 0, 0, 0)
        self.plotly_widget_slack_dist = PlotlyWidget()
        tab_slack_dist_layout.addWidget(self.plotly_widget_slack_dist)
        from src.visualization.plotter import create_slack_distribution_plot
        fig_slack_dist = create_slack_distribution_plot([], [], "Répartition slack")
        self.plotly_widget_slack_dist.update_figure(fig_slack_dist)
        self.top_tabs.addTab(self.tab_slack_dist, "Répartition slack")

        layout.addWidget(self.top_tabs)
        
        # Onglets en bas de la colonne centrale
        self.bottom_tabs = QTabWidget()
        
        # Onglet 1 : Tension
        self.tab1 = QWidget()
        tab1_layout = QVBoxLayout(self.tab1)
        tab1_layout.setContentsMargins(0, 0, 0, 0)
        
        # Widget Plotly pour le graphique "Tension"
        self.plotly_widget_tension = PlotlyWidget()
        tab1_layout.addWidget(self.plotly_widget_tension)
        
        # Graphique initial "Tension" (vide au départ)
        from src.visualization.plotter import create_tension_curvilinear_plot
        fig_tension = create_tension_curvilinear_plot([], [], "Tension")
        self.plotly_widget_tension.update_figure(fig_tension)
        
        
        # Onglet 2 : Courant
        self.tab2 = QWidget()
        tab2_layout = QVBoxLayout(self.tab2)
        tab2_layout.setContentsMargins(0, 0, 0, 0)
        
        # Widget Plotly pour le graphique "Courant"
        self.plotly_widget_current = PlotlyWidget()
        tab2_layout.addWidget(self.plotly_widget_current)
        
        # Graphique initial "Courant"
        self.update_current_profile_plot()
        
        
        # Onglet 4 : Tension vs cible
        self.tab4 = QWidget()
        tab4_layout = QVBoxLayout(self.tab4)
        tab4_layout.setContentsMargins(0, 0, 0, 0)
        self.plotly_widget_tension_vs_target = PlotlyWidget()
        tab4_layout.addWidget(self.plotly_widget_tension_vs_target)
        from src.visualization.plotter import create_tension_vs_target_plot
        fig_tension_vs_target = create_tension_vs_target_plot(
            [],
            [],
            [],
            [],
            [],
            [],
            "Tension vs cible",
        )
        self.plotly_widget_tension_vs_target.update_figure(fig_tension_vs_target)
        self.bottom_tabs.addTab(self.tab4, "Tension vs cible")
        self.bottom_tabs.addTab(self.tab1, "Tension")
        self.bottom_tabs.addTab(self.tab2, "Courant")
        self.bottom_tabs.setCurrentWidget(self.tab4)
        
        # Ajouter les onglets au layout
        layout.addWidget(self.bottom_tabs)
        
        return panel
    
    def create_metrics_panel(self):
        """Crée le panneau de métriques"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Groupe : Métriques temps réel
        metrics_group = QGroupBox("Métriques temps réel")
        metrics_layout = QGridLayout(metrics_group)
        
        # Labels pour les métriques (dans l'ordre demandé)
        # 1. Temps écoulé
        metrics_layout.addWidget(QLabel("<b>Temps écoulé:</b>"), 0, 0)
        self.time_label = QLabel("0.00 s")
        metrics_layout.addWidget(self.time_label, 0, 1)
        
        # Trait de séparation
        separator = QLabel("─" * 30)
        separator.setStyleSheet("color: #ccc;")
        metrics_layout.addWidget(separator, 1, 0, 1, 2)
        
        # Sous-titre "Câble"
        cable_subtitle = QLabel("<b>Câble</b>")
        cable_subtitle.setStyleSheet("color: #666; font-size: 10pt; margin-top: 5px;")
        metrics_layout.addWidget(cable_subtitle, 2, 0, 1, 2)
        
        # 2. Long. câble commande
        metrics_layout.addWidget(QLabel("<b>Long. câble commande:</b>"), 3, 0)
        self.length_label = QLabel("0.00 m")
        metrics_layout.addWidget(self.length_label, 3, 1)
        
        # 3. Long. câble segments
        metrics_layout.addWidget(QLabel("<b>Long. câble segments:</b>"), 4, 0)
        self.length_segments_label = QLabel("0.00 m")
        metrics_layout.addWidget(self.length_segments_label, 4, 1)
        
        # 4. Dérive longueur câble
        metrics_layout.addWidget(QLabel("<b>Dérive longueur câble:</b>"), 5, 0)
        self.length_drift_label = QLabel("0.00 m")
        metrics_layout.addWidget(self.length_drift_label, 5, 1)
        
        # 5. Long. droite
        metrics_layout.addWidget(QLabel("<b>Long. droite:</b>"), 6, 0)
        self.length_straight_label = QLabel("0.00 m")
        metrics_layout.addWidget(self.length_straight_label, 6, 1)
        
        # 6. Slack
        metrics_layout.addWidget(QLabel("<b>Slack:</b>"), 7, 0)
        self.slack_label = QLabel("0.00 m")
        metrics_layout.addWidget(self.slack_label, 7, 1)
        
        # 7. Mode câble
        metrics_layout.addWidget(QLabel("<b>Mode câble:</b>"), 8, 0)
        self.cable_mode_label = QLabel("-")
        metrics_layout.addWidget(self.cable_mode_label, 8, 1)
        
        # 8. Tension bateau
        metrics_layout.addWidget(QLabel("<b>Tension bateau:</b>"), 9, 0)
        self.tension_boat_label = QLabel("0.00 N")
        metrics_layout.addWidget(self.tension_boat_label, 9, 1)
        
        # 9. Tension ROV
        metrics_layout.addWidget(QLabel("<b>Tension ROV:</b>"), 10, 0)
        self.tension_rov_label = QLabel("0.00 N")
        metrics_layout.addWidget(self.tension_rov_label, 10, 1)
        
        # 10. Tension max
        metrics_layout.addWidget(QLabel("<b>Tension max:</b>"), 11, 0)
        self.tension_label = QLabel("0.00 N")
        metrics_layout.addWidget(self.tension_label, 11, 1)
        
        # 11. Direction câble (Bateau)
        metrics_layout.addWidget(QLabel("<b>Direction câble (Bateau):</b>"), 12, 0)
        self.cable_dir_boat_label = QLabel("(0.00, 0.00)")
        metrics_layout.addWidget(self.cable_dir_boat_label, 12, 1)
        
        # 12. Direction câble (ROV)
        metrics_layout.addWidget(QLabel("<b>Direction câble (ROV):</b>"), 13, 0)
        self.cable_dir_rov_label = QLabel("(0.00, 0.00)")
        metrics_layout.addWidget(self.cable_dir_rov_label, 13, 1)

        # Trait de séparation
        separator_cable_forces = QLabel("─" * 30)
        separator_cable_forces.setStyleSheet("color: #ccc;")
        metrics_layout.addWidget(separator_cable_forces, 14, 0, 1, 2)

        # Sous-titre "Forces s'exerçant sur le câble"
        row = 15
        cable_forces_subtitle = QLabel("<b>Forces s'exerçant sur le câble</b>")
        cable_forces_subtitle.setStyleSheet("color: #666; font-size: 10pt; margin-top: 5px;")
        metrics_layout.addWidget(cable_forces_subtitle, row, 0, 1, 2)
        row += 1
        
        # 6. Traction bateau sur câble
        metrics_layout.addWidget(QLabel("<b>Traction bateau sur câble:</b>"), row, 0)
        self.traction_boat_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.traction_boat_label, row, 1)
        row += 1
        
        # 7. Traction ROV sur câble
        metrics_layout.addWidget(QLabel("<b>Traction ROV sur câble:</b>"), row, 0)
        self.traction_rov_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.traction_rov_label, row, 1)
        row += 1
        
        # 8. Poids apparent
        metrics_layout.addWidget(QLabel("<b>Poids apparent:</b>"), row, 0)
        self.cable_apparent_weight_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.cable_apparent_weight_label, row, 1)
        row += 1
        
        # 9. Force traînée câble
        metrics_layout.addWidget(QLabel("<b>Force traînée câble:</b>"), row, 0)
        self.cable_drag_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.cable_drag_label, row, 1)
        row += 1
        
        # 10. Somme forces câble
        metrics_layout.addWidget(QLabel("<b>Somme forces câble:</b>"), row, 0)
        self.cable_total_forces_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.cable_total_forces_label, row, 1)
        row += 1
        
        # Trait de séparation
        separator2 = QLabel("─" * 30)
        separator2.setStyleSheet("color: #ccc;")
        metrics_layout.addWidget(separator2, row, 0, 1, 2)
        
        row += 1
        # Sous-titre "ROV"
        rov_subtitle = QLabel("<b>ROV</b>")
        rov_subtitle.setStyleSheet("color: #666; font-size: 10pt; margin-top: 5px;")
        metrics_layout.addWidget(rov_subtitle, row, 0, 1, 2)
        row += 1
        
        # 6. Position ROV
        metrics_layout.addWidget(QLabel("<b>Position ROV:</b>"), row, 0)
        self.position_label = QLabel("(0.00, 0.00) m")
        metrics_layout.addWidget(self.position_label, row, 1)
        row += 1
        
        # 7. Vitesse ROV
        metrics_layout.addWidget(QLabel("<b>Vitesse ROV:</b>"), row, 0)
        self.velocity_label = QLabel("(0.00, 0.00) m/s")
        metrics_layout.addWidget(self.velocity_label, row, 1)
        row += 1
        
        # Trait de séparation
        separator3 = QLabel("─" * 30)
        separator3.setStyleSheet("color: #ccc;")
        metrics_layout.addWidget(separator3, row, 0, 1, 2)
        row += 1
        
        # Sous-titre "Forces s'exerçant sur le ROV"
        forces_subtitle = QLabel("<b>Forces s'exerçant sur le ROV</b>")
        forces_subtitle.setStyleSheet("color: #666; font-size: 10pt; margin-top: 5px;")
        metrics_layout.addWidget(forces_subtitle, row, 0, 1, 2)
        row += 1
        
        # Variables de commande (Fx ROV, Fy ROV)
        metrics_layout.addWidget(QLabel("<b>Commande ROV:</b>"), row, 0)
        self.command_rov_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.command_rov_label, row, 1)
        row += 1
        
        # Forces de traction du câble sur le ROV
        metrics_layout.addWidget(QLabel("<b>Traction câble sur ROV:</b>"), row, 0)
        self.traction_cable_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.traction_cable_label, row, 1)
        row += 1
        
        # 8. Force traînée ROV
        metrics_layout.addWidget(QLabel("<b>Force traînée ROV:</b>"), row, 0)
        self.drag_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.drag_label, row, 1)
        row += 1
        
        # 9. Poids apparent (modèle, vers le bas)
        metrics_layout.addWidget(QLabel("<b>Poids apparent (modèle):</b>"), row, 0)
        self.apparent_weight_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.apparent_weight_label, row, 1)
        row += 1
        
        # 10. Somme des forces ROV (modèle)
        metrics_layout.addWidget(QLabel("<b>Somme forces ROV (modèle):</b>"), row, 0)
        self.total_forces_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.total_forces_label, row, 1)
        row += 1

        # 11. Gamma ROV (accélération verticale)
        metrics_layout.addWidget(QLabel("<b>Gamma ROV (y):</b>"), row, 0)
        self.gamma_rov_label = QLabel("0.00 m/s²")
        metrics_layout.addWidget(self.gamma_rov_label, row, 1)
        row += 1
        
        # Trait de séparation
        separator4 = QLabel("─" * 30)
        separator4.setStyleSheet("color: #ccc;")
        metrics_layout.addWidget(separator4, row, 0, 1, 2)
        row += 1
        
        # Sous-titre "Coordonnées Bateau"
        boat_subtitle = QLabel("<b>Coordonnées Bateau</b>")
        boat_subtitle.setStyleSheet("color: #666; font-size: 10pt; margin-top: 5px;")
        metrics_layout.addWidget(boat_subtitle, row, 0, 1, 2)
        row += 1
        
        # Bateau
        metrics_layout.addWidget(QLabel("<b>Bateau:</b>"), row, 0)
        self.position_boat_label = QLabel("(0.00, 0.00) m")
        metrics_layout.addWidget(self.position_boat_label, row, 1)
        row += 1
        
        # Vitesse Bateau
        metrics_layout.addWidget(QLabel("<b>Vitesse Bateau:</b>"), row, 0)
        self.velocity_boat_label = QLabel("(0.00, 0.00) m/s")
        metrics_layout.addWidget(self.velocity_boat_label, row, 1)
        
        layout.addWidget(metrics_group)
        
        layout.addStretch()
        
        return panel

    def create_profile_panel(self):
        """Crée le panneau du profil câble"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.plotly_widget = PlotlyWidget()
        self.plotly_widget.setMinimumHeight(800)
        self.plotly_widget.setMaximumHeight(1200)
        self.plotly_widget.setMaximumWidth(400)
        self.plotly_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        layout.addWidget(self.plotly_widget, 1)

        # Graphique initial "Profil" - sera remplacé lors de l'initialisation complète
        from src.visualization.plotter import create_system_plot
        y_rov_init = self.main_window.init_params.get('y_rov_init', -10.0)
        x_rov_init = self.main_window.init_params.get('x_rov_init', 0.0)
        fig = create_system_plot(
            x_rov_init,
            y_rov_init,
            [],
            [],
            x_rov_init,
            0,
            "Profil",
            T_cable=None,
        )
        self.plotly_widget.update_figure(fig)

        return panel

    def create_right_panel(self):
        """Crée le panneau de droite avec onglets Métriques et Profil câble"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)

        tabs = QTabWidget()
        tabs.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        tabs.addTab(self.create_metrics_panel(), "Métriques")
        tabs.addTab(self.create_profile_panel(), "Profil câble")
        tabs.addTab(self.create_profile_fond_panel(), "Profil fond")

        layout.addWidget(tabs)
        panel.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        return panel

    def create_profile_fond_panel(self):
        """Crée le panneau du profil fond (zoom près du ROV)."""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.plotly_widget_profile_fond = PlotlyWidget()
        self.plotly_widget_profile_fond.setMinimumHeight(800)
        self.plotly_widget_profile_fond.setMaximumHeight(1200)
        self.plotly_widget_profile_fond.setMaximumWidth(400)
        self.plotly_widget_profile_fond.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        layout.addWidget(self.plotly_widget_profile_fond, 1)

        from src.visualization.plotter import create_rov_local_plot
        y_rov_init = self.main_window.init_params.get('y_rov_init', -10.0)
        x_rov_init = self.main_window.init_params.get('x_rov_init', 0.0)
        fig = create_rov_local_plot(
            x_rov_init,
            y_rov_init,
            [],
            [],
            None,
            None,
            None,
            "Profil fond",
        )
        self.plotly_widget_profile_fond.update_figure(fig)

        return panel
    
    def start_simulation(self):
        """Démarre la simulation"""
        try:
            sc_fx_rov = self._get_sc_fx_rov()

            # Vérifier que les paramètres sont chargés
            if self.main_window.parameters is None:
                QMessageBox.warning(self, "Erreur", "Veuillez charger les paramètres d'environnement.")
                return
            
            # Arrêter la simulation précédente si nécessaire
            if self.simulation_thread and self.simulation_thread.isRunning():
                self.stop_simulation()
            
            # Réinitialiser les plages d'axes pour une nouvelle simulation
            self.current_x_range = None
            self.current_y_range = None
            
            # Récupérer les valeurs de commande depuis les QDoubleSpinBox
            fx_rov = self.fx_rov_input.value()
            fy_rov = self.fy_rov_input.value()
            vx_boat_cmd = self.vx_boat_cmd_input.value()
            dl_dt = self.dl_dt_input.value()
            
            # Stocker les valeurs initiales dans simulation_state
            state = self.main_window.get_simulation_state()
            state['fx_rov'] = fx_rov
            state['fy_rov'] = fy_rov
            state['vx_boat_cmd'] = vx_boat_cmd
            state['dl_dt'] = dl_dt
            state['fx_rov_source'] = "user"
            state['fy_rov_source'] = "user"
            state['vx_boat_cmd_source'] = "user"
            state['dl_dt_source'] = "user"
            state['dl_dt_mode'] = "auto" if self.dl_dt_mode_btn.isChecked() else "scen"
            self.main_window.update_simulation_state(state)

            # Réinitialiser les styles des commandes au démarrage
            self.fx_rov_input.setStyleSheet("color: #000000;")
            self.fy_rov_input.setStyleSheet("color: #000000;")
            self.vx_boat_cmd_input.setStyleSheet("color: #000000;")
            self.dl_dt_input.setStyleSheet("color: #000000;")
            
            # Créer et démarrer le thread de simulation
            self.simulation_thread = SimulationThread(
                self.main_window.parameters,
                self.main_window.calc_params,
                self.main_window.init_params,
                self.main_window.get_simulation_state(),
                fx_rov=fx_rov,
                fy_rov=fy_rov,
                vx_boat_cmd=vx_boat_cmd,
                dl_dt=dl_dt,
                sc_fx_rov=sc_fx_rov,
                sc_fy_rov=self._get_sc_fy_rov(),
                sc_v_bateau=self._get_sc_v_bateau(),
                sc_v_moulinet=self._get_sc_v_moulinet(),
                mission_name=self.main_window.mission_data.get('current') if hasattr(self.main_window, 'mission_data') else None
            )
            
            # Connecter les signaux
            self.simulation_thread.simulation_updated.connect(self.on_simulation_updated)
            self.simulation_thread.simulation_finished.connect(self.on_simulation_finished)
            self.simulation_thread.error_occurred.connect(self.on_simulation_error)
            
            # Démarrer le thread
            self.simulation_thread.start()
            
            # Mettre à jour l'état
            state = self.main_window.get_simulation_state()
            state['running'] = True
            state['paused'] = False
            state['mission_ended'] = False
            if 'mission_end_time' in state:
                state.pop('mission_end_time', None)
            self._mission_end_latched = False
            self._scenario_trigger_index = 0
            self.main_window.update_simulation_state(state)
            self._set_scenario_texts()
            
            # Mettre à jour les boutons
            self.btn_start.setEnabled(False)
            self.btn_pause.setEnabled(True)
            self.btn_restart.setEnabled(False)
            self.status_label.setText("▶ Simulation en cours...")
            self.status_label.setStyleSheet("padding: 8px; background-color: #d4edda; border: 1px solid #c3e6cb;")
            self._update_refresh_button_state()
            
            # Fenêtre Snapshot : bouton Pause -> même effet que « Pause » dans cet onglet
            app = QApplication.instance()
            if app is not None:
                app._cable_snapshot_request_pause = self._pause_simulation_from_snapshot
            
            self.simulation_started.emit()
            
        except Exception as e:
            QMessageBox.critical(self, "Erreur", f"Erreur lors du démarrage de la simulation:\n{str(e)}")
            import traceback
            traceback.print_exc()
    
    def _pause_simulation_from_snapshot(self):
        """Utilisé par la fenêtre modale Snapshot : met en pause sans fermer le dialogue."""
        if not self.simulation_thread or not self.simulation_thread.isRunning():
            return
        state = self.main_window.get_simulation_state()
        if state.get("mission_ended") or state.get("paused"):
            return
        state["paused"] = True
        self.main_window.update_simulation_state("paused", True)
        self.simulation_thread.pause()
        self.btn_pause.setText("▶ Reprendre")
        self.status_label.setText("⏸ Simulation en pause")
        self.status_label.setStyleSheet(
            "padding: 8px; background-color: #fff3cd; border: 1px solid #ffeaa7;"
        )
        self.btn_restart.setEnabled(True)
        self._update_refresh_button_state()

    def stop_simulation(self):
        """Arrête la simulation"""
        app = QApplication.instance()
        if app is not None and getattr(app, "_cable_snapshot_request_pause", None) is self._pause_simulation_from_snapshot:
            app._cable_snapshot_request_pause = None
        # Ne pas réinitialiser les plages à l'arrêt pour garder la vue finale
        if self.simulation_thread and self.simulation_thread.isRunning():
            self.simulation_thread.stop()
            self.simulation_thread.wait(5000)  # Attendre jusqu'à 5 secondes
        
        # Mettre à jour l'état
        state = self.main_window.get_simulation_state()
        state['running'] = False
        state['paused'] = False
        self.main_window.update_simulation_state(state)
        
        # Mettre à jour les boutons
        self.btn_start.setEnabled(True)
        self.btn_pause.setEnabled(False)
        self.btn_restart.setEnabled(True)
        self.btn_pause.setText("⏸ Pause")
        if state.get("mission_ended"):
            self.status_label.setText("✓ Mission terminée (evt x)")
            self.status_label.setStyleSheet("padding: 8px; background-color: #d1ecf1; border: 1px solid #bee5eb;")
            if hasattr(self, "progress_bar"):
                self.progress_bar.setValue(100)
        else:
            self.status_label.setText("Simulation arrêtée")
            self.status_label.setStyleSheet("padding: 8px; background-color: #f8f9fa; border: 1px solid #dee2e6;")
        self._update_refresh_button_state()
        
        self.simulation_stopped.emit()
    
    def pause_resume_simulation(self):
        """Met en pause ou reprend la simulation"""
        if not self.simulation_thread or not self.simulation_thread.isRunning():
            return
        
        state = self.main_window.get_simulation_state()
        if state.get("mission_ended"):
            return
        state['paused'] = not state.get('paused', False)
        self.main_window.update_simulation_state('paused', state['paused'])
        
        # Mettre à jour le thread
        if state['paused']:
            self.simulation_thread.pause()
            self.btn_pause.setText("▶ Reprendre")
            self.status_label.setText("⏸ Simulation en pause")
            self.status_label.setStyleSheet("padding: 8px; background-color: #fff3cd; border: 1px solid #ffeaa7;")
            self.btn_restart.setEnabled(True)
        else:
            self.simulation_thread.resume()
            self.btn_pause.setText("⏸ Pause")
            self.status_label.setText("▶ Simulation en cours...")
            self.status_label.setStyleSheet("padding: 8px; background-color: #d4edda; border: 1px solid #c3e6cb;")
            self.btn_restart.setEnabled(False)
        self._update_refresh_button_state()
    
    def restart_simulation(self):
        """Réinitialise la simulation et remet les commandes à zéro."""
        self.stop_simulation()
        # Synchroniser les paramètres depuis l'onglet Paramètres avant réinitialisation
        params_tab = getattr(self.main_window, "all_parameters_tab", None)
        if params_tab is not None:
            params_tab.save_all_parameters_from_display()
        # Réinitialiser les plages d'axes pour une nouvelle simulation
        self.current_x_range = None
        self.current_y_range = None
        # Réinitialiser les données
        state = self.main_window.get_simulation_state()
        state['data'] = {
            'time': [],
            'x_rov': [],
            'y_rov': [],
            'vx_rov': [],
            'vy_rov': [],
            'x_boat': [],
            'L': [],
            'T_rov': [],
            'T_boat': [],
            'T_max': [],
            'Fx_drag_rov': [],
            'Fy_drag_rov': [],
            'Fx_traction_rov': [],
            'Fy_traction_rov': [],
            'Fy_rov_app_w': [],
            'Fx_rov_total': [],
            'Fy_rov_total': [],
            'x_cable_curr': None,
            'y_cable_curr': None,
        }
        # Remettre les commandes à zéro
        self.fx_rov_input.setValue(0.0)
        self.fy_rov_input.setValue(0.0)
        self.vx_boat_cmd_input.setValue(0.0)
        self.dl_dt_input.setValue(0.0)
        self.fx_rov_input.setStyleSheet("color: #000000;")
        self.fy_rov_input.setStyleSheet("color: #000000;")
        self.vx_boat_cmd_input.setStyleSheet("color: #000000;")
        self.dl_dt_input.setStyleSheet("color: #000000;")
        state['fx_rov'] = 0.0
        state['fy_rov'] = 0.0
        state['vx_boat_cmd'] = 0.0
        state['dl_dt'] = 0.0
        state['fx_rov_source'] = "user"
        state['fy_rov_source'] = "user"
        state['vx_boat_cmd_source'] = "user"
        state['dl_dt_source'] = "user"
        self.main_window.update_simulation_state(state)
        # Réinitialiser le système et les métriques avec les paramètres courants
        self.initialize_system()
        self.update_display()
        if hasattr(self, 'status_label'):
            self.status_label.setText("Lancer la simulation")
            self.status_label.setStyleSheet("padding: 8px; background-color: #f8f9fa; border: 1px solid #dee2e6;")
        if hasattr(self, 'progress_bar'):
            self.progress_bar.setValue(0)
    
    def analyse_mission(self):
        """Analyse les fichiers trace d'une mission et génère un rapport Markdown."""
        state = self.main_window.get_simulation_state()
        running = bool(state.get('running'))
        paused = bool(state.get('paused'))
        if running and not paused:
            QMessageBox.warning(
                self,
                "Analyse mission",
                "Analyse disponible uniquement après la fin de la simulation."
            )
            return
        mission_name = None
        if isinstance(self.main_window.mission_data, dict):
            mission_name = self.main_window.mission_data.get('current')
        if not mission_name or not str(mission_name).strip():
            QMessageBox.warning(self, "Analyse mission", "Aucune mission sélectionnée.")
            return

        mission_dir = get_missions_directory() / str(mission_name)
        if not mission_dir.exists():
            QMessageBox.warning(self, "Analyse mission", f"Répertoire mission introuvable: {mission_dir}")
            return

        csv_files = sorted(mission_dir.glob("*trace.csv"), key=lambda p: p.stat().st_mtime)
        msg_files = sorted(mission_dir.glob("*mess.txt"), key=lambda p: p.stat().st_mtime)

        if not csv_files:
            QMessageBox.warning(self, "Analyse mission", "Aucun fichier trace CSV trouvé.")
            return
        if not msg_files:
            QMessageBox.warning(self, "Analyse mission", "Aucun fichier messages (.txt) trouvé.")
            return

        trace_csv = csv_files[-1]
        trace_msg = msg_files[-1]

        # Lire CSV
        try:
            with open(trace_csv, "r", encoding="utf-8") as f:
                reader = csv.reader(f, delimiter=",")
                header = next(reader, [])
                rows = list(reader)
        except Exception as e:
            QMessageBox.critical(self, "Analyse mission", f"Erreur lecture CSV: {e}")
            return

        def idx(name):
            try:
                return header.index(name)
            except ValueError:
                return None

        i_t = idx("t")
        i_dl = idx("dL/dt")
        i_explain = idx("Explain")
        i_tbat = idx("Tbat")
        i_tmax = idx("Tmax")
        i_trov = idx("Trov")

        def to_float(val):
            try:
                return float(val)
            except Exception:
                return None

        times = []
        dl_vals = []
        tbat_vals = []
        tmax_vals = []
        trov_vals = []
        explain_vals = []
        for row in rows:
            if i_t is not None and i_t < len(row):
                t_val = to_float(row[i_t])
                if t_val is not None:
                    times.append(t_val)
            if i_dl is not None and i_dl < len(row):
                dl_val = to_float(row[i_dl])
                if dl_val is not None:
                    dl_vals.append(dl_val)
            if i_tbat is not None and i_tbat < len(row):
                val = to_float(row[i_tbat])
                if val is not None:
                    tbat_vals.append(val)
            if i_tmax is not None and i_tmax < len(row):
                val = to_float(row[i_tmax])
                if val is not None:
                    tmax_vals.append(val)
            if i_trov is not None and i_trov < len(row):
                val = to_float(row[i_trov])
                if val is not None:
                    trov_vals.append(val)
            if i_explain is not None and i_explain < len(row):
                explain_vals.append((row[i_explain] or "").strip())

        if not rows:
            QMessageBox.warning(self, "Analyse mission", "Le CSV est vide.")
            return

        # Vérifier si toute la mission est en Auto
        if not explain_vals or any(val == "" for val in explain_vals):
            QMessageBox.warning(
                self,
                "Analyse mission",
                "Analyse annulée: la mission n'a pas été exécutée entièrement en mode Auto."
            )
            return

        # Calculs simples
        t_final = times[-1] if times else None
        max_tbat = max(tbat_vals) if tbat_vals else None
        max_tmax = max(tmax_vals) if tmax_vals else None
        max_trov = max(trov_vals) if trov_vals else None

        # Accélération moulinet (|Δ(dL/dt)| / Δt)
        max_acc = None
        if len(times) > 1 and len(dl_vals) > 1:
            acc_vals = []
            for i in range(1, min(len(times), len(dl_vals))):
                dt = times[i] - times[i - 1]
                if dt > 0:
                    acc_vals.append(abs(dl_vals[i] - dl_vals[i - 1]) / dt)
            if acc_vals:
                max_acc = max(acc_vals)

        trupt = None
        try:
            trupt = float(self.main_window.parameters.get("cable", {}).get("tension_rupture", None))
        except Exception:
            trupt = None
        tcible = None
        try:
            tcible = float(self.main_window.calc_params.get("Tcible", None))
        except Exception:
            tcible = None
        gamma_min = None
        gamma_max = None
        try:
            boat_params = self.main_window.parameters.get("boat", {})
            gamma_min = float(boat_params.get("gamma_moulinet_min", None))
        except Exception:
            gamma_min = None
        try:
            boat_params = self.main_window.parameters.get("boat", {})
            gamma_max = float(boat_params.get("gamma_moulinet_max", None))
        except Exception:
            gamma_max = None

        # Analyse messages
        error_lines = []
        try:
            with open(trace_msg, "r", encoding="utf-8") as f:
                for line in f:
                    if "Erreur" in line or "Traceback" in line:
                        error_lines.append(line.strip())
        except Exception:
            error_lines = []

        auto_L_name = "auto_L_1"
        try:
            val = self.main_window.calc_params.get("auto_L", "auto_L_1")
            auto_L_name = val if isinstance(val, str) else f"auto_L_{int(val)}"
        except Exception:
            pass

        # Rédiger le rapport
        timestamp_str = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        report_path = mission_dir / f"{mission_name}_{timestamp_str}_analyse.MD"

        lines = []
        lines.append(f"# Analyse mission {mission_name}")
        lines.append("")
        lines.append("## Fichiers analysés")
        lines.append(f"- CSV: `{trace_csv.name}`")
        lines.append(f"- Messages: `{trace_msg.name}`")
        lines.append("")
        lines.append("## Résumé")
        lines.append(f"- Durée: {t_final:.2f} s" if t_final is not None else "- Durée: n/a")
        lines.append(f"- auto_L utilisé: {auto_L_name}")
        lines.append("")
        lines.append("## Tension câble")
        lines.append(f"- Tbat max: {max_tbat:.2f} N" if max_tbat is not None else "- Tbat max: n/a")
        lines.append(f"- Trov max: {max_trov:.2f} N" if max_trov is not None else "- Trov max: n/a")
        lines.append(f"- Tmax max: {max_tmax:.2f} N" if max_tmax is not None else "- Tmax max: n/a")
        if trupt is not None and max_tmax is not None:
            lines.append(f"- Trupt: {trupt:.2f} N (max/Trupt = {max_tmax / trupt:.3f})")
        if tcible is not None and max_tbat is not None:
            lines.append(f"- Tcible: {tcible:.2f} N (écart max = {abs(max_tbat - tcible):.2f} N)")
        lines.append("")
        lines.append("## Commande dL/dt")
        if max_acc is not None:
            lines.append(f"- Accélération max |d(dL/dt)/dt|: {max_acc:.3f} m/s²")
            if gamma_min is not None and gamma_max is not None:
                lines.append(
                    f"- Gamma_moulinet_min/max: {gamma_min:.3f} / {gamma_max:.3f} m/s²"
                )
            elif gamma_max is not None:
                lines.append(f"- Gamma_moulinet_max: {gamma_max:.3f} m/s²")
        else:
            lines.append("- Accélération max: n/a")
        lines.append("")
        lines.append("## Messages / erreurs")
        if error_lines:
            lines.append(f"- {len(error_lines)} lignes contiennent 'Erreur' ou 'Traceback'")
            lines.append("- Dernières occurrences:")
            for line in error_lines[-5:]:
                lines.append(f"  - {line}")
        else:
            lines.append("- Aucun message d'erreur détecté")
        lines.append("")
        lines.append("## Pistes d'amélioration auto_L")
        lines.append("- Si Tmax/Trupt est trop élevé: augmenter l'agressivité de la libération du câble.")
        lines.append("- Si l'accélération dépasse les bornes: renforcer la limitation d'accélération.")
        lines.append("- Flèche du câble: non analysable sans la profondeur minimale du câble dans le CSV.")

        try:
            with open(report_path, "w", encoding="utf-8") as f:
                f.write("\n".join(lines))
        except Exception as e:
            QMessageBox.critical(self, "Analyse mission", f"Erreur écriture rapport: {e}")
            return

        QMessageBox.information(
            self,
            "Analyse mission",
            f"Rapport créé: {report_path.name}"
        )
    
    def on_simulation_updated(self, data):
        """Appelé quand la simulation met à jour les données"""
        state = self.main_window.get_simulation_state()
        # Créer des copies indépendantes des listes pour éviter les problèmes de référence
        data_copy = {}
        for key, value in data.items():
            if key in ['x_cable_curr', 'y_cable_curr', 'T_cable_curr']:
                # Créer une copie indépendante des données du câble
                if value is not None:
                    if isinstance(value, list):
                        data_copy[key] = list(value)  # Copie de la liste
                    else:
                        data_copy[key] = value
                else:
                    data_copy[key] = None
            elif isinstance(value, list):
                # Pour les autres listes, créer une copie si nécessaire
                data_copy[key] = list(value) if value else []
            else:
                data_copy[key] = value
        state['data'].update(data_copy)
        state['current_time'] = data.get('current_time', state.get('current_time', 0.0))
        self.main_window.update_simulation_state(state)
        if state.get('fx_rov_source') == "scenario":
            fx_value = state.get('fx_rov')
            if fx_value is not None:
                self.apply_scenario_fx_command(float(fx_value))
        if state.get('fy_rov_source') == "scenario":
            fy_value = state.get('fy_rov')
            if fy_value is not None:
                self.apply_scenario_fy_command(float(fy_value))
        if state.get('vx_boat_cmd_source') == "scenario":
            vx_value = state.get('vx_boat_cmd')
            if vx_value is not None:
                self.apply_scenario_vx_boat_command(float(vx_value))
        if state.get('dl_dt_source') == "auto":
            dl_value = state.get('dl_dt')
            if dl_value is not None:
                self.apply_auto_dl_dt_command(float(dl_value))
        elif state.get('dl_dt_source') == "scenario":
            dl_value = state.get('dl_dt')
            if dl_value is not None:
                self.apply_scenario_dl_dt_command(float(dl_value))
        self.simulation_updated.emit(state)
    
    def on_simulation_finished(self):
        """Appelé quand la simulation est terminée"""
        self.stop_simulation()
        state = self.main_window.get_simulation_state()
        if state.get("mission_ended"):
            self.status_label.setText("✓ Mission terminée (evt x)")
        else:
            self.status_label.setText("✓ Simulation terminée")
        self.status_label.setStyleSheet("padding: 8px; background-color: #d1ecf1; border: 1px solid #bee5eb;")
        if hasattr(self, "progress_bar"):
            self.progress_bar.setValue(100)
        self.btn_export.setEnabled(True)
    
    def update_command_in_state(self):
        """Met à jour les valeurs de commande dans simulation_state quand les widgets changent"""
        state = self.main_window.get_simulation_state()
        sender = self.sender()
        if sender is None:
            state['fx_rov'] = self.fx_rov_input.value()
            state['fy_rov'] = self.fy_rov_input.value()
            state['vx_boat_cmd'] = self.vx_boat_cmd_input.value()
            state['dl_dt'] = self.dl_dt_input.value()
        elif sender is self.fx_rov_input:
            state['fx_rov'] = self.fx_rov_input.value()
        elif sender is self.fy_rov_input:
            state['fy_rov'] = self.fy_rov_input.value()
        elif sender is self.vx_boat_cmd_input:
            state['vx_boat_cmd'] = self.vx_boat_cmd_input.value()
        elif sender is self.dl_dt_input:
            state['dl_dt'] = self.dl_dt_input.value()
        self.main_window.update_simulation_state(state)
        if sender is self.fx_rov_input:
            if self._fx_rov_scenario_update:
                self._fx_rov_scenario_update = False
            else:
                self.fx_rov_input.setStyleSheet("color: #000000;")
                state['fx_rov_source'] = "user"
        if sender is self.fy_rov_input:
            if self._fy_rov_scenario_update:
                self._fy_rov_scenario_update = False
            else:
                self.fy_rov_input.setStyleSheet("color: #000000;")
                state['fy_rov_source'] = "user"
        if sender is self.vx_boat_cmd_input:
            if self._vx_boat_scenario_update:
                self._vx_boat_scenario_update = False
            else:
                self.vx_boat_cmd_input.setStyleSheet("color: #000000;")
                state['vx_boat_cmd_source'] = "user"
        if sender is self.dl_dt_input:
            if self._dl_dt_scenario_update:
                self._dl_dt_scenario_update = False
            elif self._dl_dt_auto_update:
                self._dl_dt_auto_update = False
            else:
                self.dl_dt_input.setStyleSheet("color: #000000;")
                state['dl_dt_source'] = "user"

    def apply_scenario_fx_command(self, value: float) -> None:
        """Applique une commande Fx issue d'un scénario et l'affiche en bleu."""
        self._fx_rov_scenario_update = True
        self.fx_rov_input.setStyleSheet("color: #1e6bb8;")
        if self.fx_rov_input.value() != value:
            self.fx_rov_input.setValue(value)

    def apply_scenario_fy_command(self, value: float) -> None:
        """Applique une commande Fy issue d'un scénario et l'affiche en bleu."""
        self._fy_rov_scenario_update = True
        self.fy_rov_input.setStyleSheet("color: #1e6bb8;")
        if self.fy_rov_input.value() != value:
            self.fy_rov_input.setValue(value)

    def apply_scenario_vx_boat_command(self, value: float) -> None:
        """Applique une commande Vx bateau issue d'un scénario et l'affiche en bleu."""
        self._vx_boat_scenario_update = True
        self.vx_boat_cmd_input.setStyleSheet("color: #1e6bb8;")
        if self.vx_boat_cmd_input.value() != value:
            self.vx_boat_cmd_input.setValue(value)

    def apply_scenario_dl_dt_command(self, value: float) -> None:
        """Applique une commande dL/dt issue d'un scénario et l'affiche en bleu."""
        self._dl_dt_scenario_update = True
        self.dl_dt_input.setStyleSheet("color: #1e6bb8;")
        if self.dl_dt_input.value() != value:
            self.dl_dt_input.setValue(value)

    def apply_auto_dl_dt_command(self, value: float) -> None:
        """Applique une commande dL/dt issue du mode auto et l'affiche en rouge."""
        self._dl_dt_auto_update = True
        self.dl_dt_input.setStyleSheet("color: #d9534f;")
        if self.dl_dt_input.value() != value:
            self.dl_dt_input.setValue(value)

    def _get_sc_fx_rov(self) -> str:
        """Retourne le scénario Fx ROV depuis l'UI si disponible, sinon via calc_params."""
        tab = getattr(self.main_window, "all_parameters_tab", None)
        if tab is not None and hasattr(tab, "sc_fx_rov"):
            if hasattr(tab.sc_fx_rov, "toPlainText"):
                text = (tab.sc_fx_rov.toPlainText() or "").strip()
            else:
                text = (tab.sc_fx_rov.text() or "").strip()
            if text:
                return text
        return (self.main_window.calc_params.get('sc_fx_rov', '') or "").strip()

    def _get_sc_fy_rov(self) -> str:
        """Retourne le scénario Fy ROV depuis l'UI si disponible, sinon via calc_params."""
        tab = getattr(self.main_window, "all_parameters_tab", None)
        if tab is not None and hasattr(tab, "sc_fy_rov"):
            if hasattr(tab.sc_fy_rov, "toPlainText"):
                text = (tab.sc_fy_rov.toPlainText() or "").strip()
            else:
                text = (tab.sc_fy_rov.text() or "").strip()
            if text:
                return text
        return (self.main_window.calc_params.get('sc_fy_rov', '') or "").strip()

    def _get_sc_v_bateau(self) -> str:
        """Retourne le scénario Vx bateau depuis l'UI si disponible, sinon via calc_params."""
        tab = getattr(self.main_window, "all_parameters_tab", None)
        if tab is not None and hasattr(tab, "sc_v_bateau"):
            text = (tab.sc_v_bateau.text() or "").strip()
            if text:
                return text
        return (self.main_window.calc_params.get('sc_v_bateau', '') or "").strip()

    def _get_sc_v_moulinet(self) -> str:
        """Retourne le scénario dL/dt depuis l'UI si disponible, sinon via calc_params."""
        tab = getattr(self.main_window, "all_parameters_tab", None)
        if tab is not None and hasattr(tab, "sc_v_moulinet"):
            text = (tab.sc_v_moulinet.text() or "").strip()
            if text:
                return text
        return (self.main_window.calc_params.get('sc_v_moulinet', '') or "").strip()

    def _render_scenario_html(self, raw_text: str, last_couple: str | None) -> str:
        raw_text = raw_text or ""
        last_couple = (last_couple or "").strip()
        text_html = None
        if last_couple and last_couple in raw_text:
            before, after = raw_text.rsplit(last_couple, 1)
            highlighted = (
                '<b><span style="color:#1e6bb8;">'
                + html.escape(last_couple)
                + "</span></b>"
            )
            text_html = html.escape(before) + highlighted + html.escape(after)
        elif last_couple:
            def _find_span_ignore_spaces(text: str, needle: str):
                text_norm = []
                idx_map = []
                for i, ch in enumerate(text):
                    if not ch.isspace():
                        text_norm.append(ch)
                        idx_map.append(i)
                needle_norm = "".join(ch for ch in needle if not ch.isspace())
                if not needle_norm:
                    return None
                norm_str = "".join(text_norm)
                pos = norm_str.rfind(needle_norm)
                if pos < 0:
                    return None
                start = idx_map[pos]
                end = idx_map[pos + len(needle_norm) - 1] + 1
                return start, end

            span = _find_span_ignore_spaces(raw_text, last_couple)
            if span:
                start, end = span
                before = raw_text[:start]
                middle = raw_text[start:end]
                after = raw_text[end:]
                highlighted = (
                    '<b><span style="color:#1e6bb8;">'
                    + html.escape(middle)
                    + "</span></b>"
                )
                text_html = html.escape(before) + highlighted + html.escape(after)
        if text_html is None:
            text_html = html.escape(raw_text)
        return text_html.replace("\n", "<br>")

    def _update_scenario_field(self, label: str) -> None:
        raw_text = self._scenario_raw_text.get(label, "")
        last_couple = self._scenario_last_triggers.get(label)
        html_text = self._render_scenario_html(raw_text, last_couple)
        if label == "Fx" and hasattr(self, "scenario_fx_text"):
            self.scenario_fx_text.setHtml(html_text)
        elif label == "Fy" and hasattr(self, "scenario_fy_text"):
            self.scenario_fy_text.setHtml(html_text)
        elif label == "Bat" and hasattr(self, "scenario_boat_text"):
            self.scenario_boat_text.setHtml(html_text)
        elif label == "Moul" and hasattr(self, "scenario_moulinet_text"):
            self.scenario_moulinet_text.setHtml(html_text)

    def _set_scenario_texts(self) -> None:
        self._scenario_raw_text = {
            "Fx": self._get_sc_fx_rov(),
            "Fy": self._get_sc_fy_rov(),
            "Bat": self._get_sc_v_bateau(),
            "Moul": self._get_sc_v_moulinet(),
        }
        for label in ("Fx", "Fy", "Bat", "Moul"):
            self._update_scenario_field(label)

    def on_dl_dt_mode_toggled(self, checked: bool) -> None:
        """Bascule le mode de commande dL/dt entre Auto et Scen."""
        self.dl_dt_mode_btn.setText("Auto" if checked else "Scen")
        state = self.main_window.get_simulation_state()
        state['dl_dt_mode'] = "auto" if checked else "scen"
        self.main_window.update_simulation_state(state)
    
    def on_simulation_error(self, error_msg):
        """Appelé en cas d'erreur dans la simulation"""
        QMessageBox.critical(self, "Erreur de simulation", f"Une erreur est survenue:\n{error_msg}")
        self.stop_simulation()

    def _update_refresh_button_state(self):
        state = self.main_window.get_simulation_state()
        running = bool(state.get('running'))
        paused = bool(state.get('paused'))
        self.btn_refresh_graphs.setEnabled(running or paused)

    def refresh_graphs(self):
        """Force le rafraîchissement de tous les graphiques."""
        try:
            for widget in (
                getattr(self, 'plotly_widget', None),
                getattr(self, 'plotly_widget_tension', None),
                getattr(self, 'plotly_widget_slack', None),
                getattr(self, 'plotly_widget_current', None),
                getattr(self, 'plotly_widget_dl_dt', None),
                getattr(self, 'plotly_widget_tension_vs_target', None),
                getattr(self, 'plotly_widget_profile_fond', None),
            ):
                if widget is not None:
                    widget.is_first_update = True
            if hasattr(self, '_tension_graph_initialized'):
                delattr(self, '_tension_graph_initialized')
            self.last_plot_update_time = None
            
            # Récupérer les données depuis simulation_state AVANT d'appeler update_display
            # pour s'assurer qu'on utilise les données les plus récentes
            state = self.main_window.get_simulation_state()
            data = state.get('data', {})
            
            # Mettre à jour l'affichage général
            self.update_display()
            
            # Forcer le rafraîchissement explicite des graphiques avec les données récupérées
            if data.get('dl_dt_cmd') is not None:
                self._refresh_dl_dt_plot(data)
            if data.get('time'):
                self._refresh_tension_vs_target_plot(data)
        except Exception as e:
            trace_print(8, f"Erreur lors du rafraîchissement des graphiques: {e}")

    def _refresh_dl_dt_plot(self, data):
        if data.get('dl_dt_cmd') is None:
            return
        try:
            from src.visualization.plotter import create_dl_dt_plot
            fig_dl_dt = create_dl_dt_plot(
                data.get('time', []),
                data.get('dl_dt_cmd', []),
                "Commande dL/dt",
                data.get('cable_mode', []),
                data.get('scenario_triggers', []),
            )
            self.plotly_widget_dl_dt.update_figure(fig_dl_dt)
        except Exception as e:
            trace_print(8, f"Erreur lors de la mise à jour du graphique dL/dt: {e}")

    def _refresh_tension_vs_target_plot(self, data):
        if not data.get('time'):
            return
        try:
            from src.visualization.plotter import create_tension_vs_target_plot

            times = data.get('time', [])
            t_rupture = None
            try:
                t_rupture = float(self.main_window.parameters.get('cable', {}).get('tension_rupture', 50.0))
            except Exception:
                t_rupture = 50.0
            t_cible = None
            try:
                t_cible = float(self.main_window.calc_params.get('Tcible', None))
            except Exception:
                t_cible = None
            if t_cible is None or t_cible == 0:
                t_cible = t_rupture / 2.0 if t_rupture is not None else None

            t_boat = data.get('T_boat', []) or []
            t_rov = data.get('T_rov', []) or []
            t_max = data.get('T_max', []) or []

            min_len = min(len(times), len(t_boat), len(t_rov), len(t_max))
            if min_len == 0:
                return
            times = times[:min_len]
            t_boat = t_boat[:min_len]
            t_rov = t_rov[:min_len]
            t_max = t_max[:min_len]
            cable_mode = (data.get('cable_mode') or [])[:min_len]
            scenario_triggers = (data.get('scenario_triggers') or [])[:min_len]

            max_points = 1500
            step = max(1, len(times) // max_points)
            times_ds = times[::step]
            t_rupture_series = [t_rupture] * len(times_ds) if t_rupture is not None else []
            t_cible_series = [t_cible] * len(times_ds) if t_cible is not None else []
            cable_mode_ds = cable_mode[::step] if cable_mode else []
            scenario_triggers_ds = scenario_triggers[::step] if scenario_triggers else []

            fig_tension_vs_target = create_tension_vs_target_plot(
                times_ds,
                t_rupture_series,
                t_cible_series,
                t_boat[::step],
                t_rov[::step],
                t_max[::step],
                "Tension vs cible",
                cable_mode_ds,
                scenario_triggers_ds,
            )
            self.plotly_widget_tension_vs_target.update_figure(fig_tension_vs_target)
        except Exception as e:
            trace_print(8, f"Erreur lors de la mise à jour du graphique Tension vs cible: {e}")
    
    def update_current_profile_plot(self, y_ref=None):
        """Met à jour le graphique du profil de courant"""
        try:
            import numpy as np
            from src.models.environment import Environment
            from src.visualization.plotter import create_current_profile_plot
            
            state = self.main_window.get_simulation_state()
            system = state.get('system')
            if system is not None:
                env = system.environment
                v_courant = getattr(env, "v_courant_raw", self.main_window.init_params.get('v_courant', "0.0"))
            else:
                env = Environment({})
                v_courant = self.main_window.init_params.get('v_courant', "0.0")
            
            if y_ref is None:
                y_ref = self.main_window.init_params.get('y_rov_init', -10.0)
            
            depth_min = min(float(y_ref), -10.0)
            parsed = env._parse_current_profile_string(v_courant)
            if isinstance(parsed, tuple):
                profile_depths, _ = parsed
                if profile_depths:
                    if any(d < 0 for d in profile_depths):
                        depth_min = min(depth_min, float(min(profile_depths)))
                    else:
                        depth_min = min(depth_min, -float(max(profile_depths)))
            
            depths = np.linspace(0.0, depth_min, 60)
            speeds = [env.get_current_velocity(y, v_courant) for y in depths]
            
            fig_current = create_current_profile_plot(
                depths.tolist(), speeds, "Profil du courant"
            )
            self.plotly_widget_current.update_figure(fig_current)
        except Exception as e:
            trace_print(8, f"Erreur lors de la mise à jour du graphique Courant: {e}")
            import traceback
            traceback.print_exc()
    
    def update_display(self):
        """Met à jour l'affichage avec les dernières données"""
        # Mettre à jour l'affichage de la mission
        self.update_mission_display()
        
        state = self.main_window.get_simulation_state()
        data = state.get('data', {})
        if state.get("mission_ended") or self._mission_end_latched:
            self._mission_end_latched = True
            if state.get("running") or state.get("paused"):
                state['running'] = False
                state['paused'] = False
                self.main_window.update_simulation_state(state)
            if hasattr(self, "status_label"):
                self.status_label.setText("✓ Mission terminée (evt x)")
                self.status_label.setStyleSheet("padding: 8px; background-color: #d1ecf1; border: 1px solid #bee5eb;")
            if hasattr(self, "progress_bar"):
                self.progress_bar.setValue(100)

        # Mettre à jour l'affichage des scénarios déclenchés
        triggers = data.get("scenario_triggers") or []
        if triggers and len(triggers) > self._scenario_trigger_index:
            new_steps = triggers[self._scenario_trigger_index :]
            for step in new_steps:
                for trigger_text in step or []:
                    if not isinstance(trigger_text, str):
                        continue
                    if ":" not in trigger_text:
                        continue
                    label, couple = trigger_text.split(":", 1)
                    label = label.strip()
                    couple = couple.strip()
                    if label in {"Fx", "Fy", "Bat", "Moul"}:
                        self._scenario_last_triggers[label] = couple
                        self._update_scenario_field(label)
            self._scenario_trigger_index = len(triggers)

        # Répartition du slack le long du câble
        if getattr(self, "plotly_widget_slack_dist", None) is not None:
            try:
                x_cable_raw = data.get('x_cable_curr')
                y_cable_raw = data.get('y_cable_curr')
                if x_cable_raw is not None and y_cable_raw is not None:
                    import numpy as np
                    x_cable = np.asarray(x_cable_raw, dtype=float)
                    y_cable = np.asarray(y_cable_raw, dtype=float)
                    if len(x_cable) > 1 and len(y_cable) > 1:
                        dx = np.diff(x_cable)
                        dy = np.diff(y_cable)
                        ds = np.hypot(dx, dy)
                        n_seg = len(ds)
                        total_ds = float(np.sum(ds))
                        L_current = None
                        if data.get('L'):
                            try:
                                L_current = float(data['L'][-1])
                            except Exception:
                                L_current = None
                        if L_current is None or L_current <= 1e-9:
                            L_current = total_ds
                        scale = L_current / total_ds if total_ds > 1e-9 else 1.0
                        ds_eff = ds * scale
                        L_straight = float(np.hypot(x_cable[-1] - x_cable[0], y_cable[-1] - y_cable[0]))
                        ds_straight = L_straight / max(n_seg, 1)
                        slack_local = (ds_eff - ds_straight).tolist()
                        s_cum = np.cumsum(ds_eff)
                        s_mid = (s_cum - 0.5 * ds_eff).tolist()
                        from src.visualization.plotter import create_slack_distribution_plot
                        fig_slack_dist = create_slack_distribution_plot(
                            s_mid, slack_local, "Répartition slack"
                        )
                        self.plotly_widget_slack_dist.update_figure(fig_slack_dist)
            except Exception as e:
                trace_print(8, f"Erreur lors de la mise à jour du graphique Répartition slack: {e}")
        
        
        # Mettre à jour les métriques
        if data.get('time'):
            current_time = data['time'][-1] if data['time'] else 0.0
            self.time_label.setText(f"{current_time:.2f} s")
            
            # Calculer le pourcentage de progression
            t_final = self.main_window.calc_params.get('t_final', 60.0)
            if not self._mission_end_latched:
                progress = int((current_time / t_final) * 100) if t_final > 0 else 0
                self.progress_bar.setValue(min(progress, 100))
            
            if data.get('x_rov') and data.get('y_rov'):
                x_rov = data['x_rov'][-1]
                y_rov = data['y_rov'][-1]
                self.position_label.setText(f"({x_rov:.2f}, {y_rov:.2f}) m")
            
            if data.get('vx_rov') and data.get('vy_rov'):
                vx_rov = data['vx_rov'][-1]
                vy_rov = data['vy_rov'][-1]
                self.velocity_label.setText(f"({vx_rov:.2f}, {vy_rov:.2f}) m/s")
            
            if data.get('L_step') is not None:
                L = float(data.get('L_step', 0.0))
                self.length_label.setText(f"{L:.2f} m")
            elif data.get('L'):
                L = data['L'][-1]
                self.length_label.setText(f"{L:.2f} m")
            else:
                L = 0.0
                self.length_label.setText("0.00 m")

            # Longueur droite bateau -> ROV
            # Utiliser D_straight depuis les données (calculé après recalcul du ROV dans simulation_thread)
            # D_straight est calculé avec les valeurs recalculées de x_rov et y_rov
            if data.get('D_straight') and len(data['D_straight']) > 0:
                try:
                    D_straight = float(data['D_straight'][-1])
                    self.length_straight_label.setText(f"{D_straight:.2f} m")
                    # Slack = L - D_straight (formule correcte)
                    # Si L < D_straight, le slack est négatif (câble tendu)
                    # Si L > D_straight, le slack est positif (câble avec courbure)
                    slack = L - D_straight
                    self.slack_label.setText(f"{slack:.2f} m")
                    if slack < 0.0:
                        self.slack_label.setStyleSheet("color: #c62828; font-weight: bold;")
                    else:
                        self.slack_label.setStyleSheet("")
                except Exception:
                    self.length_straight_label.setText("0.00 m")
                    self.slack_label.setText("0.00 m")
                    self.slack_label.setStyleSheet("")
            else:
                # Fallback : calculer depuis les positions si D_straight n'est pas disponible
                if data.get('x_boat') and data.get('x_rov') and data.get('y_rov'):
                    try:
                        x_boat = float(data['x_boat'][-1])
                        x_rov = float(data['x_rov'][-1])
                        y_rov = float(data['y_rov'][-1])
                        straight_len = (x_rov - x_boat) ** 2 + (y_rov - 0.0) ** 2
                        D_straight = straight_len ** 0.5
                        self.length_straight_label.setText(f"{D_straight:.2f} m")
                        slack = L - D_straight
                        self.slack_label.setText(f"{slack:.2f} m")
                        if slack < 0.0:
                            self.slack_label.setStyleSheet("color: #c62828; font-weight: bold;")
                        else:
                            self.slack_label.setStyleSheet("")
                    except Exception:
                        self.length_straight_label.setText("0.00 m")
                        self.slack_label.setText("0.00 m")
                        self.slack_label.setStyleSheet("")
                else:
                    self.length_straight_label.setText("0.00 m")
                    self.slack_label.setText("0.00 m")
                    self.slack_label.setStyleSheet("")

            if data.get('cable_mode'):
                last_mode = data['cable_mode'][-1]
                mode_label = "Caténaire" if last_mode == "catenary" else "Ligne droite"
                self.cable_mode_label.setText(mode_label)
            else:
                self.cable_mode_label.setText("-")

            x_cable_raw = data.get('x_cable_curr')
            y_cable_raw = data.get('y_cable_curr')
            if x_cable_raw is not None and y_cable_raw is not None:
                try:
                    import numpy as np
                    x_cable = np.asarray(x_cable_raw)
                    y_cable = np.asarray(y_cable_raw)
                    if len(x_cable) > 1 and len(y_cable) > 1:
                        ds = np.hypot(np.diff(x_cable), np.diff(y_cable))
                        length_segments = float(np.sum(ds))
                        drift = float(length_segments - L)
                        self.length_segments_label.setText(f"{length_segments:.2f} m")
                        self.length_drift_label.setText(f"{drift:.2f} m")
                        if abs(drift) > 1e-2:
                            self.length_drift_label.setStyleSheet("color: #c62828; font-weight: bold;")
                        else:
                            self.length_drift_label.setStyleSheet("")
                    else:
                        self.length_segments_label.setText("0.00 m")
                        drift = float(0.0 - L)
                        self.length_drift_label.setText(f"{drift:.2f} m")
                        if abs(drift) > 1e-2:
                            self.length_drift_label.setStyleSheet("color: #c62828; font-weight: bold;")
                        else:
                            self.length_drift_label.setStyleSheet("")
                except Exception:
                    self.length_segments_label.setText("0.00 m")
                    drift = float(0.0 - L)
                    self.length_drift_label.setText(f"{drift:.2f} m")
                    if abs(drift) > 1e-2:
                        self.length_drift_label.setStyleSheet("color: #c62828; font-weight: bold;")
                    else:
                        self.length_drift_label.setStyleSheet("")
            else:
                self.length_segments_label.setText("0.00 m")
                drift = float(0.0 - L)
                self.length_drift_label.setText(f"{drift:.2f} m")
                if abs(drift) > 1e-2:
                    self.length_drift_label.setStyleSheet("color: #c62828; font-weight: bold;")
                else:
                    self.length_drift_label.setStyleSheet("")
            
            if data.get('T_max'):
                T_max = data['T_max'][-1] if data['T_max'] else 0.0
                self.tension_label.setText(f"{T_max:.2f} N")
            
            if data.get('T_boat'):
                T_boat = data['T_boat'][-1] if data['T_boat'] else 0.0
                self.tension_boat_label.setText(f"{T_boat:.2f} N")
            else:
                self.tension_boat_label.setText("0.00 N")
            
            if data.get('T_rov'):
                T_rov = data['T_rov'][-1] if data['T_rov'] else 0.0
                self.tension_rov_label.setText(f"{T_rov:.2f} N")
            else:
                self.tension_rov_label.setText("0.00 N")

            if data.get('dl_dt_cmd') is not None:
                try:
                    from src.visualization.plotter import create_dl_dt_plot

                    fig_dl_dt = create_dl_dt_plot(
                        data.get('time', []),
                        data.get('dl_dt_cmd', []),
                        "Commande dL/dt",
                        data.get('cable_mode', []),
                        data.get('scenario_triggers', []),
                    )
                    self.plotly_widget_dl_dt.update_figure(fig_dl_dt)
                except Exception as e:
                    trace_print(8, f"Erreur lors de la mise à jour du graphique dL/dt: {e}")

            if data.get('time'):
                try:
                    from src.visualization.plotter import create_slack_plot
                    times = data.get('time', [])
                    x_rov = data.get('x_rov', [])
                    y_rov = data.get('y_rov', [])
                    x_boat = data.get('x_boat', [])
                    L_series = data.get('L', [])
                    t_boat = data.get('T_boat', [])
                    dl_dt_series = data.get('dl_dt_cmd', [])

                    n = min(len(times), len(x_rov), len(y_rov), len(x_boat), len(L_series))
                    slack = []
                    slack_ratio = []
                    hover_data = []
                    for i in range(n):
                        l_val = float(L_series[i])
                        straight = float((x_rov[i] - x_boat[i]) ** 2 + (y_rov[i] - 0.0) ** 2) ** 0.5
                        s_val = l_val - straight
                        slack.append(s_val)
                        slack_ratio.append(s_val / l_val if l_val > 1e-6 else 0.0)
                        t_bat_val = float(t_boat[i]) if i < len(t_boat) else 0.0
                        dl_val = float(dl_dt_series[i]) if i < len(dl_dt_series) else 0.0
                        hover_data.append([
                            float(x_rov[i]),
                            float(y_rov[i]),
                            t_bat_val,
                            dl_val,
                        ])

                    fig_slack = create_slack_plot(
                        times[:n],
                        slack,
                        slack_ratio,
                        hover_data,
                        "Slack / L  (%)",
                    )
                    self.plotly_widget_slack.update_figure(fig_slack)
                except Exception as e:
                    trace_print(8, f"Erreur lors de la mise à jour du graphique Slack: {e}")

            if data.get('time'):
                try:
                    self._refresh_tension_vs_target_plot(data)
                except Exception as e:
                    trace_print(8, f"Erreur lors de la mise à jour du graphique Tension vs cible: {e}")

            
            # Tensions du câble (forces exercées sur le câble)
            traction_boat = (0.0, 0.0)
            traction_rov = (0.0, 0.0)
            if data.get('T_boat'):
                T_boat = data['T_boat'][-1] if data['T_boat'] else 0.0
                x_cable_raw = data.get('x_cable_curr')
                y_cable_raw = data.get('y_cable_curr')
                if x_cable_raw is not None and y_cable_raw is not None:
                    import numpy as np
                    x_cable = np.asarray(x_cable_raw)
                    y_cable = np.asarray(y_cable_raw)
                    if len(x_cable) > 1 and len(y_cable) > 1:
                        x_boat = data['x_boat'][-1] if data.get('x_boat') else 0.0
                        x_rov = data['x_rov'][-1]
                        y_rov = data['y_rov'][-1]
                        dist_first_to_boat = np.sqrt((x_cable[0] - x_boat)**2 + (y_cable[0] - 0.0)**2)
                        dist_first_to_rov = np.sqrt((x_cable[0] - x_rov)**2 + (y_cable[0] - y_rov)**2)
                        if dist_first_to_rov < dist_first_to_boat:
                            x_cable = np.flip(x_cable)
                            y_cable = np.flip(y_cable)
                        
                        dx_boat = x_cable[1] - x_cable[0]
                        dy_boat = y_cable[1] - y_cable[0]
                        ds_boat = np.sqrt(dx_boat**2 + dy_boat**2)
                        if ds_boat > 1e-6:
                            uboat_x = dx_boat / ds_boat
                            uboat_y = dy_boat / ds_boat
                            # Force exercée par le bateau sur le câble = opposée à U_bateau (cohérent avec _trace_cable_equilibrium_forces)
                            traction_boat = (-T_boat * uboat_x, -T_boat * uboat_y)
                            self.traction_boat_label.setText(f"({traction_boat[0]:.2f}, {traction_boat[1]:.2f}) N")
                        else:
                            self.traction_boat_label.setText("(0.00, 0.00) N")
                else:
                    self.traction_boat_label.setText("(0.00, 0.00) N")
            
            if data.get('T_rov'):
                T_rov = data['T_rov'][-1] if data['T_rov'] else 0.0
                x_cable_raw = data.get('x_cable_curr')
                y_cable_raw = data.get('y_cable_curr')
                if x_cable_raw is not None and y_cable_raw is not None:
                    import numpy as np
                    x_cable = np.asarray(x_cable_raw)
                    y_cable = np.asarray(y_cable_raw)
                    if len(x_cable) > 1 and len(y_cable) > 1:
                        x_boat = data['x_boat'][-1] if data.get('x_boat') else 0.0
                        x_rov = data['x_rov'][-1]
                        y_rov = data['y_rov'][-1]
                        dist_first_to_boat = np.sqrt((x_cable[0] - x_boat)**2 + (y_cable[0] - 0.0)**2)
                        dist_first_to_rov = np.sqrt((x_cable[0] - x_rov)**2 + (y_cable[0] - y_rov)**2)
                        if dist_first_to_rov < dist_first_to_boat:
                            x_cable = np.flip(x_cable)
                            y_cable = np.flip(y_cable)
                        
                        dx_rov = x_cable[-1] - x_cable[-2]
                        dy_rov = y_cable[-1] - y_cable[-2]
                        ds_rov = np.sqrt(dx_rov**2 + dy_rov**2)
                        if ds_rov > 1e-6:
                            urov_x = dx_rov / ds_rov
                            urov_y = dy_rov / ds_rov
                            # Force exercée par le ROV sur le câble = même sens que U_rov (cohérent avec _trace_cable_equilibrium_forces)
                            traction_rov = (T_rov * urov_x, T_rov * urov_y)
                            self.traction_rov_label.setText(f"({traction_rov[0]:.2f}, {traction_rov[1]:.2f}) N")
                        else:
                            self.traction_rov_label.setText("(0.00, 0.00) N")
                else:
                    self.traction_rov_label.setText("(0.00, 0.00) N")
            
            # Poids apparent et force traînée du câble
            try:
                system = state.get('system')
                L_cable = data['L'][-1] if data.get('L') else None
                if system is not None and L_cable is not None:
                    # Calculer le poids apparent total (positif si le câble est plus dense que l'eau, vers le bas)
                    N_segments = max(len(x_cable_raw) - 1, 1) if x_cable_raw is not None else 1
                    ds = L_cable / N_segments if N_segments > 0 else L_cable
                    from src.solvers.forces import compute_cable_apparent_weight
                    Fy_weight_per_seg = compute_cable_apparent_weight(
                        system.cable.rho_cable,
                        system.environment.rho_eau,
                        system.cable.A_cable,
                        system.environment.g,
                        ds
                    )
                    Fy_weight_total = float(Fy_weight_per_seg * N_segments)
                    # Convention cohérente avec _trace_cable_equilibrium_forces : poids apparent positif vers le bas
                    cable_weight_vec = (0.0, Fy_weight_total)
                    self.cable_apparent_weight_label.setText(f"({cable_weight_vec[0]:.2f}, {cable_weight_vec[1]:.2f}) N")
                    
                    use_thread_drag = False
                    cable_drag_hist = data.get('cable_drag')
                    if cable_drag_hist:
                        last_drag = cable_drag_hist[-1]
                        if isinstance(last_drag, (list, tuple)) and len(last_drag) == 2:
                            cable_drag_vec = (float(last_drag[0]), float(last_drag[1]))
                            self.cable_drag_label.setText(
                                f"({cable_drag_vec[0]:.2f}, {cable_drag_vec[1]:.2f}) N"
                            )
                            total_fx = traction_rov[0] + traction_boat[0] + cable_weight_vec[0] + cable_drag_vec[0]
                            total_fy = traction_rov[1] + traction_boat[1] + cable_weight_vec[1] + cable_drag_vec[1]
                            self.cable_total_forces_label.setText(f"({total_fx:.2f}, {total_fy:.2f}) N")
                            use_thread_drag = True

                    x_cable_raw = data.get('x_cable_curr')
                    y_cable_raw = data.get('y_cable_curr')
                    is_paused = bool(state.get('paused')) if isinstance(state, dict) else False
                    if not use_thread_drag and x_cable_raw is not None and y_cable_raw is not None:
                        import numpy as np
                        from src.solvers.forces import compute_cable_forces
                        
                        x_cable = list(x_cable_raw) if not isinstance(x_cable_raw, list) else x_cable_raw
                        y_cable = list(y_cable_raw) if not isinstance(y_cable_raw, list) else y_cable_raw
                        if len(x_cable) > 1 and len(y_cable) > 1:
                            # Estimer les vitesses du câble à partir de deux positions successives
                            vx_cable = np.zeros(len(x_cable))
                            vy_cable = np.zeros(len(x_cable))
                            t_now = data['time'][-1] if data.get('time') else None
                            if (
                                t_now is not None
                                and self._prev_cable_time is not None
                                and self._prev_cable_x is not None
                                and self._prev_cable_y is not None
                            ):
                                dt = float(t_now) - float(self._prev_cable_time)
                                is_paused = bool(state.get('paused')) if isinstance(state, dict) else False
                                if dt > 1e-6 and len(self._prev_cable_x) == len(x_cable) and not is_paused:
                                    vx_cable = (np.asarray(x_cable) - np.asarray(self._prev_cable_x)) / dt
                                    vy_cable = (np.asarray(y_cable) - np.asarray(self._prev_cable_y)) / dt
                                    # Limiter les vitesses extrêmes (sauts numériques)
                                    vx_rov = data['vx_rov'][-1] if data.get('vx_rov') else 0.0
                                    vy_rov = data['vy_rov'][-1] if data.get('vy_rov') else 0.0
                                    vx_boat = data['vx_boat'][-1] if data.get('vx_boat') else 0.0
                                    v_cap = max(1.0, 2.0 * max(abs(vx_rov), abs(vy_rov), abs(vx_boat)))
                                    vx_cable = np.clip(vx_cable, -v_cap, v_cap)
                                    vy_cable = np.clip(vy_cable, -v_cap, v_cap)
                                params_cable = {
                                    'd': system.cable.d,
                                    'rho_cable': system.cable.rho_cable,
                                    'Cx_cable': system.cable.Cx_cable,
                                    'Cf_cable': system.cable.Cf_cable
                                }
                            if is_paused and self._prev_cable_drag is not None:
                                cable_drag_vec = self._prev_cable_drag
                            else:
                                Fx_segments, Fy_segments, _, _, _, _ = compute_cable_forces(
                                    x_cable, y_cable, vx_cable, vy_cable, system.environment, params_cable, L_cable
                                )
                                F_drag_cable_x = float(np.sum(Fx_segments)) if len(Fx_segments) > 0 else 0.0
                                # Retirer le poids apparent pour ne garder que la traînée verticale
                                from src.solvers.forces import compute_cable_apparent_weight
                                N_segments = max(len(x_cable) - 1, 1)
                                ds = L_cable / N_segments if N_segments > 0 else L_cable
                                Fy_weight_seg = compute_cable_apparent_weight(
                                    system.cable.rho_cable,
                                    system.environment.rho_eau,
                                    system.cable.A_cable,
                                    system.environment.g,
                                    ds
                                )
                                F_weight_total = Fy_weight_seg * N_segments
                                F_drag_cable_y = (float(np.sum(Fy_segments)) - F_weight_total) if len(Fy_segments) > 0 else 0.0
                                cable_drag_vec = (F_drag_cable_x, F_drag_cable_y)
                                # Lissage simple de la traînée pour éviter le bagottement visuel
                                if self._prev_cable_drag is not None:
                                    alpha = 0.3
                                    cable_drag_vec = (
                                        alpha * cable_drag_vec[0] + (1.0 - alpha) * self._prev_cable_drag[0],
                                        alpha * cable_drag_vec[1] + (1.0 - alpha) * self._prev_cable_drag[1],
                                    )
                                self._prev_cable_drag = cable_drag_vec

                            if cable_drag_vec is not None:
                                self.cable_drag_label.setText(
                                    f"({cable_drag_vec[0]:.2f}, {cable_drag_vec[1]:.2f}) N"
                                )
                            
                            total_fx = traction_rov[0] + traction_boat[0] + cable_weight_vec[0] + cable_drag_vec[0]
                            total_fy = traction_rov[1] + traction_boat[1] + cable_weight_vec[1] + cable_drag_vec[1]
                            self.cable_total_forces_label.setText(f"({total_fx:.2f}, {total_fy:.2f}) N")
                            
                            # Mémoriser la dernière géométrie pour estimer les vitesses
                            if t_now is not None:
                                self._prev_cable_time = float(t_now)
                                self._prev_cable_x = list(x_cable)
                                self._prev_cable_y = list(y_cable)
            except Exception:
                pass
            
            # Direction du câble (vecteur unitaire, sens s croissant : bateau -> ROV)
            x_cable_raw = data.get('x_cable_curr')
            y_cable_raw = data.get('y_cable_curr')
            if x_cable_raw is not None and y_cable_raw is not None:
                import numpy as np
                x_cable = np.asarray(x_cable_raw)
                y_cable = np.asarray(y_cable_raw)
                if len(x_cable) > 1 and len(y_cable) > 1:
                    x_boat = data['x_boat'][-1] if data.get('x_boat') else 0.0
                    x_rov = data['x_rov'][-1]
                    y_rov = data['y_rov'][-1]
                    dist_first_to_boat = np.sqrt((x_cable[0] - x_boat)**2 + (y_cable[0] - 0.0)**2)
                    dist_first_to_rov = np.sqrt((x_cable[0] - x_rov)**2 + (y_cable[0] - y_rov)**2)
                    if dist_first_to_rov < dist_first_to_boat:
                        x_cable = np.flip(x_cable)
                        y_cable = np.flip(y_cable)
                    
                    dx_boat = x_cable[1] - x_cable[0]
                    dy_boat = y_cable[1] - y_cable[0]
                    ds_boat = np.sqrt(dx_boat**2 + dy_boat**2)
                    if ds_boat > 1e-6:
                        self.cable_dir_boat_label.setText(f"({dx_boat/ds_boat:.2f}, {dy_boat/ds_boat:.2f})")
                    else:
                        self.cable_dir_boat_label.setText("(0.00, 0.00)")
                    
                    dx_rov = x_cable[-1] - x_cable[-2]
                    dy_rov = y_cable[-1] - y_cable[-2]
                    ds_rov = np.sqrt(dx_rov**2 + dy_rov**2)
                    if ds_rov > 1e-6:
                        self.cable_dir_rov_label.setText(f"({dx_rov/ds_rov:.2f}, {dy_rov/ds_rov:.2f})")
                    else:
                        self.cable_dir_rov_label.setText("(0.00, 0.00)")
            
            # Variables de commande (Fx ROV, Fy ROV)
            fx_rov_cmd = self.fx_rov_input.value()
            fy_rov_cmd = self.fy_rov_input.value()
            self.command_rov_label.setText(f"({fx_rov_cmd:.2f}, {fy_rov_cmd:.2f}) N")
            
            # Forces de traction du câble sur le ROV (déjà orientées câble -> ROV)
            if data.get('Fx_traction_rov') and data.get('Fy_traction_rov'):
                Fx_traction = data['Fx_traction_rov'][-1] if data['Fx_traction_rov'] else 0.0
                Fy_traction = data['Fy_traction_rov'][-1] if data['Fy_traction_rov'] else 0.0
                Fx_traction_display = Fx_traction
                Fy_traction_display = Fy_traction
                self.traction_cable_label.setText(f"({Fx_traction_display:.2f}, {Fy_traction_display:.2f}) N")
            else:
                Fx_traction_display = 0.0
                Fy_traction_display = 0.0
            
            # Force de traînée (horizontale et verticale combinées)
            if data.get('Fx_drag_rov') and data.get('Fy_drag_rov'):
                Fx_drag = data['Fx_drag_rov'][-1] if data['Fx_drag_rov'] else 0.0
                Fy_drag = data['Fy_drag_rov'][-1] if data['Fy_drag_rov'] else 0.0
                self.drag_label.setText(f"({Fx_drag:.2f}, {Fy_drag:.2f}) N")
            else:
                Fx_drag = 0.0
                Fy_drag = 0.0
            
            # Poids apparent (modèle, vers le bas)
            F_apparent_down = 0.0
            if data.get('Fy_rov_app_w'):
                F_apparent_down = data['Fy_rov_app_w'][-1] if data['Fy_rov_app_w'] else 0.0
            self.apparent_weight_label.setText(f"(0.00, {F_apparent_down:.2f}) N")

            # Somme des forces appliquées au ROV (modèle)
            # Convention : toutes les forces verticales sont positives vers le haut (surface)
            # F_apparent_down est maintenant F_apparent_weight (positif si flottabilité positive, vers le haut)
            Fx_total = data['Fx_rov_total'][-1] if data.get('Fx_rov_total') else (Fx_drag + Fx_traction_display + fx_rov_cmd)
            F_apparent_weight = F_apparent_down  # Renommage pour cohérence
            Fy_total = data['Fy_rov_total'][-1] if data.get('Fy_rov_total') else (F_apparent_weight + Fy_drag + Fy_traction_display + fy_rov_cmd)
            self.total_forces_label.setText(f"({Fx_total:.2f}, {Fy_total:.2f}) N")

            # Accélération verticale (gamma_rov_y)
            mass = None
            try:
                mass = float(self.main_window.parameters.get('rov', {}).get('m', 0.0)) if self.main_window.parameters else 0.0
            except Exception:
                mass = 0.0
            if mass and mass > 0.0:
                gamma_rov_y = Fy_total / mass
                self.gamma_rov_label.setText(f"{gamma_rov_y:.2f} m/s²")
            else:
                self.gamma_rov_label.setText("0.00 m/s²")
            
            # Coordonnées Bateau
            if data.get('x_boat'):
                x_boat = data['x_boat'][-1]
                # Le bateau est toujours à la surface (y = 0)
                self.position_boat_label.setText(f"({x_boat:.2f}, 0.00) m")
            
            # Vitesse Bateau
            if data.get('vx_boat'):
                vx_boat = data['vx_boat'][-1] if data.get('vx_boat') else 0.0
                # Le bateau ne se déplace que horizontalement (vy = 0)
                self.velocity_boat_label.setText(f"({vx_boat:.2f}, 0.00) m/s")
            
            # Panneau "Forces Bateau" supprimé
        
        # Mettre à jour le graphique (limiter la fréquence pour éviter le clignotement)
        if data.get('x_rov') and data.get('y_rov') and len(data['x_rov']) > 0:
            try:
                current_time = data['time'][-1] if data.get('time') else 0.0
                # Ne rafraîchir le graphique que si le temps a suffisamment avancé
                # ou si last_plot_update_time is None (initialisation ou reset)
                if (
                    self.last_plot_update_time is None
                    or current_time - self.last_plot_update_time >= 0.5  # toutes les 0,5 s
                ):
                    from src.visualization.plotter import create_system_plot
                    
                    x_rov = data['x_rov'][-1]
                    y_rov = data['y_rov'][-1]
                    x_boat = data['x_boat'][-1] if data.get('x_boat') else 0.0
                    L = data['L'][-1] if data.get('L') else 0.0
                    
                    # Récupérer les données du câble - vérifier si elles existent et ne sont pas vides
                    x_cable_raw = data.get('x_cable_curr')
                    y_cable_raw = data.get('y_cable_curr')
                    T_cable_raw = data.get('T_cable_curr')
                    point_indices_raw = data.get('cable_point_indices')
                    
                    # Convertir en listes si nécessaire et vérifier qu'elles ne sont pas vides
                    if x_cable_raw is not None:
                        x_cable = list(x_cable_raw) if not isinstance(x_cable_raw, list) else x_cable_raw
                        if len(x_cable) == 0:
                            x_cable = []
                    else:
                        x_cable = []
                    
                    if y_cable_raw is not None:
                        y_cable = list(y_cable_raw) if not isinstance(y_cable_raw, list) else y_cable_raw
                        if len(y_cable) == 0:
                            y_cable = []
                    else:
                        y_cable = []
                    
                    if T_cable_raw is not None:
                        T_cable = list(T_cable_raw) if not isinstance(T_cable_raw, list) else T_cable_raw
                        if len(T_cable) == 0:
                            T_cable = None
                    else:
                        T_cable = None
                    
                    if point_indices_raw is not None and isinstance(point_indices_raw, list):
                        cable_point_indices = list(point_indices_raw)
                        if len(cable_point_indices) != len(x_cable):
                            cable_point_indices = list(range(len(x_cable)))
                    else:
                        cable_point_indices = list(range(len(x_cable)))
                    
                    # Ne mettre à jour les graphiques que si on a des données de câble
                    if len(x_cable) > 0 and len(y_cable) > 0:
                        t_final = self.main_window.calc_params.get('t_final', 60.0)
                        title = f"Système ROV - t = {current_time:.2f} s / {t_final:.2f} s"
                        
                        # Debug: vérifier les longueurs de segments réellement utilisées par le plot
                        if len(x_cable) > 1:
                            ds_list = []
                            for i in range(1, len(x_cable)):
                                dx = x_cable[i] - x_cable[i-1]
                                dy = y_cable[i] - y_cable[i-1]
                                ds_list.append(np.sqrt(dx**2 + dy**2))
                            if ds_list:
                                trace_print(1, f"[DEBUG] Plot ds min/max (n={len(ds_list)}): "
                                    f"{min(ds_list):.6f} ; {max(ds_list):.6f} à t={current_time:.2f}s"
                                )

                        # Passer les plages actuelles pour qu'elles ne changent que si nécessaire
                        cable_mode = None
                        if data.get('cable_mode'):
                            cable_mode = data['cable_mode'][-1]
                        fig = create_system_plot(
                            x_rov,
                            y_rov,
                            x_cable,
                            y_cable,
                            x_boat,
                            L,
                            title,
                            x_range=self.current_x_range,
                            y_range=self.current_y_range,
                            T_cable=T_cable,
                            cable_mode=cable_mode,
                            point_indices=cable_point_indices,
                        )
                        self.plotly_widget.update_figure(fig)

                        # Mettre à jour le graphique local "Profil fond" (zoom près du ROV),
                        # avec le même temps courant pour garantir la cohérence.
                        try:
                            from src.visualization.plotter import create_rov_local_plot

                            x_cable_local = list(x_cable)
                            y_cable_local = list(y_cable)
                            t_cable_local = list(T_cable) if T_cable is not None else []
                            point_indices_local = list(cable_point_indices)
                            s_cable_local = []

                            if x_cable_local and y_cable_local:
                                # Le câble dans data est déjà ordonné bateau -> ROV et recollé aux extrémités
                                # (point 0 = bateau, dernier point = ROV). On ne réoriente donc pas ici,
                                # pour conserver exactement les mêmes indices globaux que dans "Profil câble".

                                # Abscisse curviligne locale sur le tronçon affiché
                                if len(x_cable_local) > 1:
                                    dx_loc = np.diff(np.asarray(x_cable_local, dtype=float))
                                    dy_loc = np.diff(np.asarray(y_cable_local, dtype=float))
                                    ds_loc = np.sqrt(dx_loc**2 + dy_loc**2)
                                    s_cable_local = np.concatenate(([0.0], np.cumsum(ds_loc))).tolist()
                                elif x_cable_local:
                                    s_cable_local = [0.0]

                                # Ne garder que la queue proche du ROV (10 derniers points au maximum)
                                n_seg = min(10, len(x_cable_local))
                                x_cable_local = x_cable_local[-n_seg:]
                                y_cable_local = y_cable_local[-n_seg:]
                                if t_cable_local:
                                    t_cable_local = t_cable_local[-n_seg:]
                                if s_cable_local:
                                    s_cable_local = s_cable_local[-n_seg:]
                                if point_indices_local:
                                    point_indices_local = point_indices_local[-n_seg:]

                            title_fond = f"Profil fond - t = {current_time:.2f} s / {t_final:.2f} s"
                            fig_fond = create_rov_local_plot(
                                x_rov,
                                y_rov,
                                x_cable_local,
                                y_cable_local,
                                t_cable_local if t_cable_local else None,
                                s_cable_local if s_cable_local else None,
                                point_indices_local if point_indices_local else None,
                                title_fond,
                            )
                            self.plotly_widget_profile_fond.update_figure(fig_fond)
                        except Exception as e:
                            trace_print(8, f"Erreur lors de la mise à jour du graphique Profil fond: {e}")
                    
                    # Mettre à jour les plages stockées avec les nouvelles valeurs du graphique
                    # (elles ont été recalculées dans create_system_plot si nécessaire)
                    # S'assurer que les plages sont bien arrondies par paliers de 10 mètres
                    import numpy as np
                    if 'fig' in locals() and fig is not None:
                        if fig.layout.xaxis.range is not None:
                            x_min, x_max = fig.layout.xaxis.range
                            # Arrondir par paliers de 10 mètres
                            x_min = np.floor(x_min / 10.0) * 10.0
                            x_max = np.ceil(x_max / 10.0) * 10.0
                            self.current_x_range = [x_min, x_max]
                        if fig.layout.yaxis.range is not None:
                            y_max, y_min = fig.layout.yaxis.range  # Note: reversed
                            # Arrondir par paliers de 10 mètres
                            y_min = np.floor(y_min / 10.0) * 10.0
                            y_max = np.ceil(y_max / 10.0) * 10.0
                            self.current_y_range = [y_max, y_min]  # Stocker dans l'ordre reversed
                    
                    # Mettre à jour le graphique "Courant"
                    self.update_current_profile_plot(y_ref=y_rov)
                    
                    # Mettre à jour le graphique "Tension"
                    T_cable = data.get('T_cable_curr', [])
                    from src.visualization.plotter import create_tension_curvilinear_plot
                    
                    # Toujours mettre à jour le graphique, même s'il n'y a pas de données
                    # Cela garantit que le graphique est initialisé
                    if len(x_cable) > 0 and len(y_cable) > 0 and len(T_cable) > 0:
                        try:
                            # Calculer l'abscisse curviligne s le long du câble
                            # s = 0 au bateau, s = L au ROV
                            # Convention : le câble va du bateau (index 0) au ROV (index -1)
                            x_cable_arr = np.asarray(x_cable)
                            y_cable_arr = np.asarray(y_cable)
                            T_cable_arr = np.asarray(T_cable)
                            
                            # Si les tensions sont définies par segment (N) au lieu de nœud (N+1),
                            # interpoler sur les nœuds pour garantir l'affichage.
                            if len(x_cable_arr) == len(y_cable_arr) and len(T_cable_arr) == len(x_cable_arr) - 1:
                                s_curvilinear = np.zeros(len(x_cable_arr))
                                for i in range(1, len(x_cable_arr)):
                                    dx = x_cable_arr[i] - x_cable_arr[i-1]
                                    dy = y_cable_arr[i] - y_cable_arr[i-1]
                                    ds = np.sqrt(dx**2 + dy**2)
                                    s_curvilinear[i] = s_curvilinear[i-1] + ds
                                s_seg = 0.5 * (s_curvilinear[:-1] + s_curvilinear[1:])
                                T_cable_arr = np.interp(
                                    s_curvilinear, s_seg, T_cable_arr,
                                    left=T_cable_arr[0], right=T_cable_arr[-1]
                                )
                            
                            # Vérifier que les dimensions correspondent
                            if len(x_cable_arr) == len(y_cable_arr) == len(T_cable_arr):
                                # IMPORTANT: Après correction de _compute_catenary_tensions :
                                # Les tensions T retournées par le solveur suivent l'ordre des positions :
                                # T[0] = tension au bateau, T[-1] = tension au ROV
                                # (cohérent avec x_cable[0] = bateau, x_cable[-1] = ROV)
                                
                                # Vérifier l'ordre du câble et le réorganiser si nécessaire
                                # Le câble doit aller du bateau (y=0) au ROV (y=y_rov)
                                dist_first_to_boat = np.sqrt((x_cable_arr[0] - x_boat)**2 + (y_cable_arr[0] - 0.0)**2)
                                dist_first_to_rov = np.sqrt((x_cable_arr[0] - x_rov)**2 + (y_cable_arr[0] - y_rov)**2)
                                
                                # Si le premier point est plus proche du ROV, inverser l'ordre
                                if dist_first_to_rov < dist_first_to_boat:
                                    # Le câble est dans l'ordre inverse : ROV en premier, bateau en dernier
                                    # Il faut inverser les positions ET les tensions pour maintenir la cohérence
                                    x_cable_arr = np.flip(x_cable_arr)
                                    y_cable_arr = np.flip(y_cable_arr)
                                    T_cable_arr = np.flip(T_cable_arr)
                                    # Après inversion : T_cable_arr[0] = ancienne T[-1] = tension au bateau ✓
                                    #                  T_cable_arr[-1] = ancienne T[0] = tension au ROV ✓
                                # Sinon, le câble est déjà dans le bon ordre et les tensions aussi, pas besoin d'inverser
                                
                                # CORRECTION : Forcer le dernier point du câble à correspondre exactement à la position du ROV
                                # Cela évite les sauts dans l'abscisse curviligne et les slack négatifs
                                if len(x_cable_arr) > 0:
                                    x_cable_arr[-1] = x_rov
                                    y_cable_arr[-1] = y_rov
                                
                                # CORRECTION : Forcer le premier point du câble à correspondre exactement à la position du bateau
                                if len(x_cable_arr) > 0:
                                    x_cable_arr[0] = x_boat
                                    y_cable_arr[0] = 0.0
                                
                                # Calculer l'abscisse curviligne s en cumulant les distances
                                s_curvilinear = np.zeros(len(x_cable_arr))
                                for i in range(1, len(x_cable_arr)):
                                    dx = x_cable_arr[i] - x_cable_arr[i-1]
                                    dy = y_cable_arr[i] - y_cable_arr[i-1]
                                    ds = np.sqrt(dx**2 + dy**2)
                                    s_curvilinear[i] = s_curvilinear[i-1] + ds
                                
                                # Calculer les composantes Tx et Ty de la tension
                                # Pour chaque point du câble, calculer l'angle local
                                Tx_cable = np.zeros(len(x_cable_arr))
                                Ty_cable = np.zeros(len(y_cable_arr))
                                
                                for i in range(len(x_cable_arr)):
                                    if i == 0:
                                        # Premier point (bateau) : utiliser le segment suivant
                                        if len(x_cable_arr) > 1:
                                            dx = x_cable_arr[1] - x_cable_arr[0]
                                            dy = y_cable_arr[1] - y_cable_arr[0]
                                        else:
                                            dx, dy = 0, 0
                                    elif i == len(x_cable_arr) - 1:
                                        # Dernier point (ROV) : utiliser le segment précédent
                                        dx = x_cable_arr[i] - x_cable_arr[i-1]
                                        dy = y_cable_arr[i] - y_cable_arr[i-1]
                                    else:
                                        # Point intermédiaire : moyenne des deux segments adjacents
                                        dx = (x_cable_arr[i+1] - x_cable_arr[i-1]) / 2.0
                                        dy = (y_cable_arr[i+1] - y_cable_arr[i-1]) / 2.0
                                    
                                    ds = np.sqrt(dx**2 + dy**2)
                                    if ds > 1e-6:
                                        cos_theta = dx / ds
                                        sin_theta = dy / ds
                                        Tx_cable[i] = T_cable_arr[i] * cos_theta
                                        Ty_cable[i] = T_cable_arr[i] * sin_theta
                                    else:
                                        Tx_cable[i] = 0
                                        Ty_cable[i] = 0
                                
                                # Créer le graphique Tension
                                # Forcer la recréation complète si c'est la première fois avec des données
                                if not hasattr(self, '_tension_graph_initialized'):
                                    self.plotly_widget_tension.is_first_update = True
                                    self._tension_graph_initialized = True
                                
                                fig_tension = create_tension_curvilinear_plot(
                                    s_curvilinear.tolist(), T_cable_arr.tolist(), "Tension",
                                    s_range=self.current_s_range, T_range=self.current_T_range,
                                    Tx=Tx_cable.tolist(), Ty=Ty_cable.tolist(),
                                    x_cable=x_cable_arr.tolist(), y_cable=y_cable_arr.tolist()
                                )
                                self.plotly_widget_tension.update_figure(fig_tension)
                                
                                # Mettre à jour les plages stockées pour le graphique Tension
                                # Maintenant : s en abscisse (X), T en ordonnée (Y)
                                if fig_tension.layout.xaxis.range is not None:
                                    s_min, s_max = fig_tension.layout.xaxis.range
                                    # Arrondir par paliers de 10 mètres
                                    s_min = np.floor(s_min / 10.0) * 10.0
                                    s_max = np.ceil(s_max / 10.0) * 10.0
                                    self.current_s_range = [s_min, s_max]
                                if fig_tension.layout.yaxis.range is not None:
                                    T_min, T_max = fig_tension.layout.yaxis.range
                                    # Arrondir par paliers de 10 N
                                    T_min = np.floor(T_min / 10.0) * 10.0
                                    T_max = np.ceil(T_max / 10.0) * 10.0
                                    self.current_T_range = [T_min, T_max]
                            else:
                                # Dimensions incompatibles, créer un graphique vide
                                trace_print(8, f"Warning: Dimensions incompatibles - x_cable: {len(x_cable_arr)}, y_cable: {len(y_cable_arr)}, T_cable: {len(T_cable_arr)}")
                                fig_tension = create_tension_curvilinear_plot([], [], "Tension")
                                self.plotly_widget_tension.update_figure(fig_tension)
                        except Exception as e:
                            trace_print(8, f"Erreur lors de la mise à jour du graphique Tension: {e}")
                            import traceback
                            traceback.print_exc()
                            # Créer un graphique vide en cas d'erreur
                            fig_tension = create_tension_curvilinear_plot([], [], "Tension")
                            self.plotly_widget_tension.update_figure(fig_tension)
                    else:
                        # Pas de données disponibles, créer un graphique vide
                        # Forcer la recréation complète si c'est la première fois
                        if not hasattr(self, '_tension_graph_initialized'):
                            self.plotly_widget_tension.is_first_update = True
                            self._tension_graph_initialized = True
                        fig_tension = create_tension_curvilinear_plot([], [], "Tension")
                        self.plotly_widget_tension.update_figure(fig_tension)
                    
                    self.last_plot_update_time = current_time
            except Exception as e:
                trace_print(8, f"Erreur lors de la mise à jour du graphique: {e}")
                import traceback
                traceback.print_exc()
