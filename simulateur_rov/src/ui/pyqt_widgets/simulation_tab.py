"""
Onglet de simulation avec contrôles et visualisations
"""
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                             QLabel, QGroupBox, QScrollArea, QGridLayout,
                             QMessageBox, QProgressBar, QDoubleSpinBox, QSpinBox, QToolButton, QApplication,
                             QTabWidget)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QPoint
from PyQt6.QtGui import QFont, QCursor
import sys
import os

# Ajouter le répertoire parent au path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

from .plotly_widget import PlotlyWidget
from .simulation_thread import SimulationThread
from src.utils.logger import trace_print, set_trace_level


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
        
        # Colonne droite : Métriques
        right_panel = self.create_metrics_panel()
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
        self.fx_rov_input.setRange(-100.0, 100.0)
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
        self.fy_rov_input.setRange(-100.0, 100.0)
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
        self.dl_dt_input.setRange(-1.0, 1.0)
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
        
        # Export CSV
        export_row = QHBoxLayout()
        self.btn_export = QPushButton("📊 Export CSV")
        self.btn_export.setStyleSheet("background-color: #6c757d; color: white; padding: 8px;")
        self.btn_export.setEnabled(False)
        self.btn_export.clicked.connect(self.export_csv)
        export_row.addWidget(self.btn_export)
        export_help = HelpButton("Exporte les données de simulation au format CSV.\n"
                                 "Les données incluent toutes les métriques enregistrées pendant la simulation.", self)
        export_row.addWidget(export_help, 0)
        controls_layout.addLayout(export_row)

        # Niveau de trace
        trace_row = QHBoxLayout()
        trace_label = QLabel("Niveau trace")
        self.trace_level_input = QSpinBox()
        self.trace_level_input.setRange(0, 10)
        self.trace_level_input.setSingleStep(1)
        self.trace_level_input.setValue(10)
        set_trace_level(10)
        self.trace_level_input.valueChanged.connect(lambda v: set_trace_level(int(v)))
        trace_row.addWidget(trace_label)
        trace_row.addWidget(self.trace_level_input)
        trace_help = HelpButton("Définit le niveau global de trace.\n"
                                "Un message s'affiche si son niveau >= niveau trace.", self)
        trace_row.addWidget(trace_help, 0)
        controls_layout.addLayout(trace_row)
        
        layout.addWidget(controls_group)
        
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
    
    def initialize_system(self):
        """Initialise le système ROV et met à jour les métriques temps réel"""
        try:
            import numpy as np  # Import numpy au début de la fonction
            
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
            
            from src.models.system_model import ROVSystem
            from src.utils.initial_conditions import get_initial_state
            
            # Créer le système ROV
            N_segments = int(self.main_window.calc_params.get('N_segments', 50))
            system = ROVSystem(self.main_window.parameters, N_segments=N_segments)
            
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
            F_apparent_weight_down = F_weight - F_buoyancy
            F_buoyancy_net = -F_apparent_weight_down
            
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
            # Force exercée PAR le câble SUR le ROV = -T_rov * Urov (opposée à Urov)
            Fx_traction = -T_rov * Urov_x
            Fy_traction = -T_rov * Urov_y
            
            # Variables de commande (initialisées à 0)
            Fx_cmd = 0.0
            Fy_cmd = 0.0
            
            # Somme des forces ROV
            Fx_total = Fx_drag_rov + Fx_traction + Fx_cmd
            Fy_total = Fy_drag_rov + Fy_traction + F_apparent_weight_down + Fy_cmd
            
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
                'T0': [T_rov],
                'T_rov': [T_rov],
                'T_boat': [T_boat],
                'T_max': [T_max],
                'Fx_drag': [Fx_drag_rov],
                'Fy_drag': [Fy_drag_rov],
                'Fx_traction': [Fx_traction],
                'Fy_traction': [Fy_traction],
                'F_apparent_weight': [F_apparent_weight_down],
                'F_buoyancy_net': [F_buoyancy_net],
                'Fx_total': [Fx_total],
                'Fy_total': [Fy_total],
                'Fx_traction_boat': [Fx_traction_boat],
                'Fy_traction_boat': [Fy_traction_boat],
                'F_prop_boat': [F_prop_boat],
                'Fx_total_boat': [Fx_total_boat],
                'Fy_total_boat': [Fy_total_boat],
                'x_cable_curr': x_cable.tolist() if hasattr(x_cable, 'tolist') else list(x_cable),
                'y_cable_curr': y_cable.tolist() if hasattr(y_cable, 'tolist') else list(y_cable),
                'T_cable_curr': T.tolist() if hasattr(T, 'tolist') else list(T),
            }
            state['current_time'] = 0.0
            state['system'] = system
            state['y_current'] = y0
            # Mettre la simulation en pause après l'initialisation pour permettre l'analyse
            state['running'] = False
            state['paused'] = False
            self.main_window.update_simulation_state(state)
            
            # Mettre à jour les boutons pour refléter l'état initialisé mais en pause
            if hasattr(self, 'btn_start'):
                self.btn_start.setEnabled(True)
                self.btn_pause.setEnabled(False)
                self.btn_restart.setEnabled(True)
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

        # Widget Plotly pour le graphique "Profil"
        self.plotly_widget = PlotlyWidget()
        layout.addWidget(self.plotly_widget)
        layout.setAlignment(self.plotly_widget, Qt.AlignmentFlag.AlignTop)
        
        # Graphique initial "Profil" - sera remplacé lors de l'initialisation complète
        # Créer un graphique initial avec le bateau et le ROV, mais sans câble pour éviter la trace "pas de données"
        # Le graphique sera remplacé lors de l'appel à initialize_system() -> update_display()
        from src.visualization.plotter import create_system_plot
        # Récupérer la profondeur initiale du ROV depuis les paramètres
        y_rov_init = self.main_window.init_params.get('y_rov_init', -10.0)  # Profondeur négative
        x_rov_init = self.main_window.init_params.get('x_rov_init', 0.0)
        # Le bateau est toujours à la surface (y=0)
        # Passer None pour T_cable pour éviter les problèmes
        fig = create_system_plot(x_rov_init, y_rov_init, [], [], x_rov_init, 0, "Profil", T_cable=None)
        self.plotly_widget.update_figure(fig)
        
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
        
        self.bottom_tabs.addTab(self.tab1, "Tension")
        
        # Onglet 2 : Courant
        self.tab2 = QWidget()
        tab2_layout = QVBoxLayout(self.tab2)
        tab2_layout.setContentsMargins(0, 0, 0, 0)
        
        # Widget Plotly pour le graphique "Courant"
        self.plotly_widget_current = PlotlyWidget()
        tab2_layout.addWidget(self.plotly_widget_current)
        
        # Graphique initial "Courant"
        self.update_current_profile_plot()
        
        self.bottom_tabs.addTab(self.tab2, "Courant")
        
        # Onglet 3 : Commande dL/dt
        self.tab3 = QWidget()
        tab3_layout = QVBoxLayout(self.tab3)
        tab3_layout.setContentsMargins(0, 0, 0, 0)
        self.plotly_widget_dl_dt = PlotlyWidget()
        tab3_layout.addWidget(self.plotly_widget_dl_dt)
        from src.visualization.plotter import create_dl_dt_plot
        fig_dl_dt = create_dl_dt_plot([], [], "Commande dL/dt", None)
        self.plotly_widget_dl_dt.update_figure(fig_dl_dt)
        self.bottom_tabs.addTab(self.tab3, "Commande dL/dt")

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
        
        # 6. Mode câble
        metrics_layout.addWidget(QLabel("<b>Mode câble:</b>"), 7, 0)
        self.cable_mode_label = QLabel("-")
        metrics_layout.addWidget(self.cable_mode_label, 7, 1)
        
        # 7. Tension bateau
        metrics_layout.addWidget(QLabel("<b>Tension bateau:</b>"), 8, 0)
        self.tension_boat_label = QLabel("0.00 N")
        metrics_layout.addWidget(self.tension_boat_label, 8, 1)
        
        # 8. Tension ROV
        metrics_layout.addWidget(QLabel("<b>Tension ROV:</b>"), 9, 0)
        self.tension_rov_label = QLabel("0.00 N")
        metrics_layout.addWidget(self.tension_rov_label, 9, 1)
        
        # 9. Tension max
        metrics_layout.addWidget(QLabel("<b>Tension max:</b>"), 10, 0)
        self.tension_label = QLabel("0.00 N")
        metrics_layout.addWidget(self.tension_label, 10, 1)
        
        # 10. Direction câble (Bateau)
        metrics_layout.addWidget(QLabel("<b>Direction câble (Bateau):</b>"), 11, 0)
        self.cable_dir_boat_label = QLabel("(0.00, 0.00)")
        metrics_layout.addWidget(self.cable_dir_boat_label, 11, 1)
        
        # 11. Direction câble (ROV)
        metrics_layout.addWidget(QLabel("<b>Direction câble (ROV):</b>"), 12, 0)
        self.cable_dir_rov_label = QLabel("(0.00, 0.00)")
        metrics_layout.addWidget(self.cable_dir_rov_label, 12, 1)

        # Trait de séparation
        separator_cable_forces = QLabel("─" * 30)
        separator_cable_forces.setStyleSheet("color: #ccc;")
        metrics_layout.addWidget(separator_cable_forces, 13, 0, 1, 2)

        # Sous-titre "Forces s'exerçant sur le câble"
        cable_forces_subtitle = QLabel("<b>Forces s'exerçant sur le câble</b>")
        cable_forces_subtitle.setStyleSheet("color: #666; font-size: 10pt; margin-top: 5px;")
        metrics_layout.addWidget(cable_forces_subtitle, 14, 0, 1, 2)
        
        # 6. Traction bateau sur câble
        metrics_layout.addWidget(QLabel("<b>Traction bateau sur câble:</b>"), 15, 0)
        self.traction_boat_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.traction_boat_label, 15, 1)
        
        # 7. Traction ROV sur câble
        metrics_layout.addWidget(QLabel("<b>Traction ROV sur câble:</b>"), 16, 0)
        self.traction_rov_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.traction_rov_label, 16, 1)
        
        # 8. Poids apparent
        metrics_layout.addWidget(QLabel("<b>Poids apparent:</b>"), 17, 0)
        self.cable_apparent_weight_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.cable_apparent_weight_label, 17, 1)
        
        # 9. Force traînée câble
        metrics_layout.addWidget(QLabel("<b>Force traînée câble:</b>"), 18, 0)
        self.cable_drag_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.cable_drag_label, 18, 1)
        
        # 10. Somme forces câble
        metrics_layout.addWidget(QLabel("<b>Somme forces câble:</b>"), 19, 0)
        self.cable_total_forces_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.cable_total_forces_label, 19, 1)
        
        # Trait de séparation
        separator2 = QLabel("─" * 30)
        separator2.setStyleSheet("color: #ccc;")
        metrics_layout.addWidget(separator2, 20, 0, 1, 2)
        
        # Sous-titre "ROV"
        rov_subtitle = QLabel("<b>ROV</b>")
        rov_subtitle.setStyleSheet("color: #666; font-size: 10pt; margin-top: 5px;")
        metrics_layout.addWidget(rov_subtitle, 21, 0, 1, 2)
        
        # 6. Position ROV
        metrics_layout.addWidget(QLabel("<b>Position ROV:</b>"), 22, 0)
        self.position_label = QLabel("(0.00, 0.00) m")
        metrics_layout.addWidget(self.position_label, 22, 1)
        
        # 7. Vitesse ROV
        metrics_layout.addWidget(QLabel("<b>Vitesse ROV:</b>"), 23, 0)
        self.velocity_label = QLabel("(0.00, 0.00) m/s")
        metrics_layout.addWidget(self.velocity_label, 23, 1)
        
        # Trait de séparation
        separator3 = QLabel("─" * 30)
        separator3.setStyleSheet("color: #ccc;")
        metrics_layout.addWidget(separator3, 24, 0, 1, 2)
        
        # Sous-titre "Forces s'exerçant sur le ROV"
        forces_subtitle = QLabel("<b>Forces s'exerçant sur le ROV</b>")
        forces_subtitle.setStyleSheet("color: #666; font-size: 10pt; margin-top: 5px;")
        metrics_layout.addWidget(forces_subtitle, 25, 0, 1, 2)
        
        # Variables de commande (Fx ROV, Fy ROV)
        metrics_layout.addWidget(QLabel("<b>Commande ROV:</b>"), 26, 0)
        self.command_rov_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.command_rov_label, 26, 1)
        
        # Forces de traction du câble sur le ROV
        metrics_layout.addWidget(QLabel("<b>Traction câble sur ROV:</b>"), 27, 0)
        self.traction_cable_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.traction_cable_label, 27, 1)
        
        # 8. Force traînée ROV
        metrics_layout.addWidget(QLabel("<b>Force traînée ROV:</b>"), 28, 0)
        self.drag_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.drag_label, 28, 1)
        
        # 9. Poids apparent
        metrics_layout.addWidget(QLabel("<b>Poids apparent:</b>"), 29, 0)
        self.apparent_weight_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.apparent_weight_label, 29, 1)
        
        # 10. Somme des forces ROV
        metrics_layout.addWidget(QLabel("<b>Somme forces ROV (modèle):</b>"), 30, 0)
        self.total_forces_label = QLabel("(0.00, 0.00) N")
        metrics_layout.addWidget(self.total_forces_label, 30, 1)
        
        # Trait de séparation
        separator4 = QLabel("─" * 30)
        separator4.setStyleSheet("color: #ccc;")
        metrics_layout.addWidget(separator4, 31, 0, 1, 2)
        
        # Sous-titre "Coordonnées Bateau"
        boat_subtitle = QLabel("<b>Coordonnées Bateau</b>")
        boat_subtitle.setStyleSheet("color: #666; font-size: 10pt; margin-top: 5px;")
        metrics_layout.addWidget(boat_subtitle, 32, 0, 1, 2)
        
        # Bateau
        metrics_layout.addWidget(QLabel("<b>Bateau:</b>"), 33, 0)
        self.position_boat_label = QLabel("(0.00, 0.00) m")
        metrics_layout.addWidget(self.position_boat_label, 33, 1)
        
        # Vitesse Bateau
        metrics_layout.addWidget(QLabel("<b>Vitesse Bateau:</b>"), 34, 0)
        self.velocity_boat_label = QLabel("(0.00, 0.00) m/s")
        metrics_layout.addWidget(self.velocity_boat_label, 34, 1)
        
        layout.addWidget(metrics_group)
        
        layout.addStretch()
        
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
            self.main_window.update_simulation_state(state)
            
            # Mettre à jour les boutons
            self.btn_start.setEnabled(False)
            self.btn_pause.setEnabled(True)
            self.btn_restart.setEnabled(False)
            self.status_label.setText("▶ Simulation en cours...")
            self.status_label.setStyleSheet("padding: 8px; background-color: #d4edda; border: 1px solid #c3e6cb;")
            
            self.simulation_started.emit()
            
        except Exception as e:
            QMessageBox.critical(self, "Erreur", f"Erreur lors du démarrage de la simulation:\n{str(e)}")
            import traceback
            traceback.print_exc()
    
    def stop_simulation(self):
        """Arrête la simulation"""
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
        self.status_label.setText("Simulation arrêtée")
        self.status_label.setStyleSheet("padding: 8px; background-color: #f8f9fa; border: 1px solid #dee2e6;")
        
        self.simulation_stopped.emit()
    
    def pause_resume_simulation(self):
        """Met en pause ou reprend la simulation"""
        if not self.simulation_thread or not self.simulation_thread.isRunning():
            return
        
        state = self.main_window.get_simulation_state()
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
            'T0': [],
            'T_rov': [],
            'T_boat': [],
            'T_max': [],
            'Fx_drag': [],
            'Fy_drag': [],
            'Fx_traction': [],
            'Fy_traction': [],
            'F_apparent_weight': [],
            'Fx_total': [],
            'Fy_total': [],
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
    
    def export_csv(self):
        """Exporte les données en CSV"""
        # TODO: Implémenter l'export CSV
        QMessageBox.information(self, "Export", "Fonctionnalité d'export CSV à implémenter.")
    
    def on_simulation_updated(self, data):
        """Appelé quand la simulation met à jour les données"""
        state = self.main_window.get_simulation_state()
        state['data'].update(data)
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
        self.status_label.setText("✓ Simulation terminée")
        self.status_label.setStyleSheet("padding: 8px; background-color: #d1ecf1; border: 1px solid #bee5eb;")
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
        
        
        # Mettre à jour les métriques
        if data.get('time'):
            current_time = data['time'][-1] if data['time'] else 0.0
            self.time_label.setText(f"{current_time:.2f} s")
            
            # Calculer le pourcentage de progression
            t_final = self.main_window.calc_params.get('t_final', 60.0)
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
            
            if data.get('L'):
                L = data['L'][-1]
                self.length_label.setText(f"{L:.2f} m")
            else:
                L = 0.0
                self.length_label.setText("0.00 m")

            # Longueur droite bateau -> ROV
            if data.get('x_boat') and data.get('x_rov') and data.get('y_rov'):
                try:
                    x_boat = float(data['x_boat'][-1])
                    x_rov = float(data['x_rov'][-1])
                    y_rov = float(data['y_rov'][-1])
                    straight_len = (x_rov - x_boat) ** 2 + (y_rov - 0.0) ** 2
                    self.length_straight_label.setText(f"{straight_len ** 0.5:.2f} m")
                except Exception:
                    self.length_straight_label.setText("0.00 m")
            else:
                self.length_straight_label.setText("0.00 m")

            if data.get('cable_mode'):
                last_mode = data['cable_mode'][-1]
                mode_label = "Caténaire" if last_mode == "catenary" else "Ligne droite"
                self.cable_mode_label.setText(mode_label)
            else:
                self.cable_mode_label.setText("-")
            
            x_cable_raw = data.get('x_cable_curr')
            y_cable_raw = data.get('y_cable_curr')
            if x_cable_raw is not None and y_cable_raw is not None:
                import numpy as np
                x_cable = np.asarray(x_cable_raw)
                y_cable = np.asarray(y_cable_raw)
                if len(x_cable) > 1 and len(y_cable) > 1:
                    ds = np.hypot(np.diff(x_cable), np.diff(y_cable))
                    length_segments = float(np.sum(ds))
                    self.length_segments_label.setText(f"{length_segments:.2f} m")
                    self.length_drift_label.setText(f"{(length_segments - L):.2f} m")
                else:
                    self.length_segments_label.setText("0.00 m")
                    self.length_drift_label.setText(f"{(0.0 - L):.2f} m")
            else:
                self.length_segments_label.setText("0.00 m")
                self.length_drift_label.setText(f"{(0.0 - L):.2f} m")
            
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
                    from src.visualization.plotter import create_tension_vs_target_plot

                    times = data.get('time', [])
                    t_rupture = None
                    try:
                        t_rupture = float(self.main_window.parameters.get('cable', {}).get('tension_rupture', 50.0))
                    except Exception:
                        t_rupture = 50.0
                    t_cible = t_rupture / 2.0 if t_rupture is not None else None

                    t_boat = data.get('T_boat', []) or []
                    t_rov = data.get('T_rov', []) or []
                    t_max = data.get('T_max', []) or []

                    # Aligner les longueurs pour éviter les erreurs de rafraîchissement
                    min_len = min(len(times), len(t_boat), len(t_rov), len(t_max))
                    if min_len == 0:
                        return
                    times = times[:min_len]
                    t_boat = t_boat[:min_len]
                    t_rov = t_rov[:min_len]
                    t_max = t_max[:min_len]

                    # Réduire le nombre de points pour éviter les rafraîchissements lents
                    max_points = 1500
                    step = max(1, len(times) // max_points)
                    times_ds = times[::step]
                    t_rupture_series = [t_rupture] * len(times_ds) if t_rupture is not None else []
                    t_cible_series = [t_cible] * len(times_ds) if t_cible is not None else []

                    fig_tension_vs_target = create_tension_vs_target_plot(
                        times_ds,
                        t_rupture_series,
                        t_cible_series,
                        t_boat[::step],
                        t_rov[::step],
                        t_max[::step],
                        "Tension vs cible",
                    )
                    self.plotly_widget_tension_vs_target.update_figure(fig_tension_vs_target)
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
                            # Force exercée par le bateau sur le câble = opposée à U_bateau
                            traction_boat = (-abs(T_boat) * uboat_x, -abs(T_boat) * uboat_y)
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
                            # Force exercée par le ROV sur le câble = même sens que U_rov
                            traction_rov = (abs(T_rov) * urov_x, abs(T_rov) * urov_y)
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
                    weight_per_unit = (system.cable.rho_cable - system.environment.rho_eau) * system.cable.A_cable * system.environment.g
                    F_apparent_cable = weight_per_unit * L_cable
                    # Convention: poids apparent vers le bas (composante Y négative)
                    cable_weight_vec = (0.0, -F_apparent_cable)
                    self.cable_apparent_weight_label.setText(f"({cable_weight_vec[0]:.2f}, {cable_weight_vec[1]:.2f}) N")
                    
                    x_cable_raw = data.get('x_cable_curr')
                    y_cable_raw = data.get('y_cable_curr')
                    if x_cable_raw is not None and y_cable_raw is not None:
                        import numpy as np
                        from src.solvers.forces import compute_cable_forces
                        
                        x_cable = list(x_cable_raw) if not isinstance(x_cable_raw, list) else x_cable_raw
                        y_cable = list(y_cable_raw) if not isinstance(y_cable_raw, list) else y_cable_raw
                        if len(x_cable) > 1 and len(y_cable) > 1:
                            vx_cable = np.zeros(len(x_cable))
                            vy_cable = np.zeros(len(x_cable))
                            params_cable = {
                                'd': system.cable.d,
                                'rho_cable': system.cable.rho_cable,
                                'Cx_cable': system.cable.Cx_cable
                            }
                            Fx_segments, Fy_segments = compute_cable_forces(
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
                            self.cable_drag_label.setText(f"({cable_drag_vec[0]:.2f}, {cable_drag_vec[1]:.2f}) N")
                            
                            total_fx = traction_rov[0] + traction_boat[0] + cable_weight_vec[0] + cable_drag_vec[0]
                            total_fy = traction_rov[1] + traction_boat[1] + cable_weight_vec[1] + cable_drag_vec[1]
                            self.cable_total_forces_label.setText(f"({total_fx:.2f}, {total_fy:.2f}) N")
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
            if data.get('Fx_traction') and data.get('Fy_traction'):
                Fx_traction = data['Fx_traction'][-1] if data['Fx_traction'] else 0.0
                Fy_traction = data['Fy_traction'][-1] if data['Fy_traction'] else 0.0
                Fx_traction_display = Fx_traction
                Fy_traction_display = Fy_traction
                self.traction_cable_label.setText(f"({Fx_traction_display:.2f}, {Fy_traction_display:.2f}) N")
            else:
                Fx_traction_display = 0.0
                Fy_traction_display = 0.0
            
            # Force de traînée (horizontale et verticale combinées)
            if data.get('Fx_drag') and data.get('Fy_drag'):
                Fx_drag = data['Fx_drag'][-1] if data['Fx_drag'] else 0.0
                Fy_drag = data['Fy_drag'][-1] if data['Fy_drag'] else 0.0
                self.drag_label.setText(f"({Fx_drag:.2f}, {Fy_drag:.2f}) N")
            else:
                Fx_drag = 0.0
                Fy_drag = 0.0
            
            # Poids apparent (format vectoriel pour cohérence)
            F_apparent_down = 0.0
            if data.get('F_apparent_weight'):
                F_apparent_down = data['F_apparent_weight'][-1] if data['F_apparent_weight'] else 0.0
            if data.get('F_buoyancy_net'):
                F_buoy_display = data['F_buoyancy_net'][-1] if data['F_buoyancy_net'] else 0.0
                self.apparent_weight_label.setText(f"(0.00, {F_buoy_display:.2f}) N")
            else:
                F_buoy_display = -F_apparent_down
                self.apparent_weight_label.setText(f"(0.00, {F_buoy_display:.2f}) N")
            
            # Somme des forces appliquées au ROV
            Fx_total = Fx_drag + Fx_traction_display + fx_rov_cmd
            Fy_total = Fy_drag + Fy_traction_display + F_apparent_down + fy_rov_cmd
            self.total_forces_label.setText(f"({Fx_total:.2f}, {Fy_total:.2f}) N")
            
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
                    
                    # Ne mettre à jour le graphique que si on a des données de câble ou si c'est la première fois
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
                                trace_print(
                                    1,
                                    f"[DEBUG] Plot ds min/max (n={len(ds_list)}): "
                                    f"{min(ds_list):.6f} ; {max(ds_list):.6f} à t={current_time:.2f}s"
                                )

                        # Passer les plages actuelles pour qu'elles ne changent que si nécessaire
                        fig = create_system_plot(x_rov, y_rov, x_cable, y_cable, x_boat, L, title,
                                               x_range=self.current_x_range, y_range=self.current_y_range,
                                               T_cable=T_cable)
                        self.plotly_widget.update_figure(fig)
                    
                    # Mettre à jour les plages stockées avec les nouvelles valeurs du graphique
                    # (elles ont été recalculées dans create_system_plot si nécessaire)
                    # S'assurer que les plages sont bien arrondies par paliers de 10 mètres
                    import numpy as np
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
