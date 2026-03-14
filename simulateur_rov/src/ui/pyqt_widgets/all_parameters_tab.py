"""
Onglet unifié pour tous les paramètres (environnement, calcul, conditions initiales)
Organisé en 3 colonnes
"""
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                             QLineEdit, QPushButton, QGroupBox, QGridLayout,
                             QFileDialog, QMessageBox, QScrollArea, QComboBox,
                             QInputDialog, QTextEdit)
from PyQt6.QtCore import Qt, QTimer, QPoint
from PyQt6.QtGui import QCursor
from pathlib import Path
import sys
import os
import json
from datetime import datetime
import math
from src.utils.logger import trace_print
from src.utils.scenario_utils import verifier_syntaxe_scenario, find_first_invalid_couple, list_auto_L_functions, get_auto_L_function
import inspect

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from src.ui.mission_utils import get_missions_directory, list_missions, create_mission, load_mission_description
from src.ui.pyqt_widgets.simulation_tab import HelpButton


class MissionToolTip(QLabel):
    """Tooltip personnalisé persistant pour les descriptions de missions"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.WindowType.ToolTip | Qt.WindowType.FramelessWindowHint)
        self.setStyleSheet("""
            QLabel {
                background-color: #f4e4bc;
                color: #555;
                border: 1px solid #d4c4a4;
                padding: 8px;
                border-radius: 3px;
                font-size: 10pt;
            }
        """)
        self.setWordWrap(True)
        self.setMaximumWidth(400)
        self.hide()


class AutoLToolTip(QLabel):
    """Tooltip personnalisé persistant pour les docstrings des fonctions auto_L"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.WindowType.ToolTip | Qt.WindowType.FramelessWindowHint)
        self.setStyleSheet("""
            QLabel {
                background-color: #e8f4f8;
                color: #333;
                border: 1px solid #b8d4e0;
                padding: 8px;
                border-radius: 3px;
                font-size: 10pt;
            }
        """)
        self.setWordWrap(True)
        self.setMaximumWidth(500)
        self.hide()


class AllParametersTab(QWidget):
    """Onglet unifié pour tous les paramètres"""
    
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.mission_descriptions = {}  # Dictionnaire pour stocker les descriptions des missions
        self.mission_tooltip = MissionToolTip(self)  # Tooltip persistant
        self.auto_L_docstrings = {}  # Dictionnaire pour stocker les docstrings des fonctions auto_L
        self.auto_L_tooltip = AutoLToolTip(self)  # Tooltip persistant pour auto_L
        self._suppress_auto_L_tooltip = False
        self.init_ui()
        self.load_all_parameters_display()

    def eventFilter(self, obj, event):
        """Intercepte certains événements pour gérer les tooltips persistants."""
        try:
            from PyQt6.QtCore import QEvent
        except Exception:
            return super().eventFilter(obj, event)

        # Cacher le tooltip auto_L quand la souris quitte la liste déroulante
        if hasattr(self, "auto_L_combo") and self.auto_L_combo is not None:
            view = self.auto_L_combo.view()
            if view is not None and obj is view.viewport():
                if event.type() in (QEvent.Type.Leave, QEvent.Type.FocusOut, QEvent.Type.Hide):
                    self.auto_L_tooltip.hide()

        return super().eventFilter(obj, event)
    
    def init_ui(self):
        """Initialise l'interface avec 3 colonnes"""
        main_layout = QVBoxLayout(self)
        
        # Groupe : Sélection de mission (en haut, sur toute la largeur)
        mission_group = QGroupBox("🎯 Mission")
        mission_layout = QVBoxLayout()

        mission_columns_layout = QHBoxLayout()

        mission_left_layout = QVBoxLayout()
        mission_select_layout = QHBoxLayout()
        mission_select_layout.addWidget(QLabel("Mission:"))

        # Créer d'abord le label de statut
        self.mission_status_label = QLabel("Aucune mission sélectionnée")
        self.mission_status_label.setStyleSheet("padding: 5px; background-color: #f0f0f0; border: 1px solid #ccc;")

        self.mission_combo = QComboBox()
        self.mission_combo.setEditable(False)
        mission_select_layout.addWidget(self.mission_combo, 1)

        btn_new_mission = QPushButton("➕ Nouvelle mission")
        btn_new_mission.clicked.connect(self.create_new_mission)
        mission_select_layout.addWidget(btn_new_mission)

        mission_left_layout.addLayout(mission_select_layout)
        mission_left_layout.addWidget(self.mission_status_label)

        mission_right_layout = QVBoxLayout()
        self.mission_description = QTextEdit()
        self.mission_description.setPlaceholderText("Description libre de la mission...")
        self.mission_description.setFixedHeight(48)
        mission_right_layout.addWidget(self.mission_description)
        mission_right_layout.addStretch()

        mission_columns_layout.addLayout(mission_left_layout, 1)
        mission_columns_layout.addLayout(mission_right_layout, 2)

        mission_layout.addLayout(mission_columns_layout)
        
        mission_group.setLayout(mission_layout)
        main_layout.addWidget(mission_group)
        
        # Remplir la liste et connecter le signal
        self.update_mission_list()
        self.mission_combo.currentTextChanged.connect(self.on_mission_changed)
        self.mission_combo.highlighted.connect(self.on_mission_highlighted)
        self.mission_combo.activated.connect(self.on_mission_activated)  # Cache le tooltip quand on sélectionne
        
        # Boutons de chargement/sauvegarde (en haut, sur toute la largeur)
        button_layout = QHBoxLayout()
        
        btn_load = QPushButton("📂 Charger Param_mission")
        btn_load.clicked.connect(self.load_from_mission)
        button_layout.addWidget(btn_load)
        
        btn_save = QPushButton("💾 Sauvegarder Param_mission")
        btn_save.clicked.connect(self.save_to_mission)
        button_layout.addWidget(btn_save)
        
        btn_load_file = QPushButton("📂 Charger depuis fichier...")
        btn_load_file.clicked.connect(self.load_from_file)
        button_layout.addWidget(btn_load_file)
        
        btn_save_file = QPushButton("💾 Sauvegarder vers fichier...")
        btn_save_file.clicked.connect(self.save_to_file)
        button_layout.addWidget(btn_save_file)
        
        main_layout.addLayout(button_layout)
        
        # Zone principale avec 3 colonnes
        columns_layout = QHBoxLayout()
        columns_layout.setSpacing(10)
        
        # Colonne 1 : Paramètres d'environnement
        col1 = self.create_environment_column()
        columns_layout.addWidget(col1, 1)
        
        # Colonne 2 : Paramètres de calcul
        col2 = self.create_calc_column()
        columns_layout.addWidget(col2, 1)
        
        # Colonne 3 : Conditions initiales
        col3 = self.create_init_column()
        columns_layout.addWidget(col3, 1)
        
        main_layout.addLayout(columns_layout, 1)
        self._connect_free_fall_inputs()
    
    def _update_auto_L_combo(self):
        """Remplit le combo Auto_L avec la liste des fonctions auto_L_xxx du code source."""
        self.auto_L_combo.clear()
        self.auto_L_docstrings.clear()
        names = list_auto_L_functions()
        if names:
            self.auto_L_combo.addItems(names)
            # Charger les docstrings pour chaque fonction
            for name in names:
                func = get_auto_L_function(name)
                if func:
                    docstring = inspect.getdoc(func) or "Aucune documentation disponible."
                    self.auto_L_docstrings[name] = docstring
        else:
            self.auto_L_combo.addItem("auto_L_1")
            func = get_auto_L_function("auto_L_1")
            if func:
                docstring = inspect.getdoc(func) or "Aucune documentation disponible."
                self.auto_L_docstrings["auto_L_1"] = docstring
    
    def create_environment_column(self):
        """Crée la colonne des paramètres d'environnement"""
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        layout = QVBoxLayout(scroll_content)
        
        # Paramètres ROV
        rov_group = self.create_rov_parameters_group()
        layout.addWidget(rov_group)
        
        # Paramètres Câble
        cable_group = self.create_cable_parameters_group()
        layout.addWidget(cable_group)
        
        # Paramètres Bateau
        boat_group = self.create_boat_parameters_group()
        layout.addWidget(boat_group)
        
        # Paramètres Environnement
        env_group = self.create_environment_parameters_group()
        layout.addWidget(env_group)
        
        layout.addStretch()
        scroll.setWidget(scroll_content)
        return scroll
    
    def create_calc_column(self):
        """Crée la colonne des paramètres de calcul"""
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        layout = QVBoxLayout(scroll_content)
        
        # Méthode d'intégration
        method_group = QGroupBox("Méthode d'intégration")
        method_layout = QGridLayout()
        method_layout.addWidget(QLabel("Méthode:"), 0, 0)
        self.method_combo = QComboBox()
        self.method_combo.addItems(['RK45', 'RK23', 'DOP853', 'Radau'])
        self.method_combo.currentTextChanged.connect(self.on_method_changed)
        method_layout.addWidget(self.method_combo, 0, 1)
        method_layout.addWidget(
            HelpButton(
                "Choix de l’algorithme d’intégration numérique (solve_ivp). "
                "RK45 est un bon compromis. RK23 est plus rapide mais moins précis. "
                "DOP853 est plus précis pour des dynamiques raides. Radau est "
                "implicite et stable mais plus coûteux.",
                self,
            ),
            0,
            2,
        )
        method_group.setLayout(method_layout)
        layout.addWidget(method_group)
        
        # Tolérances
        tolerance_group = QGroupBox("Tolérances")
        tolerance_layout = QGridLayout()
        tolerance_layout.addWidget(QLabel("Tolérance relative (rtol):"), 0, 0)
        self.rtol = QLineEdit()
        tolerance_layout.addWidget(self.rtol, 0, 1)
        tolerance_layout.addWidget(
            HelpButton(
                "Tolérance relative de l’intégrateur. Plus elle est petite, plus "
                "la solution est précise mais plus le calcul est long. Valeurs "
                "trop faibles peuvent ralentir fortement ou entraîner des pas "
                "d’intégration très petits.",
                self,
            ),
            0,
            2,
        )
        tolerance_layout.addWidget(QLabel("Tolérance absolue (atol):"), 1, 0)
        self.atol = QLineEdit()
        tolerance_layout.addWidget(self.atol, 1, 1)
        tolerance_layout.addWidget(
            HelpButton(
                "Tolérance absolue de l’intégrateur. Utile lorsque les variables "
                "peuvent être proches de zéro. Réduire cette valeur augmente la "
                "précision, mais peut forcer des pas plus petits et ralentir la "
                "simulation.",
                self,
            ),
            1,
            2,
        )
        tolerance_group.setLayout(tolerance_layout)
        layout.addWidget(tolerance_group)
        
        # Pas de temps
        timestep_group = QGroupBox("Pas de temps")
        timestep_layout = QGridLayout()
        timestep_layout.addWidget(QLabel("Pas maximum (max_step):"), 0, 0)
        self.max_step = QLineEdit()
        timestep_layout.addWidget(self.max_step, 0, 1)
        timestep_layout.addWidget(
            HelpButton(
                "Pas maximum autorisé par l’intégrateur. Limite la taille des "
                "pas temporels pour éviter de “sauter” des dynamiques rapides. "
                "Une valeur trop petite ralentit la simulation, trop grande peut "
                "dégrader la précision.",
                self,
            ),
            0,
            2,
        )
        timestep_layout.addWidget(QLabel("Pas maximum dt_max (s):"), 1, 0)
        self.dt_max = QLineEdit()
        timestep_layout.addWidget(self.dt_max, 1, 1)
        timestep_layout.addWidget(
            HelpButton(
                "Pas maximum utilisé par la boucle de simulation pour avancer "
                "le temps entre deux mises à jour. Sert de garde‑fou global, "
                "indépendamment de l’intégrateur, pour stabiliser la boucle UI.",
                self,
            ),
            1,
            2,
        )
        timestep_group.setLayout(timestep_layout)
        layout.addWidget(timestep_group)
        
        # Simulation
        sim_group = QGroupBox("Paramètres de simulation")
        sim_layout = QGridLayout()
        sim_layout.addWidget(QLabel("Temps final (s):"), 0, 0)
        self.t_final = QLineEdit()
        sim_layout.addWidget(self.t_final, 0, 1)
        sim_layout.addWidget(
            HelpButton(
                "Durée totale de la simulation. La boucle s’arrête lorsque "
                "le temps atteint cette valeur. Augmenter ce paramètre allonge "
                "la durée de calcul et le volume de données enregistrées.",
                self,
            ),
            0,
            2,
        )
        sim_layout.addWidget(QLabel("Pas par mise à jour:"), 1, 0)
        self.steps_per_update = QLineEdit()
        sim_layout.addWidget(self.steps_per_update, 1, 1)
        sim_layout.addWidget(
            HelpButton(
                "Nombre de pas d’intégration entre deux rafraîchissements de "
                "l’interface. Plus la valeur est grande, plus l’UI est fluide "
                "mais moins réactive; plus petite, l’UI est plus précise mais "
                "peut ralentir.",
                self,
            ),
            1,
            2,
        )

        # Paramètres de performance / taille des données
        sim_layout.addWidget(QLabel("Taille max données (points):"), 2, 0)
        self.max_data_size = QLineEdit()
        sim_layout.addWidget(self.max_data_size, 2, 1)
        sim_layout.addWidget(
            HelpButton(
                "Nombre maximum de points conservés dans les séries temporelles "
                "(`data`). Les plus anciens points sont supprimés au‑delà de cette "
                "limite pour éviter une explosion mémoire (défaut: 10000).",
                self,
            ),
            2,
            2,
        )

        sim_layout.addWidget(QLabel("Intervalle écriture Excel (itérations):"), 3, 0)
        self.excel_write_interval = QLineEdit()
        sim_layout.addWidget(self.excel_write_interval, 3, 1)
        sim_layout.addWidget(
            HelpButton(
                "Nombre de pas de simulation entre deux écritures dans le fichier "
                "Excel `trace_dic.xlsx`. Augmenter cette valeur réduit le coût "
                "d’entrées/sorties disque (défaut: 50).",
                self,
            ),
            3,
            2,
        )

        sim_layout.addWidget(QLabel("Points max graphiques (plot_points_limit):"), 4, 0)
        self.plot_points_limit = QLineEdit()
        sim_layout.addWidget(self.plot_points_limit, 4, 1)
        sim_layout.addWidget(
            HelpButton(
                "Nombre maximum de points envoyés à l’interface pour chaque "
                "courbe. Les graphiques n’affichent que les N derniers points "
                "pour limiter le volume de données (défaut: 1000).",
                self,
            ),
            4,
            2,
        )

        sim_group.setLayout(sim_layout)
        layout.addWidget(sim_group)
        
        # Discrétisation
        disc_group = QGroupBox("Discrétisation du câble")
        disc_layout = QGridLayout()
        disc_layout.addWidget(QLabel("Nombre de segments (N_segments):"), 0, 0)
        self.n_segments = QLineEdit()
        disc_layout.addWidget(self.n_segments, 0, 1)
        disc_layout.addWidget(
            HelpButton(
                "Nombre de segments pour discrétiser le câble. Augmenter N améliore "
                "la fidélité géométrique et les forces locales, mais augmente le "
                "temps de calcul. Réduire N accélère la simulation mais lisse la "
                "forme du câble.",
                self,
            ),
            0,
            2,
        )
        disc_group.setLayout(disc_layout)
        layout.addWidget(disc_group)

        # Visualisation
        visual_group = QGroupBox("Visualisation")
        visual_layout = QGridLayout()
        visual_layout.addWidget(QLabel("Lissage transition mode (0–1):"), 0, 0)
        self.transition_alpha = QLineEdit()
        visual_layout.addWidget(self.transition_alpha, 0, 1)
        visual_layout.addWidget(
            HelpButton(
                "Facteur de lissage (0–1) appliqué à la transition caténaire → "
                "ligne droite. 0 désactive le lissage, 1 applique le lissage "
                "complet prévu par le modèle. Utile pour éviter des variations "
                "brutales de tension.",
                self,
            ),
            0,
            2,
        )
        visual_group.setLayout(visual_layout)
        layout.addWidget(visual_group)
        
        layout.addStretch()
        scroll.setWidget(scroll_content)
        return scroll
    
    def create_init_column(self):
        """Crée la colonne des conditions initiales"""
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        layout = QVBoxLayout(scroll_content)
        
        # Position ROV
        rov_group = QGroupBox("Position initiale du ROV")
        rov_layout = QGridLayout()
        rov_layout.addWidget(QLabel("Position horizontale x_rov_init (m):"), 0, 0)
        self.x_rov_init = QLineEdit()
        rov_layout.addWidget(self.x_rov_init, 0, 1)
        rov_layout.addWidget(QLabel("Profondeur y_rov_init (m, négatif):"), 1, 0)
        self.y_rov_init = QLineEdit()
        rov_layout.addWidget(self.y_rov_init, 1, 1)
        rov_group.setLayout(rov_layout)
        layout.addWidget(rov_group)
        
        # Position Bateau
        boat_group = QGroupBox("Position initiale du Bateau")
        boat_layout = QGridLayout()
        boat_layout.addWidget(QLabel("Position horizontale x_boat_init (m):"), 0, 0)
        self.x_boat_init = QLineEdit()
        boat_layout.addWidget(self.x_boat_init, 0, 1)
        boat_group.setLayout(boat_layout)
        layout.addWidget(boat_group)
        
        # Câble
        cable_group = QGroupBox("Paramètres du câble")
        cable_layout = QGridLayout()
        cable_layout.addWidget(QLabel("Longueur initiale L_init (m):"), 0, 0)
        self.L_init = QLineEdit()
        cable_layout.addWidget(self.L_init, 0, 1)
        cable_group.setLayout(cable_layout)
        layout.addWidget(cable_group)
        
        # Environnement
        env_group = QGroupBox("Environnement")
        env_layout = QGridLayout()
        env_layout.addWidget(QLabel("Vitesse du courant v_courant (m/s):"), 0, 0)
        self.v_courant = QLineEdit()
        env_layout.addWidget(self.v_courant, 0, 1)
        env_group.setLayout(env_layout)
        layout.addWidget(env_group)
        
        # Scénario
        scenario_group = self.create_scenario_group()
        layout.addWidget(scenario_group)
        
        layout.addStretch()
        scroll.setWidget(scroll_content)
        return scroll
    
    def create_rov_parameters_group(self):
        """Crée le groupe de paramètres ROV"""
        group = QGroupBox("Paramètres ROV")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("Masse (kg):"), 0, 0)
        self.rov_m = QLineEdit()
        layout.addWidget(self.rov_m, 0, 1)
        
        layout.addWidget(QLabel("Volume vol (m³, optionnel):"), 1, 0)
        self.rov_vol = QLineEdit()
        self.rov_vol.setPlaceholderText("laisser vide pour utiliser a*b*h")
        layout.addWidget(self.rov_vol, 1, 1)
        
        layout.addWidget(QLabel("Largeur a (m):"), 2, 0)
        self.rov_a = QLineEdit()
        layout.addWidget(self.rov_a, 2, 1)
        
        layout.addWidget(QLabel("Longueur b (m):"), 3, 0)
        self.rov_b = QLineEdit()
        layout.addWidget(self.rov_b, 3, 1)
        
        layout.addWidget(QLabel("Hauteur h (m):"), 4, 0)
        self.rov_h = QLineEdit()
        layout.addWidget(self.rov_h, 4, 1)
        
        layout.addWidget(QLabel("Coeff. traînée Cx, Cy:"), 5, 0)
        self.rov_cx = QLineEdit()
        self.rov_cy = QLineEdit()
        drag_layout = QHBoxLayout()
        drag_layout.setContentsMargins(0, 0, 0, 0)
        drag_layout.addWidget(self.rov_cx)
        drag_layout.addWidget(self.rov_cy)
        layout.addLayout(drag_layout, 5, 1)

        layout.addWidget(QLabel("Poids apparent:"), 6, 0)
        self.rov_apparent_weight = QLineEdit()
        self.rov_apparent_weight.setReadOnly(True)
        self.rov_apparent_weight.setPlaceholderText("calculé automatiquement")
        layout.addWidget(self.rov_apparent_weight, 6, 1)

        layout.addWidget(QLabel("dy/dt libre limite:"), 7, 0)
        self.rov_free_fall_speed = QLineEdit()
        self.rov_free_fall_speed.setReadOnly(True)
        self.rov_free_fall_speed.setPlaceholderText("calculée automatiquement")
        layout.addWidget(self.rov_free_fall_speed, 7, 1)
        layout.addWidget(
            HelpButton(
                "Vitesse limite incluant l'effet de la force de traînée. "
                "Une vitesse positive indique une flottabilité positive; le ROV remonte.",
                self,
            ),
            7,
            2,
        )
        
        group.setLayout(layout)
        return group
    
    def create_cable_parameters_group(self):
        """Crée le groupe de paramètres Câble"""
        group = QGroupBox("Paramètres Câble")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("Diamètre d (m):"), 0, 0)
        self.cable_d = QLineEdit()
        layout.addWidget(self.cable_d, 0, 1)
        
        layout.addWidget(QLabel("Masse volumique (kg/m³):"), 1, 0)
        self.cable_rho = QLineEdit()
        layout.addWidget(self.cable_rho, 1, 1)
        
        layout.addWidget(QLabel("Coeff. traînée Cx:"), 2, 0)
        self.cable_cx = QLineEdit()
        layout.addWidget(self.cable_cx, 2, 1)

        layout.addWidget(QLabel("Coeff. frottement Cf:"), 3, 0)
        self.cable_cf = QLineEdit()
        layout.addWidget(self.cable_cf, 3, 1)

        layout.addWidget(QLabel("Tension de rupture (N):"), 4, 0)
        self.cable_break_tension = QLineEdit()
        layout.addWidget(self.cable_break_tension, 4, 1)
        
        group.setLayout(layout)
        return group
    
    def create_boat_parameters_group(self):
        """Crée le groupe de paramètres Bateau"""
        group = QGroupBox("Paramètres Bateau")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("Masse (kg):"), 0, 0)
        self.boat_m = QLineEdit()
        layout.addWidget(self.boat_m, 0, 1)
        
        layout.addWidget(QLabel("Coeff. traînée:"), 1, 0)
        self.boat_drag = QLineEdit()
        layout.addWidget(self.boat_drag, 1, 1)

        layout.addWidget(QLabel("Bornes dL/dt (min, max):"), 2, 0)
        self.boat_dl_dt_min = QLineEdit()
        self.boat_dl_dt_max = QLineEdit()
        layout.addWidget(self.boat_dl_dt_min, 2, 1)
        layout.addWidget(self.boat_dl_dt_max, 2, 2)

        layout.addWidget(QLabel("Bornes d²L/dt² (min, max):"), 3, 0)
        self.boat_gamma_min = QLineEdit()
        self.boat_gamma_max = QLineEdit()
        layout.addWidget(self.boat_gamma_min, 3, 1)
        layout.addWidget(self.boat_gamma_max, 3, 2)
        
        group.setLayout(layout)
        return group
    
    def create_environment_parameters_group(self):
        """Crée le groupe de paramètres Environnement"""
        group = QGroupBox("Paramètres Environnement")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("Masse volumique eau (kg/m³):"), 0, 0)
        self.env_rho = QLineEdit()
        layout.addWidget(self.env_rho, 0, 1)
        
        layout.addWidget(QLabel("Gravité g (m/s²):"), 1, 0)
        self.env_g = QLineEdit()
        layout.addWidget(self.env_g, 1, 1)
        
        layout.addWidget(QLabel("Viscosité μ (Pa·s):"), 2, 0)
        self.env_mu = QLineEdit()
        layout.addWidget(self.env_mu, 2, 1)
        
        group.setLayout(layout)
        return group
    
    def create_scenario_group(self):
        """Crée le groupe de paramètres Scénario"""
        group = QGroupBox("Scénarios")
        layout = QGridLayout()
        
        layout.addWidget(QLabel("ROV Fx:"), 0, 0)
        self.sc_fx_rov = QTextEdit()
        self.sc_fx_rov.setAcceptRichText(False)
        fx_height = self.sc_fx_rov.fontMetrics().lineSpacing() * 3 + 8
        self.sc_fx_rov.setFixedHeight(fx_height)
        layout.addWidget(self.sc_fx_rov, 0, 1)
        layout.addWidget(
            HelpButton("Force horizontale ROV imposée par le scénario (N).", self),
            0, 2
        )
        
        layout.addWidget(QLabel("ROV Fy:"), 1, 0)
        self.sc_fy_rov = QTextEdit()
        self.sc_fy_rov.setAcceptRichText(False)
        fy_height = self.sc_fy_rov.fontMetrics().lineSpacing() * 3 + 8
        self.sc_fy_rov.setFixedHeight(fy_height)
        layout.addWidget(self.sc_fy_rov, 1, 1)
        layout.addWidget(
            HelpButton("Force verticale ROV imposée par le scénario (N).", self),
            1, 2
        )
        
        layout.addWidget(QLabel("Bateau:"), 2, 0)
        self.sc_v_bateau = QLineEdit()
        layout.addWidget(self.sc_v_bateau, 2, 1)
        layout.addWidget(
            HelpButton("Vitesse commande bateau imposée par le scénario (m/s).", self),
            2, 2
        )
        
        layout.addWidget(QLabel("Moulinet"), 3, 0)
        self.sc_v_moulinet = QLineEdit()
        layout.addWidget(self.sc_v_moulinet, 3, 1)
        layout.addWidget(
            HelpButton("Vitesse de déroulement du câble imposée par le scénario (m/s).", self),
            3, 2
        )

        layout.addWidget(QLabel("Auto_L"), 4, 0)
        self.auto_L_combo = QComboBox()
        self.auto_L_combo.setEditable(False)
        self._update_auto_L_combo()
        # Tooltips persistants pour les fonctions auto_L
        self.auto_L_combo.highlighted.connect(self.on_auto_L_highlighted)
        self.auto_L_combo.activated.connect(self.on_auto_L_activated)  # Cache le tooltip quand on sélectionne
        # Cacher le tooltip quand la souris quitte la liste déroulante
        if self.auto_L_combo.view() is not None:
            self.auto_L_combo.view().viewport().installEventFilter(self)
        layout.addWidget(self.auto_L_combo, 4, 1)
        layout.addWidget(
            HelpButton(
                "Fonction auto_L_xxx utilisée en mode Auto (xxx = identifiant alphanumérique).",
                self,
            ),
            4, 2
        )

        layout.addWidget(QLabel("Tension cible:"), 5, 0)
        self.tcible = QLineEdit()
        layout.addWidget(self.tcible, 5, 1)
        layout.addWidget(
            HelpButton(
                "Tension cible au niveau du bateau (N), utilisée par les lois auto_L_n.",
                self,
            ),
            5, 2
        )

        group.setLayout(layout)
        return group
    
    def load_all_parameters_display(self):
        """Charge tous les paramètres dans l'affichage"""
        # Paramètres d'environnement
        params = self.main_window.parameters
        if params:
            rov = params.get('rov', {})
            self.rov_m.setText(str(rov.get('m', '')))
            # Volume : peut être None (utiliser a*b*h) ou une valeur numérique
            vol = rov.get('vol', None)
            self.rov_vol.setText("" if vol is None else str(vol))
            self.rov_a.setText(str(rov.get('a', '')))
            self.rov_b.setText(str(rov.get('b', '')))
            self.rov_h.setText(str(rov.get('h', '')))
            self.rov_cx.setText(str(rov.get('Cx', '')))
            self.rov_cy.setText(str(rov.get('Cy', '')))
            
            cable = params.get('cable', {})
            self.cable_d.setText(str(cable.get('d', '')))
            self.cable_rho.setText(str(cable.get('rho_cable', '')))
            self.cable_cx.setText(str(cable.get('Cx_cable', '')))
            self.cable_cf.setText(str(cable.get('Cf_cable', '')))
            self.cable_break_tension.setText(str(cable.get('tension_rupture', '')))
            
            boat = params.get('boat', {})
            self.boat_m.setText(str(boat.get('m', '')))
            self.boat_drag.setText(str(boat.get('drag_coefficient', '')))
            if hasattr(self, "boat_dl_dt_min"):
                self.boat_dl_dt_min.setText(str(boat.get('dl_dt_min', '')))
            if hasattr(self, "boat_dl_dt_max"):
                self.boat_dl_dt_max.setText(str(boat.get('dl_dt_max', '')))
            if hasattr(self, "boat_gamma_min"):
                self.boat_gamma_min.setText(str(boat.get('gamma_moulinet_min', '')))
            if hasattr(self, "boat_gamma_max"):
                self.boat_gamma_max.setText(str(boat.get('gamma_moulinet_max', '')))
            
            env = params.get('environment', {})
            self.env_rho.setText(str(env.get('rho_eau', '')))
            self.env_g.setText(str(env.get('g', '')))
            self.env_mu.setText(str(env.get('mu', '')))
        
        # Paramètres de calcul
        calc_params = self.main_window.calc_params
        self.method_combo.setCurrentText(calc_params.get('method', 'RK45'))
        self.rtol.setText(str(calc_params.get('rtol', 1e-5)))
        self.atol.setText(str(calc_params.get('atol', 1e-7)))
        self.max_step.setText(str(calc_params.get('max_step', 0.1)))
        self.dt_max.setText(str(calc_params.get('dt_max', 0.1)))
        self.t_final.setText(str(calc_params.get('t_final', 60.0)))
        self.steps_per_update.setText(str(calc_params.get('steps_per_update', 5)))
        self.n_segments.setText(str(calc_params.get('N_segments', 50)))
        self.transition_alpha.setText(str(calc_params.get('straight_blend_alpha', 1.0)))
        # Nouveaux paramètres de performance / taille des données
        if hasattr(self, "max_data_size"):
            self.max_data_size.setText(str(calc_params.get('max_data_size', 10000)))
        if hasattr(self, "excel_write_interval"):
            self.excel_write_interval.setText(str(calc_params.get('excel_write_interval', 50)))
        if hasattr(self, "plot_points_limit"):
            self.plot_points_limit.setText(str(calc_params.get('plot_points_limit', 1000)))
        self.sc_fx_rov.setPlainText(str(calc_params.get('sc_fx_rov', '')))
        self.sc_fy_rov.setPlainText(str(calc_params.get('sc_fy_rov', '')))
        self.sc_v_bateau.setText(str(calc_params.get('sc_v_bateau', '')))
        self.sc_v_moulinet.setText(str(calc_params.get('sc_v_moulinet', '')))
        if hasattr(self, "auto_L_combo"):
            auto_L_val = calc_params.get('auto_L', 'auto_L_1')
            if isinstance(auto_L_val, int):
                auto_L_val = f"auto_L_{auto_L_val}"
            self._suppress_auto_L_tooltip = True
            self.auto_L_tooltip.hide()
            self.auto_L_combo.blockSignals(True)
            idx = self.auto_L_combo.findText(auto_L_val)
            if idx >= 0:
                self.auto_L_combo.setCurrentIndex(idx)
            else:
                self.auto_L_combo.setCurrentIndex(0)
            self.auto_L_combo.blockSignals(False)
            self._suppress_auto_L_tooltip = False
        if hasattr(self, "tcible"):
            self.tcible.setText(str(calc_params.get('Tcible', "")))

        if hasattr(self, "mission_description"):
            self.mission_description.setPlainText(str(self.main_window.mission_data.get('description', "")))
        
        # Conditions initiales
        init_params = self.main_window.init_params
        self.x_rov_init.setText(str(init_params.get('x_rov_init', 0.0)))
        self.y_rov_init.setText(str(init_params.get('y_rov_init', -10.0)))  # Profondeur négative
        self.x_boat_init.setText(str(init_params.get('x_boat_init', 0.0)))
        self.L_init.setText(str(init_params.get('L_init', 50.0)))
        self.v_courant.setText(str(init_params.get('v_courant', 0.0)))
        self.update_free_fall_speed()

    def _connect_free_fall_inputs(self):
        inputs = [
            self.env_rho,
            self.env_g,
            self.rov_m,
            self.rov_vol,
            self.rov_a,
            self.rov_b,
            self.rov_h,
            self.rov_cy,
        ]
        for widget in inputs:
            widget.textChanged.connect(self.update_free_fall_speed)

    def update_free_fall_speed(self):
        metrics = self._compute_free_fall_metrics()
        if metrics is None:
            self.rov_apparent_weight.setText("")
            self.rov_free_fall_speed.setText("")
            return
        effective_weight, speed = metrics
        apparent_weight_display = -effective_weight
        self.rov_apparent_weight.setText(f"{apparent_weight_display:.1f} N")
        self.rov_free_fall_speed.setText(f"{speed:.3f} m/s")

    def _compute_free_fall_metrics(self):
        def read_float(widget):
            try:
                return float((widget.text() or "").strip())
            except ValueError:
                return None

        rho = read_float(self.env_rho)
        mass = read_float(self.rov_m)
        g = read_float(self.env_g)
        a = read_float(self.rov_a)
        b = read_float(self.rov_b)
        h = read_float(self.rov_h)
        cy = read_float(self.rov_cy)
        vol_text = (self.rov_vol.text() or "").strip()
        volume = read_float(self.rov_vol) if vol_text != "" else None

        if None in (rho, mass, g, a, b, h, cy):
            return None
        if rho <= 0 or mass <= 0 or g <= 0 or a <= 0 or b <= 0 or h <= 0 or cy <= 0:
            return None
        if volume is not None and volume <= 0:
            return None

        area = a * b
        if area <= 0:
            return None

        if volume is None:
            volume = a * b * h

        effective_weight = (mass - rho * volume) * g
        if effective_weight == 0:
            return 0.0, 0.0

        speed = math.sqrt((2.0 * abs(effective_weight)) / (rho * cy * area))
        speed = speed if effective_weight < 0 else -speed
        return effective_weight, speed
    
    def save_all_parameters_from_display(self):
        """Sauvegarde tous les paramètres depuis l'affichage"""
        try:
            def parse_float(value, default, label):
                text = (value or "").strip()
                if text == "":
                    return default
                try:
                    return float(text)
                except ValueError as exc:
                    raise ValueError(f"{label}: {text}") from exc
            
            def parse_int(value, default, label):
                text = (value or "").strip()
                if text == "":
                    return default
                try:
                    return int(text)
                except ValueError as exc:
                    raise ValueError(f"{label}: {text}") from exc

            # Paramètres d'environnement
            # Volume : champ optionnel, vide => None (on utilisera a*b*h dans le modèle)
            vol_text = (self.rov_vol.text() or "").strip()
            vol_value = None
            if vol_text != "":
                vol_value = parse_float(vol_text, None, "Volume vol (m³)")

            self.main_window.parameters = {
                'rov': {
                    'm': parse_float(self.rov_m.text(), 100.0, "Masse ROV (kg)"),
                    'vol': vol_value,
                    'a': parse_float(self.rov_a.text(), 0.5, "Largeur a (m)"),
                    'b': parse_float(self.rov_b.text(), 1.0, "Longueur b (m)"),
                    'h': parse_float(self.rov_h.text(), 0.5, "Hauteur h (m)"),
                    'Cx': parse_float(self.rov_cx.text(), 0.8, "Coeff. traînée Cx"),
                    'Cy': parse_float(self.rov_cy.text(), 1.0, "Coeff. traînée Cy")
                },
                'cable': {
                    'd': parse_float(self.cable_d.text(), 0.01, "Diamètre câble d (m)"),
                    'rho_cable': parse_float(self.cable_rho.text(), 1500.0, "Masse volumique câble (kg/m³)"),
                    'Cx_cable': parse_float(self.cable_cx.text(), 1.2, "Coeff. traînée câble Cx"),
                    'Cf_cable': parse_float(self.cable_cf.text(), 0.04, "Coeff. frottement câble Cf"),
                    'tension_rupture': parse_float(
                        self.cable_break_tension.text(),
                        50.0,
                        "Tension de rupture (N)"
                    )
                },
                'boat': {
                    'm': parse_float(self.boat_m.text(), 10000.0, "Masse bateau (kg)"),
                    'drag_coefficient': parse_float(self.boat_drag.text(), 0.5, "Coeff. traînée bateau"),
                    'dl_dt_min': parse_float(
                        self.boat_dl_dt_min.text(), -1.0, "Borne dL/dt min (m/s)"
                    ) if hasattr(self, "boat_dl_dt_min") else -1.0,
                    'dl_dt_max': parse_float(
                        self.boat_dl_dt_max.text(), 1.0, "Borne dL/dt max (m/s)"
                    ) if hasattr(self, "boat_dl_dt_max") else 1.0,
                    'gamma_moulinet_min': parse_float(
                        self.boat_gamma_min.text(), -0.5, "Borne d²L/dt² min (m/s²)"
                    ) if hasattr(self, "boat_gamma_min") else -0.5,
                    'gamma_moulinet_max': parse_float(
                        self.boat_gamma_max.text(), 0.5, "Borne d²L/dt² max (m/s²)"
                    ) if hasattr(self, "boat_gamma_max") else 0.5,
                },
                'environment': {
                    'rho_eau': parse_float(self.env_rho.text(), 1025.0, "Masse volumique eau (kg/m³)"),
                    'g': parse_float(self.env_g.text(), 9.81, "Gravité g (m/s²)"),
                    'mu': parse_float(self.env_mu.text(), 1e-3, "Viscosité μ (Pa·s)")
                }
            }
            
            # Paramètres de calcul
            self.main_window.calc_params = {
                'method': self.method_combo.currentText(),
                'rtol': parse_float(self.rtol.text(), 1e-5, "Tolérance relative (rtol)"),
                'atol': parse_float(self.atol.text(), 1e-7, "Tolérance absolue (atol)"),
                'max_step': parse_float(self.max_step.text(), 0.1, "Pas maximum (max_step)"),
                'dt_max': parse_float(self.dt_max.text(), 0.1, "Pas maximum dt_max (s)"),
                't_final': parse_float(self.t_final.text(), 60.0, "Temps final (s)"),
                'steps_per_update': parse_int(self.steps_per_update.text(), 5, "Pas par mise à jour"),
                'N_segments': parse_int(self.n_segments.text(), 50, "Nombre de segments (N_segments)"),
                'straight_blend_alpha': parse_float(
                    self.transition_alpha.text(), 1.0, "Lissage transition mode"
                ),
                # Nouveaux paramètres de performance / taille des données
                'max_data_size': parse_int(
                    self.max_data_size.text(), 10000, "Taille max données (max_data_size)"
                ),
                'excel_write_interval': parse_int(
                    self.excel_write_interval.text(), 50, "Intervalle écriture Excel (excel_write_interval)"
                ),
                'plot_points_limit': parse_int(
                    self.plot_points_limit.text(), 1000, "Points max graphiques (plot_points_limit)"
                ),
                'sc_fx_rov': (self.sc_fx_rov.toPlainText() or "").strip(),
                'sc_fy_rov': (self.sc_fy_rov.toPlainText() or "").strip(),
                'sc_v_bateau': (self.sc_v_bateau.text() or "").strip(),
                'sc_v_moulinet': (self.sc_v_moulinet.text() or "").strip(),
                'auto_L': self.auto_L_combo.currentText() if hasattr(self, "auto_L_combo") else "auto_L_1",
                'Tcible': parse_float(self.tcible.text(), None, "Tension cible") if hasattr(self, "tcible") else None,
            }

            if not self._validate_scenarios(self.main_window.calc_params):
                return False
            
            # Conditions initiales
            v_courant_text = (self.v_courant.text() or "").strip()
            v_courant_value = v_courant_text if v_courant_text != "" else "0.0"
            
            self.main_window.init_params = {
                'x_rov_init': parse_float(self.x_rov_init.text(), 0.0, "Position x_rov_init (m)"),
                'y_rov_init': parse_float(self.y_rov_init.text(), -10.0, "Profondeur y_rov_init (m)"),  # Profondeur négative
                'x_boat_init': parse_float(self.x_boat_init.text(), 0.0, "Position x_boat_init (m)"),
                'L_init': parse_float(self.L_init.text(), 50.0, "Longueur initiale L_init (m)"),
                'v_courant': v_courant_value
            }
            
            # Réinitialiser automatiquement le système après sauvegarde des paramètres
            if hasattr(self.main_window, 'simulation_tab'):
                self.main_window.simulation_tab.initialize_system()
            
            return True
        except ValueError as e:
            QMessageBox.warning(self, "Erreur", f"Valeur invalide: {str(e)}")
            return False
    
    # Méthodes de gestion des missions (reprises de parameters_tab.py)
    def update_mission_list(self):
        """Met à jour la liste des missions disponibles"""
        self.mission_combo.clear()
        self.mission_descriptions.clear()
        missions = list_missions()
        
        # Charger les descriptions pour chaque mission
        for mission_name in missions:
            description = load_mission_description(mission_name)
            self.mission_descriptions[mission_name] = description
        
        self.mission_combo.addItems(missions)
        
        current_mission = self.main_window.mission_data.get('current') if self.main_window.mission_data else None
        if current_mission and current_mission in missions:
            self.mission_combo.blockSignals(True)
            self.mission_combo.setCurrentText(current_mission)
            self.mission_combo.blockSignals(False)
    
    def on_mission_highlighted(self, index):
        """Affiche un tooltip persistant avec la description de la mission survolée"""
        if index >= 0 and index < self.mission_combo.count():
            mission_name = self.mission_combo.itemText(index)
            description = self.mission_descriptions.get(mission_name, "")
            
            if description:
                # Mettre à jour le texte du tooltip
                self.mission_tooltip.setText(description)
                self.mission_tooltip.adjustSize()
                
                # Positionner le tooltip près du curseur
                cursor_pos = QCursor.pos()
                tooltip_pos = QPoint(cursor_pos.x() + 15, cursor_pos.y() + 15)
                self.mission_tooltip.move(tooltip_pos)
                self.mission_tooltip.show()
            else:
                # Cacher le tooltip si pas de description
                self.mission_tooltip.hide()
        else:
            self.mission_tooltip.hide()
    
    def on_mission_activated(self, index):
        """Cache le tooltip quand une mission est sélectionnée (la liste se ferme)"""
        self.mission_tooltip.hide()
    
    def on_auto_L_highlighted(self, index):
        """Affiche un tooltip persistant avec la docstring de la fonction auto_L survolée"""
        if self._suppress_auto_L_tooltip:
            self.auto_L_tooltip.hide()
            return

        view = self.auto_L_combo.view() if hasattr(self, "auto_L_combo") else None
        if view is None or not view.isVisible():
            self.auto_L_tooltip.hide()
            return

        if index >= 0 and index < self.auto_L_combo.count():
            func_name = self.auto_L_combo.itemText(index)
            docstring = self.auto_L_docstrings.get(func_name, "")
            
            if docstring:
                # Convertir les retours à la ligne en <br/> pour l'affichage HTML
                docstring_html = docstring.replace('\n', '<br/>')
                # Mettre à jour le texte du tooltip avec formatage HTML
                self.auto_L_tooltip.setText(f"<b>{func_name}</b><br/><br/>{docstring_html}")
                self.auto_L_tooltip.adjustSize()
                
                # Positionner le tooltip près du curseur
                cursor_pos = QCursor.pos()
                tooltip_pos = QPoint(cursor_pos.x() + 15, cursor_pos.y() + 15)
                self.auto_L_tooltip.move(tooltip_pos)
                self.auto_L_tooltip.show()
            else:
                # Cacher le tooltip si pas de docstring
                self.auto_L_tooltip.hide()
        else:
            self.auto_L_tooltip.hide()
    
    def on_auto_L_activated(self, index):
        """Cache le tooltip quand une fonction auto_L est sélectionnée (la liste se ferme)"""
        self.auto_L_tooltip.hide()

    def _validate_scenarios(self, calc_params: dict, strict: bool = True, use_fields: bool = True) -> bool:
        """Valide la syntaxe des scénarios et affiche un message si invalide."""
        field_map = {
            "ROV Fx": self.sc_fx_rov,
            "ROV Fy": self.sc_fy_rov,
            "Bateau": self.sc_v_bateau,
            "Moulinet": self.sc_v_moulinet,
        }
        scenarios = {}
        if use_fields:
            for label, field in field_map.items():
                if hasattr(field, "toPlainText"):
                    raw = (field.toPlainText() or "").strip()
                else:
                    raw = (field.text() or "").strip()
                normalized = raw.replace(";", ":")
                if normalized != raw:
                    if hasattr(field, "setPlainText"):
                        field.setPlainText(normalized)
                    else:
                        field.setText(normalized)
                scenarios[label] = normalized
        else:
            scenarios = {
                "ROV Fx": (calc_params.get('sc_fx_rov') or "").strip(),
                "ROV Fy": (calc_params.get('sc_fy_rov') or "").strip(),
                "Bateau": (calc_params.get('sc_v_bateau') or "").strip(),
                "Moulinet": (calc_params.get('sc_v_moulinet') or "").strip(),
            }
            for label, scen in scenarios.items():
                scenarios[label] = scen.replace(";", ":")
        calc_params['sc_fx_rov'] = scenarios.get("ROV Fx", "")
        calc_params['sc_fy_rov'] = scenarios.get("ROV Fy", "")
        calc_params['sc_v_bateau'] = scenarios.get("Bateau", "")
        calc_params['sc_v_moulinet'] = scenarios.get("Moulinet", "")
        invalid = [label for label, scen in scenarios.items()
                   if scen and not verifier_syntaxe_scenario(scen)]
        if use_fields:
            for label, field in field_map.items():
                if label in invalid:
                    field.setStyleSheet("border: 1px solid #d9534f;")
                else:
                    field.setStyleSheet("")
        if invalid:
            details = []
            for label in invalid:
                scen = scenarios.get(label, "")
                first_err = find_first_invalid_couple(scen) or "(couple invalide)"
                details.append(f"- {label}: {first_err}")
            detail_text = "\n".join(details)
            if strict:
                QMessageBox.warning(
                    self,
                    "Scénario invalide",
                    "Syntaxe invalide pour:\n"
                    f"{detail_text}\n"
                    "Corrigez les scénarios avant de charger/sauvegarder."
                )
                return False
            QMessageBox.warning(
                self,
                "Scénario invalide",
                "Syntaxe invalide pour:\n"
                f"{detail_text}\n"
                "La mission est chargée pour correction."
            )
        return True
    
    def get_current_mission_directory(self):
        """Retourne le répertoire de la mission actuellement sélectionnée"""
        current_mission = self.mission_combo.currentText()
        if current_mission:
            missions_dir = get_missions_directory()
            mission_dir = missions_dir / current_mission
            mission_dir.mkdir(exist_ok=True)
            return mission_dir
        return None
    
    def on_mission_changed(self, mission_name):
        """Appelé quand la mission sélectionnée change"""
        if mission_name and str(mission_name).strip():
            # Debug: Afficher le nom de la mission sélectionnée
            trace_print(1, f"[DEBUG] Mission sélectionnée: '{mission_name}'")
            self.main_window.mission_data['current'] = mission_name
            self.main_window.mission_data['loaded'] = False
            self.update_mission_status(mission_name)
            
            mission_dir = self.get_current_mission_directory()
            if mission_dir:
                param_file = mission_dir / "Param_mission.json"
                if param_file.exists():
                    if self.load_param_mission_file(param_file):
                        # Réinitialiser le système après chargement des paramètres
                        if hasattr(self.main_window, 'simulation_tab'):
                            self.main_window.simulation_tab.initialize_system()
        else:
            self.main_window.mission_data['current'] = None
            self.update_mission_status(None)
    
    def update_mission_status(self, mission_name):
        """Met à jour l'affichage du statut de la mission"""
        if not hasattr(self, 'mission_status_label'):
            return
            
        if mission_name:
            mission_dir = self.get_current_mission_directory()
            param_file = mission_dir / "Param_mission.json" if mission_dir else None
            if param_file and param_file.exists():
                if self.main_window.mission_data.get('loaded'):
                    self.mission_status_label.setText(
                        f"Mission: {mission_name} | Paramètres chargés"
                    )
                    self.mission_status_label.setStyleSheet(
                        "padding: 5px; background-color: #d4edda; border: 1px solid #c3e6cb;"
                    )
                else:
                    self.mission_status_label.setText(
                        f"Mission: {mission_name} | Param_mission.json existe"
                    )
                    self.mission_status_label.setStyleSheet(
                        "padding: 5px; background-color: #d4edda; border: 1px solid #c3e6cb;"
                    )
            else:
                self.mission_status_label.setText(
                    f"Mission: {mission_name} | Param_mission.json n'existe pas encore"
                )
                self.mission_status_label.setStyleSheet(
                    "padding: 5px; background-color: #fff3cd; border: 1px solid #ffeaa7;"
                )
        else:
            self.mission_status_label.setText("Aucune mission sélectionnée")
            self.mission_status_label.setStyleSheet(
                "padding: 5px; background-color: #f0f0f0; border: 1px solid #ccc;"
            )
    
    def create_new_mission(self):
        """Crée une nouvelle mission"""
        mission_name, ok = QInputDialog.getText(
            self,
            "Nouvelle mission",
            "Nom de la mission:",
        )
        
        if ok and mission_name:
            mission_name = mission_name.strip()
            if not mission_name:
                QMessageBox.warning(self, "Erreur", "Le nom de la mission ne peut pas être vide.")
                return
            
            missions = list_missions()
            if mission_name in missions:
                QMessageBox.warning(self, "Erreur", f"La mission '{mission_name}' existe déjà.")
                return
            
            try:
                create_mission(mission_name)
                self.update_mission_list()
                self.mission_combo.setCurrentText(mission_name)
                QMessageBox.information(self, "Succès", f"Mission '{mission_name}' créée avec succès.")
            except Exception as e:
                QMessageBox.critical(self, "Erreur", f"Erreur lors de la création de la mission:\n{str(e)}")
    
    def load_param_mission_file(self, filepath):
        """Charge un fichier Param_mission.json"""
        try:
            # Debug: Extraire le nom de la mission depuis le chemin du fichier
            from pathlib import Path
            filepath_obj = Path(filepath) if not isinstance(filepath, Path) else filepath
            mission_name = filepath_obj.parent.name
            trace_print(1, f"[DEBUG] Chargement de la mission: '{mission_name}'")
            
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)

            parameters = data.get("parameters") or data.get("parametres") or data.get("paramètres")
            calc_params = data.get("calc_params") or data.get("parametres_calcul") or data.get("paramètres_calcul")
            init_params = data.get("init_params") or data.get("conditions_initiales")
            
            # Conversion automatique pour compatibilité : y_rov_init positif -> négatif
            # Convention : y < 0 = profondeur (sous la surface)
            if init_params and "y_rov_init" in init_params:
                y_val = init_params["y_rov_init"]
                if isinstance(y_val, (int, float)) and y_val > 0:
                    init_params["y_rov_init"] = -y_val
                    trace_print(8, f"⚠️  Conversion automatique : y_rov_init={y_val} -> {init_params['y_rov_init']} (profondeur négative)")

            if not isinstance(parameters, dict) or not isinstance(calc_params, dict) or not isinstance(init_params, dict):
                raise ValueError("Le fichier ne contient pas les sections attendues (parameters, calc_params, init_params).")

            # Migration: tcible -> Tcible
            if "Tcible" not in calc_params and "tcible" in calc_params:
                calc_params["Tcible"] = calc_params.get("tcible")
                calc_params.pop("tcible", None)
            # Migration: Gamma_moulinet_max -> boat.gamma_moulinet_min/max
            if "Gamma_moulinet_max" in calc_params:
                try:
                    boat_params = parameters.setdefault("boat", {})
                    gamma_max_val = calc_params.get("Gamma_moulinet_max")
                    if boat_params.get("gamma_moulinet_max") is None and gamma_max_val is not None:
                        boat_params["gamma_moulinet_max"] = float(gamma_max_val)
                    if boat_params.get("gamma_moulinet_min") is None and gamma_max_val is not None:
                        boat_params["gamma_moulinet_min"] = -abs(float(gamma_max_val))
                except Exception:
                    pass
                calc_params.pop("Gamma_moulinet_max", None)

            self._validate_scenarios(calc_params, strict=False, use_fields=False)

            description = str(data.get("description", ""))
            self.main_window.parameters = parameters
            self.main_window.calc_params = calc_params
            self.main_window.init_params = init_params
            self.main_window.mission_data['description'] = description
            self.main_window.mission_data['loaded'] = True
            self.mission_description.setPlainText(description)

            self.load_all_parameters_display()
            self.update_mission_status(self.mission_combo.currentText())
            
            # Réinitialiser automatiquement le système après chargement des paramètres
            if hasattr(self.main_window, 'simulation_tab'):
                self.main_window.simulation_tab.initialize_system()
            
            return True
        except Exception as e:
            QMessageBox.critical(self, "Erreur", f"Erreur lors du chargement du fichier de mission :\n{str(e)}")
            return False
    
    def load_from_mission(self):
        """Charge Param_mission.json depuis le répertoire de la mission sélectionnée"""
        mission_name = self.mission_combo.currentText()
        
        # Debug: Afficher le nom de la mission chargée
        trace_print(1, f"[DEBUG] Chargement de la mission: '{mission_name}'")
        
        mission_dir = self.get_current_mission_directory()
        if not mission_dir:
            QMessageBox.warning(self, "Aucune mission", "Veuillez sélectionner une mission d'abord.")
            return
        
        param_file = mission_dir / "Param_mission.json"
        if not param_file.exists():
            QMessageBox.warning(
                self, 
                "Fichier introuvable", 
                f"Le fichier Param_mission.json n'existe pas pour la mission '{self.mission_combo.currentText()}'."
            )
            return
        
        if self.load_param_mission_file(param_file):
            # Réinitialiser le système après chargement des paramètres
            if hasattr(self.main_window, 'simulation_tab'):
                self.main_window.simulation_tab.initialize_system()
            if hasattr(self, 'mission_status_label'):
                self.mission_status_label.setText(
                    f"Mission {self.mission_combo.currentText()} | Paramètres chargés"
                )
                self.mission_status_label.setStyleSheet(
                    "padding: 5px; background-color: #d4edda; border: 1px solid #c3e6cb;"
                )
    
    def save_to_mission(self):
        """Sauvegarde Param_mission.json dans le répertoire de la mission sélectionnée"""
        mission_dir = self.get_current_mission_directory()
        if not mission_dir:
            QMessageBox.warning(self, "Aucune mission", "Veuillez sélectionner une mission d'abord.")
            return
        
        if not self.save_all_parameters_from_display():
            return

        param_file = mission_dir / "Param_mission.json"
        
        try:
            description = self.mission_description.toPlainText()
            self.main_window.mission_data['description'] = description
            data = {
                "date_sauvegarde": datetime.now().isoformat(),
                "mission": self.mission_combo.currentText(),
                "description": description,
                "parameters": self.main_window.parameters,
                "calc_params": self.main_window.calc_params,
                "init_params": self.main_window.init_params,
            }

            with open(param_file, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=4)

            self.update_mission_status(self.mission_combo.currentText())
            QMessageBox.information(
                self, 
                "Succès", 
                f"Paramètres sauvegardés dans la mission '{self.mission_combo.currentText()}'."
            )
        except Exception as e:
            QMessageBox.critical(self, "Erreur", f"Erreur lors de la sauvegarde :\n{str(e)}")
    
    def load_from_file(self):
        """Charge depuis un fichier JSON"""
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Charger les paramètres de mission",
            "",
            "Fichiers JSON de mission (*.json)"
        )
        
        if filename:
            if self.load_param_mission_file(filename):
                QMessageBox.information(self, "Succès", "Paramètres de mission chargés avec succès.")

    def save_to_file(self):
        """Sauvegarde les paramètres vers un fichier JSON choisi par l'utilisateur"""
        if not self.save_all_parameters_from_display():
            return

        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Sauvegarder les paramètres",
            "",
            "Fichiers JSON de mission (*.json)"
        )

        if not filename:
            return

        if not filename.lower().endswith(".json"):
            filename = f"{filename}.json"

        try:
            description = self.mission_description.toPlainText()
            self.main_window.mission_data['description'] = description
            data = {
                "date_sauvegarde": datetime.now().isoformat(),
                "mission": self.mission_combo.currentText(),
                "description": description,
                "parameters": self.main_window.parameters,
                "calc_params": self.main_window.calc_params,
                "init_params": self.main_window.init_params,
            }

            with open(filename, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=4)

            QMessageBox.information(self, "Succès", "Paramètres sauvegardés dans le fichier choisi.")
        except Exception as e:
            QMessageBox.critical(self, "Erreur", f"Erreur lors de la sauvegarde :\n{str(e)}")
    
    def reset_to_defaults(self):
        """Réinitialise les paramètres par défaut"""
        self.main_window.load_default_parameters()
        
        # Réinitialiser aussi les paramètres de calcul et conditions initiales
        self.main_window.calc_params = {
            'method': 'RK45',
            'rtol': 1e-5,
            'atol': 1e-7,
            'max_step': 0.1,
            'steps_per_update': 5,
            't_final': 60.0,
            'dt_max': 0.1,
            'N_segments': 50,
            'straight_blend_alpha': 1.0,
            'auto_L': 'auto_L_1',
            'Tcible': None,
        }
        self.main_window.init_params = {
            'x_rov_init': 0.0,
            'y_rov_init': -10.0,  # Profondeur négative (convention : y < 0 = sous la surface)
            'x_boat_init': 0.0,
            'L_init': 50.0,
            'v_courant': "0.0"
        }
        
        self.load_all_parameters_display()
        
        # Réinitialiser la mission sélectionnée
        self.main_window.mission_data['current'] = None
        
        # Réinitialiser le ComboBox de mission (bloquer les signaux pour éviter de déclencher on_mission_changed)
        self.mission_combo.blockSignals(True)
        self.mission_combo.setCurrentIndex(-1)  # Aucune sélection
        self.mission_combo.blockSignals(False)
        
        # Mettre à jour le statut de la mission
        self.update_mission_status(None)
        
        # Réinitialiser automatiquement le système après réinitialisation des paramètres
        if hasattr(self.main_window, 'simulation_tab'):
            self.main_window.simulation_tab.initialize_system()
        
        # Mettre à jour l'affichage de la mission dans l'onglet Simulation
        if hasattr(self.main_window, 'simulation_tab'):
            self.main_window.simulation_tab.update_mission_display()
        
        QMessageBox.information(self, "Succès", "Paramètres réinitialisés.")
    
    def on_method_changed(self, text):
        """Appelé quand la méthode change"""
        self.save_all_parameters_from_display()
