"""
Onglet pour les paramètres d'environnement
"""
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                             QLineEdit, QPushButton, QGroupBox, QGridLayout,
                             QFileDialog, QMessageBox, QScrollArea, QComboBox,
                             QInputDialog)
from PyQt6.QtCore import Qt
from pathlib import Path
import sys
import os
import json
from datetime import datetime
from src.utils.logger import trace_print

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from src.ui.mission_utils import get_missions_directory, list_missions, create_mission


class ParametersTab(QWidget):
    """Onglet pour configurer les paramètres d'environnement"""
    
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.init_ui()
        self.load_parameters_display()
        
        # Synchroniser la mission sélectionnée avec main_window
        # (Cette synchronisation est déjà faite dans update_mission_list, donc pas besoin de la refaire ici)
    
    def init_ui(self):
        """Initialise l'interface"""
        layout = QVBoxLayout(self)
        
        # Groupe : Sélection de mission
        mission_group = QGroupBox("🎯 Mission")
        mission_layout = QVBoxLayout()
        
        mission_select_layout = QHBoxLayout()
        mission_select_layout.addWidget(QLabel("Mission:"))
        
        # Créer d'abord le label de statut pour qu'il existe avant update_mission_list()
        self.mission_status_label = QLabel("Aucune mission sélectionnée")
        self.mission_status_label.setStyleSheet("padding: 5px; background-color: #f0f0f0; border: 1px solid #ccc;")
        
        self.mission_combo = QComboBox()
        self.mission_combo.setEditable(False)
        # Ne pas connecter le signal avant d'avoir rempli la liste pour éviter les appels prématurés
        mission_select_layout.addWidget(self.mission_combo, 1)
        
        btn_new_mission = QPushButton("➕ Nouvelle mission")
        btn_new_mission.clicked.connect(self.create_new_mission)
        mission_select_layout.addWidget(btn_new_mission)
        
        mission_layout.addLayout(mission_select_layout)
        
        # Afficher la mission actuelle
        mission_layout.addWidget(self.mission_status_label)
        
        mission_group.setLayout(mission_layout)
        layout.addWidget(mission_group)
        
        # Maintenant qu'on a créé mission_status_label, on peut remplir la liste et connecter le signal
        self.update_mission_list()
        self.mission_combo.currentTextChanged.connect(self.on_mission_changed)
        
        # Boutons de chargement/sauvegarde
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
        
        btn_reset = QPushButton("🔄 Réinitialiser")
        btn_reset.clicked.connect(self.reset_to_defaults)
        button_layout.addWidget(btn_reset)
        
        layout.addLayout(button_layout)
        
        # Zone scrollable pour les paramètres
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        
        # Paramètres ROV
        rov_group = self.create_rov_parameters_group()
        scroll_layout.addWidget(rov_group)
        
        # Paramètres Câble
        cable_group = self.create_cable_parameters_group()
        scroll_layout.addWidget(cable_group)
        
        # Paramètres Bateau
        boat_group = self.create_boat_parameters_group()
        scroll_layout.addWidget(boat_group)
        
        # Paramètres Environnement
        env_group = self.create_environment_parameters_group()
        scroll_layout.addWidget(env_group)
        
        scroll_layout.addStretch()
        
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)
    
    def create_rov_parameters_group(self):
        """Crée le groupe de paramètres ROV"""
        group = QGroupBox("Paramètres ROV")
        layout = QGridLayout()
        
        # Masse
        layout.addWidget(QLabel("Masse (kg):"), 0, 0)
        self.rov_m = QLineEdit()
        layout.addWidget(self.rov_m, 0, 1)
        
        # Dimensions
        layout.addWidget(QLabel("Largeur a (m):"), 1, 0)
        self.rov_a = QLineEdit()
        layout.addWidget(self.rov_a, 1, 1)
        
        layout.addWidget(QLabel("Longueur b (m):"), 2, 0)
        self.rov_b = QLineEdit()
        layout.addWidget(self.rov_b, 2, 1)
        
        layout.addWidget(QLabel("Hauteur h (m):"), 3, 0)
        self.rov_h = QLineEdit()
        layout.addWidget(self.rov_h, 3, 1)
        
        # Coefficients de traînée
        layout.addWidget(QLabel("Coeff. traînée Cx:"), 4, 0)
        self.rov_cx = QLineEdit()
        layout.addWidget(self.rov_cx, 4, 1)
        
        layout.addWidget(QLabel("Coeff. traînée Cy:"), 5, 0)
        self.rov_cy = QLineEdit()
        layout.addWidget(self.rov_cy, 5, 1)
        
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
    
    def load_parameters_display(self):
        """Charge les paramètres dans l'affichage"""
        params = self.main_window.parameters
        if not params:
            return
        
        # ROV
        rov = params.get('rov', {})
        self.rov_m.setText(str(rov.get('m', '')))
        self.rov_a.setText(str(rov.get('a', '')))
        self.rov_b.setText(str(rov.get('b', '')))
        self.rov_h.setText(str(rov.get('h', '')))
        self.rov_cx.setText(str(rov.get('Cx', '')))
        self.rov_cy.setText(str(rov.get('Cy', '')))
        
        # Câble
        cable = params.get('cable', {})
        self.cable_d.setText(str(cable.get('d', '')))
        self.cable_rho.setText(str(cable.get('rho_cable', '')))
        self.cable_cx.setText(str(cable.get('Cx_cable', '')))
        
        # Bateau
        boat = params.get('boat', {})
        self.boat_m.setText(str(boat.get('m', '')))
        self.boat_drag.setText(str(boat.get('drag_coefficient', '')))
        
        # Environnement
        env = params.get('environment', {})
        self.env_rho.setText(str(env.get('rho_eau', '')))
        self.env_g.setText(str(env.get('g', '')))
        self.env_mu.setText(str(env.get('mu', '')))
    
    def save_parameters_from_display(self):
        """Sauvegarde les paramètres depuis l'affichage"""
        try:
            params = {
                'rov': {
                    'm': float(self.rov_m.text() or 100.0),
                    'a': float(self.rov_a.text() or 0.5),
                    'b': float(self.rov_b.text() or 1.0),
                    'h': float(self.rov_h.text() or 0.5),
                    'Cx': float(self.rov_cx.text() or 0.8),
                    'Cy': float(self.rov_cy.text() or 1.0)
                },
                'cable': {
                    'd': float(self.cable_d.text() or 0.01),
                    'rho_cable': float(self.cable_rho.text() or 1500.0),
                    'Cx_cable': float(self.cable_cx.text() or 1.2)
                },
                'boat': {
                    'm': float(self.boat_m.text() or 10000.0),
                    'drag_coefficient': float(self.boat_drag.text() or 0.5)
                },
                'environment': {
                    'rho_eau': float(self.env_rho.text() or 1025.0),
                    'g': float(self.env_g.text() or 9.81),
                    'mu': float(self.env_mu.text() or 1e-3)
                }
            }
            
            self.main_window.parameters = params
            return True
        except ValueError as e:
            QMessageBox.warning(self, "Erreur", f"Valeur invalide: {str(e)}")
            return False
    
    def load_from_file(self):
        """
        Charge l'ensemble des paramètres (environnement + calcul + conditions
        initiales) depuis un fichier JSON unique de type « Param_mission ».
        """
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Charger les paramètres de mission",
            "",
            "Fichiers JSON de mission (*.json)"
        )
        
        if filename:
            try:
                # Lire le fichier JSON unique
                with open(filename, "r", encoding="utf-8") as f:
                    data = json.load(f)

                # Récupérer les trois blocs de paramètres
                parameters = data.get("parameters") or data.get("parametres") or data.get("paramètres")
                calc_params = data.get("calc_params") or data.get("parametres_calcul") or data.get("paramètres_calcul")
                init_params = data.get("init_params") or data.get("conditions_initiales")

                if not isinstance(parameters, dict) or not isinstance(calc_params, dict) or not isinstance(init_params, dict):
                    raise ValueError("Le fichier ne contient pas les sections attendues (parameters, calc_params, init_params).")

                # Mettre à jour l'état global de la fenêtre principale
                self.main_window.parameters = parameters
                self.main_window.calc_params = calc_params
                self.main_window.init_params = init_params

                # Rafraîchir l'affichage des trois onglets
                self.load_parameters_display()
                if hasattr(self.main_window, "calc_tab"):
                    self.main_window.calc_tab.load_params_display()
                if hasattr(self.main_window, "init_tab"):
                    self.main_window.init_tab.load_params_display()

                QMessageBox.information(self, "Succès", "Paramètres de mission chargés avec succès.")
            except Exception as e:
                QMessageBox.critical(self, "Erreur", f"Erreur lors du chargement du fichier de mission :\n{str(e)}")
    
    def save_to_file(self):
        """
        Sauvegarde l'ensemble des paramètres (environnement + calcul +
        conditions initiales) dans un fichier JSON unique de type « Param_mission ».
        """
        # Mettre à jour les structures de données à partir des trois onglets
        if not self.save_parameters_from_display():
            return

        # Paramètres de calcul
        if hasattr(self.main_window, "calc_tab"):
            self.main_window.calc_tab.save_params_from_display()

        # Conditions initiales
        if hasattr(self.main_window, "init_tab"):
            self.main_window.init_tab.save_params_from_display()

        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Sauvegarder les paramètres de mission",
            "Param_mission.json",
            "Fichiers JSON de mission (*.json)"
        )
        
        if filename:
            try:
                # Construire la structure globale
                data = {
                    "parameters": self.main_window.parameters,
                    "calc_params": self.main_window.calc_params,
                    "init_params": self.main_window.init_params,
                }

                # Sauvegarder dans un seul fichier JSON
                with open(filename, "w", encoding="utf-8") as f:
                    json.dump(data, f, ensure_ascii=False, indent=4)

                QMessageBox.information(self, "Succès", "Paramètres de mission sauvegardés avec succès.")
            except Exception as e:
                QMessageBox.critical(self, "Erreur", f"Erreur lors de la sauvegarde du fichier de mission :\n{str(e)}")
    
    def update_mission_list(self):
        """Met à jour la liste des missions disponibles"""
        self.mission_combo.clear()
        missions = list_missions()
        self.mission_combo.addItems(missions)
        
        # Sélectionner la mission actuelle si elle existe
        current_mission = self.main_window.mission_data.get('current') if self.main_window.mission_data else None
        if current_mission and current_mission in missions:
            # Bloquer temporairement le signal pour éviter les appels prématurés
            self.mission_combo.blockSignals(True)
            self.mission_combo.setCurrentText(current_mission)
            self.mission_combo.blockSignals(False)
            # Mettre à jour le statut seulement si mission_status_label existe
            if hasattr(self, 'mission_status_label'):
                self.update_mission_status(current_mission)
        else:
            if hasattr(self, 'mission_status_label'):
                self.update_mission_status(None)
    
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
        if mission_name:
            self.main_window.mission_data = {'current': mission_name}
            self.update_mission_status(mission_name)
            
            # Charger automatiquement Param_mission.json si il existe
            mission_dir = self.get_current_mission_directory()
            if mission_dir:
                param_file = mission_dir / "Param_mission.json"
                if param_file.exists():
                    reply = QMessageBox.question(
                        self, 
                        "Charger les paramètres ?",
                        f"Le fichier Param_mission.json existe pour la mission '{mission_name}'.\n"
                        "Voulez-vous charger ces paramètres ?",
                        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
                    )
                    if reply == QMessageBox.StandardButton.Yes:
                        self.load_param_mission_file(param_file)
        else:
            self.update_mission_status(None)
    
    def update_mission_status(self, mission_name):
        """Met à jour l'affichage du statut de la mission"""
        # Vérifier que mission_status_label existe (au cas où cette méthode serait appelée avant l'initialisation complète)
        if not hasattr(self, 'mission_status_label'):
            return
            
        if mission_name:
            mission_dir = self.get_current_mission_directory()
            param_file = mission_dir / "Param_mission.json" if mission_dir else None
            if param_file and param_file.exists():
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
            
            # Vérifier si la mission existe déjà
            missions = list_missions()
            if mission_name in missions:
                QMessageBox.warning(
                    self, 
                    "Erreur", 
                    f"La mission '{mission_name}' existe déjà."
                )
                return
            
            # Créer la mission
            try:
                create_mission(mission_name)
                self.update_mission_list()
                self.mission_combo.setCurrentText(mission_name)
                QMessageBox.information(
                    self, 
                    "Succès", 
                    f"Mission '{mission_name}' créée avec succès."
                )
            except Exception as e:
                QMessageBox.critical(
                    self, 
                    "Erreur", 
                    f"Erreur lors de la création de la mission:\n{str(e)}"
                )
    
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

            # Récupérer les trois blocs de paramètres
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

            # Mettre à jour l'état global de la fenêtre principale
            self.main_window.parameters = parameters
            self.main_window.calc_params = calc_params
            self.main_window.init_params = init_params

            # Rafraîchir l'affichage des trois onglets
            self.load_parameters_display()
            if hasattr(self.main_window, "calc_tab"):
                self.main_window.calc_tab.load_params_display()
            if hasattr(self.main_window, "init_tab"):
                self.main_window.init_tab.load_params_display()
            
            self.update_mission_status(self.mission_combo.currentText())
            
            return True
        except Exception as e:
            QMessageBox.critical(
                self, 
                "Erreur", 
                f"Erreur lors du chargement du fichier de mission :\n{str(e)}"
            )
            return False
    
    def load_from_mission(self):
        """Charge Param_mission.json depuis le répertoire de la mission sélectionnée"""
        mission_dir = self.get_current_mission_directory()
        if not mission_dir:
            QMessageBox.warning(
                self, 
                "Aucune mission", 
                "Veuillez sélectionner une mission d'abord."
            )
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
            QMessageBox.information(
                self, 
                "Succès", 
                f"Paramètres chargés depuis la mission '{self.mission_combo.currentText()}'."
            )
    
    def save_to_mission(self):
        """Sauvegarde Param_mission.json dans le répertoire de la mission sélectionnée"""
        mission_dir = self.get_current_mission_directory()
        if not mission_dir:
            QMessageBox.warning(
                self, 
                "Aucune mission", 
                "Veuillez sélectionner une mission d'abord."
            )
            return
        
        # Mettre à jour les structures de données à partir des trois onglets
        if not self.save_parameters_from_display():
            return

        # Paramètres de calcul
        if hasattr(self.main_window, "calc_tab"):
            self.main_window.calc_tab.save_params_from_display()

        # Conditions initiales
        if hasattr(self.main_window, "init_tab"):
            self.main_window.init_tab.save_params_from_display()

        param_file = mission_dir / "Param_mission.json"
        
        try:
            # Construire la structure globale
            data = {
                "date_sauvegarde": datetime.now().isoformat(),
                "mission": self.mission_combo.currentText(),
                "parameters": self.main_window.parameters,
                "calc_params": self.main_window.calc_params,
                "init_params": self.main_window.init_params,
            }

            # Sauvegarder dans le répertoire de la mission
            with open(param_file, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=4)

            self.update_mission_status(self.mission_combo.currentText())
            QMessageBox.information(
                self, 
                "Succès", 
                f"Paramètres sauvegardés dans la mission '{self.mission_combo.currentText()}'."
            )
        except Exception as e:
            QMessageBox.critical(
                self, 
                "Erreur", 
                f"Erreur lors de la sauvegarde :\n{str(e)}"
            )
    
    def reset_to_defaults(self):
        """Réinitialise les paramètres par défaut"""
        self.main_window.load_default_parameters()
        self.load_parameters_display()
        QMessageBox.information(self, "Succès", "Paramètres réinitialisés.")
