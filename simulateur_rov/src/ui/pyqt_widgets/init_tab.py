"""
Onglet pour les conditions initiales
"""
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QLabel, QLineEdit,
                             QGroupBox, QGridLayout)
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))


class InitTab(QWidget):
    """Onglet pour configurer les conditions initiales"""
    
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.init_ui()
        self.load_params_display()
    
    def init_ui(self):
        """Initialise l'interface"""
        layout = QVBoxLayout(self)
        
        # Groupe : Position ROV
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
        
        # Groupe : Position Bateau
        boat_group = QGroupBox("Position initiale du Bateau")
        boat_layout = QGridLayout()
        
        boat_layout.addWidget(QLabel("Position horizontale x_boat_init (m):"), 0, 0)
        self.x_boat_init = QLineEdit()
        boat_layout.addWidget(self.x_boat_init, 0, 1)
        
        boat_group.setLayout(boat_layout)
        layout.addWidget(boat_group)
        
        # Groupe : Câble
        cable_group = QGroupBox("Paramètres du câble")
        cable_layout = QGridLayout()
        
        cable_layout.addWidget(QLabel("Longueur initiale L_init (m):"), 0, 0)
        self.L_init = QLineEdit()
        cable_layout.addWidget(self.L_init, 0, 1)
        
        cable_group.setLayout(cable_layout)
        layout.addWidget(cable_group)
        
        # Groupe : Environnement
        env_group = QGroupBox("Environnement")
        env_layout = QGridLayout()
        
        env_layout.addWidget(QLabel("Vitesse du courant v_courant (m/s):"), 0, 0)
        self.v_courant = QLineEdit()
        env_layout.addWidget(self.v_courant, 0, 1)
        
        env_group.setLayout(env_layout)
        layout.addWidget(env_group)
        
        layout.addStretch()
    
    def load_params_display(self):
        """Charge les paramètres dans l'affichage"""
        params = self.main_window.init_params
        
        self.x_rov_init.setText(str(params.get('x_rov_init', 0.0)))
        self.y_rov_init.setText(str(params.get('y_rov_init', -10.0)))  # Profondeur négative
        self.x_boat_init.setText(str(params.get('x_boat_init', 0.0)))
        self.L_init.setText(str(params.get('L_init', 50.0)))
        self.v_courant.setText(str(params.get('v_courant', 0.0)))
    
    def save_params_from_display(self):
        """Sauvegarde les paramètres depuis l'affichage"""
        try:
            v_courant_text = (self.v_courant.text() or "").strip()
            v_courant_value = v_courant_text if v_courant_text != "" else "0.0"
            self.main_window.init_params = {
                'x_rov_init': float(self.x_rov_init.text() or 0.0),
                'y_rov_init': float(self.y_rov_init.text() or -10.0),  # Profondeur négative
                'x_boat_init': float(self.x_boat_init.text() or 0.0),
                'L_init': float(self.L_init.text() or 50.0),
                'v_courant': v_courant_value
            }
            return True
        except ValueError:
            return False
    
    def showEvent(self, event):
        """Sauvegarde les paramètres quand l'onglet est affiché"""
        self.save_params_from_display()
        super().showEvent(event)
