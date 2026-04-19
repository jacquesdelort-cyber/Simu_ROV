"""
Onglet pour les paramètres de calcul
"""
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QLabel, QLineEdit,
                             QGroupBox, QGridLayout, QComboBox)
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))


class CalcTab(QWidget):
    """Onglet pour configurer les paramètres de calcul"""
    
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.init_ui()
        self.load_params_display()
    
    def init_ui(self):
        """Initialise l'interface"""
        layout = QVBoxLayout(self)
        
        # Groupe : Méthode d'intégration
        method_group = QGroupBox("Méthode d'intégration")
        method_layout = QGridLayout()
        
        method_layout.addWidget(QLabel("Méthode:"), 0, 0)
        self.method_combo = QComboBox()
        self.method_combo.addItems(['RK45', 'RK23', 'DOP853', 'Radau'])
        self.method_combo.currentTextChanged.connect(self.on_method_changed)
        method_layout.addWidget(self.method_combo, 0, 1)
        
        method_group.setLayout(method_layout)
        layout.addWidget(method_group)
        
        # Groupe : Tolérances
        tolerance_group = QGroupBox("Tolérances")
        tolerance_layout = QGridLayout()
        
        tolerance_layout.addWidget(QLabel("Tolérance relative (rtol):"), 0, 0)
        self.rtol = QLineEdit()
        tolerance_layout.addWidget(self.rtol, 0, 1)
        
        tolerance_layout.addWidget(QLabel("Tolérance absolue (atol):"), 1, 0)
        self.atol = QLineEdit()
        tolerance_layout.addWidget(self.atol, 1, 1)
        
        tolerance_group.setLayout(tolerance_layout)
        layout.addWidget(tolerance_group)
        
        # Groupe : Pas de temps
        timestep_group = QGroupBox("Pas de temps")
        timestep_layout = QGridLayout()
        
        timestep_layout.addWidget(QLabel("Pas maximum (max_step):"), 0, 0)
        self.max_step = QLineEdit()
        timestep_layout.addWidget(self.max_step, 0, 1)
        
        timestep_layout.addWidget(QLabel("Pas maximum dt_max (s):"), 1, 0)
        self.dt_max = QLineEdit()
        timestep_layout.addWidget(self.dt_max, 1, 1)
        
        timestep_group.setLayout(timestep_layout)
        layout.addWidget(timestep_group)
        
        # Groupe : Simulation
        sim_group = QGroupBox("Paramètres de simulation")
        sim_layout = QGridLayout()
        
        sim_layout.addWidget(QLabel("Temps final (s):"), 0, 0)
        self.t_final = QLineEdit()
        sim_layout.addWidget(self.t_final, 0, 1)
        
        sim_layout.addWidget(QLabel("Pas par mise à jour:"), 1, 0)
        self.steps_per_update = QLineEdit()
        sim_layout.addWidget(self.steps_per_update, 1, 1)
        
        sim_group.setLayout(sim_layout)
        layout.addWidget(sim_group)
        
        # Groupe : Discrétisation
        disc_group = QGroupBox("Discrétisation du câble")
        disc_layout = QGridLayout()
        
        disc_layout.addWidget(QLabel("Nombre de segments (N_segments):"), 0, 0)
        self.n_segments = QLineEdit()
        disc_layout.addWidget(self.n_segments, 0, 1)
        
        disc_group.setLayout(disc_layout)
        layout.addWidget(disc_group)
        
        layout.addStretch()
    
    def load_params_display(self):
        """Charge les paramètres dans l'affichage"""
        params = self.main_window.calc_params
        
        self.method_combo.setCurrentText(params.get('method', 'RK45'))
        self.rtol.setText(str(params.get('rtol', 1e-5)))
        self.atol.setText(str(params.get('atol', 1e-7)))
        self.max_step.setText(str(params.get('max_step', 0.1)))
        self.dt_max.setText(str(params.get('dt_max', 0.1)))
        self.t_final.setText(str(params.get('t_final', 60.0)))
        self.steps_per_update.setText(str(params.get('steps_per_update', 5)))
        self.n_segments.setText(str(params.get('N_segments', 50)))
    
    def save_params_from_display(self):
        """Sauvegarde les paramètres depuis l'affichage"""
        try:
            prev = dict(self.main_window.calc_params) if isinstance(self.main_window.calc_params, dict) else {}
            self.main_window.calc_params = {
                'method': self.method_combo.currentText(),
                'rtol': float(self.rtol.text() or 1e-5),
                'atol': float(self.atol.text() or 1e-7),
                'max_step': float(self.max_step.text() or 0.1),
                'dt_max': float(self.dt_max.text() or 0.1),
                't_final': float(self.t_final.text() or 60.0),
                'steps_per_update': int(self.steps_per_update.text() or 5),
                'N_segments': int(self.n_segments.text() or 50),
                # Préserver les paramètres avancés non édités dans cet onglet.
                'guard_profile': prev.get('guard_profile', 'soft'),
                'guard_enable': bool(prev.get('guard_enable', True)),
                'guard_tension_spike_factor': float(prev.get('guard_tension_spike_factor', 12.0)),
                'guard_tension_abs': float(prev.get('guard_tension_abs', 1200.0)),
                'guard_geom_ds_ratio': float(prev.get('guard_geom_ds_ratio', 3.2)),
                'guard_geom_rel_L': float(prev.get('guard_geom_rel_L', 0.07)),
                'guard_hard_block_duration_s': float(prev.get('guard_hard_block_duration_s', 1.5)),
                'trace_boost_level': prev.get('trace_boost_level', 7),
                'trace_boost_duration_s': float(prev.get('trace_boost_duration_s', 2.0)),
            }
            return True
        except ValueError as e:
            return False
    
    def on_method_changed(self, text):
        """Appelé quand la méthode change"""
        self.save_params_from_display()
    
    def showEvent(self, event):
        """Sauvegarde les paramètres quand l'onglet est affiché"""
        self.save_params_from_display()
        super().showEvent(event)
