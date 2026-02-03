"""
Application PyQt principale pour le simulateur ROV
"""
import sys
import os
from PyQt6.QtWidgets import QApplication, QMainWindow, QTabWidget, QVBoxLayout, QWidget
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QIcon

# Ajouter le répertoire parent au path pour les imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from .pyqt_widgets.simulation_tab import SimulationTab
from .pyqt_widgets.all_parameters_tab import AllParametersTab


class ROVSimulatorApp(QMainWindow):
    """Fenêtre principale de l'application simulateur ROV"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("🌊 Simulateur ROV - Application Windows")
        self.setMinimumSize(1400, 900)
        
        # État global de l'application
        self.simulation_state = {
            'running': False,
            'paused': False,
            'current_time': 0.0,
            'system': None,
            'y_current': None,
            't_current': 0.0,
            'data': {
                'time': [],
                'x_rov': [],
                'y_rov': [],
                'vx_rov': [],
                'vy_rov': [],
                'x_boat': [],
                'L': [],
                'T0': [],
                'T_boat': [],
                'T_max': [],
                'x_cable_curr': None,
                'y_cable_curr': None,
            }
        }
        
        self.parameters = None
        self.calc_params = {
            'method': 'RK45',
            'rtol': 1e-5,
            'atol': 1e-7,
            'max_step': 0.1,
            'steps_per_update': 5,
            't_final': 60.0,
            'dt_max': 0.1,
            'N_segments': 50
        }
        
        self.init_params = {
            'x_rov_init': 0.0,
            'y_rov_init': -10.0,  # Profondeur négative (convention : y < 0 = sous la surface)
            'x_boat_init': 0.0,
            'L_init': 50.0,
            'v_courant': "0.0"
        }
        
        self.mission_data = {'current': None}
        
        # Ne pas sélectionner automatiquement une mission au démarrage
        # L'utilisateur doit sélectionner explicitement une mission dans l'onglet Paramètres
        
        # Charger les paramètres par défaut
        self.load_default_parameters()
        
        # Créer l'interface
        self.init_ui()
    
    def load_default_parameters(self):
        """Charge les paramètres par défaut"""
        from src.utils.parameters import get_default_parameters
        self.parameters = get_default_parameters()
    
    def init_ui(self):
        """Initialise l'interface utilisateur"""
        # Widget central avec onglets
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout(central_widget)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # Créer les onglets
        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.TabPosition.North)
        
        # Onglet Simulation
        self.simulation_tab = SimulationTab(self)
        self.tabs.addTab(self.simulation_tab, "🎮 Simulation")
        
        # Onglet Paramètres (unifié avec 3 colonnes)
        self.all_parameters_tab = AllParametersTab(self)
        self.tabs.addTab(self.all_parameters_tab, "⚙️ Paramètres")
        
        layout.addWidget(self.tabs)
    
    def get_simulation_state(self):
        """Retourne l'état de la simulation"""
        return self.simulation_state
    
    def update_simulation_state(self, key, value=None):
        """Met à jour l'état de la simulation"""
        if isinstance(key, dict):
            # Si key est un dictionnaire, mettre à jour tout l'état
            self.simulation_state.update(key)
        elif value is not None:
            # Si key est une clé et value est fourni, mettre à jour une seule clé
            self.simulation_state[key] = value
        else:
            # Si seulement key est fourni et que ce n'est pas un dict, c'est une erreur
            raise ValueError("update_simulation_state() requires either a dict or (key, value) pair")


def main():
    """Point d'entrée principal de l'application"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Style moderne
    
    # Personnaliser le style des tooltips (fond couleur sable, texte gris moyen)
    # Note: Sur Windows, les tooltips peuvent utiliser le style système
    # Essayons d'appliquer le style de manière plus agressive
    tooltip_style = """
        QToolTip {
            background-color: rgb(244, 228, 188);
            color: rgb(85, 85, 85);
            border: 1px solid rgb(212, 196, 164);
            padding: 5px;
            border-radius: 3px;
            font-size: 10pt;
        }
    """
    # Appliquer le style à l'application
    app.setStyleSheet(tooltip_style)
    
    # Créer et afficher la fenêtre principale
    window = ROVSimulatorApp()
    window.show()
    
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
