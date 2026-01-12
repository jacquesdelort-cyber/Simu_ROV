"""Layout pour l'onglet de simulation"""
from dash import html, dcc
import dash_bootstrap_components as dbc
from ..dash_components.mission_controls import create_mission_controls
from ..dash_components.simulation_controls import create_simulation_controls
from ..dash_components.visualizations import create_system_view, create_real_time_metrics


def create_simulation_layout():
    """Crée le layout pour l'onglet de simulation"""
    
    return dbc.Container([
        dbc.Row([
            # Colonne gauche : Mission et commandes
            dbc.Col([
                create_mission_controls(),
                html.Hr(),
                create_simulation_controls(),
            ], width=3, className="border-end"),
            
            # Colonne centrale : Visualisation
            dbc.Col([
                create_system_view(),
            ], width=6),
            
            # Colonne droite : Métriques temps réel
            dbc.Col([
                create_real_time_metrics(),
            ], width=3),
        ])
    ], fluid=True, className="mt-3")
