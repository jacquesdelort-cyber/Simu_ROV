"""Composants pour les contrôles de simulation"""
from dash import html
import dash_bootstrap_components as dbc


def create_simulation_controls():
    """Crée les contrôles de simulation (boutons Démarrer, Arrêter, Pause, etc.)"""
    
    return html.Div([
        html.H5("Commandes de simulation", className="mb-3"),
        
        # Bouton Démarrer
        dbc.Button(
            "▶ Démarrer simulation",
            id="start-simulation-button",
            color="success",
            className="w-100 mb-2",
            disabled=False
        ),
        
        # Boutons de contrôle (affichés quand la simulation est en cours)
        html.Div(id="simulation-control-buttons", children=[
            dbc.Button(
                "⏹ Arrêter",
                id="stop-simulation-button",
                color="danger",
                className="w-100 mb-2",
                disabled=True
            ),
            dbc.Button(
                "⏸ Pause",
                id="pause-simulation-button",
                color="warning",
                className="w-100 mb-2",
                disabled=True
            ),
            dbc.Button(
                "🔄 Relancer",
                id="restart-simulation-button",
                color="info",
                className="w-100 mb-2",
                disabled=True
            ),
            dbc.Button(
                "📊 Export CSV",
                id="export-csv-button",
                color="secondary",
                className="w-100 mb-2",
                disabled=True
            ),
        ]),
        
        # Messages d'état
        html.Div(id="simulation-status-message", className="mt-3"),
    ])
