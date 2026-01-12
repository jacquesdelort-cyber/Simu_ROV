"""Composants Dash pour la gestion des missions"""
from dash import html, dcc
import dash_bootstrap_components as dbc
from ..mission_utils import list_missions


def create_mission_controls():
    """Crée les composants de contrôle pour la gestion des missions"""
    
    try:
        missions = list_missions()
    except Exception as e:
        print(f"Erreur lors de la récupération des missions: {e}")
        import traceback
        traceback.print_exc()
        missions = []  # Liste vide en cas d'erreur
    
    return html.Div([
        # Affichage de la mission actuelle
        html.Div(id="mission-display"),
        
        # Gestion des missions
        dbc.Collapse([
            dbc.Card([
                dbc.CardBody([
                    # Liste des missions existantes
                    html.Div([
                        dbc.Label("Sélectionner une mission existante"),
                        dcc.Dropdown(
                            id="select-mission-dropdown",
                            options=[{"label": "", "value": ""}] + [
                                {"label": m, "value": m} for m in missions
                            ],
                            value="",
                            placeholder="Choisir une mission...",
                        ),
                    ], className="mb-3"),
                    
                    # Créer une nouvelle mission
                    html.Div([
                        dbc.Label("Nom de la nouvelle mission"),
                        dbc.Input(
                            id="new-mission-name",
                            type="text",
                            placeholder="Ex: Mission_2024_01_15",
                        ),
                    ], className="mb-3"),
                    
                    dbc.Button(
                        "➕ Créer une nouvelle mission",
                        id="create-mission-button",
                        color="primary",
                        className="w-100 mb-3",
                    ),
                    
                    # Messages de retour
                    html.Div(id="mission-messages"),
                ])
            ])
        ], id="mission-collapse", is_open=False),
        
        dbc.Button(
            "📁 Gérer les missions",
            id="toggle-mission-collapse",
            color="secondary",
            outline=True,
            className="w-100 mt-2",
        ),
    ])
