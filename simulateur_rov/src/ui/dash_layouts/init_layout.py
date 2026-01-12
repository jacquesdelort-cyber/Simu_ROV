"""Layout pour l'onglet Conditions initiales"""
from dash import html
import dash_bootstrap_components as dbc


def create_init_layout():
    """Crée le layout pour l'onglet Conditions initiales"""
    
    return dbc.Container([
        dbc.Row([
            dbc.Col([
                html.H5("Conditions initiales"),
                dbc.Label("Position horizontale ROV (m)"),
                dbc.Input(id="x_rov_init", type="number", value=0.0, min=0.0, max=100.0, step=1.0, className="mb-3"),
                
                dbc.Label("Profondeur ROV (m)"),
                dbc.Input(id="y_rov_init", type="number", value=10.0, min=0.0, max=200.0, step=1.0, className="mb-3"),
                
                html.Hr(),
                html.H5("Câble"),
                dbc.Label("Longueur câble (m)"),
                dbc.Input(id="L_init", type="number", value=50.0, min=10.0, max=500.0, step=1.0, className="mb-3"),
                
                html.Hr(),
                html.H5("Environnement"),
                dbc.Label("Vitesse du courant (m/s)"),
                dbc.Input(id="v_courant", type="number", value=0.0, min=-10.0, max=10.0, step=0.1),
            ], width=6),
            
            dbc.Col([
                # Colonne vide pour l'instant
                html.Div([])
            ], width=6),
        ]),
        dbc.Row([
            dbc.Col([
                dbc.Button(
                    "💾 Sauvegarder les conditions initiales",
                    id="save-init-params-button",
                    color="primary",
                    className="mt-3 w-100"
                ),
                html.Div(id="save-init-params-message", className="mt-2"),
            ], width=12)
        ])
    ], fluid=True, className="mt-3")
