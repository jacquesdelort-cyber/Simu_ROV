"""Layout pour l'onglet Paramètres calcul"""
from dash import html, dcc
import dash_bootstrap_components as dbc


def create_calc_layout():
    """Crée le layout pour l'onglet Paramètres calcul"""
    
    methods = ['RK45', 'RK23', 'DOP853', 'Radau', 'BDF', 'LSODA']
    
    return dbc.Container([
        dbc.Row([
            dbc.Col([
                html.H5("Méthode d'intégration"),
                dbc.Label("Méthode d'intégration"),
                dcc.Dropdown(
                    id="method",
                    options=[{"label": m, "value": m} for m in methods],
                    value="RK45",
                    className="mb-3"
                ),
                
                html.Hr(),
                html.H5("Tolérances"),
                dbc.Label("Tolérance relative (rtol)"),
                dbc.Input(id="rtol", type="number", value=1e-5, min=1e-10, max=1e-1, step=1e-6, className="mb-3"),
                
                dbc.Label("Tolérance absolue (atol)"),
                dbc.Input(id="atol", type="number", value=1e-7, min=1e-12, max=1e-3, step=1e-8, className="mb-3"),
                
                html.Hr(),
                html.H5("Performance"),
                dbc.Label("Pas de calcul par mise à jour d'affichage"),
                dbc.Input(id="steps_per_update", type="number", value=5, min=1, max=50, step=1),
            ], width=6),
            
            dbc.Col([
                html.H5("Paramètres de simulation"),
                dbc.Label("Temps final (s)"),
                dbc.Input(id="t_final", type="number", value=60.0, min=10.0, max=300.0, step=1.0, className="mb-3"),
                
                dbc.Label("Pas de temps max (s)"),
                dbc.Input(id="dt_max", type="number", value=0.1, min=0.01, max=1.0, step=0.01, className="mb-3"),
                
                dbc.Label("Nombre de segments du câble"),
                dbc.Input(id="N_segments", type="number", value=50, min=10, max=100, step=1, className="mb-3"),
                
                html.Hr(),
                html.H5("Paramètres d'intégration"),
                dbc.Label("Pas de temps maximum intégrateur (s)"),
                dbc.Input(id="max_step", type="number", value=0.1, min=0.001, max=10.0, step=0.01),
            ], width=6),
        ]),
        dbc.Row([
            dbc.Col([
                dbc.Button(
                    "💾 Sauvegarder les paramètres de calcul",
                    id="save-calc-params-button",
                    color="primary",
                    className="mt-3 w-100"
                ),
                html.Div(id="save-calc-params-message", className="mt-2"),
            ], width=12)
        ])
    ], fluid=True, className="mt-3")
