"""
Exemple minimal d'application Dash pour le simulateur ROV (version simplifiée)
Ce fichier sert de point de départ pour la migration Streamlit → Dash
"""
import dash
from dash import dcc, html
import dash_bootstrap_components as dbc
from dash.dependencies import Input, Output, State

# Initialisation de l'application Dash
app = dash.Dash(
    __name__,
    external_stylesheets=[dbc.themes.BOOTSTRAP],
    suppress_callback_exceptions=True
)

# Layout principal (version simplifiée sans callbacks complexes)
app.layout = html.Div([
    # Header
    dbc.NavbarSimple(
        brand="Simulateur ROV - Dash",
        brand_href="#",
        color="primary",
        dark=True,
        
    ),
    
    # Contenu principal
    dbc.Container([
        dbc.Row([
            dbc.Col([
                # Navigation par onglets
                dbc.Tabs([
                    dbc.Tab(label="Configuration", tab_id="config", children=[
                        html.Div([
                            html.H3("Configuration"),
                            html.P("Onglet de configuration (à migrer)"),
                            dbc.Alert("Cette interface sera migrée depuis Streamlit", color="info"),
                        ], style={'padding': '20px'})
                    ]),
                    dbc.Tab(label="Simulation", tab_id="simulation", children=[
                        html.Div([
                            html.H3("Simulation"),
                            html.P("Onglet de simulation (à migrer)"),
                            
                            # Exemple de contrôles (sans callbacks pour le moment)
                            dbc.Row([
                                dbc.Col([
                                    dbc.Button("Démarrer", id="start-button", color="success", className="me-2"),
                                    dbc.Button("Arrêter", id="stop-button", color="danger", className="me-2"),
                                    dbc.Button("Pause", id="pause-button", color="warning"),
                                ])
                            ], className="mt-3"),
                            
                            # Affichage statique (pour test)
                            html.Div([
                                dbc.Alert("Simulation arrêtée", color="secondary", id="status-display"),
                            ], className="mt-3"),
                            
                            # Graphique statique (pour test)
                            html.Div([
                                dcc.Graph(
                                    id="system-plot",
                                    figure={
                                        'data': [{
                                            'x': [0, 1, 2, 3, 4],
                                            'y': [0, 1, 4, 9, 16],
                                            'type': 'scatter',
                                            'mode': 'lines+markers',
                                            'name': 'Exemple'
                                        }],
                                        'layout': {
                                            'title': 'Graphique système (exemple)',
                                            'xaxis': {'title': 'X'},
                                            'yaxis': {'title': 'Y'}
                                        }
                                    }
                                )
                            ]),
                        ], style={'padding': '20px'})
                    ]),
                    dbc.Tab(label="Analyse", tab_id="analyse", children=[
                        html.Div([
                            html.H3("Analyse"),
                            html.P("Onglet d'analyse (à migrer)"),
                            dbc.Alert("Cette interface sera migrée depuis Streamlit", color="info"),
                        ], style={'padding': '20px'})
                    ]),
                ], id="main-tabs", active_tab="config"),
            ], width=12)
        ])
    ], fluid=True, className="mt-3"),
])

# Callback simple pour tester les boutons (optionnel)
@app.callback(
    Output('status-display', 'children'),
    Input('start-button', 'n_clicks'),
    Input('stop-button', 'n_clicks'),
    Input('pause-button', 'n_clicks'),
    prevent_initial_call=True
)
def update_status(start, stop, pause):
    """Met à jour le statut de la simulation"""
    from dash import callback_context
    
    ctx = callback_context
    if not ctx.triggered:
        return dbc.Alert("Simulation arrêtée", color="secondary")
    
    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    
    if button_id == 'start-button':
        return dbc.Alert("Simulation démarrée !", color="success")
    elif button_id == 'stop-button':
        return dbc.Alert("Simulation arrêtée", color="secondary")
    elif button_id == 'pause-button':
        return dbc.Alert("Simulation en pause", color="warning")
    
    return dbc.Alert("Simulation arrêtée", color="secondary")

if __name__ == '__main__':
    # Lancer l'application
    print("\n" + "="*50)
    print("Application Dash démarrée !")
    print("Ouvrez votre navigateur à l'adresse :")
    print("http://localhost:8050")
    print("="*50)
    print("Note: Les messages 'Bad request version' sont normaux")
    print("et peuvent être ignorés (tentatives de connexion automatiques)")
    print("="*50 + "\n")
    # Utiliser use_reloader=False pour réduire les messages en développement
    app.run(debug=True, port=8050, use_reloader=False)
