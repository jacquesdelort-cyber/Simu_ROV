"""
Exemple minimal d'application Dash pour le simulateur ROV
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

# Stores pour état global (équivalent à st.session_state)
stores = [
    dcc.Store(id='simulation-state', data={
        'running': False,
        'paused': False,
        'current_time': 0.0,
        'data': {
            'time': [],
            'x_rov': [],
            'y_rov': [],
        }
    }),
    dcc.Store(id='parameters-store', data={}),
    dcc.Store(id='mission-store', data={'current': None}),
]

# Layout principal
app.layout = html.Div([
    *stores,
    
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
                        ], style={'padding': '20px'})
                    ]),
                    dbc.Tab(label="Simulation", tab_id="simulation", children=[
                        html.Div([
                            html.H3("Simulation"),
                            html.P("Onglet de simulation (à migrer)"),
                            
                            # Exemple de contrôles
                            dbc.Row([
                                dbc.Col([
                                    dbc.Button("Démarrer", id="start-button", color="success", className="me-2"),
                                    dbc.Button("Arrêter", id="stop-button", color="danger", className="me-2"),
                                    dbc.Button("Pause", id="pause-button", color="warning"),
                                ])
                            ], className="mt-3"),
                            
                            # Affichage temps (exemple)
                            html.Div([
                                dbc.Alert("Simulation arrêtée", color="secondary")
                            ], id="time-display", className="mt-3"),
                            
                            # Graphique (exemple) - avec figure initiale pour éviter "Loading..."
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
                            ),
                        ], style={'padding': '20px'})
                    ]),
                    dbc.Tab(label="Analyse", tab_id="analyse", children=[
                        html.Div([
                            html.H3("Analyse"),
                            html.P("Onglet d'analyse (à migrer)"),
                        ], style={'padding': '20px'})
                    ]),
                ], id="main-tabs", active_tab="config"),
            ], width=12)
        ])
    ], fluid=True, className="mt-3"),
    
    # Interval pour mises à jour (désactivé par défaut)
    dcc.Interval(
        id='simulation-interval',
        interval=100,  # ms
        n_intervals=0,
        disabled=True
    ),
    
    dcc.Interval(
        id='display-interval',
        interval=1000,  # ms
        n_intervals=0,
        disabled=True
    ),
])

# Exemple de callback pour contrôles simulation
@app.callback(
    [Output('simulation-state', 'data', allow_duplicate=True),
     Output('simulation-interval', 'disabled'),
     Output('display-interval', 'disabled')],
    Input('start-button', 'n_clicks'),
    Input('stop-button', 'n_clicks'),
    Input('pause-button', 'n_clicks'),
    State('simulation-state', 'data'),
    prevent_initial_call=True
)
def control_simulation(start, stop, pause, state):
    """Gère les contrôles de simulation"""
    from dash import callback_context
    
    ctx = callback_context
    if not ctx.triggered:
        return state, True, True
    
    button_id = ctx.triggered[0]['prop_id'].split('.')[0]
    
    if button_id == 'start-button':
        state['running'] = True
        state['paused'] = False
        return state, False, False  # Activer les intervals
    elif button_id == 'stop-button':
        state['running'] = False
        state['paused'] = False
        return state, True, True  # Désactiver les intervals
    elif button_id == 'pause-button':
        state['paused'] = not state['paused']
        return state, state['paused'], False
    
    return state, True, True

# Exemple de callback pour mise à jour temps réel
@app.callback(
    Output('time-display', 'children'),
    Input('display-interval', 'n_intervals'),
    State('simulation-state', 'data'),
    prevent_initial_call=True
)
def update_time_display(n, state):
    """Met à jour l'affichage du temps"""
    if state and state.get('running', False):
        current_time = state.get('current_time', 0.0)
        return dbc.Alert(f"Temps écoulé : {current_time:.2f} s", color="info")
    return dbc.Alert("Simulation arrêtée", color="secondary")

# Valeur initiale pour le graphique (évite le "Loading...")
initial_figure = {
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

# Exemple de callback pour graphique
@app.callback(
    Output('system-plot', 'figure'),
    Input('display-interval', 'n_intervals'),
    State('simulation-state', 'data'),
    prevent_initial_call=True
)
def update_plot(n, state):
    """Met à jour le graphique (exemple)"""
    import plotly.graph_objects as go
    
    # Graphique exemple (à remplacer par create_system_plot)
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=[0, 1, 2, 3, 4],
        y=[0, 1, 4, 9, 16],
        mode='lines+markers',
        name='Exemple'
    ))
    fig.update_layout(
        title="Graphique système (exemple)",
        xaxis_title="X",
        yaxis_title="Y"
    )
    return fig

if __name__ == '__main__':
    # Lancer l'application
    print("\n" + "="*50)
    print("Application Dash démarrée !")
    print("Ouvrez votre navigateur à l'adresse :")
    print("http://localhost:8050")
    print("="*50 + "\n")
    app.run(debug=True, port=8050)
