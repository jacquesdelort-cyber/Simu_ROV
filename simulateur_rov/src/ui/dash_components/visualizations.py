"""Composants pour les visualisations"""
from dash import html, dcc
import dash_bootstrap_components as dbc
import plotly.graph_objects as go


def create_system_view():
    """Crée la vue système 2D avec graphique Plotly"""
    # Figure initiale vide
    initial_figure = go.Figure()
    initial_figure.update_layout(
        title="Vue du système",
        xaxis_title="X (m)",
        yaxis_title="Y (m)",
        template="plotly_white",
        height=500,
    )
    
    return html.Div([
        html.H5("Vue du système", className="mb-3"),
        dcc.Graph(id='system-plot', figure=initial_figure, style={'height': '500px'}),
    ])


def create_real_time_metrics():
    """Crée les métriques temps réel"""
    import dash_bootstrap_components as dbc
    return html.Div([
        html.H5("Métriques temps réel", className="mb-3"),
        html.Div(id='real-time-metrics', children=[
            html.Div(id='time-display', className="mb-2", children=dbc.Alert("Temps: 0.00 s", color="info", className="mb-2 p-2")),
            html.Div(id='rov-position', className="mb-2", children=dbc.Alert("ROV: (0.0, 0.0) m", color="info", className="mb-2 p-2")),
            html.Div(id='cable-length', className="mb-2", children=dbc.Alert("Câble: 0.0 m", color="info", className="mb-2 p-2")),
            html.Div(id='tension-max', className="mb-2", children=dbc.Alert("Tension max: 0.0 N", color="info", className="mb-2 p-2")),
        ]),
    ])


def create_analysis_plots():
    """Crée les graphiques d'analyse (positions, vitesses, tensions)"""
    return html.Div([
        dbc.Row([
            dbc.Col([
                html.H6("Position ROV"),
                dcc.Graph(id='position-plot', style={'height': '300px'}),
            ], width=6),
            dbc.Col([
                html.H6("Vitesse ROV"),
                dcc.Graph(id='velocity-plot', style={'height': '300px'}),
            ], width=6),
        ], className="mb-3"),
        dbc.Row([
            dbc.Col([
                html.H6("Tensions"),
                dcc.Graph(id='tension-plot', style={'height': '300px'}),
            ], width=12),
        ]),
    ])
