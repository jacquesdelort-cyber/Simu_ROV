"""Composants Dash pour les inputs de paramètres"""
from dash import html
import dash_bootstrap_components as dbc


def create_rov_parameters_inputs():
    """Crée les inputs pour les paramètres ROV"""
    return html.Div([
        html.H5("ROV", className="mb-3"),
        dbc.Label("Masse du ROV (kg)"),
        dbc.Input(id="m_rov", type="number", value=100.0, min=1.0, max=1000.0, step=0.1, className="mb-3"),
        
        dbc.Label("Largeur du ROV (m)"),
        dbc.Input(id="a_rov", type="number", value=0.5, min=0.1, max=5.0, step=0.001, className="mb-3"),
        
        dbc.Label("Longueur du ROV (m)"),
        dbc.Input(id="b_rov", type="number", value=1.0, min=0.1, max=10.0, step=0.001, className="mb-3"),
        
        dbc.Label("Hauteur du ROV (m)"),
        dbc.Input(id="h_rov", type="number", value=0.5, min=0.1, max=5.0, step=0.001, className="mb-3"),
        
        dbc.Label("Cx_ROV"),
        dbc.Input(id="cx_rov", type="number", value=0.8, min=0.1, max=2.0, step=0.01, className="mb-3"),
        
        dbc.Label("Cy_ROV"),
        dbc.Input(id="cy_rov", type="number", value=1.0, min=0.1, max=2.0, step=0.01),
    ])


def create_cable_parameters_inputs():
    """Crée les inputs pour les paramètres Câble"""
    return html.Div([
        html.H5("Câble", className="mb-3"),
        dbc.Label("Diamètre du câble (m)"),
        dbc.Input(id="d_cable", type="number", value=0.01, min=0.001, max=0.1, step=0.0001, className="mb-3"),
        
        dbc.Label("Masse volumique du câble (kg/m³)"),
        dbc.Input(id="rho_cable", type="number", value=1500.0, min=100.0, max=10000.0, step=1.0, className="mb-3"),
        
        dbc.Label("Coefficient de traînée du câble"),
        dbc.Input(id="cx_cable", type="number", value=1.2, min=0.1, max=2.0, step=0.01),
    ])


def create_boat_parameters_inputs():
    """Crée les inputs pour les paramètres Bateau"""
    return html.Div([
        html.H5("Bateau", className="mb-3"),
        dbc.Label("Masse du bateau (kg)"),
        dbc.Input(id="m_boat", type="number", value=10000.0, min=1000.0, max=100000.0, step=100.0),
    ])


def create_environment_parameters_inputs():
    """Crée les inputs pour les paramètres Environnement"""
    return html.Div([
        html.H5("Environnement", className="mb-3"),
        dbc.Label("Masse volumique de l'eau (kg/m³)"),
        dbc.Input(id="rho_eau", type="number", value=1025.0, min=900.0, max=1100.0, step=0.1, className="mb-3"),
        
        dbc.Label("Accélération de la pesanteur (m/s²)"),
        dbc.Input(id="g", type="number", value=9.81, min=1.0, max=20.0, step=0.01, className="mb-3"),
        
        dbc.Label("Viscosité dynamique de l'eau (Pa·s)"),
        dbc.Input(id="mu", type="number", value=0.001, min=0.0001, max=0.01, step=0.000001),
    ])
