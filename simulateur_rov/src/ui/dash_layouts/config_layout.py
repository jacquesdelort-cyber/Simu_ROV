"""Layout pour l'onglet de configuration"""
from dash import html
import dash_bootstrap_components as dbc
from ..dash_components.parameter_inputs import (
    create_rov_parameters_inputs,
    create_cable_parameters_inputs,
    create_boat_parameters_inputs,
    create_environment_parameters_inputs
)


def create_config_layout():
    """Crée le layout pour l'onglet de configuration"""
    
    return dbc.Container([
        dbc.Row([
            dbc.Col([
                create_rov_parameters_inputs()
            ], width=3),
            dbc.Col([
                create_cable_parameters_inputs()
            ], width=3),
            dbc.Col([
                create_boat_parameters_inputs()
            ], width=3),
            dbc.Col([
                create_environment_parameters_inputs()
            ], width=3),
        ]),
        dbc.Row([
            dbc.Col([
                dbc.Button(
                    "💾 Sauvegarder les paramètres",
                    id="save-params-button",
                    color="primary",
                    className="mt-3 w-100"
                ),
                html.Div(id="save-params-message", className="mt-2"),
            ], width=12)
        ])
    ], fluid=True, className="mt-3")
