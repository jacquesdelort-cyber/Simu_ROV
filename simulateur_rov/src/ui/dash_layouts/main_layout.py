"""Layout principal de l'application Dash ROV"""
from dash import html, dcc
import dash_bootstrap_components as dbc
from .config_layout import create_config_layout
from .simulation_layout import create_simulation_layout
from .calc_layout import create_calc_layout
from .init_layout import create_init_layout


def create_main_layout():
    """Crée le layout principal de l'application"""
    
    return html.Div([
        # Header
        dbc.NavbarSimple(
            brand="🌊 Simulateur ROV - Main layout",
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
                        dbc.Tab(
                            label="Simulation",
                            tab_id="simulation",
                            children=create_simulation_layout()
                        ),
                        dbc.Tab(
                            label="Paramètres d'environnement",
                            tab_id="config",
                            children=create_config_layout()
                        ),
                        dbc.Tab(
                            label="Paramètres calcul",
                            tab_id="calc",
                            children=create_calc_layout()
                        ),
                        dbc.Tab(
                            label="Conditions initiales",
                            tab_id="init",
                            children=create_init_layout()
                        ),
                    ], id="main-tabs", active_tab="simulation"),
                ], width=12)
            ])
        ], fluid=True, className="mt-3"),
        
        # Intervals pour mises à jour (désactivés par défaut)
        dcc.Interval(
            id='simulation-interval',
            interval=2000,  # ms - Vérification toutes les 2 secondes (pour tests de débogage)
            n_intervals=0,
            disabled=True
        ),
        dcc.Interval(
            id='metrics-interval',
            interval=3000,  # ms - Mise à jour des métriques toutes les 3 secondes (pour tests de débogage)
            n_intervals=0,
            disabled=True
        ),
    ])
