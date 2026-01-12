"""
Application Dash principale pour le simulateur ROV
Migration depuis Streamlit vers Dash
"""
import dash
from dash import dcc, html
import dash_bootstrap_components as dbc
from dash.dependencies import Input, Output, State

# Configuration pour les Background Callbacks (calculs longs) - Dash 3.x
try:
    import diskcache
    import os
    
    # Créer un répertoire pour le cache si nécessaire
    cache_dir = os.path.join(os.getcwd(), 'cache')
    os.makedirs(cache_dir, exist_ok=True)
    
    # Initialiser le gestionnaire de callbacks en arrière-plan avec diskcache
    cache = diskcache.Cache(cache_dir)
    background_callback_manager = dash.DiskcacheManager(cache)
    
    # Initialisation de l'application Dash avec support des Background Callbacks
    app = dash.Dash(
        __name__,
        external_stylesheets=[dbc.themes.BOOTSTRAP],
        suppress_callback_exceptions=True,
        background_callback_manager=background_callback_manager
    )
    print("✓ Background Callbacks activés (Dash 3.x avec diskcache)")
except ImportError as e:
    # Fallback si diskcache n'est pas installé
    print(f"⚠️  diskcache non disponible ({e}) - installation recommandée: pip install diskcache")
    print("⚠️  Les Background Callbacks ne seront pas disponibles")
    app = dash.Dash(
        __name__,
        external_stylesheets=[dbc.themes.BOOTSTRAP],
        suppress_callback_exceptions=True
    )
except Exception as e:
    # Autre erreur lors de l'initialisation
    print(f"⚠️  Erreur lors de l'initialisation des Background Callbacks: {e}")
    print("⚠️  L'application continuera sans Background Callbacks")
    import traceback
    traceback.print_exc()
    app = dash.Dash(
        __name__,
        external_stylesheets=[dbc.themes.BOOTSTRAP],
        suppress_callback_exceptions=True
    )

# Import des layouts et callbacks
from .dash_layouts.main_layout import create_main_layout
from .dash_callbacks.mission_callbacks import register_mission_callbacks
from .dash_callbacks.parameter_callbacks import register_parameter_callbacks
from .dash_callbacks.simulation_callbacks import register_simulation_callbacks
from .dash_callbacks.data_callbacks import register_data_callbacks

# Configuration de la page
app.title = "Simulateur ROV !"

# Stores pour état global (équivalent à st.session_state)
stores = [
    dcc.Store(id='simulation-state', data={
        'running': False,
        'paused': False,
        'current_time': 0.0,
        'system': None,
        'y_current': None,
        't_current': 0.0,
        'data': {
            'time': [],
            'x_rov': [],
            'y_rov': [],
            'vx_rov': [],
            'vy_rov': [],
            'x_boat': [],
            'L': [],
            'T0': [],
            'T_boat': [],
            'T_max': [],
            'x_cable_curr': None,
            'y_cable_curr': None,
        }
    }),
    dcc.Store(id='parameters-store', data={}),
    dcc.Store(id='calc-params-store', data={
        'method': 'RK45',
        'rtol': 1e-5,
        'atol': 1e-7,
        'max_step': 0.1,
        'steps_per_update': 5,
        't_final': 60.0,
        'dt_max': 0.1,
        'N_segments': 50
    }),
    dcc.Store(id='mission-store', data={'current': None}),
    dcc.Store(id='init-params-store', data={
        'x_rov_init': 0.0,
        'y_rov_init': 10.0,
        'L_init': 50.0,
        'v_courant': 0.0
    }),
]

# Layout principal
app.layout = html.Div([
    *stores,
    create_main_layout()
])

# Enregistrer tous les callbacks
try:
    print("Enregistrement des callbacks...")
    register_mission_callbacks(app)
    print("  ✓ Callbacks de mission")
    register_parameter_callbacks(app)
    print("  ✓ Callbacks de paramètres")
    register_simulation_callbacks(app)
    print("  ✓ Callbacks de simulation")
    register_data_callbacks(app)  # Callbacks de données (simulation avec thread séparé)
    print("  ✓ Callbacks de données")
    print("✓ Tous les callbacks activés")
    print("ℹ️  Simulation exécutée dans un thread séparé pour éviter les timeouts")
    print("ℹ️  Visualisations temps réel activées")
except Exception as e:
    print(f"❌ Erreur lors de l'enregistrement des callbacks: {e}")
    import traceback
    traceback.print_exc()
    print("⚠️  L'application peut ne pas fonctionner correctement")

if __name__ == '__main__':
    print("\n" + "="*50)
    print("Application Dash démarrée !!!")
    print("Ouvrez votre navigateur à l'adresse :")
    print("http://localhost:8050")
    print("="*50 + "\n")
    app.run(
        debug=True, 
        port=8050, 
        use_reloader=False,
        dev_tools_hot_reload=False,
        dev_tools_ui=True,
        dev_tools_serve_dev_bundles=True
    )