# Plan de Migration : Streamlit → Dash

## 📋 Table des matières

1. [Analyse de l'existant](#analyse-de-lexistant)
2. [Architecture Dash proposée](#architecture-dash-proposée)
3. [Plan de migration par phases](#plan-de-migration-par-phases)
4. [Détails techniques par composant](#détails-techniques-par-composant)
5. [Checklist de migration](#checklist-de-migration)
6. [Estimation de temps](#estimation-de-temps)
7. [Exemples de code](#exemples-de-code)

---

## 📊 Analyse de l'existant

### Structure actuelle Streamlit

**Fichiers principaux :**
- `src/ui/streamlit_app.py` (~1481 lignes)
- `src/visualization/plotter.py` (réutilisable)
- `src/models/` (réutilisable)
- `src/utils/` (réutilisable)

### Composants UI identifiés

#### 1. **Gestion des Missions**
- Sélection/création de missions
- Chargement/sauvegarde de paramètres
- Navigation entre missions

#### 2. **Onglets de Configuration**
- **Paramètres ROV** : m, a, b, h, Cx, Cy
- **Paramètres Câble** : d, rho, Cx
- **Paramètres Bateau** : m
- **Paramètres Environnement** : rho_eau, g, mu, v_courant
- **Paramètres Calcul** : method, rtol, atol, max_step, steps_per_update, t_final, dt_max, N_segments
- **Conditions Initiales** : x_rov_init, y_rov_init, L_init, v_courant

#### 3. **Onglet Simulation**
- Contrôles de simulation (Démarrer, Arrêter, Pause, Reprendre)
- Paramètres de commande (Fx_rov, Fy_rov, vx_boat_cmd, dL_dt)
- Visualisation temps réel (vue système 2D avec Plotly)
- Métriques temps réel (Temps écoulé, Profondeur, Vitesse, Tensions)
- Graphiques temps réel (positions, vitesses)

#### 4. **Analyse Détaillée**
- Graphiques post-simulation (Positions, Vitesses, Tension)
- Visualisation interactive avec slider temporel
- Export CSV

### Technologies utilisées
- **Streamlit** : Interface web
- **Plotly** : Visualisations (réutilisable)
- **NumPy/SciPy** : Calculs (réutilisable)
- **JSON** : Sauvegarde paramètres (réutilisable)
- **Pandas** : Export données (réutilisable)

---

## 🏗️ Architecture Dash proposée

### Structure des fichiers

```
simulateur_rov/
├── src/
│   ├── ui/
│   │   ├── dash_app.py              # Application Dash principale
│   │   ├── dash_components/         # Composants Dash réutilisables
│   │   │   ├── __init__.py
│   │   │   ├── mission_controls.py  # Composants gestion missions
│   │   │   ├── parameter_inputs.py  # Composants paramètres
│   │   │   ├── simulation_controls.py # Contrôles simulation
│   │   │   └── visualizations.py    # Wrappers visualisations
│   │   ├── dash_callbacks/          # Callbacks Dash
│   │   │   ├── __init__.py
│   │   │   ├── mission_callbacks.py # Callbacks missions
│   │   │   ├── parameter_callbacks.py # Callbacks paramètres
│   │   │   ├── simulation_callbacks.py # Callbacks simulation
│   │   │   └── data_callbacks.py    # Callbacks données
│   │   └── dash_layouts/            # Layouts Dash
│   │       ├── __init__.py
│   │       ├── main_layout.py       # Layout principal
│   │       ├── config_layout.py     # Layout configuration
│   │       └── simulation_layout.py # Layout simulation
│   ├── models/                      # ✅ Réutilisé tel quel
│   ├── solvers/                     # ✅ Réutilisé tel quel
│   ├── utils/                       # ✅ Réutilisé tel quel
│   └── visualization/               # ✅ Réutilisé tel quel
└── requirements_dash.txt            # Nouvelles dépendances
```

### Architecture des composants

```
┌─────────────────────────────────────────────────────────┐
│                    dash_app.py                          │
│  - Initialisation app Dash                              │
│  - Configuration serveur                                │
│  - Enregistrement callbacks                             │
│  - Layout principal                                     │
└─────────────────────────────────────────────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
┌───────▼────────┐ ┌──────▼──────┐ ┌───────▼─────────┐
│  dash_layouts/ │ │dash_components│ │dash_callbacks/ │
│  - Structure UI│ │ - Composants │ │ - Logique       │
│  - Organisation│ │ - Réutilisable│ │ - Événements   │
└────────────────┘ └──────────────┘ └─────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
┌───────▼────────┐ ┌──────▼──────┐ ┌───────▼─────────┐
│   models/      │ │visualization│ │    utils/        │
│   solvers/     │ │  plotter.py │ │  parameters.py   │
│                │ │             │ │  data_io.py      │
└────────────────┘ └─────────────┘ └──────────────────┘
```

### Gestion de l'état

**Dash utilise `dcc.Store` pour l'état global :**

```python
# Store pour état simulation
dcc.Store(id='simulation-state', data={
    'running': False,
    'paused': False,
    'current_time': 0.0,
    'system': None,
    'data': {...}
})

# Store pour paramètres
dcc.Store(id='parameters-store', data={...})

# Store pour mission
dcc.Store(id='mission-store', data={...})
```

---

## 🚀 Plan de migration par phases

### Phase 0 : Préparation (1 jour)

**Objectifs :**
- Installer Dash et dépendances
- Créer la structure de fichiers
- Configurer l'environnement de développement

**Tâches :**
- [ ] Installer `dash`, `dash-bootstrap-components`, `dash-core-components`
- [ ] Créer structure de répertoires `dash_components/`, `dash_callbacks/`, `dash_layouts/`
- [ ] Créer fichier `requirements_dash.txt`
- [ ] Configurer environnement de test parallèle

**Livrables :**
- Structure de fichiers créée
- Dépendances installées
- Application Dash minimale fonctionnelle

---

### Phase 1 : Structure de base (2 jours)

**Objectifs :**
- Créer l'application Dash de base
- Implémenter le layout principal
- Configurer le système de routing (si nécessaire)

**Tâches :**
- [ ] Créer `dash_app.py` avec app Dash de base
- [ ] Créer `dash_layouts/main_layout.py` avec structure principale
- [ ] Implémenter navigation par onglets (équivalent Streamlit tabs)
- [ ] Configurer `dcc.Store` pour état global
- [ ] Créer layout responsive avec `dbc.Row` et `dbc.Col`

**Architecture proposée :**

```python
# dash_app.py
import dash
from dash import dcc, html
import dash_bootstrap_components as dbc

app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

# Stores pour état global
stores = [
    dcc.Store(id='simulation-state'),
    dcc.Store(id='parameters-store'),
    dcc.Store(id='mission-store'),
]

# Layout principal
app.layout = html.Div([
    *stores,
    create_main_layout()  # Depuis dash_layouts/main_layout.py
])

if __name__ == '__main__':
    app.run(debug=True)
```

**Livrables :**
- Application Dash fonctionnelle avec layout de base
- Navigation par onglets
- Structure responsive

---

### Phase 2 : Gestion des Missions (1.5 jours)

**Objectifs :**
- Migrer la gestion des missions
- Implémenter sélection/création de missions
- Chargement/sauvegarde des paramètres

**Tâches :**
- [ ] Créer `dash_components/mission_controls.py`
- [ ] Créer `dash_callbacks/mission_callbacks.py`
- [ ] Migrer fonctions utilitaires (réutiliser code existant)
- [ ] Implémenter callbacks pour sélection mission
- [ ] Implémenter callbacks pour création mission
- [ ] Implémenter callbacks pour chargement paramètres

**Composants Dash équivalents :**

| Streamlit | Dash |
|-----------|------|
| `st.selectbox()` | `dcc.Dropdown()` |
| `st.button()` | `dbc.Button()` |
| `st.text_input()` | `dbc.Input()` |

**Livrables :**
- Composants gestion missions fonctionnels
- Callbacks missions opérationnels
- Chargement/sauvegarde paramètres fonctionnel

---

### Phase 3 : Onglets de Configuration (2.5 jours)

**Objectifs :**
- Migrer tous les onglets de configuration
- Implémenter les inputs de paramètres
- Sauvegarde des paramètres

**Tâches :**
- [ ] Créer `dash_components/parameter_inputs.py`
- [ ] Créer `dash_layouts/config_layout.py`
- [ ] Créer `dash_callbacks/parameter_callbacks.py`
- [ ] Migrer onglet Paramètres ROV
- [ ] Migrer onglet Paramètres Câble
- [ ] Migrer onglet Paramètres Bateau
- [ ] Migrer onglet Paramètres Environnement
- [ ] Migrer onglet Paramètres Calcul
- [ ] Migrer onglet Conditions Initiales
- [ ] Implémenter callbacks sauvegarde

**Composants Dash équivalents :**

| Streamlit | Dash |
|-----------|------|
| `st.number_input()` | `dbc.Input(type='number')` |
| `st.slider()` | `dcc.Slider()` |
| `st.selectbox()` | `dcc.Dropdown()` |
| `st.tabs()` | `dbc.Tabs()` |

**Exemple de migration :**

```python
# Streamlit (avant)
x_rov_init = st.number_input(
    "Position horizontale ROV (m)", 
    0.0, 100.0, 
    0.0, 
    1.0, 
    key="x_rov_init"
)

# Dash (après)
dbc.Input(
    id="x_rov_init",
    type="number",
    value=0.0,
    min=0.0,
    max=100.0,
    step=1.0,
    placeholder="Position horizontale ROV (m)"
)
```

**Livrables :**
- Tous les onglets de configuration migrés
- Inputs fonctionnels
- Sauvegarde paramètres opérationnelle

---

### Phase 4 : Contrôles de Simulation (2 jours)

**Objectifs :**
- Migrer les contrôles de simulation
- Implémenter les paramètres de commande
- Gérer l'état de simulation

**Tâches :**
- [ ] Créer `dash_components/simulation_controls.py`
- [ ] Créer `dash_layouts/simulation_layout.py`
- [ ] Créer `dash_callbacks/simulation_callbacks.py`
- [ ] Migrer boutons contrôle (Démarrer, Arrêter, Pause)
- [ ] Migrer inputs commande (Fx_rov, Fy_rov, vx_boat_cmd, dL_dt)
- [ ] Implémenter callbacks pour démarrage/arrêt simulation
- [ ] Implémenter gestion état simulation avec `dcc.Store`

**Gestion simulation temps réel :**

```python
# Dash utilise dcc.Interval pour mises à jour périodiques
dcc.Interval(
    id='simulation-interval',
    interval=100,  # ms
    n_intervals=0,
    disabled=True
)

@app.callback(
    Output('simulation-state', 'data'),
    Input('simulation-interval', 'n_intervals'),
    State('simulation-state', 'data')
)
def update_simulation(n, state):
    if state['running'] and not state['paused']:
        # Faire un pas de simulation
        # Mettre à jour state
        pass
    return state
```

**Livrables :**
- Contrôles simulation fonctionnels
- Paramètres commande opérationnels
- État simulation géré correctement

---

### Phase 5 : Visualisations Temps Réel (3 jours)

**Objectifs :**
- Migrer visualisations temps réel
- Implémenter graphiques Plotly
- Métriques temps réel

**Tâches :**
- [ ] Créer `dash_components/visualizations.py`
- [ ] Wrapper pour graphiques Plotly (réutiliser `plotter.py`)
- [ ] Migrer vue système 2D (Plotly)
- [ ] Migrer métriques temps réel
- [ ] Migrer graphiques temps réel (positions, vitesses)
- [ ] Implémenter callbacks pour mises à jour graphiques
- [ ] Optimiser performance (éviter recalculs inutiles)

**Réutilisation Plotly :**

```python
# plotter.py est réutilisable tel quel !
from src.visualization.plotter import create_system_plot

# Dans callback Dash
@app.callback(
    Output('system-plot', 'figure'),
    Input('simulation-state', 'data')
)
def update_system_plot(state):
    # Extraire données depuis state
    x_rov, y_rov, x_cable, y_cable, ... = extract_from_state(state)
    
    # Réutiliser fonction existante
    fig = create_system_plot(x_rov, y_rov, x_cable, y_cable, ...)
    return fig
```

**Mises à jour temps réel :**

```python
# Dash utilise dcc.Interval + callbacks pour temps réel
dcc.Interval(
    id='display-interval',
    interval=1000,  # Mise à jour toutes les secondes
    n_intervals=0
)

@app.callback(
    [Output('time-display', 'children'),
     Output('system-plot', 'figure')],
    Input('display-interval', 'n_intervals'),
    State('simulation-state', 'data')
)
def update_display(n, state):
    # Mettre à jour affichage
    time_str = f"{state['current_time']:.2f} s"
    fig = create_system_plot(...)
    return time_str, fig
```

**Livrables :**
- Visualisations temps réel fonctionnelles
- Graphiques Plotly intégrés
- Métriques mises à jour en temps réel
- Performance optimisée

---

### Phase 6 : Analyse Détaillée (2 jours)

**Objectifs :**
- Migrer analyse post-simulation
- Graphiques détaillés
- Export CSV

**Tâches :**
- [ ] Migrer graphiques détaillés (Positions, Vitesses, Tension)
- [ ] Migrer slider temporel interactif
- [ ] Migrer visualisation avec slider
- [ ] Implémenter export CSV
- [ ] Implémenter callbacks pour analyse

**Livrables :**
- Analyse détaillée fonctionnelle
- Export CSV opérationnel
- Visualisation interactive avec slider

---

### Phase 7 : Tests et Optimisations (1.5 jours)

**Objectifs :**
- Tester toutes les fonctionnalités
- Optimiser performance
- Corriger bugs

**Tâches :**
- [ ] Tests fonctionnels complets
- [ ] Tests de performance
- [ ] Optimisation callbacks (memoization)
- [ ] Correction bugs
- [ ] Documentation code

**Optimisations Dash :**

```python
from functools import lru_cache
from dash import callback_context

# Memoization pour éviter recalculs
@lru_cache(maxsize=128)
def expensive_computation(params):
    # Calcul coûteux
    pass

# Prévention callbacks multiples
app.config.suppress_callback_exceptions = True
```

**Livrables :**
- Application testée et fonctionnelle
- Performance optimisée
- Documentation mise à jour

---

## 🔧 Détails techniques par composant

### 1. Gestion de l'état

**Streamlit :**
```python
st.session_state['simulation_running'] = True
st.session_state['current_time'] = 0.0
```

**Dash :**
```python
# Store global
dcc.Store(id='simulation-state', data={
    'running': False,
    'paused': False,
    'current_time': 0.0,
    'system': None,
    'data': {...}
})

# Callback pour mettre à jour
@app.callback(
    Output('simulation-state', 'data'),
    Input('start-button', 'n_clicks'),
    State('simulation-state', 'data')
)
def start_simulation(n_clicks, state):
    if n_clicks:
        state['running'] = True
    return state
```

### 2. Mises à jour temps réel

**Streamlit :**
```python
st.rerun()  # Re-exécute tout le script
```

**Dash :**
```python
# Interval pour mises à jour périodiques
dcc.Interval(
    id='update-interval',
    interval=100,  # ms
    n_intervals=0
)

# Callback déclenché par interval
@app.callback(
    Output('display', 'children'),
    Input('update-interval', 'n_intervals'),
    State('simulation-state', 'data')
)
def update_display(n, state):
    # Mise à jour ciblée
    return f"{state['current_time']:.2f} s"
```

### 3. Navigation par onglets

**Streamlit :**
```python
tab1, tab2, tab3 = st.tabs(["Config", "Simulation", "Analyse"])
with tab1:
    # Contenu
```

**Dash :**
```python
dbc.Tabs([
    dbc.Tab(label="Config", tab_id="config"),
    dbc.Tab(label="Simulation", tab_id="simulation"),
    dbc.Tab(label="Analyse", tab_id="analyse")
], id="main-tabs", active_tab="config")

# Contenu conditionnel via callback
@app.callback(
    Output('tab-content', 'children'),
    Input('main-tabs', 'active_tab')
)
def update_tab_content(active_tab):
    if active_tab == "config":
        return create_config_layout()
    elif active_tab == "simulation":
        return create_simulation_layout()
    # ...
```

### 4. Visualisations Plotly

**Streamlit :**
```python
fig = create_system_plot(...)
st.plotly_chart(fig, use_container_width=True)
```

**Dash :**
```python
# Composant graph
dcc.Graph(id='system-plot')

# Callback pour mettre à jour
@app.callback(
    Output('system-plot', 'figure'),
    Input('simulation-state', 'data')
)
def update_plot(state):
    fig = create_system_plot(...)  # Réutilise code existant !
    return fig
```

### 5. Export CSV

**Streamlit :**
```python
csv = df.to_csv()
st.download_button("Télécharger", data=csv, file_name="data.csv")
```

**Dash :**
```python
# Bouton téléchargement
html.A(
    dbc.Button("Télécharger CSV"),
    id="download-link",
    download="simulation.csv",
    href="",
    target="_blank"
)

# Callback pour générer CSV
@app.callback(
    Output('download-link', 'href'),
    Input('download-button', 'n_clicks'),
    State('simulation-state', 'data')
)
def download_csv(n_clicks, state):
    if n_clicks:
        csv_string = generate_csv(state['data'])
        # Encoder en base64 pour téléchargement
        b64 = base64.b64encode(csv_string.encode()).decode()
        return f"data:text/csv;base64,{b64}"
    return ""
```

---

## ✅ Checklist de migration

### Préparation
- [ ] Backend installé (Dash, dash-bootstrap-components)
- [ ] Structure de fichiers créée
- [ ] Environnement de test configuré

### Fonctionnalités de base
- [ ] Application Dash démarre correctement
- [ ] Layout principal fonctionnel
- [ ] Navigation par onglets opérationnelle

### Gestion Missions
- [ ] Liste missions affichée
- [ ] Sélection mission fonctionnelle
- [ ] Création mission fonctionnelle
- [ ] Chargement paramètres fonctionnel
- [ ] Sauvegarde paramètres fonctionnelle

### Configuration
- [ ] Onglet Paramètres ROV migré
- [ ] Onglet Paramètres Câble migré
- [ ] Onglet Paramètres Bateau migré
- [ ] Onglet Paramètres Environnement migré
- [ ] Onglet Paramètres Calcul migré
- [ ] Onglet Conditions Initiales migré

### Simulation
- [ ] Boutons contrôle fonctionnels
- [ ] Paramètres commande fonctionnels
- [ ] État simulation géré correctement
- [ ] Simulation démarre/arrête correctement
- [ ] Pause/Reprendre fonctionne

### Visualisations
- [ ] Vue système 2D fonctionnelle
- [ ] Métriques temps réel mises à jour
- [ ] Graphiques temps réel fonctionnels
- [ ] Performance acceptable

### Analyse
- [ ] Graphiques détaillés fonctionnels
- [ ] Slider temporel fonctionnel
- [ ] Visualisation interactive fonctionnelle
- [ ] Export CSV fonctionnel

### Tests
- [ ] Tests fonctionnels passent
- [ ] Performance validée
- [ ] Bugs corrigés
- [ ] Documentation à jour

---

## ⏱️ Estimation de temps

### Par phase

| Phase | Tâches | Temps estimé |
|-------|--------|--------------|
| Phase 0 : Préparation | Setup, structure | 1 jour |
| Phase 1 : Structure de base | Layout principal | 2 jours |
| Phase 2 : Gestion Missions | Missions | 1.5 jours |
| Phase 3 : Configuration | 6 onglets | 2.5 jours |
| Phase 4 : Contrôles Simulation | Contrôles | 2 jours |
| Phase 5 : Visualisations Temps Réel | Graphiques temps réel | 3 jours |
| Phase 6 : Analyse Détaillée | Analyse post-sim | 2 jours |
| Phase 7 : Tests et Optimisations | Tests, bugs | 1.5 jours |
| **TOTAL** | | **16 jours** |

### Temps total estimé : 16 jours (3.2 semaines)

**Hypothèses :**
- Développeur familier avec Python et web frameworks
- Code existant bien structuré (réutilisation possible)
- Pas de changements majeurs de fonctionnalités
- Tests unitaires de base seulement

**Facteurs pouvant augmenter le temps :**
- Apprentissage Dash (ajouter 2-3 jours)
- Refactoring majeur nécessaire (+2-3 jours)
- Tests complets (+2-3 jours)
- Documentation détaillée (+1-2 jours)

**Facteurs pouvant réduire le temps :**
- Développeur expérimenté en Dash (-2-3 jours)
- Priorisation fonctionnalités essentielles (-3-4 jours)
- Migration minimale fonctionnelle (-2-3 jours)

---

## 📝 Exemples de code

### Exemple 1 : Application Dash de base

```python
# dash_app.py
import dash
from dash import dcc, html
import dash_bootstrap_components as dbc
from dash.dependencies import Input, Output, State

# Import layouts et callbacks
from src.ui.dash_layouts.main_layout import create_main_layout
from src.ui.dash_callbacks import (
    register_mission_callbacks,
    register_parameter_callbacks,
    register_simulation_callbacks,
    register_data_callbacks
)

# Initialisation app
app = dash.Dash(
    __name__,
    external_stylesheets=[dbc.themes.BOOTSTRAP],
    suppress_callback_exceptions=True
)

# Stores pour état global
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
            # ...
        }
    }),
    dcc.Store(id='parameters-store', data={}),
    dcc.Store(id='mission-store', data={'current': None}),
]

# Layout principal
app.layout = html.Div([
    *stores,
    create_main_layout()
])

# Enregistrer callbacks
register_mission_callbacks(app)
register_parameter_callbacks(app)
register_simulation_callbacks(app)
register_data_callbacks(app)

if __name__ == '__main__':
    app.run(debug=True, port=8050)
```

### Exemple 2 : Layout principal

```python
# dash_layouts/main_layout.py
import dash_bootstrap_components as dbc
from dash import html, dcc
from .config_layout import create_config_layout
from .simulation_layout import create_simulation_layout

def create_main_layout():
    """Crée le layout principal de l'application"""
    
    return html.Div([
        # Header
        dbc.NavbarSimple(
            brand="Simulateur ROV",
            brand_href="#",
            color="primary",
            dark=True,
        ),
        
        # Contenu principal
        dbc.Container([
            dbc.Row([
                dbc.Col([
                    dbc.Tabs([
                        dbc.Tab(label="Configuration", tab_id="config"),
                        dbc.Tab(label="Simulation", tab_id="simulation"),
                        dbc.Tab(label="Analyse", tab_id="analyse"),
                    ], id="main-tabs", active_tab="config"),
                    
                    html.Div(id="tab-content")
                ], width=12)
            ])
        ], fluid=True)
    ])

# Callback pour contenu des onglets
@app.callback(
    Output('tab-content', 'children'),
    Input('main-tabs', 'active_tab')
)
def update_tab_content(active_tab):
    if active_tab == "config":
        return create_config_layout()
    elif active_tab == "simulation":
        return create_simulation_layout()
    elif active_tab == "analyse":
        return create_analysis_layout()
    return html.Div()
```

### Exemple 3 : Callback simulation temps réel

```python
# dash_callbacks/simulation_callbacks.py
from dash import Input, Output, State
import time as time_module

def register_simulation_callbacks(app):
    """Enregistre les callbacks de simulation"""
    
    # Interval pour simulation
    app.layout.children.append(
        dcc.Interval(
            id='simulation-interval',
            interval=100,  # ms
            n_intervals=0,
            disabled=True
        )
    )
    
    # Interval pour affichage
    app.layout.children.append(
        dcc.Interval(
            id='display-interval',
            interval=1000,  # ms
            n_intervals=0,
            disabled=True
        )
    )
    
    @app.callback(
        [Output('simulation-interval', 'disabled'),
         Output('display-interval', 'disabled'),
         Output('simulation-state', 'data')],
        Input('start-button', 'n_clicks'),
        Input('stop-button', 'n_clicks'),
        Input('pause-button', 'n_clicks'),
        State('simulation-state', 'data')
    )
    def control_simulation(start, stop, pause, state):
        ctx = callback_context
        if not ctx.triggered:
            return True, True, state
        
        button_id = ctx.triggered[0]['prop_id'].split('.')[0]
        
        if button_id == 'start-button':
            state['running'] = True
            state['paused'] = False
            return False, False, state
        elif button_id == 'stop-button':
            state['running'] = False
            state['paused'] = False
            return True, True, state
        elif button_id == 'pause-button':
            state['paused'] = not state['paused']
            return state['paused'], False, state
        
        return True, True, state
    
    @app.callback(
        Output('simulation-state', 'data', allow_duplicate=True),
        Input('simulation-interval', 'n_intervals'),
        State('simulation-state', 'data'),
        prevent_initial_call=True
    )
    def update_simulation(n, state):
        if not state['running'] or state['paused']:
            return state
        
        # Faire un pas de simulation
        # (logique similaire à Streamlit)
        system = state['system']
        t_current = state['t_current']
        y_current = state['y_current']
        
        # ... code simulation ...
        
        # Mettre à jour state
        state['t_current'] = t_current + dt
        state['y_current'] = y_current
        state['current_time'] = t_current + dt
        
        return state
```

### Exemple 4 : Composant paramètres

```python
# dash_components/parameter_inputs.py
import dash_bootstrap_components as dbc
from dash import html

def create_rov_parameters_inputs():
    """Crée les inputs pour paramètres ROV"""
    
    return html.Div([
        dbc.Row([
            dbc.Col([
                dbc.Label("Masse ROV (kg)"),
                dbc.Input(
                    id="m_rov",
                    type="number",
                    value=100.0,
                    min=1.0,
                    max=1000.0,
                    step=1.0
                )
            ], width=6),
            dbc.Col([
                dbc.Label("Longueur ROV (m)"),
                dbc.Input(
                    id="a_rov",
                    type="number",
                    value=0.5,
                    min=0.1,
                    max=5.0,
                    step=0.1
                )
            ], width=6),
        ]),
        # ... autres paramètres ...
    ])
```

---

## 🎯 Prochaines étapes

1. **Valider le plan** : Revoir et ajuster selon vos contraintes
2. **Créer un prototype** : Phase 0 + Phase 1 pour valider l'approche
3. **Démarrer migration** : Suivre le plan phase par phase
4. **Tests continus** : Tester chaque phase avant de passer à la suivante
5. **Documentation** : Documenter au fur et à mesure

---

## 📚 Ressources

- **Dash Documentation** : https://dash.plotly.com/
- **Dash Bootstrap Components** : https://dash-bootstrap-components.opensource.faculty.ai/
- **Dash Callbacks** : https://dash.plotly.com/basic-callbacks
- **Dash Store** : https://dash.plotly.com/dash-core-components/store

---

## 🔄 Migration parallèle recommandée

**Recommandation :** Garder Streamlit fonctionnel pendant la migration Dash

- Créer branche Git pour migration Dash
- Tester Dash en parallèle
- Basculer une fois Dash validé
- Garder Streamlit comme backup

---

## ⚠️ Points d'attention

1. **Gestion de l'état** : Dash nécessite une approche différente (Stores vs session_state)
2. **Callbacks** : Architecture basée sur callbacks (penser à la structure)
3. **Performance** : Optimiser avec memoization si nécessaire
4. **Tests** : Tester chaque phase avant de continuer
5. **Documentation** : Documenter les callbacks complexes

---

**Date de création :** 2024
**Version :** 1.0
**Auteur :** Plan de migration Dash
