"""Fonctions de visualisation avec Plotly"""
import plotly.graph_objects as go
import numpy as np

from src.visualization.cable_hover import (
    CABLE_XY_HOVERTEMPLATE,
    cable_markers_along_s_hover_texts,
    cable_polyline_hover_texts,
    cable_vertex_tooltip,
)

# Légende horizontale au-dessus du graphique (comme « Tension vs cible », onglet Simulation)
_LAYOUT_LEGEND_TOP = dict(
    legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
    margin=dict(b=60),
)


def create_system_plot(
    x_rov,
    y_rov,
    x_cable,
    y_cable,
    x_bateau,
    L=0,
    title="Système ROV-Câble-Bateau",
    x_range=None,
    y_range=None,
    T_cable=None,
    cable_mode=None,
    point_indices=None,
):
    """
    Crée la visualisation 2D du système
    
    Parameters:
    -----------
    x_rov, y_rov : float
        Position du ROV
    x_cable, y_cable : array
        Positions du câble
    x_bateau : float
        Position du bateau
    L : float
        Longueur du câble
    title : str
        Titre du graphique
    x_range : list, optional
        Plage de l'axe X [x_min, x_max] à utiliser. Si None, calcule automatiquement.
    y_range : list, optional
        Plage de l'axe Y [y_min, y_max] à utiliser. Si None, calcule automatiquement.
    T_cable : array, optional
        Tensions le long du câble (N) pour affichage dans le tooltip
    
    Returns:
    --------
    go.Figure
        Figure Plotly
    """
    fig = go.Figure()
    
    # S'assurer que le câble est connecté au bateau et au ROV
    # Les données du câble peuvent aller du ROV au bateau ou vice versa
    x_cable_complete = None
    y_cable_complete = None
    T_cable_complete = None
    point_ids_complete = None
    
    if len(x_cable) > 0 and len(y_cable) > 0:
        # Convertir en arrays numpy si nécessaire
        x_cable_arr = np.asarray(x_cable)
        y_cable_arr = np.asarray(y_cable)
        
        # Créer un array complet incluant les extrémités
        x_cable_complete = x_cable_arr.copy()
        y_cable_complete = y_cable_arr.copy()
        # Indices globaux des points (si fournis)
        if point_indices is not None and len(point_indices) == len(x_cable_arr):
            point_ids_complete = np.asarray(point_indices).copy()
        else:
            point_ids_complete = np.arange(len(x_cable_arr), dtype=int)
        
        # Gérer les tensions si fournies
        if T_cable is not None and len(T_cable) > 0:
            T_cable_arr = np.asarray(T_cable)
            T_cable_complete = T_cable_arr.copy()
        
        # Les données du câble arrivent déjà dans l'ordre bateau -> ROV depuis simulation_thread.py
        # Ne pas inverser l'ordre ici pour garantir la cohérence avec les messages d'invariants
        
        # Ne pas modifier les coordonnées du câble pour forcer la connexion au bateau/ROV
        # On trace exactement les points du câble tels qu'ils sont
        # Vérifier si P0 coïncide avec le bateau et PN avec le ROV (avec tolérance)
        tol = 1e-6  # Tolérance pour la coïncidence
        P0_coincide_bateau = (np.abs(x_cable_complete[0] - x_bateau) < tol and 
                              np.abs(y_cable_complete[0] - 0.0) < tol)
        PN_coincide_rov = (np.abs(x_cable_complete[-1] - x_rov) < tol and 
                           np.abs(y_cable_complete[-1] - y_rov) < tol)
        
        dx = np.diff(x_cable_complete)
        dy = np.diff(y_cable_complete)
        ds = np.sqrt(dx**2 + dy**2)
        s_cumulative = np.concatenate(([0.0], np.cumsum(ds)))
        pn_list = [int(point_ids_complete[i]) for i in range(len(x_cable_complete))]
        hover_texts = cable_polyline_hover_texts(
            x_cable_complete, y_cable_complete, point_numbers=pn_list
        )
        if T_cable_complete is not None and len(T_cable_complete) > 0:
            for i in range(len(hover_texts)):
                hover_texts[i] += f"<br>Tension: {float(T_cable_complete[i]):.2f} N"

        # Câble : ligne brisée entre nœuds (pas de line.smoothing Plotly : la spline peut
        # dépasser les points et donner une fausse « bosse vers la surface » sur une chaînette).
        if cable_mode == "straight":
            cable_color = "#d9534f"
        else:
            cable_color = "#1e6bb8"

        trace_params = {
            'x': x_cable_complete,
            'y': y_cable_complete,
            'mode': 'lines+markers',
            'name': 'Câble',
            'line': dict(color=cable_color, width=3, shape='linear'),
            'marker': dict(size=3, color=cable_color, opacity=0.5, symbol='circle'),
            'showlegend': True
        }
        
        trace_params['hovertext'] = hover_texts
        trace_params['hovertemplate'] = CABLE_XY_HOVERTEMPLATE
        trace_params['hoverinfo'] = 'text'

        fig.add_trace(go.Scatter(**trace_params))
    else:
        # Si pas de données de câble, ne pas créer de trace "pas de données"
        # Le graphique affichera seulement le bateau et le ROV
        # Cela évite d'avoir une trace "pas de données" qui reste visible après l'initialisation
        pass
    
    rov_hover_kw: dict = {}
    bateau_hover_kw: dict = {}
    if x_cable_complete is not None and len(x_cable_complete) > 0:
        pn0 = int(point_ids_complete[0])
        pn1 = int(point_ids_complete[-1])
        l_first = (
            float(
                np.hypot(
                    x_cable_complete[1] - x_cable_complete[0],
                    y_cable_complete[1] - y_cable_complete[0],
                )
            )
            if len(x_cable_complete) > 1
            else None
        )
        rov_hover_kw = {
            "hovertext": [
                cable_vertex_tooltip(pn1, float(s_cumulative[-1]), None, float(x_rov), float(y_rov))
            ],
            "hovertemplate": CABLE_XY_HOVERTEMPLATE,
            "hoverinfo": "text",
        }
        bateau_hover_kw = {
            "hovertext": [
                cable_vertex_tooltip(pn0, 0.0, l_first, float(x_bateau), 0.0)
            ],
            "hovertemplate": CABLE_XY_HOVERTEMPLATE,
            "hoverinfo": "text",
        }

    # ROV (marqueur) - même taille que le bateau (20 pixels)
    fig.add_trace(
        go.Scatter(
            x=[x_rov],
            y=[y_rov],
            mode='markers',
            name='ROV',
            marker=dict(size=8, color='red', symbol='square', line=dict(width=2, color='darkred')),
            **rov_hover_kw,
        )
    )

    # Bateau - sera ajouté comme forme avec taille constante en pixels après le layout
    # Ajouter un marqueur invisible pour la légende et le hover
    fig.add_trace(
        go.Scatter(
            x=[x_bateau],
            y=[0],
            mode='markers',
            name='Bateau',
            marker=dict(size=1, opacity=0),
            showlegend=True,
            **bateau_hover_kw,
        )
    )
    
    # Surface de l'eau
    # Utiliser les données du câble complet si disponibles
    if x_cable_complete is not None and y_cable_complete is not None:
        x_min_raw = min(np.min(x_cable_complete), x_rov, x_bateau)
        x_max_raw = max(np.max(x_cable_complete), x_rov, x_bateau)
        y_values = list(y_cable_complete) + [y_rov, 0.0]  # Inclure le câble, le ROV et la surface
    else:
        x_min_raw = min(x_rov, x_bateau)
        x_max_raw = max(x_rov, x_bateau)
        y_values = [y_rov, 0.0]  # Inclure le ROV et la surface
    
    # Utiliser la plage fournie ou calculer une nouvelle plage
    if x_range is not None:
        x_min, x_max = x_range
        # Vérifier si les données sortent de la plage actuelle
        if x_min_raw < x_min or x_max_raw > x_max:
            # Recalculer la plage
            x_min = np.floor(x_min_raw / 10.0) * 10.0
            x_max = np.ceil(x_max_raw / 10.0) * 10.0
            # S'assurer que la plage minimale est de 10 mètres
            if x_max - x_min < 10.0:
                center = (x_min_raw + x_max_raw) / 2.0
                x_min = np.floor((center - 5.0) / 10.0) * 10.0
                x_max = np.ceil((center + 5.0) / 10.0) * 10.0
                if x_max - x_min < 10.0:
                    x_max = x_min + 10.0
    else:
        # Arrondir les limites de l'axe X par paliers de 10 mètres
        # Arrondir vers le bas pour x_min, vers le haut pour x_max
        x_min = np.floor(x_min_raw / 10.0) * 10.0
        x_max = np.ceil(x_max_raw / 10.0) * 10.0
        
        # S'assurer que la plage minimale est de 10 mètres
        if x_max - x_min < 10.0:
            # Centrer la plage autour du milieu
            center = (x_min_raw + x_max_raw) / 2.0
            x_min = np.floor((center - 5.0) / 10.0) * 10.0
            x_max = np.ceil((center + 5.0) / 10.0) * 10.0
            # Garantir au moins 10 mètres
            if x_max - x_min < 10.0:
                x_max = x_min + 10.0
    
    fig.add_hline(y=0, line_dash="dash", line_color="cyan", 
                  annotation_text="Surface de l'eau")
    
    # Configuration de l'axe Y
    # Convention : y < 0 = profondeur (sous la surface), y = 0 = surface
    # Orientation normale : y_max (surface, 0) en haut, y_min (profondeur, négatif) en bas
    # Utiliser range=[y_min, y_max] (ordre normal) - pas d'inversion
    yaxis_config = {}
    
    if len(y_values) > 0:
        y_min_raw = min(y_values)  # Valeur la plus négative (profondeur maximale)
        y_max_raw = max(y_values)  # Valeur la plus positive (surface = 0 ou proche)
        
        # Si toutes les valeurs sont à 0 (graphique initial), utiliser la plage par défaut
        if y_min_raw == 0 and y_max_raw == 0:
            y_min_default = -50.0  # Profondeur par défaut (négative)
            y_max_default = 0.0    # Surface par défaut (y = 0)
            # Utiliser range=[y_min, y_max] (ordre normal) - pas d'inversion
            # Cela affiche y_min (profondeur) en bas et y_max (surface) en haut
            yaxis_config['range'] = [y_min_default, y_max_default]  # [y_min, y_max] ordre normal
            yaxis_config['autorange'] = False  # Désactiver autorange
            y_range_data = [y_max_default, y_min_default]  # Pour le calcul du bateau
        else:
            # Utiliser la plage fournie ou calculer une nouvelle plage
            if y_range is not None:
                y_max, y_min = y_range  # Note: y_range est [y_max, y_min] car reversed
                # Vérifier si les données sortent de la plage actuelle
                if y_min_raw < y_min or y_max_raw > y_max:
                    # Recalculer la plage avec arrondi par paliers de 10 mètres
                    y_min = np.floor(y_min_raw / 10.0) * 10.0
                    y_max = np.ceil(y_max_raw / 10.0) * 10.0
                    # S'assurer que la plage minimale est de 10 mètres
                    if y_max - y_min < 10.0:
                        center = (y_min_raw + y_max_raw) / 2.0
                        y_min = np.floor((center - 5.0) / 10.0) * 10.0
                        y_max = np.ceil((center + 5.0) / 10.0) * 10.0
                        if y_max - y_min < 10.0:
                            y_max = y_min + 10.0
            else:
                # Arrondir les limites de l'axe Y par paliers de 10 mètres
                # Arrondir vers le bas pour y_min (profondeur), vers le haut pour y_max (surface)
                y_min = np.floor(y_min_raw / 10.0) * 10.0
                y_max = np.ceil(y_max_raw / 10.0) * 10.0
                
                # S'assurer que la plage minimale est de 10 mètres
                if y_max - y_min < 10.0:
                    # Centrer la plage autour du milieu
                    center = (y_min_raw + y_max_raw) / 2.0
                    y_min = np.floor((center - 5.0) / 10.0) * 10.0
                    y_max = np.ceil((center + 5.0) / 10.0) * 10.0
                    # Garantir au moins 10 mètres
                    if y_max - y_min < 10.0:
                        y_max = y_min + 10.0
            
            # Vérifier que les valeurs sont bien des multiples de 10 (arrondi final)
            y_min = np.floor(y_min / 10.0) * 10.0
            y_max = np.ceil(y_max / 10.0) * 10.0
            
            # Utiliser range=[y_min, y_max] (ordre normal) - pas d'inversion
            # Cela affiche y_min (profondeur) en bas et y_max (surface) en haut
            # On veut : surface (y_max ≈ 0) en haut, profondeur (y_min < 0) en bas
            yaxis_config['range'] = [y_min, y_max]  # [y_min, y_max] ordre normal
            yaxis_config['autorange'] = False  # Désactiver autorange
            y_range_data = [y_max, y_min]  # Pour le calcul du bateau
    else:
        # Plage par défaut : surface en haut, profondeur en bas
        # Arrondir par paliers de 10 mètres même pour la plage par défaut
        y_min_default = -50.0  # Profondeur par défaut (négative)
        y_max_default = 0.0    # Surface par défaut (y = 0)
        # Utiliser range=[y_min, y_max] (ordre normal) - pas d'inversion
        # Cela affiche y_min (profondeur) en bas et y_max (surface) en haut
        yaxis_config['range'] = [y_min_default, y_max_default]  # [y_min, y_max] ordre normal
        yaxis_config['autorange'] = False  # Désactiver autorange
        y_range_data = [y_max_default, y_min_default]  # Pour le calcul du bateau
    
    fig.update_layout(
        xaxis_title="Position horizontale (m)",
        yaxis_title="Profondeur (m) - Surface en haut",
        xaxis=dict(range=[x_min, x_max]),
        yaxis=yaxis_config,
        title=title,
        hovermode='closest',
        showlegend=True
        # width et height seront définis dans plotly_widget.py selon la taille du widget
    )
    
    # Ajouter une annotation avec une forme SVG pour le bateau avec taille constante en pixels
    # Convertir la position du bateau en coordonnées normalisées (paper coordinates)
    if y_range_data is not None:
        # Calculer les coordonnées normalisées (0-1) pour la position du bateau
        # Le bateau est à y = 0 (surface)
        # y_range_data = [y_max, y_min] où y_max = 0 (surface) et y_min < 0 (profondeur)
        # Avec range=[y_min, y_max] (ordre normal), y_min est en bas (y_norm = 0) et y_max en haut (y_norm = 1)
        # Pour y = 0 (y_max), on veut y_norm = 1.0 (en haut)
        x_norm = (x_bateau - x_min) / (x_max - x_min) if x_max != x_min else 0.5
        y_max_range, y_min_range = y_range_data  # [y_max, y_min]
        # Le bateau est à y = 0 = y_max_range
        # Calcul normal : y_norm = (y - y_min) / (y_max - y_min)
        # Pour y = y_max_range = 0, y_norm = (0 - y_min_range) / (y_max_range - y_min_range) = (-y_min_range) / (y_max_range - y_min_range)
        # Comme y_max_range = 0 et y_min_range < 0, y_norm = (-y_min_range) / (0 - y_min_range) = (-y_min_range) / (-y_min_range) = 1.0
        y_norm = (0.0 - y_min_range) / (y_max_range - y_min_range) if y_max_range != y_min_range else 1.0
        
        # Taille du carré en pixels (réduite de 25% : 20 -> 15)
        boat_size_pixels = 15
        # Convertir la taille en pixels en coordonnées normalisées (approximation)
        # En supposant une largeur de graphique de 800 pixels
        x_size_norm = boat_size_pixels / 800.0
        y_size_norm = boat_size_pixels / 600.0
        
        # Ajouter une annotation avec une forme rectangulaire
        fig.add_shape(
            type="rect",
            xref="paper", yref="paper",
            x0=x_norm - x_size_norm/2, x1=x_norm + x_size_norm/2,
            y0=y_norm - y_size_norm/2, y1=y_norm + y_size_norm/2,
            fillcolor='#8B4513',
            line=dict(color='#654321', width=2),
            layer="above"
        )
    
    return fig


def create_current_profile_plot(depths, speeds, title="Profil du courant"):
    """
    Crée un graphique du profil de courant en fonction de la profondeur.
    
    Parameters:
    -----------
    depths : array
        Profondeurs (m) (convention : y < 0 = profondeur)
    speeds : array
        Vitesses du courant (m/s)
    title : str
        Titre du graphique
    
    Returns:
    --------
    go.Figure
        Figure Plotly avec vitesse en abscisse (horizontal) et profondeur en ordonnée (vertical, orienté vers le haut)
    """
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=speeds,  # Abscisse : vitesse du courant (axe horizontal)
        y=depths,  # Ordonnée : profondeur (axe vertical)
        mode='lines+markers',
        name='Courant',
        line=dict(color='teal', width=3),
        marker=dict(size=4, color='teal')
    ))
    
    # Calculer les limites pour l'axe Y (profondeur) - orientation normale (valeurs positives en haut)
    depth_min = min(depths) if depths else 0.0
    depth_max = max(depths) if depths else 0.0
    # Ordre normal : valeurs négatives en bas, valeurs positives en haut
    y_range = [depth_min, depth_max] if depth_max > depth_min else None
    
    # Échelle fixe pour l'axe X (vitesse) : de -3 à +3 m/s
    x_range = [-3.0, 3.0]
    
    fig.update_layout(
        title=title,
        xaxis_title="Vitesse du courant (m/s)",  # Abscisse : axe horizontal
        yaxis_title="Profondeur (m)",  # Ordonnée : axe vertical
        xaxis=dict(
            range=x_range,  # Échelle fixe : -3 à +4 m/s
            showticklabels=True,  # Afficher les valeurs numériques sur l'axe X
            ticks="outside",  # Afficher les ticks à l'extérieur
            showline=True,  # Afficher la ligne de l'axe
        ),
        yaxis=dict(
            range=y_range,  # Orientation normale : valeurs positives en haut
        ),
        margin=dict(l=40, r=20, t=50, b=40),
        showlegend=False
    )
    
    return fig


def create_position_plot(time, x_rov, y_rov, title="Positions du ROV"):
    """
    Crée un graphique des positions du ROV
    
    Parameters:
    -----------
    time : array
        Temps (s)
    x_rov, y_rov : array
        Positions du ROV (m)
    title : str
        Titre du graphique
    
    Returns:
    --------
    go.Figure
        Figure Plotly
    """
    fig = go.Figure()
    
    fig.add_trace(go.Scatter(
        x=time, y=x_rov,
        mode='lines',
        name='Position horizontale',
        line=dict(color='blue', width=2)
    ))
    
    fig.add_trace(go.Scatter(
        x=time, y=y_rov,
        mode='lines',
        name='Profondeur',
        line=dict(color='red', width=2)
    ))
    
    fig.update_layout(
        xaxis_title="Temps (s)",
        yaxis_title="Position (m)",
        title=title,
        hovermode='x unified',
        width=600,
        height=400
    )
    
    return fig


def create_velocity_plot(
    time,
    vx_rov,
    vy_rov,
    v_rov=None,
    title="Vitesses du ROV",
    scenario_triggers: list[list[str]] | None = None,
):
    """
    Crée un graphique des vitesses du ROV
    
    Parameters:
    -----------
    time : array
        Temps (s)
    vx_rov, vy_rov : array
        Composantes de vitesse du ROV (m/s)
    v_rov : array, optional
        Module de la vitesse du ROV (m/s). Si None, est calculé avec sqrt(vx^2 + vy^2).
    title : str
        Titre du graphique
    
    Returns:
    --------
    go.Figure
        Figure Plotly
    """
    fig = go.Figure()
    
    vx_arr = np.asarray(vx_rov, dtype=float) if vx_rov is not None else np.asarray([], dtype=float)
    vy_arr = np.asarray(vy_rov, dtype=float) if vy_rov is not None else np.asarray([], dtype=float)
    n = min(len(time), len(vx_arr), len(vy_arr))
    time_arr = np.asarray(time[:n], dtype=float) if n > 0 else np.asarray([], dtype=float)
    vx_arr = vx_arr[:n]
    vy_arr = vy_arr[:n]
    if v_rov is None:
        v_arr = np.hypot(vx_arr, vy_arr)
    else:
        v_arr = np.asarray(v_rov, dtype=float)[:n]

    fig.add_trace(go.Scatter(
        x=time_arr, y=vx_arr,
        mode='lines',
        name='vx_rov',
        line=dict(color='blue', width=2)
    ))
    
    fig.add_trace(go.Scatter(
        x=time_arr, y=vy_arr,
        mode='lines',
        name='vy_rov',
        line=dict(color='red', width=2)
    ))

    fig.add_trace(go.Scatter(
        x=time_arr, y=v_arr,
        mode='lines',
        name='vrov',
        line=dict(color='green', width=2)
    ))

    max_abs = 0.0
    if v_arr.size > 0:
        finite = v_arr[np.isfinite(v_arr)]
        if finite.size > 0:
            max_abs = max(max_abs, float(np.max(np.abs(finite))))
    if vx_arr.size > 0:
        finite = vx_arr[np.isfinite(vx_arr)]
        if finite.size > 0:
            max_abs = max(max_abs, float(np.max(np.abs(finite))))
    if vy_arr.size > 0:
        finite = vy_arr[np.isfinite(vy_arr)]
        if finite.size > 0:
            max_abs = max(max_abs, float(np.max(np.abs(finite))))
    y_events = max_abs * 0.98 if max_abs > 0.0 else 0.0

    if scenario_triggers:
        n_trig = min(len(time_arr), len(scenario_triggers))
        trig_x = []
        trig_y = []
        trig_text = []
        for i in range(n_trig):
            triggers = scenario_triggers[i]
            if triggers:
                trig_x.append(float(time_arr[i]))
                trig_y.append(y_events)
                trig_text.append("<br>".join(triggers))
        if trig_x:
            fig.add_trace(
                go.Scatter(
                    x=trig_x,
                    y=trig_y,
                    mode="markers",
                    name="",
                    showlegend=False,
                    marker=dict(color="black", size=6, symbol="square"),
                    hovertemplate="%{hovertext}<extra></extra>",
                    hovertext=trig_text,
                )
            )
    
    fig.add_hline(y=0, line_dash="dash", line_color="gray")
    
    fig.update_layout(
        xaxis_title="Temps (s)",
        yaxis_title="Vitesse (m/s)",
        title=title,
        hovermode='x unified',
        width=600,
        height=400,
        **_LAYOUT_LEGEND_TOP,
    )
    
    return fig


def create_position_rov_plot(
    time,
    x_rov,
    y_rov,
    title="Position ROV",
    scenario_triggers: list[list[str]] | None = None,
):
    """
    Crée un graphique des positions du ROV.

    - x_rov : position horizontale (m)
    - y_rov : profondeur / ordonnée (m), convention habituelle y ≤ 0 sous la surface
    """
    fig = go.Figure()

    x_arr = np.asarray(x_rov, dtype=float) if x_rov is not None else np.asarray([], dtype=float)
    y_arr = np.asarray(y_rov, dtype=float) if y_rov is not None else np.asarray([], dtype=float)
    n = min(len(time), len(x_arr), len(y_arr))
    time_arr = np.asarray(time[:n], dtype=float) if n > 0 else np.asarray([], dtype=float)
    x_arr = x_arr[:n]
    y_arr = y_arr[:n]

    fig.add_trace(
        go.Scatter(
            x=time_arr,
            y=x_arr,
            mode="lines",
            name="x_rov",
            line=dict(color="blue", width=2),
        )
    )
    fig.add_trace(
        go.Scatter(
            x=time_arr,
            y=y_arr,
            mode="lines",
            name="y_rov",
            line=dict(color="red", width=2),
        )
    )

    max_abs = 0.0
    if x_arr.size > 0:
        finite = x_arr[np.isfinite(x_arr)]
        if finite.size > 0:
            max_abs = max(max_abs, float(np.max(np.abs(finite))))
    if y_arr.size > 0:
        finite = y_arr[np.isfinite(y_arr)]
        if finite.size > 0:
            max_abs = max(max_abs, float(np.max(np.abs(finite))))
    y_events = max_abs * 0.98 if max_abs > 0.0 else 0.0

    if scenario_triggers:
        n_trig = min(len(time_arr), len(scenario_triggers))
        trig_x = []
        trig_y = []
        trig_text = []
        for i in range(n_trig):
            triggers = scenario_triggers[i]
            if triggers:
                trig_x.append(float(time_arr[i]))
                trig_y.append(y_events)
                trig_text.append("<br>".join(triggers))
        if trig_x:
            fig.add_trace(
                go.Scatter(
                    x=trig_x,
                    y=trig_y,
                    mode="markers",
                    name="",
                    showlegend=False,
                    marker=dict(color="black", size=6, symbol="square"),
                    hovertemplate="%{hovertext}<extra></extra>",
                    hovertext=trig_text,
                )
            )

    fig.add_hline(y=0, line_dash="dash", line_color="gray")
    fig.update_layout(
        xaxis_title="Temps (s)",
        yaxis_title="x_rov, y_rov (m)",
        title=title,
        hovermode="x unified",
        width=600,
        height=400,
        **_LAYOUT_LEGEND_TOP,
    )
    return fig


def create_tension_plot(time, T, title="Tension du câble"):
    """
    Crée un graphique de la tension du câble
    
    Parameters:
    -----------
    time : array
        Temps (s)
    T : array
        Tensions (N)
    title : str
        Titre du graphique
    
    Returns:
    --------
    go.Figure
        Figure Plotly
    """
    fig = go.Figure()
    
    if T.ndim == 2:
        # Plusieurs points le long du câble
        for i in range(0, T.shape[1], max(1, T.shape[1] // 10)):
            fig.add_trace(go.Scatter(
                x=time, y=T[:, i],
                mode='lines',
                name=f'Tension point {i}',
                line=dict(width=1)
            ))
    else:
        # Une seule courbe
        fig.add_trace(go.Scatter(
            x=time, y=T,
            mode='lines',
            name='Tension',
            line=dict(color='purple', width=2)
        ))
    
    fig.update_layout(
        xaxis_title="Temps (s)",
        yaxis_title="Tension (N)",
        title=title,
        hovermode='x unified',
        width=600,
        height=400
    )
    
    return fig


def create_dl_dt_plot(
    time,
    dl_dt,
    title="Commande dL/dt",
    cable_mode: list[str] | None = None,
    scenario_triggers: list[list[str]] | None = None,
):
    """
    Crée un graphique de la commande dL/dt en fonction du temps.
    """
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=time,
        y=dl_dt,
        mode='lines',
        name='dL/dt',
        line=dict(color='red', width=2)
    ))
    max_time = max(time) if len(time) > 0 else 0.0
    k = max(1, int(np.ceil(max_time / 30.0)))
    x_max = 30.0 * k
    max_val = max([abs(v) for v in dl_dt], default=0.0)
    n = max(1, int(np.ceil(max_val)))
    # Limite verticale utilisée pour l'axe Y
    y_range_max = float(n) * 1.05
    # Position des marqueurs d'événements : juste sous le bord supérieur de l'axe
    y_events = y_range_max * 0.98

    # On ne trace plus la ligne horizontale en haut du diagramme pour le mode câble
    # afin de ne laisser visibles que les marqueurs d'événements.

    if scenario_triggers:
        n_trig = min(len(time), len(scenario_triggers))
        trig_x = []
        trig_y = []
        trig_text = []
        for i in range(n_trig):
            triggers = scenario_triggers[i]
            if triggers:
                trig_x.append(time[i])
                trig_y.append(y_events)
                trig_text.append("<br>".join(triggers))
        if trig_x:
            fig.add_trace(
                go.Scatter(
                    x=trig_x,
                    y=trig_y,
                    mode="markers",
                    name="",
                    showlegend=False,
                    marker=dict(color="black", size=6, symbol="square"),
                    hovertemplate="%{hovertext}<extra></extra>",
                    hovertext=trig_text,
                )
            )

    fig.update_layout(
        xaxis_title="Temps (s)",
        yaxis_title="dL/dt (m/s)",
        xaxis=dict(range=[0.0, x_max]),
        yaxis=dict(range=[-y_range_max, y_range_max]),
        title=title,
        hovermode='x unified',
        width=600,
        height=400,
        **_LAYOUT_LEGEND_TOP,
    )
    return fig


def create_slack_plot(
    time,
    slack,
    slack_ratio,
    hover_data=None,
        title="Slack / L  (%)",
):
    """
    Crée un graphique du slack (L - L_straight) et du ratio Slack/L.
    """
    fig = go.Figure()
    hover_data = hover_data or []
    fig.add_trace(go.Scatter(
        x=time,
        y=[100.0 * float(val) for val in slack_ratio],
        mode='lines',
        name='Slack/L',
        line=dict(color='orange', width=2, dash='dot'),
        yaxis='y2',
        customdata=hover_data,
        hovertemplate=(
            "t=%{x:.2f}s<br>"
            "Slack/L=%{y:.2f} %<br>"
            "x_rov=%{customdata[0]:.2f}<br>"
            "y_rov=%{customdata[1]:.2f}<br>"
            "T_bat=%{customdata[2]:.2f} N<br>"
            "dL/dt=%{customdata[3]:.2f} m/s"
            "<extra></extra>"
        ),
    ))

    fig.update_layout(
        xaxis_title="Temps (s)",
        yaxis=dict(title="Slack/L (%)"),
        title=title,
        hovermode='x unified',
        width=600,
        height=400,
        **_LAYOUT_LEGEND_TOP,
    )

    return fig


def create_slack_distribution_plot(s_m, slack_local, title="Répartition slack"):
    """
    Crée un graphique de répartition locale du slack le long du câble.
    """
    fig = go.Figure()
    slack_local_mm = [1000.0 * float(val) for val in slack_local]
    fig.add_trace(go.Scatter(
        x=s_m,
        y=slack_local_mm,
        mode='lines',
        name='Slack local',
        line=dict(color='orange', width=2),
        hovertemplate="s=%{x:.2f} m<br>Slack local=%{y:.4f} mm<extra></extra>",
    ))
    fig.update_layout(
        xaxis_title="s (m)",
        yaxis_title="Slack local (mm)",
        yaxis=dict(tickformat=".3f"),
        title=title,
        hovermode='x unified',
        width=600,
        height=400
    )
    return fig


def create_tension_vs_target_plot(
    time,
    t_rupture,
    t_cible,
    t_boat,
    t_rov,
    t_max,
    title="Tension vs cible",
    cable_mode: list[str] | None = None,
    scenario_triggers: list[list[str]] | None = None,
):
    fig = go.Figure()
    hover_tpl = "%{y:.2f} N<extra></extra>"
    fig.add_trace(go.Scatter(x=time, y=t_rupture, mode="lines", name="Tension rupture", hovertemplate=hover_tpl))
    fig.add_trace(go.Scatter(x=time, y=t_cible, mode="lines", name="Tension cible", hovertemplate=hover_tpl))
    fig.add_trace(go.Scatter(x=time, y=t_boat, mode="lines", name="Tension bateau", hovertemplate=hover_tpl))
    fig.add_trace(go.Scatter(x=time, y=t_rov, mode="lines", name="Tension ROV", hovertemplate=hover_tpl))
    fig.add_trace(
        go.Scatter(
            x=time,
            y=t_max,
            mode="lines",
            name="Tension max",
            line=dict(dash="dash"),
            hovertemplate=hover_tpl,
        )
    )

    def _safe_max(series):
        if series is None:
            return 0.0
        vals = []
        for val in series:
            if isinstance(val, (int, float, np.floating)) and np.isfinite(val):
                vals.append(float(val))
        return max(vals) if vals else 0.0

    max_time = _safe_max(time)
    k = max(1, int(np.ceil(max_time / 30.0)))
    x_max = 30.0 * k
    max_val = max(
        _safe_max(t_rupture),
        _safe_max(t_cible),
        _safe_max(t_boat),
        _safe_max(t_rov),
        _safe_max(t_max),
    )
    # Position des marqueurs d'événements : juste sous le maximum des courbes
    y_events = float(max_val) * 0.98 if max_val > 0.0 else 0.0

    # On ne trace plus la ligne horizontale en haut du diagramme pour le mode câble
    # afin de ne laisser visibles que les marqueurs d'événements.

    if scenario_triggers:
        n_trig = min(len(time), len(scenario_triggers))
        trig_x = []
        trig_y = []
        trig_text = []
        for i in range(n_trig):
            triggers = scenario_triggers[i]
            if triggers:
                trig_x.append(time[i])
                trig_y.append(y_events)
                trig_text.append("<br>".join(triggers))
        if trig_x:
            fig.add_trace(
                go.Scatter(
                    x=trig_x,
                    y=trig_y,
                    mode="markers",
                    name="",
                    showlegend=False,
                    marker=dict(color="black", size=6, symbol="square"),
                    hovertemplate="%{hovertext}<extra></extra>",
                    hovertext=trig_text,
                )
            )

    fig.update_layout(
        xaxis=dict(
            title="Temps (s)",
            showticklabels=True,
            ticks="outside",
            showline=True,
            automargin=True,
            range=[0.0, x_max],
        ),
        yaxis_title="Tension (N)",
        title=title,
        hovermode="x unified",
        width=600,
        height=400,
        **_LAYOUT_LEGEND_TOP,
    )
    return fig


def create_tension_curvilinear_plot(s_curvilinear, T, title="Tension", 
                                    s_range=None, T_range=None, Tx=None, Ty=None,
                                    x_cable=None, y_cable=None):
    """
    Crée un graphique de la tension en fonction de l'abscisse curviligne
    
    Parameters:
    -----------
    s_curvilinear : array
        Abscisse curviligne le long du câble (m) - depuis le bateau (s=0) jusqu'au ROV (s=L)
    T : array
        Tensions totales le long du câble (N)
    title : str
        Titre du graphique
    s_range : list, optional
        Plage de l'axe S (abscisse curviligne) [s_min, s_max] à utiliser. Si None, calcule automatiquement.
    T_range : list, optional
        Plage de l'axe T (tension) [T_min, T_max] à utiliser. Si None, calcule automatiquement.
    Tx : array, optional
        Composantes horizontales de la tension (N)
    Ty : array, optional
        Composantes verticales de la tension (N)
    x_cable : array, optional
        Coordonnées X des points du câble (m)
    y_cable : array, optional
        Coordonnées Y des points du câble (m)
    
    Returns:
    --------
    go.Figure
        Figure Plotly avec s en abscisse (axe X) et T en ordonnée (axe Y)
    """
    fig = go.Figure()
    
    if len(s_curvilinear) > 0 and len(T) > 0:
        # Convertir en arrays numpy si nécessaire
        s_arr = np.asarray(s_curvilinear)
        T_arr = np.asarray(T)
        
        # Préparer les coordonnées X et Y si disponibles
        x_arr = None
        y_arr = None
        if x_cable is not None and y_cable is not None and len(x_cable) == len(s_arr) and len(y_cable) == len(s_arr):
            x_arr = np.asarray(x_cable)
            y_arr = np.asarray(y_cable)
        
        if x_arr is not None and y_arr is not None:
            hover_texts_T = cable_markers_along_s_hover_texts(
                s_arr, x_arr=x_arr, y_arr=y_arr
            )
        else:
            hover_texts_T = cable_markers_along_s_hover_texts(s_arr)
        for i in range(len(hover_texts_T)):
            hover_texts_T[i] += f"<br>T: {float(T_arr[i]):.2f} N"

        # Tracer la tension totale
        trace_params_T = {
            'x': s_arr,
            'y': T_arr,
            'mode': 'lines+markers',
            'name': 'T',
            'line': dict(color='purple', width=2),
            'marker': dict(size=3, color='purple', opacity=0.5, symbol='circle'),
            'showlegend': True,
            'hovertext': hover_texts_T,
            'hovertemplate': CABLE_XY_HOVERTEMPLATE,
            'hoverinfo': 'text',
        }
        fig.add_trace(go.Scatter(**trace_params_T))

        # Tracer Tx si fourni
        if Tx is not None and len(Tx) > 0:
            Tx_arr = np.asarray(Tx)
            if x_arr is not None and y_arr is not None:
                hover_texts_Tx = cable_markers_along_s_hover_texts(
                    s_arr, x_arr=x_arr, y_arr=y_arr
                )
            else:
                hover_texts_Tx = cable_markers_along_s_hover_texts(s_arr)
            for i in range(len(hover_texts_Tx)):
                hover_texts_Tx[i] += f"<br>Tx: {float(Tx_arr[i]):.2f} N"
            trace_params_Tx = {
                'x': s_arr,
                'y': Tx_arr,
                'mode': 'lines+markers',
                'name': 'Tx',
                'line': dict(color='blue', width=2),
                'marker': dict(size=3, color='blue', opacity=0.5, symbol='circle'),
                'showlegend': True,
                'hovertext': hover_texts_Tx,
                'hovertemplate': CABLE_XY_HOVERTEMPLATE,
                'hoverinfo': 'text',
            }
            fig.add_trace(go.Scatter(**trace_params_Tx))

        # Tracer Ty si fourni
        if Ty is not None and len(Ty) > 0:
            Ty_arr = np.asarray(Ty)
            if x_arr is not None and y_arr is not None:
                hover_texts_Ty = cable_markers_along_s_hover_texts(
                    s_arr, x_arr=x_arr, y_arr=y_arr
                )
            else:
                hover_texts_Ty = cable_markers_along_s_hover_texts(s_arr)
            for i in range(len(hover_texts_Ty)):
                hover_texts_Ty[i] += f"<br>Ty: {float(Ty_arr[i]):.2f} N"
            trace_params_Ty = {
                'x': s_arr,
                'y': Ty_arr,
                'mode': 'lines+markers',
                'name': 'Ty',
                'line': dict(color='red', width=2),
                'marker': dict(size=3, color='red', opacity=0.5, symbol='circle'),
                'showlegend': True,
                'hovertext': hover_texts_Ty,
                'hovertemplate': CABLE_XY_HOVERTEMPLATE,
                'hoverinfo': 'text',
            }
            fig.add_trace(go.Scatter(**trace_params_Ty))
    
    # Configuration des axes
    # Abscisse curviligne en abscisse (axe X) avec paliers de 10 m
    # Tension en ordonnée (axe Y) avec paliers de 10 N
    if len(s_curvilinear) > 0:
        s_min_raw = np.min(s_curvilinear)
        s_max_raw = np.max(s_curvilinear)
        
        if s_range is not None:
            s_min, s_max = s_range
            # Vérifier si les données sortent de la plage actuelle
            if s_min_raw < s_min or s_max_raw > s_max:
                # Recalculer la plage avec arrondi par paliers de 10 mètres
                s_min = np.floor(s_min_raw / 10.0) * 10.0
                s_max = np.ceil(s_max_raw / 10.0) * 10.0
                # S'assurer que la plage minimale est de 10 mètres
                if s_max - s_min < 10.0:
                    center = (s_min_raw + s_max_raw) / 2.0
                    s_min = np.floor((center - 5.0) / 10.0) * 10.0
                    s_max = np.ceil((center + 5.0) / 10.0) * 10.0
                    if s_max - s_min < 10.0:
                        s_max = s_min + 10.0
        else:
            # Arrondir les limites de l'axe S par paliers de 10 mètres
            s_min = np.floor(s_min_raw / 10.0) * 10.0
            s_max = np.ceil(s_max_raw / 10.0) * 10.0
            
            # S'assurer que la plage minimale est de 10 mètres
            if s_max - s_min < 10.0:
                center = (s_min_raw + s_max_raw) / 2.0
                s_min = np.floor((center - 5.0) / 10.0) * 10.0
                s_max = np.ceil((center + 5.0) / 10.0) * 10.0
                if s_max - s_min < 10.0:
                    s_max = s_min + 10.0
    else:
        # Plage par défaut
        s_min = 0.0
        s_max = 50.0
    
    if len(T) > 0:
        # Calculer les valeurs min/max en tenant compte de T, Tx et Ty
        all_values = [T]
        if Tx is not None and len(Tx) > 0:
            all_values.append(Tx)
        if Ty is not None and len(Ty) > 0:
            all_values.append(Ty)
        
        all_values_flat = np.concatenate([np.asarray(v).ravel() for v in all_values])
        finite_values = all_values_flat[np.isfinite(all_values_flat)]
        if finite_values.size == 0:
            T_min_raw, T_max_raw = 0.0, 100.0
        else:
            T_min_raw = float(np.min(finite_values))
            T_max_raw = float(np.max(finite_values))
        
        if T_range is not None:
            T_min, T_max = T_range
            # Vérifier si les données sortent de la plage actuelle
            if T_min_raw < T_min or T_max_raw > T_max:
                # Recalculer la plage avec arrondi par paliers de 10 N
                T_min = np.floor(T_min_raw / 10.0) * 10.0
                T_max = np.ceil(T_max_raw / 10.0) * 10.0
                # S'assurer que la plage minimale est de 10 N
                if T_max - T_min < 10.0:
                    center = (T_min_raw + T_max_raw) / 2.0
                    T_min = np.floor((center - 5.0) / 10.0) * 10.0
                    T_max = np.ceil((center + 5.0) / 10.0) * 10.0
                    if T_max - T_min < 10.0:
                        T_max = T_min + 10.0
        else:
            # Arrondir les limites de l'axe T par paliers de 10 N
            T_min = np.floor(T_min_raw / 10.0) * 10.0
            T_max = np.ceil(T_max_raw / 10.0) * 10.0
            
            # S'assurer que la plage minimale est de 10 N
            if T_max - T_min < 10.0:
                center = (T_min_raw + T_max_raw) / 2.0
                T_min = np.floor((center - 5.0) / 10.0) * 10.0
                T_max = np.ceil((center + 5.0) / 10.0) * 10.0
                if T_max - T_min < 10.0:
                    T_max = T_min + 10.0
    else:
        # Plage par défaut
        T_min = 0.0
        T_max = 100.0
    
    fig.update_layout(
        xaxis_title="Abscisse curviligne (m)",
        yaxis_title="Tension (N)",
        xaxis=dict(range=[s_min, s_max], autorange=False),
        yaxis=dict(range=[T_min, T_max], tick0=0, dtick=10, tickformat=".0f"),
        title=title,
        hovermode='closest',
        showlegend=True
    )
    
    return fig


def create_rov_local_plot(
    x_rov,
    y_rov,
    x_cable,
    y_cable,
    T_cable=None,
    s_cable=None,
    point_indices=None,
    title="Profil fond",
    margin=2.0,
):
    """
    Crée un graphique local autour du ROV avec une portion du câble proche du ROV.
    Les axes x/y sont à la même échelle pour conserver les proportions.
    """
    fig = go.Figure()

    x_vals = list(x_cable) if x_cable is not None else []
    y_vals = list(y_cable) if y_cable is not None else []
    if point_indices is not None:
        point_nums = [int(point_indices[i]) for i in range(len(x_vals))]
    else:
        point_nums = list(range(1, len(x_vals) + 1))

    if x_vals and y_vals:
        trace_params = {
            'x': x_vals,
            'y': y_vals,
            'mode': 'lines+markers',
            'name': 'Câble (proche ROV)',
            'line': dict(color='#1e6bb8', width=2),
            'marker': dict(size=4, color='#1e6bb8', opacity=0.7, symbol='circle'),
        }

        if s_cable is not None and len(s_cable) == len(x_vals):
            s_values = np.asarray(s_cable, dtype=float)
            hover_texts = cable_markers_along_s_hover_texts(
                s_values, x_arr=x_vals, y_arr=y_vals, point_numbers=point_nums
            )
        else:
            hover_texts = cable_polyline_hover_texts(x_vals, y_vals, point_numbers=point_nums)

        if T_cable is not None and len(T_cable) == len(x_vals):
            for i in range(len(hover_texts)):
                hover_texts[i] += f"<br>Tension: {float(T_cable[i]):.2f} N"
        trace_params['hovertext'] = hover_texts
        trace_params['hovertemplate'] = CABLE_XY_HOVERTEMPLATE
        trace_params['hoverinfo'] = 'text'

        fig.add_trace(go.Scatter(**trace_params))

    fig.add_trace(go.Scatter(
        x=[x_rov],
        y=[y_rov],
        mode='markers',
        name='ROV',
        marker=dict(color='red', size=8),
    ))

    x_min = min(x_vals + [x_rov]) if x_vals or x_rov is not None else -1.0
    x_max = max(x_vals + [x_rov]) if x_vals or x_rov is not None else 1.0
    y_min = min(y_vals + [y_rov]) if y_vals or y_rov is not None else -1.0
    y_max = max(y_vals + [y_rov]) if y_vals or y_rov is not None else 1.0

    x_min -= margin
    x_max += margin
    y_min -= margin
    y_max += margin

    fig.update_layout(
        xaxis=dict(
            title="x (m)",
            range=[x_min, x_max],
            scaleanchor="y",
            scaleratio=1,
        ),
        yaxis=dict(
            title="y (m)",
            range=[y_min, y_max],
        ),
        title=title,
        hovermode='closest',
        width=600,
        height=400,
        showlegend=False,
    )

    return fig

