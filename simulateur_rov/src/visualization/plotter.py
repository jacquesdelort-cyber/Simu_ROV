"""Fonctions de visualisation avec Plotly"""
import plotly.graph_objects as go
import numpy as np


def create_system_plot(x_rov, y_rov, x_cable, y_cable, x_bateau, L=0, title="Système ROV-Câble-Bateau"):
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
    
    Returns:
    --------
    go.Figure
        Figure Plotly
    """
    fig = go.Figure()
    
    # Câble (ligne courbe)
    fig.add_trace(go.Scatter(
        x=x_cable, y=y_cable,
        mode='lines+markers',
        name='Câble',
        line=dict(color='blue', width=2),
        marker=dict(size=3, color='blue')
    ))
    
    # ROV (marqueur)
    fig.add_trace(go.Scatter(
        x=[x_rov], y=[y_rov],
        mode='markers',
        name='ROV',
        marker=dict(size=20, color='red', symbol='square',
                   line=dict(width=2, color='darkred'))
    ))
    
    # Bateau - sera ajouté comme forme avec taille constante en pixels après le layout
    # Ajouter un marqueur invisible pour la légende et le hover
    fig.add_trace(go.Scatter(
        x=[x_bateau], y=[0],
        mode='markers',
        name='Bateau',
        marker=dict(size=1, opacity=0),  # Invisible mais présent pour la légende
        showlegend=True,
        hovertemplate='Bateau<br>Position: %{x:.2f} m<extra></extra>'
    ))
    
    # Surface de l'eau
    if len(x_cable) > 0:
        x_min = min(np.min(x_cable), x_rov - 10, x_bateau - 10)
        x_max = max(np.max(x_cable), x_rov + 10, x_bateau + 10)
    else:
        x_min = min(x_rov - 10, x_bateau - 10)
        x_max = max(x_rov + 10, x_bateau + 10)
    
    fig.add_hline(y=0, line_dash="dash", line_color="cyan", 
                  annotation_text="Surface de l'eau")
    
    # Configuration de l'axe
    # Calculer la plage Y pour forcer l'inversion (valeurs négatives vers le bas)
    y_values = list(y_cable) + [y_rov, 0]  # Inclure le ROV, le câble et la surface
    yaxis_config = {'autorange': 'reversed'}  # Profondeur vers le bas
    
    if len(y_values) > 0:
        y_min = min(y_values)
        y_max = max(y_values)
        # Ajouter une marge
        y_margin = (y_max - y_min) * 0.1 if y_max != y_min else 5.0
        # Inverser la plage : max (moins négatif, moins profond) en haut, min (plus négatif, plus profond) en bas
        yaxis_config['range'] = [y_max + y_margin, y_min - y_margin]
        y_range_data = [y_max + y_margin, y_min - y_margin]
    else:
        y_range_data = None
    
    fig.update_layout(
        xaxis_title="Position horizontale (m)",
        yaxis_title="Profondeur (m)",
        yaxis=yaxis_config,
        title=title,
        hovermode='closest',
        width=800,
        height=600,
        showlegend=True
    )
    
    # Ajouter une annotation avec une forme SVG pour le bateau avec taille constante en pixels
    # Convertir la position du bateau en coordonnées normalisées (paper coordinates)
    if y_range_data is not None:
        # Calculer les coordonnées normalisées (0-1) pour la position du bateau
        # Utiliser les valeurs calculées x_min et x_max
        x_norm = (x_bateau - x_min) / (x_max - x_min) if x_max != x_min else 0.5
        y_norm = (0 - y_range_data[0]) / (y_range_data[1] - y_range_data[0]) if y_range_data[1] != y_range_data[0] else 0.0
        
        # Taille du carré en pixels
        boat_size_pixels = 20
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


def create_velocity_plot(time, vx_rov, vy_rov, title="Vitesses du ROV"):
    """
    Crée un graphique des vitesses du ROV
    
    Parameters:
    -----------
    time : array
        Temps (s)
    vx_rov, vy_rov : array
        Vitesses du ROV (m/s)
    title : str
        Titre du graphique
    
    Returns:
    --------
    go.Figure
        Figure Plotly
    """
    fig = go.Figure()
    
    fig.add_trace(go.Scatter(
        x=time, y=vx_rov,
        mode='lines',
        name='Vitesse horizontale',
        line=dict(color='blue', width=2)
    ))
    
    fig.add_trace(go.Scatter(
        x=time, y=vy_rov,
        mode='lines',
        name='Vitesse verticale',
        line=dict(color='red', width=2)
    ))
    
    fig.add_hline(y=0, line_dash="dash", line_color="gray")
    
    fig.update_layout(
        xaxis_title="Temps (s)",
        yaxis_title="Vitesse (m/s)",
        title=title,
        hovermode='x unified',
        width=600,
        height=400
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

