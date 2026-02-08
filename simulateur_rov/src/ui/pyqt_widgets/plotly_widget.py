"""
Widget PyQt pour intégrer des graphiques Plotly
Utilise plotly.py ou une alternative pour intégrer Plotly dans PyQt
"""
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QSizePolicy
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QUrl
import plotly.graph_objects as go
from plotly.utils import PlotlyJSONEncoder
import json
import tempfile
import os
from src.utils.logger import trace_print


class PlotlyWidget(QWidget):
    """Widget pour afficher des graphiques Plotly dans PyQt"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        
        # Créer un QWebEngineView pour afficher le graphique HTML
        self.web_view = QWebEngineView()
        self.layout.addWidget(self.web_view)
        self.layout.setStretch(0, 1)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.web_view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        
        # Répertoire temporaire pour stocker les graphiques HTML
        self.temp_dir = tempfile.gettempdir()
        
        # Stocker la dernière figure pour pouvoir la mettre à jour lors du redimensionnement
        self.current_fig = None
        
        # Stocker les plages d'axes précédentes pour éviter de recréer le layout si elles n'ont pas changé
        self.previous_x_range = None
        self.previous_y_range = None
        self.is_first_update = True
        self.graph_id = f'plotly_graph_{id(self)}'  # ID fixe pour le div du graphique
    
    def update_figure(self, fig):
        """
        Met à jour le graphique affiché
        
        Parameters:
        -----------
        fig : plotly.graph_objects.Figure
            Figure Plotly à afficher
        """
        # Stocker le nombre de traces AVANT de mettre à jour current_fig
        # pour pouvoir détecter les changements
        previous_num_traces = sum(1 for trace in self.current_fig.data if hasattr(trace, 'type') and trace.type == 'scatter') if self.current_fig is not None else 0
        
        # Stocker la figure pour pouvoir la mettre à jour lors du redimensionnement
        self.current_fig = fig
        
        # Obtenir la taille du widget pour adapter le graphique
        widget_size = self.size()
        width = widget_size.width()
        height = widget_size.height()
        if width <= 0 or height <= 0:
            parent = self.parentWidget()
            if parent is not None:
                parent_size = parent.size()
                width = parent_size.width()
                height = parent_size.height()
        if width > 0 and height > 0:
            # Utiliser la taille disponible si possible
            fig.update_layout(width=width, height=height)
        else:
            # Taille par défaut si le widget n'a pas encore de taille
            fig.update_layout(width=800, height=600)
        
        # Extraire les plages d'axes actuelles
        current_x_range = fig.layout.xaxis.range
        current_y_range = fig.layout.yaxis.range
        
        # Vérifier si les plages d'axes ont changé de manière significative
        # Utiliser une tolérance pour éviter les recréations inutiles
        x_range_changed = False
        y_range_changed = False
        if self.previous_x_range is not None and current_x_range is not None:
            # Vérifier si la différence est significative (plus de 5% de changement)
            x_range_diff = abs(current_x_range[1] - current_x_range[0])
            if x_range_diff > 0:
                prev_range_diff = abs(self.previous_x_range[1] - self.previous_x_range[0])
                if prev_range_diff > 0:
                    x_change_ratio = abs((current_x_range[0] - self.previous_x_range[0]) / prev_range_diff)
                    x_range_changed = x_change_ratio > 0.05  # Plus de 5% de changement
        else:
            x_range_changed = (self.previous_x_range != current_x_range)
        
        if self.previous_y_range is not None and current_y_range is not None:
            # Vérifier si la différence est significative (plus de 5% de changement)
            y_range_diff = abs(current_y_range[1] - current_y_range[0])
            if y_range_diff > 0:
                prev_range_diff = abs(self.previous_y_range[1] - self.previous_y_range[0])
                if prev_range_diff > 0:
                    y_change_ratio = abs((current_y_range[0] - self.previous_y_range[0]) / prev_range_diff)
                    y_range_changed = y_change_ratio > 0.05  # Plus de 5% de changement
        else:
            y_range_changed = (self.previous_y_range != current_y_range)
        
        ranges_changed = x_range_changed or y_range_changed
        
        # Toujours utiliser Plotly.update pour éviter le clignotement
        # Recréer le graphique seulement lors de la première mise à jour
        # OU si le nombre de traces a changé (pour éviter les problèmes avec restyle)
        num_scatter_traces = sum(1 for trace in fig.data if hasattr(trace, 'type') and trace.type == 'scatter')
        
        # Forcer la recréation si le nombre de traces a changé, si c'est la première mise à jour
        # OU si le graphique précédent n'existe pas encore
        force_recreate = self.is_first_update or (previous_num_traces == 0) or (num_scatter_traces != previous_num_traces)

        # Recréer si les shapes ont changé (ex: ajout/suppression de lignes de mode)
        prev_shapes = getattr(self.current_fig.layout, "shapes", None) if self.current_fig is not None else None
        curr_shapes = getattr(fig.layout, "shapes", None)
        prev_shapes_len = len(prev_shapes) if prev_shapes is not None else 0
        curr_shapes_len = len(curr_shapes) if curr_shapes is not None else 0
        if prev_shapes_len != curr_shapes_len:
            force_recreate = True
        
        if force_recreate:
            # Convertir la figure en HTML avec un ID fixe pour le div
            # Utiliser div_id pour avoir un ID fixe et pouvoir mettre à jour via JavaScript
            graph_id = f'plotly_graph_{id(self)}'
            html_str = fig.to_html(include_plotlyjs='inline', div_id=graph_id)
            
            # Ajouter du CSS pour supprimer les barres de défilement
            css_injection = """
            <style>
                body {
                    margin: 0;
                    padding: 0;
                    overflow: hidden;
                }
            </style>
            """
            
            # Insérer le CSS juste après la balise <head>
            if '<head>' in html_str:
                html_str = html_str.replace('<head>', '<head>' + css_injection)
            else:
                # Si pas de <head>, l'ajouter avant le <body>
                html_str = css_injection + html_str
            
            # Sauvegarder dans un fichier temporaire
            temp_file = os.path.join(self.temp_dir, f'plotly_{id(self)}.html')
            with open(temp_file, 'w', encoding='utf-8') as f:
                f.write(html_str)
            
            # Charger le fichier dans le QWebEngineView
            try:
                self.web_view.setUrl(QUrl.fromLocalFile(temp_file))
                if self.web_view.url().isEmpty():
                    trace_print(1, "[DEBUG] PlotlyWidget: setUrl vide, fallback setHtml.")
                    self.web_view.setHtml(html_str, QUrl.fromLocalFile(self.temp_dir + os.sep))
            except Exception:
                trace_print(1, "[DEBUG] PlotlyWidget: setUrl échoué, fallback setHtml.")
                self.web_view.setHtml(html_str, QUrl.fromLocalFile(self.temp_dir + os.sep))
            
            # Mettre à jour les plages stockées
            self.previous_x_range = current_x_range
            self.previous_y_range = current_y_range
            self.is_first_update = False
            self.graph_id = graph_id
        else:
            # Mettre à jour le graphique via JavaScript sans recréation complète
            # Extraire les données des traces (seulement les traces Scatter avec x et y)
            # Créer un mapping entre l'index dans fig.data et les données à mettre à jour
            traces_data = []
            scatter_indices = []  # Indices réels des traces Scatter dans fig.data
            for i, trace in enumerate(fig.data):
                # Ne mettre à jour que les traces de type Scatter (pas les shapes)
                if hasattr(trace, 'type') and trace.type == 'scatter':
                    if hasattr(trace, 'x') and hasattr(trace, 'y'):
                        x_data = trace.x.tolist() if hasattr(trace.x, 'tolist') else list(trace.x)
                        y_data = trace.y.tolist() if hasattr(trace.y, 'tolist') else list(trace.y)
                        trace_dict = {
                            'x': x_data,
                            'y': y_data
                        }
                        if hasattr(trace, 'line') and trace.line is not None:
                            trace_dict['line'] = trace.line
                        # Inclure hovertext/hoverinfo/hovertemplate si disponibles (tooltips)
                        if hasattr(trace, 'hovertext') and trace.hovertext is not None:
                            hovertext_list = trace.hovertext.tolist() if hasattr(trace.hovertext, 'tolist') else list(trace.hovertext)
                            trace_dict['hovertext'] = hovertext_list
                        if hasattr(trace, 'hoverinfo'):
                            trace_dict['hoverinfo'] = trace.hoverinfo
                        if hasattr(trace, 'hovertemplate') and trace.hovertemplate is not None:
                            trace_dict['hovertemplate'] = trace.hovertemplate
                        traces_data.append(trace_dict)
                        scatter_indices.append(i)  # Stocker l'index réel dans fig.data
            
            # Créer le script JavaScript pour mettre à jour les données et le layout
            # Utiliser json.dumps pour sérialiser les données
            traces_json = json.dumps(traces_data, cls=PlotlyJSONEncoder)
            scatter_indices_json = json.dumps(scatter_indices)  # Indices réels des traces Scatter
            graph_id_js = self.graph_id if hasattr(self, 'graph_id') else f'plotly_graph_{id(self)}'
            
            # Préparer la mise à jour du layout (titre et plages)
            title_update = fig.layout.title.text if hasattr(fig.layout, 'title') and hasattr(fig.layout.title, 'text') else None
            
            # Préparer les mises à jour de plages
            x_range_update = None
            y_range_update = None
            if current_x_range is not None:
                x_range_update = list(current_x_range)
            if current_y_range is not None:
                y_range_update = list(current_y_range)
            
            # Extraire les shapes (pour mettre à jour la position du bateau)
            # Les shapes sont stockées dans fig.layout.shapes comme un tuple
            shapes_data = []
            if hasattr(fig.layout, 'shapes') and fig.layout.shapes is not None:
                # fig.layout.shapes est un tuple de Shape objects
                shapes_tuple = fig.layout.shapes
                # Convertir en liste pour itérer
                if isinstance(shapes_tuple, tuple):
                    shapes_list = list(shapes_tuple)
                elif isinstance(shapes_tuple, list):
                    shapes_list = shapes_tuple
                else:
                    shapes_list = [shapes_tuple]
                
                for shape in shapes_list:
                    if hasattr(shape, 'xref') and shape.xref == 'paper':
                        # C'est la shape du bateau (utilise paper coordinates)
                        shape_dict = {}
                        if hasattr(shape, 'x0'):
                            shape_dict['x0'] = float(shape.x0) if shape.x0 is not None else None
                        if hasattr(shape, 'x1'):
                            shape_dict['x1'] = float(shape.x1) if shape.x1 is not None else None
                        if hasattr(shape, 'y0'):
                            shape_dict['y0'] = float(shape.y0) if shape.y0 is not None else None
                        if hasattr(shape, 'y1'):
                            shape_dict['y1'] = float(shape.y1) if shape.y1 is not None else None
                        shapes_data.append(shape_dict if shape_dict else None)
                    else:
                        shapes_data.append(None)
            
            # Sérialiser les plages et shapes pour JavaScript
            import json as json_module
            x_range_js = json_module.dumps(x_range_update) if x_range_update else 'null'
            y_range_js = json_module.dumps(y_range_update) if y_range_update else 'null'
            shapes_json = json_module.dumps(shapes_data) if shapes_data else '[]'
            
            js_update = f"""
            (function() {{
                if (typeof Plotly !== 'undefined') {{
                    var updateData = {traces_json};
                    var scatterIndices = {scatter_indices_json};
                    var graphDiv = document.getElementById('{graph_id_js}');
                    if (!graphDiv) {{
                        console.log('Graph div not found: {graph_id_js}');
                        return;
                    }}
                    if (!graphDiv._fullLayout) {{
                        console.log('Graph not yet initialized, skipping update');
                        return;
                    }}
                    
                    // Mettre à jour chaque trace Scatter individuellement avec Plotly.restyle
                    // scatterIndices contient les indices réels des traces Scatter dans le graphique
                    // updateData contient les données correspondantes
                    for (var j = 0; j < updateData.length; j++) {{
                        if (updateData[j] !== null && j < scatterIndices.length) {{
                            var traceIndex = scatterIndices[j];  // Index réel dans le graphique
                            // Mettre à jour la trace avec ses nouvelles données
                            // Format pour une trace: {{x: [array], y: [array], hovertext: [array], hoverinfo: string}}
                            var restyleUpdate = {{
                                x: [updateData[j].x],
                                y: [updateData[j].y]
                            }};
                            // Ajouter hovertext, hoverinfo, hovertemplate si disponibles
                            if (updateData[j].hovertext !== undefined && updateData[j].hovertext !== null) {{
                                restyleUpdate.hovertext = [updateData[j].hovertext];
                            }}
                            if (updateData[j].hoverinfo !== undefined && updateData[j].hoverinfo !== null) {{
                                restyleUpdate.hoverinfo = updateData[j].hoverinfo;
                            }}
                            if (updateData[j].hovertemplate !== undefined && updateData[j].hovertemplate !== null) {{
                                restyleUpdate.hovertemplate = updateData[j].hovertemplate;
                            }}
                            if (updateData[j].line !== undefined && updateData[j].line !== null) {{
                                restyleUpdate.line = updateData[j].line;
                            }}
                            Plotly.restyle(graphDiv, restyleUpdate, traceIndex);
                        }}
                    }}
                    
                    // Mettre à jour le layout séparément avec Plotly.relayout
                    var layoutUpdate = {{}};
                    {f"layoutUpdate.title = {{text: '{title_update}'}};" if title_update else ""}
                    if ({x_range_js} !== null) {{
                        layoutUpdate['xaxis.range'] = {x_range_js};
                    }}
                    if ({y_range_js} !== null) {{
                        layoutUpdate['yaxis.range'] = {y_range_js};
                    }}
                    
                    // Mettre à jour les shapes (position du bateau)
                    var shapesData = {shapes_json};
                    if (shapesData && Array.isArray(shapesData) && shapesData.length > 0) {{
                        for (var k = 0; k < shapesData.length; k++) {{
                            if (shapesData[k] !== null && typeof shapesData[k] === 'object') {{
                                var shapePrefix = 'shapes[' + k + '].';
                                if (shapesData[k].x0 !== null && shapesData[k].x0 !== undefined) {{
                                    layoutUpdate[shapePrefix + 'x0'] = shapesData[k].x0;
                                }}
                                if (shapesData[k].x1 !== null && shapesData[k].x1 !== undefined) {{
                                    layoutUpdate[shapePrefix + 'x1'] = shapesData[k].x1;
                                }}
                                if (shapesData[k].y0 !== null && shapesData[k].y0 !== undefined) {{
                                    layoutUpdate[shapePrefix + 'y0'] = shapesData[k].y0;
                                }}
                                if (shapesData[k].y1 !== null && shapesData[k].y1 !== undefined) {{
                                    layoutUpdate[shapePrefix + 'y1'] = shapesData[k].y1;
                                }}
                            }}
                        }}
                    }}
                    
                    if (Object.keys(layoutUpdate).length > 0) {{
                        Plotly.relayout(graphDiv, layoutUpdate);
                    }}
                }}
            }})();
            """
            
            # Injecter le JavaScript dans la page actuelle
            # Utiliser runJavaScript pour exécuter le script de manière asynchrone
            self.web_view.page().runJavaScript(js_update)
            
            # Mettre à jour les plages stockées même si on n'a pas recréé le graphique
            self.previous_x_range = current_x_range
            self.previous_y_range = current_y_range
            # Mettre à jour current_fig pour la prochaine comparaison
            self.current_fig = fig
    
    def resizeEvent(self, event):
        """Appelé quand le widget est redimensionné"""
        super().resizeEvent(event)
        # Mettre à jour le graphique avec la nouvelle taille si une figure existe
        if self.current_fig is not None:
            self.update_figure(self.current_fig)
    
    def set_figure(self, fig):
        """Alias pour update_figure pour compatibilité"""
        self.update_figure(fig)
    
    def clear(self):
        """Efface le graphique"""
        # Créer une figure vide
        fig = go.Figure()
        fig.update_layout(
            title="Aucune donnée",
            xaxis_title="X",
            yaxis_title="Y",
            template="plotly_white"
        )
        self.update_figure(fig)
