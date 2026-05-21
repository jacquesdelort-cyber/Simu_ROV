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

        self.web_view = QWebEngineView()
        self.layout.addWidget(self.web_view)
        self.layout.setStretch(0, 1)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.web_view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self.temp_dir = tempfile.gettempdir()

        self.current_fig = None
        self.previous_x_range = None
        self.previous_y_range = None
        self.is_first_update = True
        self.graph_id = f"plotly_graph_{id(self)}"

        self._page_ready = False
        self._pending_figure = None
        self._js_update_pending = False

        self.web_view.loadFinished.connect(self._on_load_finished)

    def _mark_page_loading(self):
        self._page_ready = False

    def _on_load_finished(self, ok: bool):
        if not ok:
            return
        self._page_ready = True
        pending = self._pending_figure
        if pending is not None:
            self._pending_figure = None
            self.update_figure(pending)

    def _on_js_update_finished(self, _result):
        self._js_update_pending = False
        pending = self._pending_figure
        if pending is not None and self._page_ready:
            self._pending_figure = None
            self.update_figure(pending)

    def update_figure(self, fig):
        """
        Met à jour le graphique affiché

        Parameters:
        -----------
        fig : plotly.graph_objects.Figure
            Figure Plotly à afficher
        """
        previous_fig = self.current_fig
        previous_num_traces = (
            sum(
                1
                for trace in previous_fig.data
                if hasattr(trace, "type") and trace.type == "scatter"
            )
            if previous_fig is not None
            else 0
        )

        self.current_fig = fig

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
            fig.update_layout(width=width, height=height)
        else:
            fig.update_layout(width=800, height=600)

        current_x_range = fig.layout.xaxis.range
        current_y_range = fig.layout.yaxis.range

        x_range_changed = False
        y_range_changed = False
        if self.previous_x_range is not None and current_x_range is not None:
            x_range_diff = abs(current_x_range[1] - current_x_range[0])
            if x_range_diff > 0:
                prev_range_diff = abs(self.previous_x_range[1] - self.previous_x_range[0])
                if prev_range_diff > 0:
                    x_change_ratio = abs(
                        (current_x_range[0] - self.previous_x_range[0]) / prev_range_diff
                    )
                    x_range_changed = x_change_ratio > 0.05
        else:
            x_range_changed = self.previous_x_range != current_x_range

        if self.previous_y_range is not None and current_y_range is not None:
            y_range_diff = abs(current_y_range[1] - current_y_range[0])
            if y_range_diff > 0:
                prev_range_diff = abs(self.previous_y_range[1] - self.previous_y_range[0])
                if prev_range_diff > 0:
                    y_change_ratio = abs(
                        (current_y_range[0] - self.previous_y_range[0]) / prev_range_diff
                    )
                    y_range_changed = y_change_ratio > 0.05
        else:
            y_range_changed = self.previous_y_range != current_y_range

        num_scatter_traces = sum(
            1 for trace in fig.data if hasattr(trace, "type") and trace.type == "scatter"
        )

        force_recreate = (
            self.is_first_update
            or previous_num_traces == 0
            or num_scatter_traces != previous_num_traces
        )

        prev_shapes = getattr(previous_fig.layout, "shapes", None) if previous_fig is not None else None
        curr_shapes = getattr(fig.layout, "shapes", None)
        prev_shapes_len = len(prev_shapes) if prev_shapes is not None else 0
        curr_shapes_len = len(curr_shapes) if curr_shapes is not None else 0
        if prev_shapes_len != curr_shapes_len:
            force_recreate = True

        if force_recreate:
            graph_id = f"plotly_graph_{id(self)}"
            html_str = fig.to_html(include_plotlyjs="inline", div_id=graph_id)

            css_injection = """
            <style>
                body {
                    margin: 0;
                    padding: 0;
                    overflow: hidden;
                }
            </style>
            """

            if "<head>" in html_str:
                html_str = html_str.replace("<head>", "<head>" + css_injection)
            else:
                html_str = css_injection + html_str

            temp_file = os.path.join(self.temp_dir, f"plotly_{id(self)}.html")
            with open(temp_file, "w", encoding="utf-8") as f:
                f.write(html_str)

            self._mark_page_loading()
            self._js_update_pending = False
            try:
                self.web_view.setUrl(QUrl.fromLocalFile(temp_file))
                if self.web_view.url().isEmpty():
                    trace_print(1, "[DEBUG] PlotlyWidget: setUrl vide, fallback setHtml.")
                    self._mark_page_loading()
                    self.web_view.setHtml(html_str, QUrl.fromLocalFile(self.temp_dir + os.sep))
            except Exception:
                trace_print(1, "[DEBUG] PlotlyWidget: setUrl échoué, fallback setHtml.")
                self._mark_page_loading()
                self.web_view.setHtml(html_str, QUrl.fromLocalFile(self.temp_dir + os.sep))

            self.previous_x_range = current_x_range
            self.previous_y_range = current_y_range
            self.is_first_update = False
            self.graph_id = graph_id
            self._pending_figure = None
        else:
            if not self._page_ready:
                self._pending_figure = fig
                return

            if self._js_update_pending:
                self._pending_figure = fig
                return

            self._apply_js_update(fig, current_x_range, current_y_range)

    def _apply_js_update(self, fig, current_x_range, current_y_range):
        traces_data = []
        scatter_indices = []
        for i, trace in enumerate(fig.data):
            if hasattr(trace, "type") and trace.type == "scatter":
                if hasattr(trace, "x") and hasattr(trace, "y"):
                    x_data = trace.x.tolist() if hasattr(trace.x, "tolist") else list(trace.x)
                    y_data = trace.y.tolist() if hasattr(trace.y, "tolist") else list(trace.y)
                    trace_dict = {"x": x_data, "y": y_data}
                    if hasattr(trace, "line") and trace.line is not None:
                        trace_dict["line"] = trace.line
                    if hasattr(trace, "hovertext") and trace.hovertext is not None:
                        hovertext_list = (
                            trace.hovertext.tolist()
                            if hasattr(trace.hovertext, "tolist")
                            else list(trace.hovertext)
                        )
                        trace_dict["hovertext"] = hovertext_list
                    if hasattr(trace, "hoverinfo"):
                        trace_dict["hoverinfo"] = trace.hoverinfo
                    if hasattr(trace, "hovertemplate") and trace.hovertemplate is not None:
                        trace_dict["hovertemplate"] = trace.hovertemplate
                    traces_data.append(trace_dict)
                    scatter_indices.append(i)

        traces_json = json.dumps(traces_data, cls=PlotlyJSONEncoder)
        scatter_indices_json = json.dumps(scatter_indices)
        graph_id_js = self.graph_id if hasattr(self, "graph_id") else f"plotly_graph_{id(self)}"

        title_update = (
            fig.layout.title.text
            if hasattr(fig.layout, "title") and hasattr(fig.layout.title, "text")
            else None
        )

        x_range_update = list(current_x_range) if current_x_range is not None else None
        y_range_update = list(current_y_range) if current_y_range is not None else None

        shapes_data = []
        if hasattr(fig.layout, "shapes") and fig.layout.shapes is not None:
            shapes_tuple = fig.layout.shapes
            if isinstance(shapes_tuple, tuple):
                shapes_list = list(shapes_tuple)
            elif isinstance(shapes_tuple, list):
                shapes_list = shapes_tuple
            else:
                shapes_list = [shapes_tuple]

            for shape in shapes_list:
                if hasattr(shape, "xref") and shape.xref == "paper":
                    shape_dict = {}
                    if hasattr(shape, "x0"):
                        shape_dict["x0"] = float(shape.x0) if shape.x0 is not None else None
                    if hasattr(shape, "x1"):
                        shape_dict["x1"] = float(shape.x1) if shape.x1 is not None else None
                    if hasattr(shape, "y0"):
                        shape_dict["y0"] = float(shape.y0) if shape.y0 is not None else None
                    if hasattr(shape, "y1"):
                        shape_dict["y1"] = float(shape.y1) if shape.y1 is not None else None
                    shapes_data.append(shape_dict if shape_dict else None)
                else:
                    shapes_data.append(None)

        x_range_js = json.dumps(x_range_update) if x_range_update else "null"
        y_range_js = json.dumps(y_range_update) if y_range_update else "null"
        shapes_json = json.dumps(shapes_data) if shapes_data else "[]"

        plot_w = fig.layout.width
        plot_h = fig.layout.height
        if plot_w is None:
            plot_w = 800
        if plot_h is None:
            plot_h = 600
        plot_w = int(plot_w)
        plot_h = int(plot_h)

        title_js = (
            f"layoutUpdate.title = {{text: '{title_update}'}};"
            if title_update
            else ""
        )

        js_update = f"""
            (function() {{
                if (typeof Plotly === 'undefined') {{
                    return;
                }}
                var updateData = {traces_json};
                var scatterIndices = {scatter_indices_json};
                var graphDiv = document.getElementById('{graph_id_js}');
                if (!graphDiv) {{
                    return;
                }}
                if (!graphDiv._fullLayout || !Array.isArray(graphDiv.data)) {{
                    return;
                }}

                for (var j = 0; j < updateData.length; j++) {{
                    if (updateData[j] === null || j >= scatterIndices.length) {{
                        continue;
                    }}
                    var traceIndex = scatterIndices[j];
                    if (traceIndex < 0 || traceIndex >= graphDiv.data.length) {{
                        continue;
                    }}
                    var restyleUpdate = {{
                        x: [updateData[j].x],
                        y: [updateData[j].y]
                    }};
                    if (updateData[j].hovertext !== undefined && updateData[j].hovertext !== null) {{
                        restyleUpdate.hovertext = [updateData[j].hovertext];
                    }}
                    if (updateData[j].hoverinfo !== undefined && updateData[j].hoverinfo !== null) {{
                        restyleUpdate.hoverinfo = updateData[j].hoverinfo;
                    }}
                    if (updateData[j].hovertemplate !== undefined && updateData[j].hovertemplate !== null) {{
                        restyleUpdate.hovertemplate = [updateData[j].hovertemplate];
                    }}
                    if (updateData[j].line !== undefined && updateData[j].line !== null) {{
                        restyleUpdate.line = updateData[j].line;
                    }}
                    try {{
                        Plotly.restyle(graphDiv, restyleUpdate, traceIndex);
                    }} catch (e) {{
                        console.log('Plotly restyle skipped', e);
                    }}
                }}

                var layoutUpdate = {{}};
                layoutUpdate.width = {plot_w};
                layoutUpdate.height = {plot_h};
                {title_js}
                if ({x_range_js} !== null) {{
                    layoutUpdate['xaxis.range'] = {x_range_js};
                }}
                if ({y_range_js} !== null) {{
                    layoutUpdate['yaxis.range'] = {y_range_js};
                }}

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
                    try {{
                        Plotly.relayout(graphDiv, layoutUpdate);
                    }} catch (e) {{
                        console.log('Plotly relayout skipped', e);
                    }}
                }}
            }})();
            """

        self._js_update_pending = True
        self.web_view.page().runJavaScript(js_update, self._on_js_update_finished)

        self.previous_x_range = current_x_range
        self.previous_y_range = current_y_range
        self.current_fig = fig

    def resizeEvent(self, event):
        """Appelé quand le widget est redimensionné"""
        super().resizeEvent(event)
        if self.current_fig is not None:
            self.update_figure(self.current_fig)

    def set_figure(self, fig):
        """Alias pour update_figure pour compatibilité"""
        self.update_figure(fig)

    def clear(self):
        """Efface le graphique"""
        fig = go.Figure()
        fig.update_layout(
            title="Aucune donnée",
            xaxis_title="X",
            yaxis_title="Y",
            template="plotly_white",
        )
        self.update_figure(fig)
