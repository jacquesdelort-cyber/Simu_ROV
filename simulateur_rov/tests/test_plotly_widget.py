"""Tests légers pour PlotlyWidget (inspection source, sans WebEngine)."""
from __future__ import annotations

from pathlib import Path

PLOTLY_WIDGET_PATH = Path(__file__).resolve().parents[1] / "src" / "ui" / "pyqt_widgets" / "plotly_widget.py"


def _source() -> str:
    return PLOTLY_WIDGET_PATH.read_text(encoding="utf-8")


def test_plotly_widget_page_ready_and_pending():
    source = _source()
    assert "_page_ready" in source
    assert "_pending_figure" in source
    assert "_js_update_pending" in source
    assert "loadFinished.connect(self._on_load_finished)" in source
    assert "_mark_page_loading" in source


def test_plotly_widget_defers_js_when_not_ready():
    source = _source()
    assert "if not self._page_ready:" in source
    assert "self._pending_figure = fig" in source


def test_js_update_contains_restyle_guards():
    source = _source()
    assert "Array.isArray(graphDiv.data)" in source
    assert "traceIndex >= graphDiv.data.length" in source
    assert "Plotly restyle skipped" in source
    assert "Plotly relayout skipped" in source
    assert "runJavaScript(js_update, self._on_js_update_finished)" in source
