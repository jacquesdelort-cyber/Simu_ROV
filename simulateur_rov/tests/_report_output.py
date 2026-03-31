"""Première ligne d'horodatage pour les rapports (HTML Plotly, texte pytest)."""

from __future__ import annotations

import datetime
from pathlib import Path
from typing import Any


def report_generated_first_line(now: datetime.datetime | None = None) -> str:
    """Texte unique pour la ligne 1 des rapports : date et heure de génération."""
    t = now or datetime.datetime.now()
    return f"Rapport généré le {t.strftime('%Y-%m-%d %H:%M:%S')}"


def write_plotly_html_with_generation_line(
    fig: Any,
    output_path: str | Path,
    *,
    include_plotlyjs: str = "inline",
    **kwargs: Any,
) -> Path:
    """Écrit le HTML Plotly puis préfixe le fichier par un commentaire (ligne 1 = date/heure)."""
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(path), include_plotlyjs=include_plotlyjs, **kwargs)
    line = f"<!-- {report_generated_first_line()} -->\n"
    body = path.read_text(encoding="utf-8")
    path.write_text(line + body, encoding="utf-8")
    return path
