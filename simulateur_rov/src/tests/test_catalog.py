"""
Catalogue des tests disponibles pour l'onglet « Test » de l'UI.

Chaque entrée décrit :
- un identifiant stable (clé JSON),
- un nom lisible,
- une brève description,
- la commande pytest (ou script python) à lancer,
- un thème (groupe),
- éventuellement un chemin de rapport HTML généré.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List


BASE_DIR = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class TestEntry:
    """Description d'un test affiché dans l'onglet Test."""

    id: str
    group: str
    name: str
    description: str
    command: str  # Commande à exécuter depuis la racine du projet (ex: "python -m pytest tests/...")
    html_report: str | None = None  # Chemin relatif vers un rapport HTML si disponible


def _tests_utils() -> Iterable[TestEntry]:
    """Tests unitaires de fonctions utilitaires géométriques."""
    return [
        TestEntry(
            id="utils_scale_slack_unit",
            group="Géométrie câble – unitaires",
            name="scale_slack (numérique)",
            description="Tests numériques de la fonction scale_slack.",
            command="python -m pytest -q tests/test_utils_scale_slack.py",
        ),
        TestEntry(
            id="utils_supprimer_point_unit",
            group="Géométrie câble – unitaires",
            name="supprimer_point (numérique)",
            description="Tests numériques de la fonction supprimer_point.",
            command="python -m pytest -q tests/test_utils_supprimer_point.py",
        ),
    ]


def _tests_visual() -> Iterable[TestEntry]:
    """Tests avec rapports graphiques Plotly."""
    return [
        TestEntry(
            id="visual_scale_slack",
            group="Géométrie câble – rapports graphiques",
            name="scale_slack – rapport graphique",
            description="Génère un rapport HTML illustrant la fonction scale_slack.",
            command="python tests/test_scale_slack_visual.py",
            html_report="results/test_scale_slack_visual_report.html",
        ),
        TestEntry(
            id="visual_deplacer_point",
            group="Géométrie câble – rapports graphiques",
            name="deplacer_point – rapport graphique",
            description="Génère un rapport HTML illustrant la fonction deplacer_point.",
            command="python tests/test_deplacer_point_visual.py",
            html_report="results/test_deplacer_point_visual_report.html",
        ),
        TestEntry(
            id="visual_supprimer_point",
            group="Géométrie câble – rapports graphiques",
            name="supprimer_point – rapport graphique",
            description="Génère un rapport HTML illustrant la fonction supprimer_point.",
            command="python tests/test_supprimer_point_visual.py",
            html_report="results/test_supprimer_point_visual_report.html",
        ),
        TestEntry(
            id="visual_enforce_cable_segments_nb",
            group="Géométrie câble – rapports graphiques",
            name="enforce_cable_segments_nb – rapport graphique",
            description="Génère un rapport HTML illustrant la réduction du nombre de segments du câble.",
            command="python tests/test_enforce_cable_segments_nb_visual.py",
            html_report="results/test_enforce_cable_segments_nb_visual_report.html",
        ),
        TestEntry(
            id="visual_cable_normalization",
            group="Normalisation du câble – rapports graphiques",
            name="Normalisation du câble – rapport graphique",
            description="Génère le rapport HTML des tests de normalisation du câble (_normalize_cable_geometry).",
            command="python tests/test_cable_normalization_visual.py",
            html_report="results/test_cable_normalization_visual_report.html",
        ),
    ]


def get_all_tests() -> List[TestEntry]:
    """Retourne la liste complète des tests connus, ordonnés par groupe puis par nom."""
    entries: List[TestEntry] = []
    entries.extend(_tests_utils())
    entries.extend(_tests_visual())
    # Tri stable par (group, name)
    entries.sort(key=lambda e: (e.group, e.name))
    return entries


__all__ = ["TestEntry", "get_all_tests"]

