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

import re
import unicodedata
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List


BASE_DIR = Path(__file__).resolve().parents[2]


def html_report_path_from_column_name(name: str) -> str:
    """
    Chemin relatif ``results/<slug>.html`` dérivé du libellé colonne « Test / Fonction »
    (même logique que l'affichage UI).
    """
    s = unicodedata.normalize("NFKD", name)
    s = "".join(ch for ch in s if not unicodedata.combining(ch))
    s = s.replace("–", "-").replace("—", "-")
    s = s.lower()
    s = re.sub(r"[^a-z0-9]+", "_", s)
    s = re.sub(r"_+", "_", s).strip("_")
    if not s:
        s = "rapport"
    return f"results/{s}.html"


# Rapport texte agrégé de ``pytest`` (plusieurs entrées unitaires dans l'onglet Test)
TEXT_REPORT_PYTEST_AGGREGATE = "results/rapport_execution_pytest.txt"


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
            id="utils_aplatir_polyline_unit",
            group="Géométrie câble – unitaires",
            name="aplatir_polyline (numérique)",
            description="Tests numériques de la fonction CableSolver.aplatir_polyline.",
            command="python -m pytest -q tests/test_aplatir_polyline.py",
        ),
        TestEntry(
            id="utils_deformer_polyline_unit",
            group="Géométrie câble – unitaires",
            name="deformer_polyline (numérique)",
            description="Tests numériques de la fonction CableSolver.deformer_polyline (recherche de k pour une longueur cible).",
            command="python -m pytest -q tests/test_deformer_polyline.py",
        ),
        TestEntry(
            id="utils_supprimer_point_unit",
            group="Géométrie câble – unitaires",
            name="supprimer_point (numérique)",
            description="Tests numériques de la fonction supprimer_point.",
            command="python -m pytest -q tests/test_utils_supprimer_point.py",
        ),
        TestEntry(
            id="utils_create_point_with_target_length_unit",
            group="Géométrie câble – unitaires",
            name="create_point_with_target_length (numérique)",
            description="Tests numériques de la fonction create_point_with_target_length.",
            command="python -m pytest -q tests/test_utils_create_point_with_target_length.py",
        ),
        TestEntry(
            id="cable_normalize_segments_unit",
            group="Normalisation du câble – unitaires",
            name="_normalize_cable_segments (structure)",
            description="Tests de structure pour CableSolver._normalize_cable_segments (taille, extrémités, fallback).",
            command="python -m pytest -q tests/test_cable_normalize_segments.py",
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
            html_report=html_report_path_from_column_name("scale_slack – rapport graphique"),
        ),
        TestEntry(
            id="visual_deplacer_point",
            group="Géométrie câble – rapports graphiques",
            name="deplacer_point – rapport graphique",
            description="Génère un rapport HTML illustrant la fonction deplacer_point.",
            command="python tests/test_deplacer_point_visual.py",
            html_report=html_report_path_from_column_name("deplacer_point – rapport graphique"),
        ),
        TestEntry(
            id="visual_deformer_polyline",
            group="Géométrie câble – rapports graphiques",
            name="deformer_polyline – rapport graphique",
            description="Génère un rapport HTML illustrant CableSolver.deformer_polyline.",
            command="python tests/test_deformer_polyline_visual.py",
            html_report=html_report_path_from_column_name("deformer_polyline – rapport graphique"),
        ),
        TestEntry(
            id="visual_create_point_with_target_length",
            group="Géométrie câble – rapports graphiques",
            name="create_point_with_target_length – rapport graphique",
            description="Génère un rapport HTML illustrant la fonction create_point_with_target_length.",
            command="python tests/test_create_point_with_target_length_visual.py",
            html_report=html_report_path_from_column_name(
                "create_point_with_target_length – rapport graphique"
            ),
        ),
        TestEntry(
            id="utils_next_point_unit",
            group="Géométrie câble – unitaires",
            name="next_point (numérique)",
            description="Tests numériques de la fonction next_point.",
            command="python -m pytest -q tests/test_utils_next_point.py",
        ),
        TestEntry(
            id="visual_next_point",
            group="Géométrie câble – rapports graphiques",
            name="next_point – rapport graphique",
            description="Génère un rapport HTML illustrant la fonction next_point.",
            command="python tests/test_next_point_visual.py",
            html_report=html_report_path_from_column_name("next_point – rapport graphique"),
        ),
        TestEntry(
            id="visual_supprimer_point",
            group="Géométrie câble – rapports graphiques",
            name="supprimer_point – rapport graphique",
            description="Génère un rapport HTML illustrant la fonction supprimer_point.",
            command="python tests/test_supprimer_point_visual.py",
            html_report=html_report_path_from_column_name("supprimer_point – rapport graphique"),
        ),
        TestEntry(
            id="visual_enforce_cable_segments_nb",
            group="Géométrie câble – rapports graphiques",
            name="enforce_cable_segments_nb – rapport graphique",
            description="Génère un rapport HTML illustrant la réduction du nombre de segments du câble.",
            command="python tests/test_enforce_cable_segments_nb_visual.py",
            html_report=html_report_path_from_column_name(
                "enforce_cable_segments_nb – rapport graphique"
            ),
        ),
        TestEntry(
            id="visual_cable_normalization",
            group="Normalisation du câble – rapports graphiques",
            name="Normalisation du câble – rapport graphique",
            description="Génère le rapport HTML des tests de normalisation du câble (_normalize_cable_geometry).",
            command="python tests/test_cable_normalization_visual.py",
            html_report=html_report_path_from_column_name(
                "Normalisation du câble – rapport graphique"
            ),
        ),
        TestEntry(
            id="visual_cable_normalize_segments",
            group="Normalisation du câble – rapports graphiques",
            name="_normalize_cable_segments – rapport graphique",
            description="Génère un rapport HTML illustrant CableSolver._normalize_cable_segments.",
            command="python tests/test_cable_normalize_segments_visual.py",
            html_report=html_report_path_from_column_name(
                "_normalize_cable_segments – rapport graphique"
            ),
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


def get_visual_default_html_path(entry_id: str) -> str | None:
    """Chemin HTML par défaut pour un test visuel (``id`` du catalogue), ou None."""
    for e in get_all_tests():
        if e.id == entry_id and e.html_report:
            return e.html_report
    return None


__all__ = [
    "TestEntry",
    "get_all_tests",
    "get_visual_default_html_path",
    "html_report_path_from_column_name",
    "TEXT_REPORT_PYTEST_AGGREGATE",
]

