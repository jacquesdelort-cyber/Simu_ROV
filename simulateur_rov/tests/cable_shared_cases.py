"""
Jeux de données partagés pour les rapports visuels et tests autour du câble.

- ``_normalize_cable_geometry`` : ``n_segments`` = nombre de segments cible.
- ``_normalize_cable_segments`` : ``n_target`` = ``N_target`` (nombre de segments visés).

Les positions ``boat`` / ``rov`` peuvent différer des extrémités de la polyligne
``(x_cable, y_cable)`` lorsque le scénario le impose.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SharedCableCase:
    """Un cas : nom, câble, bateau/ROV, longueur cible, paramètres des deux normalisations."""

    name: str
    x_cable: np.ndarray
    y_cable: np.ndarray
    boat: tuple[float, float]
    rov: tuple[float, float]
    l_target: float
    n_segments: int
    n_target: int

    @property
    def P(self) -> np.ndarray:
        return np.stack([self.x_cable, self.y_cable], axis=1)


ALL_SHARED_CABLE_CASES: tuple[SharedCableCase, ...] = (
    SharedCableCase(
        name="Deja normalise",
        x_cable=np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0], dtype=float),
        y_cable=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float),
        l_target=5.0,
        n_segments=5,
        n_target=5,
        boat=(0.0, 0.0),
        rov=(5.0, 0.0),
    ),
    SharedCableCase(
        name="Longueur cible imposee",
        x_cable=np.array([0.0, 0.3, 1.2, 2.0, 2.7, 4.1, 5.0], dtype=float),
        y_cable=np.array([0.0, -0.4, -1.1, -1.3, -2.2, -2.6, -3.0], dtype=float),
        l_target=6.0,
        n_segments=6,
        n_target=6,
        boat=(0.0, 0.0),
        rov=(5.0, -3.0),
    ),
    SharedCableCase(
        name="Clipping surface",
        x_cable=np.array([0.0, 0.5, 1.0, 1.5, 2.0], dtype=float),
        y_cable=np.array([0.2, 0.1, -0.3, -0.8, -1.2], dtype=float),
        l_target=2.4,
        n_segments=4,
        n_target=4,
        boat=(0.0, 0.0),
        rov=(2.0, -1.2),
    ),
    SharedCableCase(
        name="Peu de segments",
        x_cable=np.array([0.0, 0.4, 1.4, 2.0], dtype=float),
        y_cable=np.array([0.0, -0.2, -0.9, -1.2], dtype=float),
        l_target=3.0,
        n_segments=3,
        n_target=3,
        boat=(0.0, 0.0),
        rov=(2.0, -1.2),
    ),
    SharedCableCase(
        name="Points repetes",
        x_cable=np.array([0.0, 0.0, 0.0, 1.5, 2.0, 2.0, 3.0], dtype=float),
        y_cable=np.array([0.0, 0.0, 0.0, -0.5, -0.8, -0.8, -1.2], dtype=float),
        l_target=3.6,
        n_segments=6,
        n_target=6,
        boat=(0.0, 0.0),
        rov=(3.0, -1.2),
    ),
    SharedCableCase(
        name="Clipping agressif",
        x_cable=np.array([0.0, 0.2, 0.7, 1.4, 2.2, 3.0], dtype=float),
        y_cable=np.array([0.8, 0.6, 0.3, -0.2, -0.7, -1.1], dtype=float),
        l_target=3.5,
        n_segments=5,
        n_target=5,
        boat=(0.0, 0.0),
        rov=(3.0, -1.1),
    ),
    SharedCableCase(
        name="Compression geometrique",
        x_cable=np.array([0.0, 1.0, 2.5, 4.0, 5.0], dtype=float),
        y_cable=np.array([0.0, -0.5, -1.2, -1.6, -2.0], dtype=float),
        l_target=2.0,
        n_segments=4,
        n_target=4,
        boat=(0.0, 0.0),
        rov=(5.0, -2.0),
    ),
    SharedCableCase(
        name="Geometrie degeneree",
        x_cable=np.array([1.0, 1.0, 1.0, 1.0, 1.0], dtype=float),
        y_cable=np.array([-2.0, -2.0, -2.0, -2.0, -2.0], dtype=float),
        l_target=4.0,
        n_segments=4,
        n_target=4,
        boat=(1.0, -2.0),
        rov=(1.0, -2.0),
    ),
    SharedCableCase(
        name="Dernier segment tres long",
        x_cable=np.array([0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 5.5], dtype=float),
        y_cable=np.array([0.0, -0.3, -0.6, -0.9, -1.2, -1.5, -2.5], dtype=float),
        l_target=6.0,
        n_segments=6,
        n_target=6,
        boat=(0.0, 0.0),
        rov=(5.5, -2.5),
    ),
    SharedCableCase(
        name="Cas simple 1",
        x_cable=np.array([0.2, 1.9, 2, 2.6, 2.8, 3.2], dtype=float),
        y_cable=np.array([-0.2, -1.8, -3.9, -5.9, -7.9, -9.9], dtype=float),
        l_target=10.8,
        n_segments=10,
        n_target=10,
        boat=(0.0, 0.0),
        rov=(3.0, -10.0),
    ),
    SharedCableCase(
        name="Cas simple 2",
        x_cable=np.array([-0.2, 1.9, 2, 2.6, 2.8, 3.2], dtype=float),
        y_cable=np.array([-0.2, -1.8, -3.9, -5.9, -7.9, -9.9], dtype=float),
        l_target=10.8,
        n_segments=10,
        n_target=10,
        boat=(0.0, 0.0),
        rov=(3.0, -10.0),
    ),
    SharedCableCase(
        name="Cas simple 3",
        x_cable=np.array([0.3, 1.9, 2, 2.6, 2.8, 2.8], dtype=float),
        y_cable=np.array([-0.2, -1.8, -3.9, -5.9, -7.9, -9.6], dtype=float),
        l_target=10.8,
        n_segments=10,
        n_target=10,
        boat=(0.0, 0.0),
        rov=(3.0, -10.0),
    ),
    SharedCableCase(
        name="Cas réaliste 1",
        x_cable=np.array([0.2, 1, 1.6, 1.8, 2.1, 2.4, 2.5, 2.7, 3, 3.1, 3.2], dtype=float),
        y_cable=np.array([-1.0, -1.9, -2.1, -3.3, -4.1, -5, -6.1, -7.1, -8.2, -9.1, -10.1], dtype=float),
        l_target=11.5,
        n_segments=5,
        n_target=5,
        boat=(0.0, 0.0),
        rov=(3.2, -9.9),
    ),
    SharedCableCase(
        name="Fallback : L_target trop court (corde bateau–ROV > cible, droite)",
        x_cable=np.array([0.0, 10.0], dtype=float),
        y_cable=np.array([0.0, -2.0], dtype=float),
        l_target=5.0,
        n_segments=6,
        n_target=6,
        boat=(0.0, 0.0),
        rov=(10.0, -2.0),
    ),
)


def get_shared_case(name: str) -> SharedCableCase:
    """Retourne un cas par nom exact (lève ``KeyError`` si inconnu)."""
    for c in ALL_SHARED_CABLE_CASES:
        if c.name == name:
            return c
    raise KeyError(name)


# Alias pratique pour les tests unitaires (même scénario que l’ancien test fallback)
CASE_FALLBACK_LT_SHORT: SharedCableCase = get_shared_case(
    "Fallback : L_target trop court (corde bateau–ROV > cible, droite)"
)

__all__ = [
    "ALL_SHARED_CABLE_CASES",
    "CASE_FALLBACK_LT_SHORT",
    "SharedCableCase",
    "get_shared_case",
]
