"""Tests de cas limites pour la normalisation du câble."""
import numpy as np

from src.models.environment import Environment
from src.solvers.cable_solver import CableSolver


def _make_solver(n_segments=6):
    """Crée un solveur minimal pour tester la renormalisation."""
    params = {
        'd': 0.01,
        'rho_cable': 1500.0,
        'Cx_cable': 1.2,
        'Cf_cable': 0.04,
    }
    env = Environment({'rho_eau': 1025.0, 'g': 9.81})
    return CableSolver(n_segments, params, env)


def _segment_lengths(x_vals, y_vals):
    """Retourne les longueurs des segments d'une polyligne."""
    x_vals = np.asarray(x_vals, dtype=float)
    y_vals = np.asarray(y_vals, dtype=float)
    dx = np.diff(x_vals)
    dy = np.diff(y_vals)
    return np.sqrt(dx**2 + dy**2)


def _ds_stats(lengths, L_target):
    """Retourne (ds_target, ratio_max) pour une longueur totale donnée."""
    n_seg = len(lengths)
    if n_seg == 0 or L_target <= 0.0:
        return 0.0, 0.0
    ds_target = L_target / float(n_seg)
    ds_max = float(np.max(lengths))
    ratio_max = ds_max / max(ds_target, 1e-9)
    return ds_target, ratio_max


def test_normalize_cable_length_handles_repeated_points():
    """La renormalisation doit gérer des points répétés sans NaN."""
    solver = _make_solver(n_segments=6)
    x_cable = np.array([0.0, 0.0, 0.0, 1.5, 2.0, 2.0, 3.0], dtype=float)
    y_cable = np.array([0.0, 0.0, 0.0, -0.5, -0.8, -0.8, -1.2], dtype=float)
    l_target = 3.6

    x_norm, y_norm = solver._normalize_cable_length(x_cable, y_cable, l_target)
    lengths = _segment_lengths(x_norm, y_norm)
    ds_target, ratio_max = _ds_stats(lengths, l_target)

    # Pas de NaN / inf
    assert np.all(np.isfinite(x_norm))
    assert np.all(np.isfinite(y_norm))

    # Longueur totale correcte
    assert np.isclose(np.sum(lengths), l_target, atol=1e-6)

    # Les segments n'ont plus besoin d'être strictement égaux, mais aucun
    # segment ne doit exploser par rapport à la longueur moyenne.
    assert ratio_max < 3.0, f"ds_max/ds_target trop grand: {ratio_max:.3f} (ds_target={ds_target:.6f})"


def test_normalize_cable_length_handles_strong_surface_clipping():
    """Le clipping agressif en surface doit laisser une géométrie exploitable."""
    solver = _make_solver(n_segments=5)
    x_cable = np.array([0.0, 0.2, 0.7, 1.4, 2.2, 3.0], dtype=float)
    y_cable = np.array([0.8, 0.6, 0.3, -0.2, -0.7, -1.1], dtype=float)
    l_target = 3.5

    x_norm, y_norm = solver._normalize_cable_length(x_cable, y_cable, l_target)
    lengths = _segment_lengths(x_norm, y_norm)
    ds_target, ratio_max = _ds_stats(lengths, l_target)

    # Tous les points doivent être sous ou au niveau de la surface
    assert np.all(y_norm <= 1e-12)

    # Longueur totale correcte et segments raisonnables
    assert np.all(np.isfinite(lengths))
    assert np.isclose(np.sum(lengths), l_target, atol=1e-6)
    assert ratio_max < 3.0, f"ds_max/ds_target trop grand: {ratio_max:.3f} (ds_target={ds_target:.6f})"


def test_normalize_cable_length_compresses_geometry_to_shorter_target():
    """Une géométrie longue doit pouvoir être compressée vers une longueur plus courte."""
    solver = _make_solver(n_segments=4)
    x_cable = np.array([0.0, 1.0, 2.5, 4.0, 5.0], dtype=float)
    y_cable = np.array([0.0, -0.5, -1.2, -1.6, -2.0], dtype=float)
    l_target = 2.0

    x_norm, y_norm = solver._normalize_cable_length(x_cable, y_cable, l_target)
    lengths = _segment_lengths(x_norm, y_norm)
    ds_target, ratio_max = _ds_stats(lengths, l_target)

    # Longueur totale correcte
    assert np.isclose(np.sum(lengths), l_target, atol=1e-6)

    # Segments raisonnablement homogènes
    assert ratio_max < 3.0, f"ds_max/ds_target trop grand: {ratio_max:.3f} (ds_target={ds_target:.6f})"

    # Clipping surface
    assert np.all(y_norm <= 1e-12)


def test_normalize_cable_length_zero_length_geometry_should_expand_to_target():
    """Cas limite documenté : une géométrie totalement dégénérée devrait atteindre L_target."""
    solver = _make_solver(n_segments=4)
    x_cable = np.array([1.0, 1.0, 1.0, 1.0, 1.0], dtype=float)
    y_cable = np.array([-2.0, -2.0, -2.0, -2.0, -2.0], dtype=float)
    l_target = 4.0

    x_norm, y_norm = solver._normalize_cable_length(x_cable, y_cable, l_target)
    lengths = _segment_lengths(x_norm, y_norm)
    ds_target, ratio_max = _ds_stats(lengths, l_target)

    # Longueur totale correcte
    assert np.isclose(np.sum(lengths), l_target, atol=1e-6)

    # Dans ce cas dégénéré, la reconstruction est rectiligne donc les
    # segments devraient être très proches de ds_target.
    assert ratio_max < 1.1, f"ds_max/ds_target inattendu pour géométrie dégénérée: {ratio_max:.3f}"

    # Tous les points doivent rester sous la surface
    assert np.all(y_norm <= 1e-12)


def test_normalize_cable_length_with_long_last_segment():
    """
    Vérifie que la normalisation fonctionne correctement quand le dernier segment
    est nettement plus long que les autres avant normalisation.
    
    Ce test vérifie spécifiquement que même si le dernier segment initial est
    beaucoup plus long (par exemple 3-4x la longueur des autres segments),
    après normalisation tous les segments ont exactement la même longueur.
    """
    solver = _make_solver(n_segments=6)
    
    # Créer une géométrie où les premiers segments sont courts (~0.5 m chacun)
    # et le dernier segment est nettement plus long (~3.0 m)
    # Longueur totale initiale : ~4.5 m
    x_cable = np.array([0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 5.5], dtype=float)
    y_cable = np.array([0.0, -0.3, -0.6, -0.9, -1.2, -1.5, -2.5], dtype=float)
    
    # Vérifier que le dernier segment est effectivement beaucoup plus long
    initial_lengths = _segment_lengths(x_cable, y_cable)
    assert len(initial_lengths) == 6
    # Les 5 premiers segments devraient être ~0.5 m chacun
    assert np.allclose(initial_lengths[:-1], 0.5, atol=0.1)
    # Le dernier segment devrait être ~3.0 m (beaucoup plus long)
    assert initial_lengths[-1] > 2.5  # Vérifie que c'est nettement plus long
    
    # Normaliser vers une longueur cible de 6.0 m
    l_target = 6.0
    x_norm, y_norm = solver._normalize_cable_length(x_cable, y_cable, l_target)
    lengths = _segment_lengths(x_norm, y_norm)
    ds_target, ratio_max = _ds_stats(lengths, l_target)

    # Vérifications après normalisation
    assert len(x_norm) == 7  # N+1 points
    assert len(y_norm) == 7
    assert len(lengths) == 6  # N segments

    # Longueur totale respectée
    assert np.isclose(np.sum(lengths), l_target, atol=1e-6), \
        f"Longueur totale incorrecte: {np.sum(lengths):.6f} au lieu de {l_target:.6f}"

    # Contrainte souple : aucun segment ne doit être beaucoup plus long que la moyenne
    assert ratio_max < 3.0, \
        f"ds_max/ds_target trop grand après normalisation: {ratio_max:.3f} (ds_target={ds_target:.6f}, lengths={lengths})"

    # Vérifier que la forme générale est préservée (les points ne sont pas tous au même endroit)
    assert np.max(x_norm) > np.min(x_norm) or np.max(y_norm) < np.min(y_norm), \
        "La géométrie semble dégénérée après normalisation"

    # Vérifier la contrainte de surface
    assert np.all(y_norm <= 1e-12), "Certains points sont au-dessus de la surface"
