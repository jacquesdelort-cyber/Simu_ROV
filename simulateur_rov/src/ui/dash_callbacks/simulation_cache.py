"""Cache global pour les objets non sérialisables de la simulation"""
# Cache global pour les objets non sérialisables (ROVSystem, etc.)
# Utilisé pour stocker les objets qui ne peuvent pas être sérialisés dans les stores Dash

_simulation_cache = {}
