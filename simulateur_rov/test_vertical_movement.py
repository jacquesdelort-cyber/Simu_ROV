"""
Script de test pour vérifier que le ROV peut se déplacer verticalement
quand une force verticale est appliquée.
"""
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from src.models.system_model import ROVSystem
from src.utils.parameters import get_default_parameters
from src.utils.initial_conditions import get_initial_state
import numpy as np

def test_vertical_movement():
    """Test que le ROV se déplace verticalement avec une force verticale appliquée"""
    print("=" * 60)
    print("Test du mouvement vertical du ROV")
    print("=" * 60)
    
    # Initialiser le système
    params = get_default_parameters()
    system = ROVSystem(params, N_segments=50)
    
    # Conditions initiales : ROV à 10 m de profondeur
    y_rov_init = 10.0
    y0 = get_initial_state(
        system,
        x_rov=0.0,
        y_rov=y_rov_init,
        x_boat=0.0,
        L=50.0
    )
    
    # Dépaqueter l'état initial
    (x_rov_init, y_rov_init_state, vx_rov_init, vy_rov_init,
     x_boat_init, vx_boat_init, x_cable_init, y_cable_init, T_init, L_init) = system.unpack_state(y0)
    
    print(f"\nÉtat initial:")
    print(f"  Position ROV: ({x_rov_init:.2f}, {y_rov_init_state:.2f}) m")
    print(f"  Vitesse ROV: ({vx_rov_init:.2f}, {vy_rov_init:.2f}) m/s")
    print(f"  Tension initiale au ROV: {T_init[0] if len(T_init) > 0 else 0.0:.2f} N")
    
    # Calculer le poids apparent
    F_buoyancy = system.rov.compute_buoyancy_force(system.environment)
    F_weight = system.rov.compute_weight_force(system.environment)
    F_apparent_weight = F_weight - F_buoyancy
    print(f"  Poids apparent: {F_apparent_weight:.2f} N")
    
    # Appliquer une force verticale vers le haut (positive = vers la surface)
    Fy_rov_applied = 20.0  # 20 N vers le haut
    print(f"\nForce verticale appliquée: {Fy_rov_applied:.2f} N (vers le haut)")
    
    # Fonction de commande avec force verticale
    def u_func(t):
        return {
            'Fx_rov': 0.0,           # Pas de force horizontale
            'Fy_rov': Fy_rov_applied,  # Force verticale vers le haut
            'vx_boat_cmd': 0.0,      # Bateau immobile
            'dL_dt': 0.0             # Longueur constante
        }
    
    # Simuler sur une courte durée (5 secondes)
    print("\nSimulation sur 5 secondes...")
    t_span = [0.0, 5.0]
    solution = system.integrate(t_span, y0, u_func, dt_max=0.1)
    
    if not solution.success:
        print(f"[ECHEC] Echec de la simulation: {solution.message}")
        return False
    
    print(f"[OK] Simulation reussie: {len(solution.t)} points calcules")
    
    # Analyser les résultats
    y_final = solution.sol(solution.t[-1])
    (x_rov_final, y_rov_final, vx_rov_final, vy_rov_final,
     x_boat_final, vx_boat_final, x_cable_final, y_cable_final, T_final, L_final) = system.unpack_state(y_final)
    
    print(f"\nEtat final (t = {solution.t[-1]:.2f} s):")
    print(f"  Position ROV: ({x_rov_final:.2f}, {y_rov_final:.2f}) m")
    print(f"  Vitesse ROV: ({vx_rov_final:.2f}, {vy_rov_final:.2f}) m/s")
    print(f"  Tension finale au ROV: {T_final[0] if len(T_final) > 0 else 0.0:.2f} N")
    
    # Vérifier le mouvement vertical
    delta_y = y_rov_final - y_rov_init_state
    print(f"\n{'=' * 60}")
    print(f"Resultat du test:")
    print(f"  Variation de profondeur: {delta_y:.3f} m")
    
    if abs(delta_y) > 0.01:  # Au moins 1 cm de mouvement
        if delta_y > 0:
            print(f"  [SUCCES] Le ROV s'est deplace vers le haut de {delta_y:.3f} m")
        else:
            print(f"  [SUCCES] Le ROV s'est deplace vers le bas de {abs(delta_y):.3f} m")
        print(f"  [SUCCES] La correction fonctionne: le mouvement vertical est possible")
        return True
    else:
        print(f"  [ECHEC] Le ROV n'a pas bouge verticalement (variation < 1 cm)")
        print(f"  [ECHEC] La correction ne fonctionne pas correctement")
        return False

if __name__ == '__main__':
    success = test_vertical_movement()
    sys.exit(0 if success else 1)
