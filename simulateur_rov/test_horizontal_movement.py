"""
Script de test pour vérifier que le ROV se déplace correctement horizontalement
quand une force horizontale positive est appliquée.
"""
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from src.models.system_model import ROVSystem
from src.utils.parameters import get_default_parameters
from src.utils.initial_conditions import get_initial_state
import numpy as np

def test_horizontal_movement():
    """Test que le ROV se déplace vers la droite avec une force horizontale positive"""
    print("=" * 60)
    print("Test du mouvement horizontal du ROV")
    print("=" * 60)
    
    # Initialiser le système
    params = get_default_parameters()
    system = ROVSystem(params, N_segments=50)
    
    # Conditions initiales : ROV à 10 m de profondeur, à l'origine
    y0 = get_initial_state(
        system,
        x_rov=0.0,
        y_rov=10.0,
        x_boat=0.0,
        L=50.0
    )
    
    # Dépaqueter l'état initial
    (x_rov_init, y_rov_init, vx_rov_init, vy_rov_init,
     x_boat_init, vx_boat_init, x_cable_init, y_cable_init, T_init, L_init) = system.unpack_state(y0)
    
    print(f"\nEtat initial:")
    print(f"  Position ROV: ({x_rov_init:.2f}, {y_rov_init:.2f}) m")
    print(f"  Vitesse ROV: ({vx_rov_init:.2f}, {vy_rov_init:.2f}) m/s")
    
    # Appliquer une force horizontale positive (vers la droite)
    Fx_rov_applied = 50.0  # 50 N vers la droite
    print(f"\nForce horizontale appliquee: {Fx_rov_applied:.2f} N (vers la droite, x positif)")
    
    # Fonction de commande avec force horizontale
    def u_func(t):
        return {
            'Fx_rov': Fx_rov_applied,  # Force horizontale vers la droite
            'Fy_rov': 0.0,             # Pas de force verticale
            'vx_boat_cmd': 0.0,        # Bateau immobile
            'dL_dt': 0.0               # Longueur constante
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
    
    # Vérifier le mouvement horizontal
    delta_x = x_rov_final - x_rov_init
    print(f"\n{'=' * 60}")
    print(f"Resultat du test:")
    print(f"  Variation horizontale: {delta_x:.3f} m")
    
    if delta_x > 0.01:
        print(f"  [SUCCES] Le ROV s'est deplace vers la droite de {delta_x:.3f} m")
        print(f"  [SUCCES] Le mouvement est correct")
        return True
    elif delta_x < -0.01:
        print(f"  [ECHEC] Le ROV s'est deplace vers la GAUCHE de {abs(delta_x):.3f} m")
        print(f"  [ECHEC] PROBLEME: Une force positive devrait deplacer vers la droite!")
        return False
    else:
        print(f"  [ECHEC] Le ROV n'a pas bouge horizontalement (variation < 1 cm)")
        return False

if __name__ == '__main__':
    success = test_horizontal_movement()
    sys.exit(0 if success else 1)
