"""
Test détaillé pour diagnostiquer le problème de mouvement horizontal
"""
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from src.models.system_model import ROVSystem
from src.utils.parameters import get_default_parameters
from src.utils.initial_conditions import get_initial_state
import numpy as np

def test_horizontal_detailed():
    """Test détaillé avec affichage des forces"""
    print("=" * 60)
    print("Test detaille du mouvement horizontal")
    print("=" * 60)
    
    # Initialiser le système
    params = get_default_parameters()
    system = ROVSystem(params, N_segments=50)
    
    # Conditions initiales
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
    print(f"  Position Bateau: ({x_boat_init:.2f}, 0.00) m")
    
    # Appliquer une force horizontale positive
    Fx_rov_applied = 50.0
    print(f"\nForce horizontale appliquee: {Fx_rov_applied:.2f} N (vers la droite)")
    
    # Fonction de commande
    def u_func(t):
        return {
            'Fx_rov': Fx_rov_applied,
            'Fy_rov': 0.0,
            'vx_boat_cmd': 0.0,
            'dL_dt': 0.0
        }
    
    # Simuler sur une courte durée
    print("\nSimulation sur 2 secondes...")
    t_span = [0.0, 2.0]
    solution = system.integrate(t_span, y0, u_func, dt_max=0.1)
    
    if not solution.success:
        print(f"[ECHEC] Echec: {solution.message}")
        return False
    
    # Analyser à plusieurs instants
    print("\nAnalyse des forces a differents instants:")
    print("-" * 60)
    
    for i in [0, len(solution.t)//4, len(solution.t)//2, len(solution.t)-1]:
        t = solution.t[i]
        y = solution.sol(t)
        (x_rov, y_rov, vx_rov, vy_rov,
         x_boat, vx_boat, x_cable, y_cable, T, L) = system.unpack_state(y)
        
        # Calculer les forces manuellement pour vérifier
        Fx_drag, Fy_drag = system.rov.compute_drag_force(vx_rov, vy_rov, y_rov, system.environment)
        
        # Calculer l'angle du câble
        if len(x_cable) > 1:
            # Après inversion: bateau à index 0, ROV à index -1
            dx_cable = x_cable[-2] - x_cable[-1]  # Du ROV vers le point adjacent (vers bateau)
            dy_cable = y_cable[-2] - y_cable[-1]
            ds = np.sqrt(dx_cable**2 + dy_cable**2)
            if ds > 1e-6:
                cos_theta0 = dx_cable / ds
                sin_theta0 = dy_cable / ds
            else:
                cos_theta0 = 0.0
                sin_theta0 = 1.0
        else:
            cos_theta0 = 0.0
            sin_theta0 = 1.0
        
        T0 = T[0] if len(T) > 0 else 0.0
        Fx_tension = T0 * cos_theta0
        Fx_total = Fx_drag + Fx_tension + Fx_rov_applied
        
        print(f"\nt = {t:.2f} s:")
        print(f"  Position ROV: ({x_rov:.2f}, {y_rov:.2f}) m")
        print(f"  Position Bateau: ({x_boat:.2f}, 0.00) m")
        print(f"  Vitesse ROV: ({vx_rov:.2f}, {vy_rov:.2f}) m/s")
        print(f"  Forces:")
        print(f"    Fx_drag: {Fx_drag:.2f} N")
        print(f"    Fx_tension (T0*cos): {Fx_tension:.2f} N (T0={T0:.2f}, cos={cos_theta0:.3f})")
        print(f"    Fx_applied: {Fx_rov_applied:.2f} N")
        print(f"    Fx_total: {Fx_total:.2f} N")
        print(f"    Accel_x: {Fx_total/system.rov.m:.3f} m/s^2")
    
    # Résultat final
    y_final = solution.sol(solution.t[-1])
    (x_rov_final, y_rov_final, vx_rov_final, vy_rov_final,
     x_boat_final, vx_boat_final, x_cable_final, y_cable_final, T_final, L_final) = system.unpack_state(y_final)
    
    delta_x = x_rov_final - x_rov_init
    print(f"\n{'=' * 60}")
    print(f"Resultat:")
    print(f"  Variation x: {delta_x:.3f} m")
    if delta_x < 0:
        print(f"  [PROBLEME] Le ROV s'est deplace vers la GAUCHE avec une force positive!")
    else:
        print(f"  [OK] Le ROV s'est deplace vers la droite")
    
    return delta_x > 0

if __name__ == '__main__':
    success = test_horizontal_detailed()
    sys.exit(0 if success else 1)
