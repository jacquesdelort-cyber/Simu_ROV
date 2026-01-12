"""
Exemple d'utilisation du simulateur ROV
"""
from src.models.system_model import ROVSystem
from src.utils.parameters import get_default_parameters
from src.utils.initial_conditions import get_initial_state
from src.visualization.plotter import (
    create_system_plot, create_position_plot, 
    create_velocity_plot
)
import numpy as np


def exemple_simulation_simple():
    """Exemple de simulation simple"""
    print("=== Exemple 1: Simulation Simple ===")
    
    # 1. Initialiser le système
    params = get_default_parameters()
    system = ROVSystem(params, N_segments=50)
    
    # 2. Définir les conditions initiales
    y0 = get_initial_state(
        system,
        x_rov=0.0,      # ROV à l'origine horizontale
        y_rov=10.0,     # Profondeur de 10 m
        x_boat=0.0,     # Bateau aussi à l'origine
        L=50.0          # Câble de 50 m
    )
    
    # 3. Définir les commandes (constantes)
    def u_func(t):
        return {
            'Fx_rov': 10.0,      # Force horizontale de 10 N
            'Fy_rov': 0.0,       # Pas de force verticale
            'vx_boat_cmd': 0.5,  # Bateau avance à 0.5 m/s
            'dL_dt': 0.0         # Longueur du câble constante
        }
    
    # 4. Lancer la simulation
    print("Lancement de la simulation...")
    solution = system.integrate([0, 60], y0, u_func, dt_max=0.1)
    print(f"Simulation terminée: {len(solution.t)} points calculés")
    
    # 5. Visualiser l'état final
    y_final = solution.sol(solution.t[-1])
    (x_rov, y_rov, _, _,
     x_boat, _, x_cable, y_cable, _, L) = system.unpack_state(y_final)
    
    fig = create_system_plot(x_rov, y_rov, x_cable, y_cable, x_boat, L)
    fig.show()
    
    return solution, system


def exemple_commande_temps_variable():
    """Exemple avec commandes variables dans le temps"""
    print("\n=== Exemple 2: Commandes Variables ===")
    
    params = get_default_parameters()
    system = ROVSystem(params, N_segments=50)
    
    y0 = get_initial_state(system, x_rov=0.0, y_rov=10.0, L=50.0)
    
    # Commande qui change avec le temps
    def u_func(t):
        if t < 20:
            # Phase 1: Descente
            return {'Fx_rov': 0.0, 'Fy_rov': 20.0, 'vx_boat_cmd': 0.0, 'dL_dt': 0.2}
        elif t < 40:
            # Phase 2: Déplacement horizontal
            return {'Fx_rov': 30.0, 'Fy_rov': 0.0, 'vx_boat_cmd': 1.0, 'dL_dt': 0.0}
        else:
            # Phase 3: Remontée
            return {'Fx_rov': 0.0, 'Fy_rov': -30.0, 'vx_boat_cmd': 0.0, 'dL_dt': -0.2}
    
    solution = system.integrate([0, 60], y0, u_func, dt_max=0.1)
    
    # Extraire les données
    time = solution.t
    n_points = len(time)
    x_rov = np.zeros(n_points)
    y_rov = np.zeros(n_points)
    vx_rov = np.zeros(n_points)
    vy_rov = np.zeros(n_points)
    
    for i, t in enumerate(time):
        y = solution.sol(t)
        x_rov[i], y_rov[i], vx_rov[i], vy_rov[i], _, _, _, _, _, _ = system.unpack_state(y)
    
    # Visualiser
    fig_pos = create_position_plot(time, x_rov, y_rov)
    fig_pos.show()
    
    fig_vel = create_velocity_plot(time, vx_rov, vy_rov)
    fig_vel.show()
    
    return solution


if __name__ == '__main__':
    # Exécuter les exemples
    solution1, system1 = exemple_simulation_simple()
    solution2 = exemple_commande_temps_variable()
    
    print("\n=== Exemples terminés ===")

