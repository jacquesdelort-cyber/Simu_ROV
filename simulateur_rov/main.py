"""Script principal pour lancer une simulation"""
from src.models.system_model import ROVSystem
from src.utils.parameters import get_default_parameters
from src.utils.initial_conditions import get_initial_state
from src.visualization.plotter import create_system_plot
from src.utils.data_io import save_results
import numpy as np


def main():
    """Fonction principale"""
    print("Initialisation du système ROV...")
    
    # Charger les paramètres
    params = get_default_parameters()
    
    # Initialiser le système
    system = ROVSystem(params, N_segments=50)
    
    # Conditions initiales
    y0 = get_initial_state(
        system,
        x_rov=0.0,
        y_rov=-10.0,  # Profondeur négative (convention : y < 0 = sous la surface)
        x_boat=0.0,
        L=50.0
    )
    
    # Fonction de commande (exemple : constantes)
    def u_func(t):
        return {
            'Fx_rov': 10.0,      # Force horizontale ROV (N)
            'Fy_rov': 0.0,       # Force verticale ROV (N)
            'vx_boat_cmd': 0.5,  # Vitesse bateau (m/s)
            'dL_dt': 0.0         # Pas de variation de longueur (m/s)
        }
    
    # Simulation
    print("Démarrage de la simulation...")
    t_span = [0, 60]  # 60 secondes
    solution = system.integrate(t_span, y0, u_func, dt_max=0.1)
    
    print(f"Simulation terminée. {len(solution.t)} points calculés.")
    
    # Visualisation
    print("Génération des graphiques...")
    
    # Obtenir l'état final
    y_final = solution.sol(solution.t[-1])
    (x_rov, y_rov, _, _,
     x_boat, _, x_cable, y_cable, _, L) = system.unpack_state(y_final)
    
    fig = create_system_plot(x_rov, y_rov, x_cable, y_cable, x_boat, L)
    fig.show()
    
    # Sauvegarde
    print("Sauvegarde des résultats...")
    save_results(system, solution, 'results/simulation_001.h5')
    print("Résultats sauvegardés dans results/simulation_001.h5")


if __name__ == '__main__':
    main()

