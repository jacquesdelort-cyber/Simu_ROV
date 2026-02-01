"""Solveur pour les équations du câble"""
import numpy as np
from scipy.optimize import fsolve, minimize_scalar, root_scalar, minimize
from src.utils.logger import trace_print


class CableSolver:
    """Résout les équations d'équilibre du câble"""
    
    def __init__(self, N_segments, params, environment):
        """
        Initialise le solveur de câble
        
        Parameters:
        -----------
        N_segments : int
            Nombre de segments pour discrétiser le câble
        params : dict
            Paramètres du câble
        environment : Environment
            Objet environnement
        """
        self.N = N_segments
        self.params = params
        self.environment = environment
        self.d = params['d']
        self.rho_cable = params['rho_cable']
        self.Cx_cable = params['Cx_cable']
        self.A_cable = np.pi * (self.d / 2)**2
    
    def solve_equilibrium_static(self, x_rov, y_rov, x_boat, L, rov_m=None, rov_vol=None):
        """
        Résout l'équilibre statique du câble (caténaire)
        
        Calcule la forme de caténaire réelle si rho_cable > rho_eau,
        sinon le câble est rectiligne (flotte).
        
        Parameters:
        -----------
        x_rov : float
            Position horizontale du ROV (m)
        y_rov : float
            Position verticale du ROV (m) (négative, profondeur : y < 0 = sous la surface)
        x_boat : float
            Position horizontale du bateau (m)
        L : float
            Longueur du câble (m)
        rov_m : float, optional
            Masse du ROV (kg) - nécessaire pour calculer la tension initiale
        rov_vol : float, optional
            Volume du ROV (m³) - nécessaire pour calculer la tension initiale
        
        Returns:
        --------
        tuple (x_cable, y_cable, T)
            Positions et tensions du câble
            x_cable, y_cable : arrays de taille (N+1)
            T : array de tensions de taille (N+1)
        """

        trace_print(1, "\n[DEBUG] Initialisation du câble : pas de prise en compte du courant (solve_equilibrium_static).")
        
        if L <= 0:
            # Câble de longueur nulle
            return np.array([x_rov, x_boat]), np.array([y_rov, 0.0]), np.array([0.0, 0.0])
        
        # Calculer le poids apparent par unité de longueur
        weight_per_unit = (self.rho_cable - self.environment.rho_eau) * self.A_cable * self.environment.g
        
        # Distance horizontale et verticale entre les extrémités
        dx = x_boat - x_rov
        dy = -y_rov  # Profondeur (y_rov est négatif)
        
        # Longueur rectiligne
        L_straight = np.sqrt(dx**2 + dy**2)
        
        # Si le câble est plus léger que l'eau ou si L < L_straight, câble tendu
        if weight_per_unit <= 0 or L < L_straight:
            # Câble tendu (rectiligne)
            # Convention : index 0 = bateau, index -1 = ROV (s=0 au bateau, s=L au ROV)
            trace_print(1, "\n[DEBUG] Initialisation du câble : câble tendu (solve_equilibrium_static).")
            if L < L_straight:
                trace_print(
                    10,
                    f"[DEBUG] ⚠️  : L < L_straight (L={L:.6f}, L_straight={L_straight:.6f}). "
                    "La géométrie rectiligne impose L = L_straight."
                )
                L = L_straight
            x_cable = np.linspace(x_boat, x_rov, self.N + 1) 
            y_cable = np.linspace(0.0, y_rov, self.N + 1)
            
            # Calculer les tensions en utilisant l'équilibre des forces
            # T_rov * Urov + T_bateau * Ubateau + Fext_stat = 0
            # où Fext_stat inclut le courant et le poids du câble
            if self.N > 0:
                # Vecteurs unitaires aux extrémités
                dx_bateau = x_cable[1] - x_cable[0]
                dy_bateau = y_cable[1] - y_cable[0]
                ds_bateau = np.sqrt(dx_bateau**2 + dy_bateau**2)
                
                dx_rov = x_cable[-1] - x_cable[-2]
                dy_rov = y_cable[-1] - y_cable[-2]
                ds_rov = np.sqrt(dx_rov**2 + dy_rov**2)
                
                if ds_bateau > 1e-6 and ds_rov > 1e-6:
                    Ubateau_x = dx_bateau / ds_bateau
                    Ubateau_y = dy_bateau / ds_bateau
                    
                    Urov_x = dx_rov / ds_rov
                    Urov_y = dy_rov / ds_rov
                    
                    # Calculer les forces externes statiques sur le câble
                    # Ces forces incluent : le courant (traînée horizontale) et le poids apparent du câble (force verticale)
                    from .forces import compute_cable_forces
                    params_cable = {
                        'd': self.d,
                        'rho_cable': self.rho_cable,
                        'Cx_cable': self.Cx_cable
                    }
                    
                    vx_cable = np.zeros(len(x_cable))
                    vy_cable = np.zeros(len(x_cable))
                    
                    Fx_courant_segments, Fy_courant_segments = compute_cable_forces(
                        x_cable, y_cable, vx_cable, vy_cable, self.environment, params_cable, L
                    )
                    
                    # Forces externes statiques totales (courant + poids du câble)
                    Fext_stat_x = np.sum(Fx_courant_segments)
                    Fext_stat_y = np.sum(Fy_courant_segments)
                    
                    # Résoudre le système d'équations
                    # Équations d'équilibre :
                    # -Ubx*T_bateau + Urx*T_rov + Fex = 0  (horizontale)
                    # -Uby*T_bateau + Ury*T_rov + Fey = 0  (verticale)
                    # où :
                    # - Fex = Force exercée par le courant sur le câble (dans le sens de la vitesse apparente du courant)
                    # - Fey = Poids apparent du câble (négatif pour un câble plus dense que l'eau, dirigé vers le bas)
                    # 
                    # Sous forme matricielle : A * [T_bateau, T_rov]^T = [-Fex, -Fey]^T
                    A = np.array([
                        [-Ubateau_x, Urov_x],
                        [-Ubateau_y, Urov_y]
                    ])
                    b = np.array([-Fext_stat_x, -Fext_stat_y])
                    
                    # Debug: Afficher les équations sous forme littérale
                    trace_print(1, "[DEBUG] Résolution du système linéaire pour tensions initiales (câble tendu)")
                    trace_print(1, "[DEBUG] Équations littérales:")
                    trace_print(1, f"  Équation 1 (horizontale): -Ubx*T_bateau + Urx*T_rov + Fex = 0")
                    trace_print(1, f"  Équation 2 (verticale):   -Uby*T_bateau + Ury*T_rov + Fey = 0")
                    trace_print(1, f"[DEBUG] Note: Fex = force du courant, Fey = poids apparent (négatif pour câble plus dense que l'eau)")
                    trace_print(1, f"[DEBUG] Coefficients: Urov_x={Urov_x:.6f}, Urov_y={Urov_y:.6f}, Ubateau_x={Ubateau_x:.6f}, Ubateau_y={Ubateau_y:.6f}")
                    trace_print(1, f"[DEBUG] Forces externes: Fext_stat_x={Fext_stat_x:.6f}, Fext_stat_y={Fext_stat_y:.6f}")
                    det_A = np.linalg.det(A)
                    straight_ratio = L_straight / max(L, 1e-12)
                    trace_print(1, f"[DEBUG] Déterminant de A: {det_A:.6f}")
                    trace_print(1, f"[DEBUG] Ratio L_straight/L: {straight_ratio:.6f}")
                    
                    try:
                        # Cas quasi-rectiligne ou système singulier : résoudre sur l'axe du câble
                        if abs(det_A) < 1e-8 or straight_ratio > 0.995:
                            trace_print(
                                10,
                                "[DEBUG] ⚠️  : câble quasi-rectiligne ou A singulière. "
                                "Projection des forces sur l'axe du câble."
                            )
                            u_dir = np.array([0.5 * (Ubateau_x + Urov_x), 0.5 * (Ubateau_y + Urov_y)])
                            norm_u = np.linalg.norm(u_dir)
                            if norm_u < 1e-12:
                                u_dir = np.array([Urov_x, Urov_y])
                                norm_u = np.linalg.norm(u_dir)
                            if norm_u < 1e-12:
                                u_dir = np.array([0.0, -1.0])
                                norm_u = 1.0
                            u_dir = u_dir / norm_u
                            
                            Fext = np.array([Fext_stat_x, Fext_stat_y])
                            Fext_parallel = float(np.dot(Fext, u_dir))
                            Fext_perp = Fext - Fext_parallel * u_dir
                            
                            if np.linalg.norm(Fext_perp) > 1e-3:
                                trace_print(
                                    10,
                                    f"[DEBUG] ⚠️  : composante perpendiculaire ignorée (|F⊥|={np.linalg.norm(Fext_perp):.6f})."
                                )
                            
                            delta_T = -Fext_parallel
                            T_base_est = max(abs(weight_per_unit) * L / 10.0 if weight_per_unit != 0 else 100.0, 10.0)
                            T_base = max(T_base_est, abs(delta_T) / 2.0)
                            T_bateau = max(0.0, T_base - 0.5 * delta_T)
                            T_rov = max(0.0, T_base + 0.5 * delta_T)
                            trace_print(
                                1,
                                f"[DEBUG] Tensions projetées: T_bateau={T_bateau:.6f}, T_rov={T_rov:.6f}, ΔT={delta_T:.6f}"
                            )
                        else:
                            T_solution = np.linalg.solve(A, b)
                            # T_solution[0] = T_bateau, T_solution[1] = T_rov
                            T_bateau = max(0.0, T_solution[0])
                            T_rov = max(0.0, T_solution[1])
                            trace_print(1, f"[DEBUG] Solution brute du système: T_bateau={T_solution[0]:.6f}, T_rov={T_solution[1]:.6f}")
                        
                        # Vérification : les tensions doivent être positives
                        if T_bateau < 1e-6:
                            T_bateau_est = max(abs(weight_per_unit) * L / 10.0 if weight_per_unit != 0 else 100.0, 10.0)
                            trace_print(
                                10,
                                f"[DEBUG] ⚠️  : T_bateau est trop petite ({T_bateau:.6f}). "
                                f"Utilisation d'une estimation: {T_bateau_est:.6f} "
                                f"(max(|w|*L/10,10) avec w={weight_per_unit:.6f}, L={L:.6f})."
                            )
                            T_bateau = T_bateau_est
                        
                        if T_rov < 1e-6:
                            T_rov_est = max(abs(weight_per_unit) * L / 10.0 if weight_per_unit != 0 else 100.0, 10.0)
                            trace_print(
                                10,
                                f"[DEBUG] ⚠️  : T_rov est trop petite ({T_rov:.6f}). "
                                f"Utilisation d'une estimation: {T_rov_est:.6f} "
                                f"(max(|w|*L/10,10) avec w={weight_per_unit:.6f}, L={L:.6f})."
                            )
                            T_rov = T_rov_est
                    except np.linalg.LinAlgError:
                        # Estimation par défaut
                        T_rov = abs(weight_per_unit) * L / 10.0 if weight_per_unit != 0 else 100.0
                        T_bateau = T_rov
                    
                    # Debug: Afficher les équations numériques avec T_rov et T_bateau remplacés par leurs valeurs
                    trace_print(1, "[DEBUG] Équations numériques:")
                    eq1_left = -Ubateau_x * T_bateau + Urov_x * T_rov + Fext_stat_x
                    eq2_left = -Ubateau_y * T_bateau + Urov_y * T_rov + Fext_stat_y
                    trace_print(1, f"  Équation 1: -{Ubateau_x:.6f}*{T_bateau:.6f} + {Urov_x:.6f}*{T_rov:.6f} + {Fext_stat_x:.6f} = 0")
                    trace_print(1, f"  Équation 2: -{Ubateau_y:.6f}*{T_bateau:.6f} + {Urov_y:.6f}*{T_rov:.6f} + {Fext_stat_y:.6f} = 0")
                    trace_print(1, f"[DEBUG] Résidus (vérification): Équation 1 = {eq1_left:.6f}, Équation 2 = {eq2_left:.6f}")
                    
                    # Debug: Afficher les équations avec les produits remplacés par leurs valeurs
                    T_bateau_Ubateau_x = T_bateau * Ubateau_x
                    T_rov_Urov_x = T_rov * Urov_x
                    T_bateau_Ubateau_y = T_bateau * Ubateau_y
                    T_rov_Urov_y = T_rov * Urov_y
                    trace_print(1, "[DEBUG] Équations avec produits remplacés par leurs valeurs:")
                    trace_print(1, f"  Équation 1: -{T_bateau_Ubateau_x:.6f} + {T_rov_Urov_x:.6f} + {Fext_stat_x:.6f} = 0")
                    trace_print(1, f"  Équation 2: -{T_bateau_Ubateau_y:.6f} + {T_rov_Urov_y:.6f} + {Fext_stat_y:.6f} = 0")
                else:
                    # Estimation par défaut
                    T_rov = abs(weight_per_unit) * L / 10.0 if weight_per_unit != 0 else 100.0
                    T_bateau = T_rov
            else:
                T_rov = abs(weight_per_unit) * L / 10.0 if weight_per_unit != 0 else 100.0
                T_bateau = T_rov
            
            # Pour un câble tendu, les tensions varient linéairement entre T_bateau et T_rov
            # T(s) = T_bateau + (T_rov - T_bateau) * (s / L)
            T = np.zeros(self.N + 1)
            T[0] = T_bateau
            if self.N > 0:
                for i in range(1, self.N + 1):
                    s_frac = i / self.N  # Fraction de la longueur
                    T[i] = T_bateau + (T_rov - T_bateau) * s_frac
            else:
                T[0] = (T_bateau + T_rov) / 2.0
            
            return x_cable, y_cable, T
        
        # Câble en caténaire (rho_cable > rho_eau)
        # Résoudre l'équation de la caténaire : y = a * cosh((x - x0) / a) + y0
        # avec les contraintes aux extrémités et la longueur L
        
        trace_print(1, "\n[DEBUG] Initialisation du câble : câble caténaire (solve_equilibrium_static).")
        x_cable, y_cable = self._solve_catenary(x_rov, y_rov, x_boat, L, weight_per_unit)
        
        # Calculer les tensions le long du câble
        T = self._compute_catenary_tensions(x_cable, y_cable, weight_per_unit, rov_m, rov_vol)
        
        self._trace_cable_equilibrium_forces(x_cable, y_cable, T, L, label="initialisation")
        return x_cable, y_cable, T

    def solve_equilibrium_static_with_current(self, x_rov, y_rov, x_boat, L, rov_m=None, rov_vol=None,
                                              max_iter=50, tol=1e-3, relax=0.2,
                                              use_full_equilibrium=True):
        """
        Résout l'équilibre statique du câble en tenant compte de la traînée du courant sur la géométrie.
        
        Approche itérative :
        1) Partir d'une caténaire initiale
        2) Calculer les forces de courant le long du câble
        3) Mettre à jour la géométrie jusqu'à convergence
        """
        trace_print(1, "\n[DEBUG] Initialisation du câble : prise en compte du courant (solve_equilibrium_static_with_current).")
        
        if L <= 0:
            return np.array([x_rov, x_boat]), np.array([y_rov, 0.0]), np.array([0.0, 0.0])
        
        # Point de départ : caténaire statique classique
        x_cable, y_cable, T_guess = self.solve_equilibrium_static(
            x_rov, y_rov, x_boat, L, rov_m=rov_m, rov_vol=rov_vol
        )
        
        N = len(x_cable) - 1
        if N <= 0:
            trace_print(1, f"[DEBUG] Nombre de segments du câble: {N}")
            return x_cable, y_cable, np.zeros(len(x_cable))
        
        from .forces import compute_cable_forces
        from .forces import compute_cable_drag_force, compute_cable_apparent_weight
        params_cable = {
            'd': self.d,
            'rho_cable': self.rho_cable,
            'Cx_cable': self.Cx_cable
        }
        
        ds = L / N if N > 0 else 0.1
        try:
            v_current = np.array([self.environment.get_current_velocity(y) for y in y_cable])
            trace_print(1, f"[DEBUG] Courant max le long du câble: {np.max(np.abs(v_current)):.6f} m/s")
        except Exception:
            trace_print(1, f"[DEBUG] Courant max le long du câble: non obtenu")
            pass
        
        def integrate_with_forces(T0x, T0y):
            x_out = np.zeros(N + 1)
            y_out = np.zeros(N + 1)
            T_out = np.zeros(N + 1)
            x_out[0] = x_boat
            y_out[0] = 0.0
            T_vec = np.array([T0x, T0y], dtype=float)
            T_out[0] = np.linalg.norm(T_vec)
            
            for i in range(N):
                v_current = self.environment.get_current_velocity(y_out[i])
                Fx = compute_cable_drag_force(v_current, self.d, self.Cx_cable,
                                              self.environment.rho_eau, ds)
                Fy = compute_cable_apparent_weight(self.rho_cable, self.environment.rho_eau,
                                                  self.A_cable, self.environment.g, ds)
                
                T_mag = np.linalg.norm(T_vec)
                if T_mag < 1e-8:
                    t_hat = np.array([0.0, -1.0])
                else:
                    t_hat = T_vec / T_mag
                
                x_out[i + 1] = x_out[i] + t_hat[0] * ds
                y_out[i + 1] = y_out[i] + t_hat[1] * ds
                
                T_vec = T_vec - np.array([Fx, Fy])
                T_out[i + 1] = np.linalg.norm(T_vec)
            
            return x_out, y_out, T_out
        
        if use_full_equilibrium:
            trace_print(1, "\n[DEBUG] solve_equilibrium_static_with_current: tentative solveur complet (charges distribuées).")
            try:
                # Estimation initiale via caténaire classique déjà calculée
                dx0 = x_cable[1] - x_cable[0]
                dy0 = y_cable[1] - y_cable[0]
                ds0 = np.hypot(dx0, dy0)
                if ds0 > 1e-8 and len(T_guess) > 0:
                    t_hat0 = np.array([dx0 / ds0, dy0 / ds0])
                else:
                    t_hat0 = np.array([0.0, -1.0])
                T0 = T_guess[0] if len(T_guess) > 0 else abs(self.rho_cable - self.environment.rho_eau) * self.A_cable * self.environment.g * L / 2.0
                T0x, T0y = T0 * t_hat0[0], T0 * t_hat0[1]
                
                def residual(T0_vec):
                    x_end, y_end, _ = integrate_with_forces(T0_vec[0], T0_vec[1])
                    return np.array([x_end[-1] - x_rov, y_end[-1] - y_rov])
                
                sol, info, ier, msg = fsolve(
                    residual,
                    [T0x, T0y],
                    full_output=True,
                    xtol=1e-6,
                    maxfev=200,
                    factor=0.1
                )
                if ier == 1:
                    trace_print(1, "[DEBUG] solve_equilibrium_static_with_current: solveur complet convergé.")
                    x_cable, y_cable, T = integrate_with_forces(sol[0], sol[1])
                    self._trace_cable_equilibrium_forces(x_cable, y_cable, T, L, label="initialisation (solveur complet)")
                    return x_cable, y_cable, T
                trace_print(5, f"[DEBUG] solve_equilibrium_static_with_current: fsolve non convergent -> {msg}")
            except Exception as e:
                trace_print(5, f"[DEBUG] solve_equilibrium_static_with_current: fsolve échoué -> {e}")
        
        trace_print(1, "[DEBUG] solve_equilibrium_static_with_current: repli vers algorithme itératif.")
        
        for _ in range(max_iter):
            vx_cable = np.zeros(len(x_cable))
            vy_cable = np.zeros(len(x_cable))
            
            Fx_segments, _ = compute_cable_forces(
                x_cable, y_cable, vx_cable, vy_cable, self.environment, params_cable, L
            )
            
            # Forces nodales (moyenne des segments voisins)
            Fx_nodes = np.zeros(len(x_cable))
            Fx_nodes[0] = Fx_segments[0]
            Fx_nodes[-1] = Fx_segments[-1]
            for i in range(1, len(x_cable) - 1):
                Fx_nodes[i] = 0.5 * (Fx_segments[i - 1] + Fx_segments[i])
            
            max_fx = np.max(np.abs(Fx_nodes)) if np.any(Fx_nodes) else 0.0
            if max_fx < 1e-9:
                trace_print(1, "[DEBUG] Courant négligeable: forces horizontales ~ 0, géométrie inchangée.")
                break
            
            scale = np.max(np.abs(Fx_nodes)) if np.any(Fx_nodes) else 1.0
            dx_update = relax * (Fx_nodes / scale) * ds
            
            x_new = x_cable + dx_update
            y_new = y_cable.copy()
            
            # Conserver les extrémités
            x_new[0] = x_boat
            y_new[0] = 0.0
            x_new[-1] = x_rov
            y_new[-1] = y_rov
            
            # Normaliser la longueur totale
            x_new, y_new = self._normalize_cable_length(x_new, y_new, L)
            
            # Convergence
            max_delta = np.max(np.abs(x_new - x_cable))
            x_cable, y_cable = x_new, y_new
            
            if max_delta < tol:
                break
        
        # Calculer les tensions sur la géométrie finale en intégrant les forces
        Fx_segments, Fy_segments = compute_cable_forces(
            x_cable, y_cable, np.zeros(len(x_cable)), np.zeros(len(x_cable)),
            self.environment, params_cable, L
        )
        T_rov_initial = T_guess[-1] if len(T_guess) > 0 else 0.0
        T = self.compute_tensions(x_cable, y_cable, L, Fx_segments, Fy_segments, T_rov_initial=T_rov_initial)
        
        self._trace_cable_equilibrium_forces(x_cable, y_cable, T, L, label="initialisation (itératif)")
        return x_cable, y_cable, T
    
    def _solve_catenary(self, x_rov, y_rov, x_boat, L, w):
        """
        Résout l'équation de la caténaire pour trouver la forme du câble
        
        Utilise une méthode itérative pour trouver les paramètres de la caténaire
        qui satisfont les conditions aux limites et la longueur L.
        
        La caténaire suit : y = a * cosh((x - x0) / a) + y0
        Longueur curviligne : s = a * sinh((x - x0) / a)
        
        Parameters:
        -----------
        x_rov, y_rov : float
            Position du ROV (y_rov < 0, profondeur)
        x_boat : float
            Position horizontale du bateau (y_boat = 0)
        L : float
            Longueur totale du câble
        w : float
            Poids apparent par unité de longueur (N/m)
        
        Returns:
        --------
        tuple (x_cable, y_cable)
            Positions du câble discrétisées (du bateau au ROV)
        """
        trace_print(1, "\n[DEBUG] Résolution de la caténaire : _solve_catenary.")

        # Convention : y < 0 = profondeur (sous la surface), y = 0 = surface
        # y_rov doit être négatif pour représenter la profondeur
        dx = x_boat - x_rov
        dy = -y_rov  # Profondeur (y_rov est négatif)
        
        # Utiliser la formule standard de la caténaire : y = a * cosh((x - x0) / a) + y0
        # Contraintes :
        # 1. y_boat = 0 : 0 = a * cosh((x_boat - x0) / a) + y0  =>  y0 = -a * cosh((x_boat - x0) / a)
        # 2. y_rov = y_rov (< 0, profondeur) : y_rov = a * cosh((x_rov - x0) / a) + y0
        #    => y_rov = a * (cosh((x_rov - x0) / a) - cosh((x_boat - x0) / a))
        # 3. L = longueur curviligne : L = a * |sinh((x_rov - x0) / a) - sinh((x_boat - x0) / a)|
        
        L_straight = np.sqrt(dx**2 + dy**2)
        
        # Approche : pour chaque valeur de a, résoudre pour x0 en utilisant la contrainte y_rov,
        # puis vérifier la longueur L
        
        def find_x0_for_a(a):
            """Trouve x0 pour un a donné en utilisant la contrainte y_rov"""
            if a <= 0:
                return None
            
            def equation_x0(x0):
                """Équation pour trouver x0 : y_rov = a * (cosh((x_rov - x0) / a) - cosh((x_boat - x0) / a))"""
                try:
                    val = a * (np.cosh((x_rov - x0) / a) - np.cosh((x_boat - x0) / a)) - y_rov
                    if not np.isfinite(val):
                        return np.inf
                    return val
                except:
                    return np.inf
            
            # x0 devrait être entre x_rov et x_boat, plus proche du ROV
            # Élargir le bracket pour être sûr de trouver la solution
            bracket_width = max(abs(dx) * 2, 10.0)
            bracket = [min(x_rov, x_boat) - bracket_width, max(x_rov, x_boat) + bracket_width]
            
            # Vérifier que l'équation change de signe dans le bracket
            try:
                f_low = equation_x0(bracket[0])
                f_high = equation_x0(bracket[1])
                
                # Si les deux valeurs ont le même signe, essayer d'élargir le bracket
                if np.sign(f_low) == np.sign(f_high) and abs(f_low) > 1e-3 and abs(f_high) > 1e-3:
                    # Essayer plusieurs brackets
                    for scale in [2, 5, 10]:
                        bracket = [min(x_rov, x_boat) - bracket_width * scale, 
                                  max(x_rov, x_boat) + bracket_width * scale]
                        f_low = equation_x0(bracket[0])
                        f_high = equation_x0(bracket[1])
                        if np.sign(f_low) != np.sign(f_high):
                            break
            except:
                pass
            
            try:
                result = root_scalar(equation_x0, bracket=bracket, method='brentq', xtol=1e-8, maxiter=200)
                if result.converged:
                    return result.root
            except:
                pass
            
            # Essayer avec fsolve si root_scalar échoue
            try:
                # Essayer plusieurs estimations initiales
                x0_guesses = [
                    x_rov + 0.2 * (x_boat - x_rov),
                    x_rov + 0.1 * (x_boat - x_rov),
                    x_rov + 0.3 * (x_boat - x_rov),
                    (x_rov + x_boat) / 2,
                    x_rov,
                ]
                
                for x0_guess in x0_guesses:
                    try:
                        result = fsolve(equation_x0, x0_guess, xtol=1e-8, maxfev=500)
                        x0_test = result[0] if isinstance(result, np.ndarray) else result
                        
                        # Vérifier que la solution est bonne
                        error = abs(equation_x0(x0_test))
                        if error < 1e-3:
                            return x0_test
                    except:
                        continue
            except:
                pass
            
            return None
        
        def compute_length_error(a):
            """Calcule l'erreur sur la longueur pour un a donné"""
            if a <= 0 or not np.isfinite(a):
                return np.inf
            
            x0 = find_x0_for_a(a)
            if x0 is None or not np.isfinite(x0):
                return np.inf
            
            try:
                s_boat = a * np.sinh((x_boat - x0) / a)
                s_rov = a * np.sinh((x_rov - x0) / a)
                
                if not (np.isfinite(s_boat) and np.isfinite(s_rov)):
                    return np.inf
                
                L_calc = abs(s_rov - s_boat)
                
                if not np.isfinite(L_calc):
                    return np.inf
                
                return abs(L_calc - L)
            except:
                return np.inf
        
        # Estimation initiale de a
        if L > L_straight * 1.5:
            a_initial = max(0.5, dy / 6.0)
        elif L > L_straight * 1.2:
            a_initial = max(1.0, dy / 4.0)
        else:
            a_initial = max(2.0, dy / 2.0)
        
        # Optimiser pour trouver a qui minimise l'erreur sur la longueur
        a_opt = a_initial
        best_error = np.inf
        
        try:
            # Essayer plusieurs valeurs initiales de a
            a_candidates = [a_initial, a_initial * 0.5, a_initial * 2.0, dy / 6.0, dy / 4.0, dy / 2.0, dy]
            a_candidates = [max(0.1, a) for a in a_candidates]
            
            for a_init in a_candidates:
                try:
                    # Vérifier d'abord que cette valeur initiale donne une erreur finie
                    test_error = compute_length_error(a_init)
                    if not np.isfinite(test_error) or test_error == np.inf:
                        continue
                    
                    # Optimiser a pour minimiser l'erreur sur la longueur
                    # Utiliser un bracket plus restreint autour de a_init
                    bracket_low = max(0.1, a_init * 0.5)
                    bracket_high = min(a_init * 5.0, 1000.0)  # Limiter à une valeur raisonnable
                    
                    # Vérifier que le bracket est valide
                    if bracket_low >= bracket_high:
                        continue
                    
                    result = minimize_scalar(
                        compute_length_error,
                        bracket=(bracket_low, bracket_high),
                        method='brent',
                        options={'xtol': 1e-5, 'maxiter': 200}
                    )
                    
                    if result.success:
                        a_test = result.x
                        if not np.isfinite(a_test) or a_test <= 0:
                            continue
                            
                        error = compute_length_error(a_test)
                        
                        if np.isfinite(error) and error < best_error:
                            best_error = error
                            a_opt = a_test
                            
                            if error < 1e-4:
                                break
                except Exception as e:
                    # Ignorer les erreurs et continuer avec la valeur suivante
                    continue
            
            if best_error > 1e-2:
                trace_print(7, f"⚠️  Avertissement: La caténaire a une erreur élevée ({best_error:.6f}). "
                      f"Utilisation des meilleures valeurs trouvées.")
        except Exception as e:
            trace_print(7, f"⚠️  Erreur lors de la résolution de la caténaire: {e}. Utilisation des valeurs initiales.")
        
        # Trouver x0 pour le a optimal
        x0_opt = find_x0_for_a(a_opt)
        if x0_opt is None:
            trace_print(7, f"⚠️  Erreur: Impossible de trouver x0 pour a={a_opt:.3f}. Utilisation d'une interpolation linéaire.")
            x_cable = np.linspace(x_rov, x_boat, self.N + 1)
            y_cable = np.linspace(y_rov, 0.0, self.N + 1)
            return x_cable, y_cable
        
        # Calculer y0
        try:
            y0_opt = -a_opt * np.cosh((x_boat - x0_opt) / a_opt)
        except:
            trace_print(7, f"⚠️  Erreur: Impossible de calculer y0. Utilisation d'une interpolation linéaire.")
            x_cable = np.linspace(x_rov, x_boat, self.N + 1)
            y_cable = np.linspace(y_rov, 0.0, self.N + 1)
            return x_cable, y_cable
        
        # Vérifier la solution
        y_rov_calc = a_opt * np.cosh((x_rov - x0_opt) / a_opt) + y0_opt
        y_boat_calc = a_opt * np.cosh((x_boat - x0_opt) / a_opt) + y0_opt
        
        # y_rov devrait être négatif (profondeur : y < 0 = sous la surface)
        if abs(y_rov_calc - y_rov) > 0.1:
            trace_print(7, f"⚠️  Avertissement: y_rov calculé ({y_rov_calc:.3f}) diffère de y_rov attendu ({y_rov:.3f}, profondeur négative)")
        
        if abs(y_boat_calc) > 0.1:
            trace_print(7, f"⚠️  Avertissement: y_boat calculé ({y_boat_calc:.3f}) diffère de y_boat attendu (0.000, surface)")
        
        # Générer les points du câble par longueur curviligne
        # Convention : s = 0 au bateau, s = L au ROV (plus simple et cohérent avec le système)
        # Calculer les longueurs curvilignes aux extrémités dans le système de référence de la caténaire
        try:
            s_boat_ref = a_opt * np.sinh((x_boat - x0_opt) / a_opt)
            s_rov_ref = a_opt * np.sinh((x_rov - x0_opt) / a_opt)
            
            # La longueur curviligne de référence dans le système de la caténaire
            s_total_ref = abs(s_rov_ref - s_boat_ref)
            
            # Déterminer l'ordre : on veut s=0 au bateau et s=L au ROV
            # Donc on mappe s (0 à L) vers s_ref (s_boat_ref à s_rov_ref)
            if s_total_ref > 1e-6:
                # Transformation linéaire : s_ref = s_boat_ref + (s / L) * (s_rov_ref - s_boat_ref)
                # Mais on doit s'assurer que s_boat_ref < s_rov_ref
                if s_rov_ref < s_boat_ref:
                    # Inverser l'ordre
                    s_boat_ref, s_rov_ref = s_rov_ref, s_boat_ref
            else:
                # Si la longueur est nulle, utiliser une approximation
                s_boat_ref = 0.0
                s_rov_ref = L
        except:
            # Si le calcul échoue, utiliser une approximation simple
            s_boat_ref = 0.0
            s_rov_ref = L
        
        # Générer les points à intervalles réguliers de longueur curviligne : s de 0 à L
        s_points = np.linspace(0.0, L, self.N + 1)
        x_cable = np.zeros(self.N + 1)
        y_cable = np.zeros(self.N + 1)
        
        for i, s in enumerate(s_points):
            try:
                # Convertir s (0 à L) en s_ref (référence de la caténaire)
                # Interpolation linéaire : s_ref = s_boat_ref + (s / L) * (s_rov_ref - s_boat_ref)
                if L > 1e-6:
                    s_ref = s_boat_ref + (s / L) * (s_rov_ref - s_boat_ref)
                else:
                    s_ref = s_boat_ref
                
                # Calculer x à partir de s_ref : s_ref = a * sinh((x - x0) / a)
                # donc x = x0 + a * arcsinh(s_ref / a)
                x = x0_opt + a_opt * np.arcsinh(s_ref / a_opt)
                # Utiliser la formule standard : y = a * cosh((x - x0) / a) + y0
                y = a_opt * np.cosh((x - x0_opt) / a_opt) + y0_opt
                
                x_cable[i] = x
                y_cable[i] = y
            except:
                # Si le calcul échoue, interpolation linéaire
                alpha = i / self.N
                x_cable[i] = x_boat + alpha * (x_rov - x_boat)
                y_cable[i] = 0.0 + alpha * (y_rov - 0.0)
        
        # Forcer les extrémités exactement (important pour la continuité)
        # Index 0 = bateau, index -1 = ROV (convention cohérente avec le système)
        x_cable[0] = x_boat
        y_cable[0] = 0.0
        x_cable[-1] = x_rov
        y_cable[-1] = y_rov
        
        # Vérifier la longueur
        L_actual = 0.0
        for i in range(self.N):
            dx_seg = x_cable[i+1] - x_cable[i]
            dy_seg = y_cable[i+1] - y_cable[i]
            L_actual += np.sqrt(dx_seg**2 + dy_seg**2)
        
        # Toujours normaliser pour garantir que la longueur est exactement L
        # (tolérance réduite pour forcer la normalisation)
        if abs(L_actual - L) / L > 1e-6:
            x_cable, y_cable = self._normalize_cable_length(x_cable, y_cable, L)
            
            # CONTRAINTE PHYSIQUE : Après normalisation, forcer y <= 0
            y_cable = np.clip(y_cable, None, 0.0)
            # Réappliquer les extrémités
            y_cable[0] = 0.0
            y_cable[-1] = y_rov
        
        # Vérification finale de la longueur
        L_final = 0.0
        for i in range(self.N):
            dx_seg = x_cable[i+1] - x_cable[i]
            dy_seg = y_cable[i+1] - y_cable[i]
            L_final += np.sqrt(dx_seg**2 + dy_seg**2)
        
        # Vérifier que la forme est bien une caténaire (pas juste deux segments)
        if len(x_cable) > 2:
            # Calculer les angles entre segments consécutifs
            angles = []
            for i in range(1, len(x_cable) - 1):
                dx1 = x_cable[i] - x_cable[i-1]
                dy1 = y_cable[i] - y_cable[i-1]
                dx2 = x_cable[i+1] - x_cable[i]
                dy2 = y_cable[i+1] - y_cable[i]
                ds1 = np.sqrt(dx1**2 + dy1**2)
                ds2 = np.sqrt(dx2**2 + dy2**2)
                if ds1 > 1e-6 and ds2 > 1e-6:
                    cos_angle = (dx1*dx2 + dy1*dy2) / (ds1 * ds2)
                    cos_angle = np.clip(cos_angle, -1.0, 1.0)
                    angle = np.arccos(cos_angle)
                    angles.append(angle)
            
            # Si tous les angles sont très petits, c'est probablement une ligne droite
            if len(angles) > 0 and np.mean(angles) < 0.01:
                trace_print(7, f"⚠️  Avertissement: Le câble semble être une ligne droite plutôt qu'une caténaire. "
                      f"Vérifiez les paramètres (L={L:.2f} m, L_straight={L_straight:.2f} m, "
                      f"rho_cable={self.rho_cable:.1f} kg/m³, rho_eau={self.environment.rho_eau:.1f} kg/m³)")
        
        # Avertissement si la longueur n'est toujours pas correcte
        if abs(L_final - L) / L > 0.02:
            trace_print(7, f"⚠️  Avertissement: Longueur du câble incorrecte après calcul de caténaire. "
                  f"Attendu: {L:.3f} m, Obtenu: {L_final:.3f} m, Erreur: {abs(L_final - L)/L*100:.1f}%")
        
        # Retourner le câble : index 0 = bateau, index -1 = ROV (convention cohérente avec le système)
        return x_cable, y_cable
    
    def _compute_catenary_tensions(self, x_cable, y_cable, weight_per_unit, rov_m=None, rov_vol=None):
        """
        Calcule les tensions le long du câble en caténaire
        
        L'équilibre des forces sur le câble s'écrit :
        T_rov * Urov + T_bateau * Ubateau + Fext_stat = 0
        # où Fext_stat inclut le courant et le poids du câble
        
        Parameters:
        -----------
        x_cable, y_cable : array
            Positions du câble (index 0 = bateau, index -1 = ROV)
        weight_per_unit : float
            Poids apparent par unité de longueur (N/m)
        rov_m, rov_vol : float, optional
            Masse et volume du ROV (non utilisés pour le calcul de tension)
        
        Returns:
        --------
        array
            Tensions le long du câble (T[0] = tension au bateau, T[-1] = tension au ROV)
        """
        trace_print(1, "\n[DEBUG] Calcul des tensions en caténaire : _compute_catenary_tensions.")

        N = len(x_cable) - 1
        T = np.zeros(N + 1)
        
        if N == 0:
            # Un seul point, tension nulle
            return T
        
        # Calculer les vecteurs unitaires aux extrémités
        # Ubateau : direction du câble depuis le bateau (index 0) vers le point suivant
        dx_bateau = x_cable[1] - x_cable[0]
        dy_bateau = y_cable[1] - y_cable[0]
        ds_bateau = np.sqrt(dx_bateau**2 + dy_bateau**2)
        
        # Urov : direction du câble vers le ROV (du point précédent vers le ROV, index -1)
        dx_rov = x_cable[-1] - x_cable[-2]
        dy_rov = y_cable[-1] - y_cable[-2]
        ds_rov = np.sqrt(dx_rov**2 + dy_rov**2)
        
        if ds_bateau < 1e-6 or ds_rov < 1e-6:
            # Câble trop court ou segments trop petits, utiliser estimation simple
            L_total = 0.0
            for i in range(N):
                dx = x_cable[i+1] - x_cable[i]
                dy = y_cable[i+1] - y_cable[i]
                L_total += np.sqrt(dx**2 + dy**2)
            T_est = max(0.0, weight_per_unit * L_total / 2.0)
            T.fill(T_est)
            return T
        
        # Vecteurs unitaires
        Ubateau_x = dx_bateau / ds_bateau
        Ubateau_y = dy_bateau / ds_bateau
        
        Urov_x = dx_rov / ds_rov
        Urov_y = dy_rov / ds_rov
        
        # Calculer les forces externes statiques sur le câble
        # Ces forces incluent : le courant (traînée horizontale) et le poids apparent du câble (force verticale)
        # Pour l'équilibre statique, les vitesses du câble sont nulles
        from .forces import compute_cable_forces
        params_cable = {
            'd': self.d,
            'rho_cable': self.rho_cable,
            'Cx_cable': self.Cx_cable
        }
        
        # Calculer la longueur totale du câble
        L_total = 0.0
        for i in range(N):
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            L_total += np.sqrt(dx**2 + dy**2)
        
        # Vitesses du câble nulles pour l'équilibre statique
        vx_cable = np.zeros(len(x_cable))
        vy_cable = np.zeros(len(x_cable))
        
        # Calculer les forces externes sur chaque segment (courant + poids du câble)
        Fx_courant_segments, Fy_courant_segments = compute_cable_forces(
            x_cable, y_cable, vx_cable, vy_cable, self.environment, params_cable, L_total
        )
        
        # Forces externes statiques totales (courant + poids du câble)
        Fext_stat_x = np.sum(Fx_courant_segments)
        Fext_stat_y = np.sum(Fy_courant_segments)
        
        # Résoudre le système d'équations d'équilibre des forces
        # Équations d'équilibre :
        # -Ubx*T_bateau + Urx*T_rov + Fex = 0  (horizontale)
        # -Uby*T_bateau + Ury*T_rov + Fey = 0  (verticale)
        # où :
        # - Fex = Force exercée par le courant sur le câble (dans le sens de la vitesse apparente du courant)
        # - Fey = Poids apparent du câble (négatif pour un câble plus dense que l'eau, dirigé vers le bas)
        # 
        # Sous forme matricielle : A * [T_bateau, T_rov]^T = [-Fex, -Fey]^T
        A = np.array([
            [-Ubateau_x, Urov_x],
            [-Ubateau_y, Urov_y]
        ])
        b = np.array([-Fext_stat_x, -Fext_stat_y])
        
        # Debug: Afficher les équations sous forme littérale
        trace_print(1, "\n[DEBUG] Résolution du système linéaire pour tensions initiales (câble en caténaire)")
        # trace_print(1, "[DEBUG] Équations littérales:")
        # trace_print(1, f"  Équation 1 (horizontale): -Ubx*T_bateau + Urx*T_rov + Fex = 0")
        # trace_print(1, f"  Équation 2 (verticale):   -Uby*T_bateau + Ury*T_rov + Fey = 0")
        # trace_print(1, f"[DEBUG] Note: Fex = force du courant, Fey = poids apparent (négatif pour câble plus dense que l'eau)")
        # trace_print(1, f"[DEBUG] Coefficients: Urov_x={Urov_x:.6f}, Urov_y={Urov_y:.6f}, Ubateau_x={Ubateau_x:.6f}, Ubateau_y={Ubateau_y:.6f}")
        # trace_print(1, f"[DEBUG] Forces externes: Fext_stat_x={Fext_stat_x:.6f}, Fext_stat_y={Fext_stat_y:.6f}")
        # trace_print(1, f"\n[DEBUG] Équations numériques:")
        # trace_print(1, f"  Équation 1 (horizontale): {-Ubateau_x:.6f}*T_bateau + {Urov_x:.6f}*T_rov + {Fext_stat_x:.6f} = 0")
        # trace_print(1, f"  Équation 2 (verticale):   {-Ubateau_y:.6f}*T_bateau + {Urov_y:.6f}*T_rov + {Fext_stat_y:.6f} = 0")
        # trace_print(1, f"[DEBUG] Déterminant de A: {np.linalg.det(A):.6f}")
        try:
            # Résoudre le système linéaire
            T_solution = np.linalg.solve(A, b)
            # trace_print(1, f"\n[DEBUG] Solution brute step 1 de la résolution du système: T_bateau={T_solution[0]:.6f}, T_rov={T_solution[1]:.6f}")
            # T_solution[0] = T_bateau, T_solution[1] = T_rov
            T_bateau = max(0.0, T_solution[0]) 
            T_rov = max(0.0, T_solution[1])
            # trace_print(1, f"[DEBUG] Solution brute step 2 de la résolution du système: T_bateau={T_bateau:.6f}, T_rov={T_rov:.6f}")
            
            # Vérification : les tensions doivent être positives
            if T_bateau < 1e-6:
                trace_print(7, f"[DEBUG] ⚠️  ERREUR CRITIQUE: T_bateau est trop petite ({T_bateau:.6f}). Utilisation d'une estimation.")
                T_bateau = max(abs(weight_per_unit) * L_total / 10.0 if weight_per_unit != 0 else 100.0, 10.0)
            
            if T_rov < 1e-6:
                trace_print(7, f"[DEBUG] ⚠️  ERREUR CRITIQUE: T_rov est trop petite ({T_rov:.6f}). Utilisation d'une estimation.")
                T_rov = max(abs(weight_per_unit) * L_total / 10.0 if weight_per_unit != 0 else 100.0, 10.0)
        except np.linalg.LinAlgError:
            # Si le système est singulier (câble vertical ou autre cas dégénéré)
            # Utiliser une estimation basée sur le poids du câble
            T_rov = max(0.0, weight_per_unit * L_total / 2.0)
            T_bateau = T_rov
        
        # Debug: Afficher les équations numériques avec T_rov et T_bateau remplacés par leurs valeurs
        eq1_left = -Ubateau_x * T_bateau + Urov_x * T_rov + Fext_stat_x
        eq2_left = -Ubateau_y * T_bateau + Urov_y * T_rov + Fext_stat_y
        trace_print(1, f"[DEBUG] Résidus (vérification): Équation 1 = {eq1_left:.6f}, Équation 2 = {eq2_left:.6f}")
        
        # Debug: Afficher les équations avec les produits remplacés par leurs valeurs
        T_bateau_Ubateau_x = T_bateau * Ubateau_x
        T_rov_Urov_x = T_rov * Urov_x
        T_bateau_Ubateau_y = T_bateau * Ubateau_y
        T_rov_Urov_y = T_rov * Urov_y
        
        # Calculer les tensions le long du câble
        # Pour une caténaire, T(s) = sqrt(H² + (w*s)²) où H est la tension horizontale
        # On calcule H à partir de T_bateau et de l'angle au bateau
        # θ_bateau est l'angle avec la verticale (comme dans simulation_thread.py)
        # Si θ_bateau est l'angle avec la verticale :
        # - Composante horizontale : H = T_bateau × sin(θ_bateau) = T_bateau × (dx/ds) = T_bateau × Ubateau_x
        # - Composante verticale : T_bateau × cos(θ_bateau) = T_bateau × (-dy/ds) = -T_bateau × Ubateau_y
        sin_theta_bateau = Ubateau_x  # sin(θ) = dx/ds où θ est l'angle avec la verticale
        cos_theta_bateau = -Ubateau_y  # cos(θ) = -dy/ds (négatif car y descend)
        
        # Tension horizontale H = T_bateau * sin(theta_bateau) où θ_bateau est l'angle avec la verticale
        H = T_bateau * abs(sin_theta_bateau) if abs(sin_theta_bateau) > 1e-6 else T_bateau
        
        # Pour une caténaire, la tension suit T(s) = sqrt(H² + (w*s)²) où s est mesuré depuis le bateau
        # Au bateau (s=0), on doit avoir T(0) = sqrt(H² + 0) = H
        # Mais nous avons T_bateau qui peut être différent de H si l'angle n'est pas petit
        # Pour assurer la continuité, on calcule s_bateau tel que T_bateau = sqrt(H² + (w*s_bateau)²)
        # Si T_bateau < H, alors s_bateau serait imaginaire, donc on utilise H = T_bateau dans ce cas
        if T_bateau < H:
            H = T_bateau
            s_bateau = 0.0
        else:
            # Calculer s_bateau : T_bateau² = H² + (w*s_bateau)²
            # donc s_bateau = sqrt((T_bateau² - H²) / w²) = sqrt(T_bateau² - H²) / w
            if weight_per_unit > 1e-6:
                s_bateau = np.sqrt(max(0.0, T_bateau**2 - H**2)) / weight_per_unit
            else:
                s_bateau = 0.0
        
        # Tension au bateau : T[0] = T_bateau
        T[0] = T_bateau
        
        # Calculer les tensions pour les points intermédiaires
        # Pour la caténaire, T(s) = sqrt(H² + (w*s)²) où s est mesuré depuis le point de tension minimale
        # s_bateau représente la distance depuis le point de tension minimale jusqu'au bateau
        # s_cumulative représente la distance curviligne accumulée depuis le bateau
        # Donc s_total = s_bateau + s_cumulative est la distance depuis le point de tension minimale
        s_cumulative = 0.0  # Initialisé à 0 car on commence au bateau
        for i in range(N):
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            ds = np.sqrt(dx**2 + dy**2)
            s_cumulative += ds  # Accumuler la distance depuis le bateau
            
            # Tension à ce point : T = sqrt(H² + (w*s_total)²)
            # où s_total est la distance depuis le point de tension minimale
            s_total = abs(-s_bateau + s_cumulative)
            T[i+1] = np.sqrt(H**2 + (weight_per_unit * s_total)**2)
        
        # Vérifier que T[-1] correspond bien à T_rov (avec tolérance)
        if abs(T[-1] - T_rov) > 0.01:
            # Ajuster pour que T[-1] = T_rov exactement
            T[-1] = T_rov
        
        return T
    
    def solve_equilibrium_dynamic(self, x_rov, y_rov, vx_rov, vy_rov,
                                  x_boat, vx_boat, L, x_cable_prev, y_cable_prev, rov_m=None, rov_vol=None):
        """
        Résout l'équilibre dynamique du câble avec forces hydrodynamiques
        
        Parameters:
        -----------
        x_rov, y_rov : float
            Position du ROV
        vx_rov, vy_rov : float
            Vitesse du ROV
        x_boat, vx_boat : float
            Position et vitesse du bateau
        L : float
            Longueur du câble
        x_cable_prev, y_cable_prev : array
            Configuration précédente du câble
        
        Returns:
        --------
        tuple (x_cable, y_cable, T)
            Nouvelle configuration du câble
        """

        trace_print(1, "\n[DEBUG] Résolution de l'équilibre dynamique : solve_equilibrium_dynamic.")

        if L <= 0:
            return np.array([x_rov, x_boat]), np.array([y_rov, 0.0]), np.array([0.0, 0.0])
        
        ds = L / self.N
        
        # Optimisation : utiliser une interpolation pondérée entre la solution statique et la configuration précédente
        # pour éviter de recalculer complètement à chaque pas
        x_cable_static, y_cable_static, T_static = self.solve_equilibrium_static(
            x_rov, y_rov, x_boat, L, rov_m=rov_m, rov_vol=rov_vol
        )
        
        # Mélanger avec la configuration précédente pour lisser les transitions
        # Coefficient de mélange : plus proche de la solution statique si les vitesses sont faibles
        v_mag = np.sqrt(vx_rov**2 + vy_rov**2 + vx_boat**2)
        alpha = 0.3 if v_mag > 0.1 else 0.7  # Plus de poids sur statique si vitesses faibles
        
        if x_cable_prev is not None and y_cable_prev is not None and len(x_cable_prev) == len(x_cable_static):
            x_cable = alpha * x_cable_static + (1 - alpha) * x_cable_prev
            y_cable = alpha * y_cable_static + (1 - alpha) * y_cable_prev
            T = T_static  # Garder les tensions de la solution statique
        else:
            x_cable = x_cable_static
            y_cable = y_cable_static
            T = T_static
        
        # Vérifier si le câble est vertical : tous les points doivent avoir la même position x
        if len(x_cable) > 0:
            x_cable_range = np.max(x_cable) - np.min(x_cable)
            if x_cable_range < 1e-6:
                # Câble vertical : forcer tous les points à la même position x
                # Utiliser la moyenne pour éviter les erreurs d'arrondi
                x_avg = np.mean(x_cable)
                x_cable[:] = x_avg
        
        # Vitesses des points du câble (interpolation linéaire entre ROV et bateau)
        vx_cable = np.linspace(vx_rov, vx_boat, self.N + 1)
        vy_cable = np.linspace(vy_rov, 0.0, self.N + 1)
        
        # Optimisation : simplifier en ne faisant qu'une seule itération d'ajustement
        # au lieu de plusieurs itérations complètes
        max_iter = 1  # Une seule itération pour performance maximale
        tolerance = 1e-2  # Tolérance plus relâchée
        
        for iteration in range(max_iter):
            x_cable_old = x_cable.copy()
            y_cable_old = y_cable.copy()
            
            # Calculer les forces
            from .forces import compute_cable_forces
            Fx, Fy = compute_cable_forces(x_cable, y_cable, vx_cable, vy_cable,
                                         self.environment, self.params, L)
            
            # Résoudre l'équilibre des forces par segment
            for i in range(1, self.N):
                # Ajuster la position pour équilibrer les forces
                # Approche simplifiée : ajustement proportionnel
                if i > 0:
                    dx_seg = x_cable[i] - x_cable[i-1]
                    dy_seg = y_cable[i] - y_cable[i-1]
                    length_seg = np.sqrt(dx_seg**2 + dy_seg**2)
                    
                    if length_seg > 1e-6:
                        # Normaliser
                        dx_seg /= length_seg
                        dy_seg /= length_seg
                        
                        # Ajuster selon les forces
                        alpha = 0.1  # Coefficient de relaxation
                        x_cable[i] += alpha * Fx[i-1] * dx_seg / (T[i] + 1e-6)
                        y_cable[i] += alpha * Fy[i-1] * dy_seg / (T[i] + 1e-6)
                
                # Assurer la longueur du segment
                if i > 0:
                    dx_seg = x_cable[i] - x_cable[i-1]
                    dy_seg = y_cable[i] - y_cable[i-1]
                    length_seg = np.sqrt(dx_seg**2 + dy_seg**2)
                    if length_seg > 1e-6:
                        scale = ds / length_seg
                        x_cable[i] = x_cable[i-1] + (x_cable[i] - x_cable[i-1]) * scale
                        y_cable[i] = y_cable[i-1] + (y_cable[i] - y_cable[i-1]) * scale
            
            # Forcer les conditions aux limites (ordre bateau -> ROV)
            x_cable[0] = x_boat
            y_cable[0] = 0.0
            x_cable[-1] = x_rov
            y_cable[-1] = y_rov
            
            # CONTRAINTE PHYSIQUE : Le câble ne peut pas être au-dessus de la surface (y > 0)
            # Forcer tous les points du câble à avoir y <= 0
            y_cable = np.clip(y_cable, None, 0.0)
            
            # Réappliquer les conditions aux limites après le clipping
            y_cable[0] = 0.0
            y_cable[-1] = y_rov
            
            # Vérifier si le câble est vertical : tous les points doivent avoir la même position x
            if len(x_cable) > 0:
                x_cable_range = np.max(x_cable) - np.min(x_cable)
                if x_cable_range < 1e-6:
                    # Câble vertical : forcer tous les points à la même position x
                    # Utiliser la moyenne pour éviter les erreurs d'arrondi
                    x_avg = np.mean(x_cable)
                    x_cable[:] = x_avg
            
            # Vérifier la convergence
            if np.max(np.abs(x_cable - x_cable_old)) < tolerance and \
               np.max(np.abs(y_cable - y_cable_old)) < tolerance:
                break
        
        # Normaliser la longueur du câble pour garantir qu'elle soit exactement égale à L
        # Calculer la longueur actuelle du câble
        L_actual = 0.0
        for i in range(len(x_cable) - 1):
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            L_actual += np.sqrt(dx**2 + dy**2)
        
        # Toujours normaliser pour garantir que la longueur est exactement L
        if L_actual > 1e-6 and abs(L_actual - L) > 1e-6:
            # Rééchantillonner le câble pour avoir exactement la longueur L
            # Conserver les extrémités fixes
            x_cable, y_cable = self._normalize_cable_length(x_cable, y_cable, L)
        
        # CONTRAINTE PHYSIQUE : Le câble ne peut pas être au-dessus de la surface (y > 0)
        # Forcer tous les points du câble à avoir y <= 0
        y_cable = np.clip(y_cable, None, 0.0)
        
        # Réappliquer les conditions aux limites après le clipping
        y_cable[0] = 0.0
        y_cable[-1] = y_rov
        
        # Vérifier et renormaliser la longueur après le clipping (le clipping peut changer la longueur)
        L_after_clip = 0.0
        for i in range(len(x_cable) - 1):
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            L_after_clip += np.sqrt(dx**2 + dy**2)
        
        if abs(L_after_clip - L) / L > 1e-6:
            x_cable, y_cable = self._normalize_cable_length(x_cable, y_cable, L)
            # Réappliquer la contrainte y <= 0 après renormalisation
            y_cable = np.clip(y_cable, None, 0.0)
            y_cable[0] = 0.0
            y_cable[-1] = y_rov
        
        # Recalculer les tensions en utilisant la tension initiale de T_static
        # pour préserver la tension correcte au ROV basée sur l'équilibre
        # CORRECTION: Après correction de _compute_catenary_tensions :
        # T_static[0] = tension au bateau, T_static[-1] = tension au ROV
        T_rov_initial = T_static[-1] if len(T_static) > 0 else 0.0
        if T_rov_initial <= 0.0:
            weight_per_unit = (self.rho_cable - self.environment.rho_eau) * self.A_cable * self.environment.g
            trace_print(
                10,
                f"[DEBUG] ⚠️  : T_rov_initial trop faible ({T_rov_initial:.6f}). "
                "Utilisation d'une estimation minimale."
            )
            T_rov_initial = max(abs(weight_per_unit) * L / 10.0 if weight_per_unit != 0 else 100.0, 10.0)
        T = self.compute_tensions(x_cable, y_cable, L, Fx, Fy, T_rov_initial=T_rov_initial)
        
        return x_cable, y_cable, T
    
    def compute_tensions(self, x_cable, y_cable, L, Fx, Fy, T_rov_initial=None):
        """
        Calcule les tensions le long du câble
        
        Parameters:
        -----------
        x_cable, y_cable : array
            Positions du câble
        L : float
            Longueur du câble
        Fx, Fy : array
            Forces par segment
        T_rov_initial : float, optional
            Tension initiale au ROV. Si None, utilise 100.0 par défaut (pour compatibilité)
        
        Returns:
        --------
        array
            Tensions le long du câble
        """

        trace_print(1, "\n[DEBUG] Calcul des tensions : compute_tensions.")

        N = len(x_cable) - 1
        ds = L / N if N > 0 else 0.1
        
        T = np.zeros(N + 1)
        
        # Intégrer les forces depuis le ROV vers le bateau
        # Convention : index 0 = bateau, index -1 = ROV
        # Utiliser la tension initiale au ROV si fournie, ou 100.0 par défaut
        if T_rov_initial is None or T_rov_initial <= 0.0:
            T_rov_initial = 100.0
        T[-1] = T_rov_initial
        
        for i in range(N - 1, -1, -1):
            # Direction du segment (i -> i+1)
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            length = np.sqrt(dx**2 + dy**2)
            
            if length > 1e-6:
                # Équilibre des forces
                dT_x = Fx[i] * ds
                dT_y = Fy[i] * ds
                
                # Mettre à jour la tension en remontant vers le bateau
                T[i] = T[i+1] + np.sqrt(dT_x**2 + dT_y**2)
            else:
                T[i] = T[i+1]
        
        return T

    def _trace_cable_equilibrium_forces(self, x_cable, y_cable, T, L, label="initialisation"):
        from .forces import compute_cable_forces, compute_cable_apparent_weight
        from datetime import datetime
        from src.ui.mission_utils import get_missions_directory
        if x_cable is None or y_cable is None or len(x_cable) < 2:
            trace_print(7, f"[DEBUG] Trace forces câble ({label}): câble insuffisant.")
            return
        if T is None or len(T) < 2:
            trace_print(7, f"[DEBUG] Trace forces câble ({label}): tensions indisponibles.")
            return

        dx_bat = x_cable[1] - x_cable[0]
        dy_bat = y_cable[1] - y_cable[0]
        ds_bat = np.hypot(dx_bat, dy_bat)
        if ds_bat > 1e-9:
            U_bat = np.array([dx_bat / ds_bat, dy_bat / ds_bat])
        else:
            U_bat = np.array([0.0, -1.0])

        dx_rov = x_cable[-1] - x_cable[-2]
        dy_rov = y_cable[-1] - y_cable[-2]
        ds_rov = np.hypot(dx_rov, dy_rov)
        if ds_rov > 1e-9:
            U_rov = np.array([dx_rov / ds_rov, dy_rov / ds_rov])
        else:
            U_rov = np.array([0.0, -1.0])

        T_bat = float(T[0])
        T_rov = float(T[-1])

        # Forces aux extrémités (convention : traction exercée par le bateau sur le câble
        # opposée à la direction du câble au bateau, traction exercée par le ROV dans le sens du câble)
        F_bateau = -T_bat * U_bat
        F_rov = T_rov * U_rov

        # Forces distribuées sur le câble
        Fx_seg, Fy_seg = compute_cable_forces(
            x_cable, y_cable, np.zeros(len(x_cable)), np.zeros(len(x_cable)),
            self.environment, self.params, L
        )
        Fx_total = float(np.sum(Fx_seg))
        Fy_total = float(np.sum(Fy_seg))

        # Décomposer poids apparent et traînée
        N = len(x_cable) - 1
        ds = L / N if N > 0 else 0.0
        Fy_weight_per_seg = compute_cable_apparent_weight(
            self.rho_cable, self.environment.rho_eau, self.A_cable, self.environment.g, ds
        )
        Fy_weight_total = float(Fy_weight_per_seg * N)
        Fy_drag_total = Fy_total - Fy_weight_total

        F_poids = np.array([0.0, Fy_weight_total])
        F_drag = np.array([Fx_total, Fy_drag_total])
        F_sum = F_bateau + F_rov + F_poids + F_drag

        def _fmt(vec):
            return f"[{vec[0]:.3f}, {vec[1]:.3f}]"

        trace_print(1, f"[DEBUG] Trace forces câble ({label})")
        trace_print(1, f"  - U_bateau       : {_fmt(U_bat)}")
        trace_print(1, f"  - U_rov          : {_fmt(U_rov)}")
        trace_print(1, f"  - Traction bateau : {_fmt(F_bateau)} N")
        trace_print(1, f"  - Traction ROV    : {_fmt(F_rov)} N")
        trace_print(1, f"  - Poids apparent  : {_fmt(F_poids)} N")
        trace_print(1, f"  - Traînée câble   : {_fmt(F_drag)} N")
        trace_print(1, f"  - Somme forces    : {_fmt(F_sum)} N")

        # Diagnostics simples de signe/direction
        if self.rho_cable > self.environment.rho_eau and F_poids[1] > 0.0:
            trace_print(7, "[DEBUG] ⚠️  Incohérence: poids apparent positif alors que câble plus dense.")
        if self.rho_cable < self.environment.rho_eau and F_poids[1] < 0.0:
            trace_print(7, "[DEBUG] ⚠️  Incohérence: poids apparent négatif alors que câble plus léger.")
        if np.dot(F_bateau, U_bat) > 0.0:
            trace_print(7, "[DEBUG] ⚠️  Incohérence: traction bateau dans le même sens que U_bateau.")
        if np.dot(F_rov, U_rov) < 0.0:
            trace_print(7, "[DEBUG] ⚠️  Incohérence: traction ROV opposée à U_rov.")

        # Longueur du câble : paramètre L0 et longueur réelle par somme des segments
        L0 = float(L)
        L_segments = 0.0
        ds_values = []
        for i in range(len(x_cable) - 1):
            dx = x_cable[i + 1] - x_cable[i]
            dy = y_cable[i + 1] - y_cable[i]
            ds_i = float(np.hypot(dx, dy))
            L_segments += ds_i
            ds_values.append(ds_i)

        trace_print(1, f"  - Longueur câble L0     : {L0:.6f} m")
        trace_print(1, f"  - Longueur segments Σds : {L_segments:.6f} m")
        if ds_values:
            trace_print(1, f"  - min/max ds            : {min(ds_values):.6f} ; {max(ds_values):.6f}")

        # Exporter la trace dans un fichier "Trace" dans le répertoire de la mission
        mission_name = getattr(self.environment, "mission_name", None)
        if mission_name and str(mission_name).strip():
            mission_dir = get_missions_directory() / str(mission_name)
            mission_dir.mkdir(exist_ok=True)
            trace_file = mission_dir / "Trace"
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            try:
                with open(trace_file, "a", encoding="utf-8") as f:
                    f.write(f"[{timestamp}] Trace forces câble ({label})\n")
                    f.write(f"  U_bateau      : {_fmt(U_bat)}\n")
                    f.write(f"  U_rov         : {_fmt(U_rov)}\n")
                    f.write(f"  Traction bateau : {_fmt(F_bateau)} N\n")
                    f.write(f"  Traction ROV    : {_fmt(F_rov)} N\n")
                    f.write(f"  Poids apparent  : {_fmt(F_poids)} N\n")
                    f.write(f"  Traînée câble   : {_fmt(F_drag)} N\n")
                    f.write(f"  Somme forces    : {_fmt(F_sum)} N\n")
                    f.write(f"  Longueur câble L0     : {L0:.6f} m\n")
                    f.write(f"  Longueur segments Σds : {L_segments:.6f} m\n")
                    if ds_values:
                        f.write(f"  min/max ds            : {min(ds_values):.6f} ; {max(ds_values):.6f}\n")
                    f.write("\n")
            except Exception as e:
                trace_print(8, f"[DEBUG] Trace forces câble: export impossible -> {e}")
    
    def _normalize_cable_length(self, x_cable, y_cable, L_target):
        """
        Normalise la longueur du câble pour qu'elle soit exactement égale à L_target
        en conservant la forme générale du câble
        
        Parameters:
        -----------
        x_cable, y_cable : array
            Positions actuelles du câble
        L_target : float
            Longueur cible (m)
        
        Returns:
        --------
        tuple (x_cable_new, y_cable_new)
            Positions normalisées du câble
        """
        # Calculer la longueur actuelle et les distances cumulatives
        N = len(x_cable) - 1
        s_cumulative = np.zeros(N + 1)  # Distance curviligne cumulative
        
        for i in range(N):
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            ds = np.sqrt(dx**2 + dy**2)
            s_cumulative[i+1] = s_cumulative[i] + ds
        
        L_actual = s_cumulative[-1]
        
        # Si la longueur est déjà correcte, retourner tel quel
        if abs(L_actual - L_target) < 1e-6:
            return x_cable.copy(), y_cable.copy()
        
        # Normaliser les distances cumulatives à la longueur cible
        if L_actual > 1e-6:
            s_cumulative = s_cumulative * (L_target / L_actual)
        else:
            # Si le câble a une longueur nulle, créer un câble rectiligne
            x_cable_new = np.linspace(x_cable[0], x_cable[-1], N + 1)
            y_cable_new = np.linspace(y_cable[0], y_cable[-1], N + 1)
            return x_cable_new, y_cable_new
        
        # Rééchantillonner le câble à intervalles réguliers selon la longueur cible
        x_cable_new = np.zeros(N + 1)
        y_cable_new = np.zeros(N + 1)
        
        # Conserver les extrémités
        x_cable_new[0] = x_cable[0]
        y_cable_new[0] = y_cable[0]
        x_cable_new[-1] = x_cable[-1]
        y_cable_new[-1] = y_cable[-1]
        
        # CONTRAINTE PHYSIQUE : Les extrémités doivent respecter y <= 0
        y_cable_new[0] = min(y_cable_new[0], 0.0)
        y_cable_new[-1] = min(y_cable_new[-1], 0.0)
        
        # Interpoler les points intermédiaires
        s_target = np.linspace(0, L_target, N + 1)
        
        for i in range(1, N):
            s_i = s_target[i]
            
            # Trouver le segment qui contient ce point
            idx = np.searchsorted(s_cumulative, s_i)
            if idx == 0:
                idx = 1
            elif idx >= len(s_cumulative):
                idx = len(s_cumulative) - 1
            
            # Interpoler linéairement dans ce segment
            s_prev = s_cumulative[idx - 1]
            s_next = s_cumulative[idx]
            
            if abs(s_next - s_prev) > 1e-6:
                alpha = (s_i - s_prev) / (s_next - s_prev)
                x_cable_new[i] = x_cable[idx - 1] + alpha * (x_cable[idx] - x_cable[idx - 1])
                y_cable_new[i] = y_cable[idx - 1] + alpha * (y_cable[idx] - y_cable[idx - 1])
            else:
                x_cable_new[i] = x_cable[idx - 1]
                y_cable_new[i] = y_cable[idx - 1]
        
        # CONTRAINTE PHYSIQUE : Forcer tous les points à y <= 0 après interpolation
        y_cable_new = np.clip(y_cable_new, None, 0.0)
        # Réappliquer les extrémités
        y_cable_new[0] = min(y_cable[0], 0.0)
        y_cable_new[-1] = min(y_cable[-1], 0.0)
        
        return x_cable_new, y_cable_new

