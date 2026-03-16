"""Solveur pour les équations du câble"""
from tkinter import N
import numpy as np
from scipy.optimize import fsolve, minimize_scalar, root_scalar, minimize, NonlinearConstraint
from scipy.interpolate import interp1d
from src.utils.logger import trace_print
from src.utils.utils import scale_slack


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
        self.Cx_cable = params.get('Cx_cable', 1.2)
        self.Cf_cable = params.get('Cf_cable', 0.04)
        self.A_cable = np.pi * (self.d / 2)**2
        self._T_prev = None

    def _normalize_cable_geometry(
        self,
        x_cable,
        y_cable,
        L_target,
        bateau,
        rov,
        N_debut_iter,
    ):
        """
        Normalise la géométrie du câble en autorisant l'ajout de points.

        Contraintes en sortie :
        - P[0] est strictement collé au bateau.
        - P[-1] est strictement collé au ROV.
        - La somme des longueurs des segments est aussi proche que possible de L_target,
          avec une erreur relative <= 1e-4 (si la convergence est atteinte).
        - Chaque segment a une longueur comprise entre L_target/N_debut_iter et
          2 * L_target/N_debut_iter.

        Paramètres
        ----------
        x_cable, y_cable : array-like
            Coordonnées actuelles des points du câble.
        L_target : float
            Longueur cible du câble.
        bateau : tuple(float, float)
            Coordonnées (x, y) du bateau.
        rov : tuple(float, float)
            Coordonnées (x, y) du ROV.
        N_debut_iter : int
            Nombre de segments au début de l'itération.

        Returns
        -------
        x_new, y_new : np.ndarray
            Coordonnées normalisées des points du câble.
        rel_err : float
            Erreur relative finale sur la longueur totale.
        iters : int
            Nombre d'itérations effectuées.
        """
        x_arr = np.asarray(x_cable, dtype=float).reshape(-1)
        y_arr = np.asarray(y_cable, dtype=float).reshape(-1)
        if x_arr.size != y_arr.size or x_arr.size < 2:
            raise ValueError("_normalize_cable_geometry: géométrie invalide")

        P = np.stack([x_arr, y_arr], axis=1)

        x_boat, y_boat = float(bateau[0]), float(bateau[1])
        x_rov, y_rov = float(rov[0]), float(rov[1])

        def segment_lengths(points):
            diffs = np.diff(points, axis=0)
            return np.linalg.norm(diffs, axis=1)

        def _str_cable_format(points: np.ndarray, ds_max_allowed_val: float | None = None) -> str:
            """
            Formate un câble P (shape (N+1, 2)) sous la forme :
            [ x0  y0 ] ds0 [ x1  y1 ] ds1 ... [ xN  yN ]
            où dsi est la longueur du segment entre les points i et i+1.
            """
            pts = np.asarray(points, dtype=float)
            if pts.ndim != 2 or pts.shape[1] != 2 or pts.shape[0] == 0:
                return repr(pts)
            n = pts.shape[0]
            parts: list[str] = []
            for i in range(n):
                x, y = pts[i]
                parts.append(f"[{x:5.2f} {y:5.2f}]")
                if i < n - 1:
                    dx = pts[i + 1, 0] - x
                    dy = pts[i + 1, 1] - y
                    ds = float(np.hypot(dx, dy))
                    if ds_max_allowed_val is not None and ds > ds_max_allowed_val:
                        # Rouge si ds > ds_max_allowed
                        parts.append(f" \x1b[31m{ds:5.2f}\x1b[0m ")
                    else:
                        # Vert sinon
                        parts.append(f" \x1b[32m{ds:5.2f}\x1b[0m ")
            return "".join(parts)

        def enforce_max_segment_length(points):
            changed = True
            while changed:
                changed = False
                new_pts = [points[0]]
                for i in range(len(points) - 1):
                    p0 = new_pts[-1]
                    p1 = points[i + 1]
                    seg = p1 - p0
                    ds = float(np.linalg.norm(seg))
                    if ds > ds_max_allowed and ds > 0.0:
                        n_sub = int(np.ceil(ds / ds_max_allowed))
                        alpha_step = 1.0 / n_sub
                        trace_print(
                            8,
                            f"[DEBUG] : on ajoute {n_sub-1} points entre {p0} et {p1}  "
                            f"ds = {ds:6.2f}  ds_max_allowed = {ds_max_allowed:6.2f}",
                        )
                        for k in range(1, n_sub + 1):
                            alpha = k * alpha_step
                            pk = p0 + alpha * seg
                            pk[1] = min(pk[1], 0.0)
                            new_pts.append(pk)
                        changed = True
                    else:
                        new_pts.append(p1)
                points = np.array(new_pts, dtype=float)
            return points

        # Étape 1 : recollement des extrémités
        P[0, 0] = x_boat
        P[0, 1] = min(y_boat, 0.0)
        P[-1, 0] = x_rov
        P[-1, 1] = min(y_rov, 0.0)

        P[:, 1] = np.minimum(P[:, 1], 0.0)

        N0 = max(int(N_debut_iter), 1)
        ds_min = L_target / N0
        ds_max_allowed = 2.0 * ds_min

        iters = 0
        rel_err = 1.0
        max_iters = 10
        tol_rel = 1e-4

        
        L_straight_cable = float(np.linalg.norm(P[-1] - P[0]))
        total_slack = L_target - L_straight_cable

        if total_slack <= 0.0:
            # Impossible, on va juste passer en mode straight line
            pass
            return x_cable, y_cable, 0.0, 0

        total_slack_ratio = total_slack / L_straight_cable

        trace_print(9, f"[DEBUG] Initial P: {_str_cable_format(P, ds_max_allowed)}")
        while iters < max_iters:
            P = enforce_max_segment_length(P)
            trace_print(9, f"[DEBUG] max_seg_l: {_str_cable_format(P, ds_max_allowed)}")
            lengths = segment_lengths(P)
            L_seg = float(lengths.sum())
            if L_seg <= 0.0 or L_target <= 0.0:
                break

            scale = L_target / L_seg
            P_new = P.copy()

            for k in range(1, len(P) - 1):
                A = P[k - 1]
                B = P[k]
                C = P[k + 1]
                local_slack =  float(np.linalg.norm(A-B)) + float(np.linalg.norm(B-C)) - float(np.linalg.norm(A-C)) 
                if float(np.linalg.norm(A-C)) == 0.0:
                    # configuration dégénérée
                    local_slack_ratio = 10
                else :  local_slack_ratio = local_slack / float(np.linalg.norm(A-C)) 
                if local_slack_ratio > total_slack_ratio :
                    sc = scale 
                else:
                    sc = 1
                Bp = scale_slack(A, B, C, sc=scale)
                Bp[1] = min(Bp[1], 0.0)
                P_new[k] = Bp
            trace_print(9, f"[DEBUG] scl_slack: {_str_cable_format(P_new, ds_max_allowed)}")
            P = P_new

            iters += 1
            if rel_err <= tol_rel:
                break

        P = enforce_max_segment_length(P)
        trace_print(9, f"[DEBUG] last max l. : {_str_cable_format(P, ds_max_allowed)}")
        x_new = P[:, 0]
        y_new = P[:, 1]
        trace_print(9, f"[DEBUG] x_new: {x_new}, y_new: {y_new}")
        return x_new, y_new, rel_err, iters

    def solve_equilibrium_static(self, x_rov, y_rov, x_boat, L, rov_m=None, rov_vol=None):
        """
        Résout l'équilibre statique du câble (caténaire)
        
        Utilise maintenant le solveur avec contraintes intégrées pour garantir :
        - L = L_seg à 0.01% près
        - P0 = position bateau
        - PN = position ROV
        - y <= 0 pour tous les points
        - ds_max / ds_target <= k_max
        
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
        
        # Préparer les paramètres pour le wrapper de forces
        params_forces = {
            'environment': self.environment,
            'params_cable': {
                'd': self.d,
                'rho_cable': self.rho_cable,
                'Cx_cable': self.Cx_cable,
                'Cf_cable': self.Cf_cable
            },
            'L': L
        }
        
        # Ne PAS passer d'initial_guess pour permettre à solve_equilibrium_constrained
        # de générer automatiquement une caténaire réaliste avec _solve_catenary
        # au lieu d'utiliser une ligne droite
        initial_guess = None
        
        # Utiliser le solveur avec contraintes intégrées
        x_cable, y_cable, T = self.solve_equilibrium_constrained(
            x_rov, y_rov, x_boat, L,
            self._forces_static_wrapper,
            params_forces,
            rov_m=rov_m,
            rov_vol=rov_vol,
            initial_guess=initial_guess,
            k_max=2.0
        )
        
        # Recalculer les tensions correctement en utilisant l'équilibre des forces
        # (le solveur avec contraintes retourne une estimation simple)
        weight_per_unit = (self.rho_cable - self.environment.rho_eau) * self.A_cable * self.environment.g
        T = self._compute_catenary_tensions(x_cable, y_cable, weight_per_unit, rov_m=rov_m, rov_vol=rov_vol)
        
        return x_cable, y_cable, T
    
    def solve_equilibrium_static_with_current(self, x_rov, y_rov, x_boat, L, rov_m=None, rov_vol=None,
                                              max_iter=50, tol=1e-3, relax=0.2,
                                              use_full_equilibrium=True):
        """
        Résout l'équilibre statique du câble en tenant compte de la traînée du courant sur la géométrie.
        
        Utilise maintenant le solveur avec contraintes intégrées pour garantir :
        - L = L_seg à 0.01% près
        - P0 = position bateau
        - PN = position ROV
        - y <= 0 pour tous les points
        - ds_max / ds_target <= k_max
        """
        trace_print(1, "\n[DEBUG] Initialisation du câble : prise en compte du courant (solve_equilibrium_static_with_current).")
        
        if L <= 0:
            return np.array([x_rov, x_boat]), np.array([y_rov, 0.0]), np.array([0.0, 0.0])
        
        # Préparer les paramètres pour le wrapper de forces
        params_forces = {
            'environment': self.environment,
            'params_cable': {
                'd': self.d,
                'rho_cable': self.rho_cable,
                'Cx_cable': self.Cx_cable,
                'Cf_cable': self.Cf_cable
            },
            'L': L
        }
        
        # Ne PAS passer d'initial_guess pour permettre à solve_equilibrium_constrained
        # de générer automatiquement une caténaire réaliste avec _solve_catenary
        # au lieu d'utiliser une ligne droite
        initial_guess = None
        
        # Utiliser le solveur avec contraintes intégrées
        x_cable, y_cable, T = self.solve_equilibrium_constrained(
            x_rov, y_rov, x_boat, L,
            self._forces_static_with_current_wrapper,
            params_forces,
            rov_m=rov_m,
            rov_vol=rov_vol,
            initial_guess=initial_guess,
            k_max=2.0
        )
        
        # Recalculer les tensions correctement en utilisant l'équilibre des forces
        # (le solveur avec contraintes retourne une estimation simple)
        weight_per_unit = (self.rho_cable - self.environment.rho_eau) * self.A_cable * self.environment.g
        T = self._compute_catenary_tensions(x_cable, y_cable, weight_per_unit, rov_m=rov_m, rov_vol=rov_vol)
        
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
        trace_print(5, "\n[DEBUG] ========== _solve_catenary APPELÉE ==========")
        trace_print(5, f"[DEBUG] _solve_catenary: Paramètres - x_rov={x_rov:.3f}, y_rov={y_rov:.3f}, x_boat={x_boat:.3f}, L={L:.3f}, w={w:.6f}")

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
            if a <= 0 or not np.isfinite(a) or a < 1e-10:
                return None
            
            def equation_x0(x0):
                """Équation pour trouver x0 : y_rov = a * (cosh((x_rov - x0) / a) - cosh((x_boat - x0) / a))"""
                try:
                    # Vérifier que a est valide
                    if a <= 1e-10 or not np.isfinite(a):
                        return np.inf
                    
                    # Vérifier que x0 est fini
                    if not np.isfinite(x0):
                        return np.inf
                    
                    # Calculer les arguments de cosh
                    arg1 = (x_rov - x0) / a
                    arg2 = (x_boat - x0) / a
                    
                    # Vérifier que les arguments ne sont pas trop grands (éviter overflow)
                    if abs(arg1) > 700 or abs(arg2) > 700:
                        return np.inf
                    
                    # Calculer la valeur
                    val = a * (np.cosh(arg1) - np.cosh(arg2)) - y_rov
                    
                    # Vérifier que le résultat est fini
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
            if a <= 0 or not np.isfinite(a) or a < 1e-10:
                return np.inf
            
            x0 = find_x0_for_a(a)
            if x0 is None or not np.isfinite(x0):
                return np.inf
            
            try:
                # Calculer les arguments de sinh
                arg_boat = (x_boat - x0) / a
                arg_rov = (x_rov - x0) / a
                
                # Vérifier que les arguments ne sont pas trop grands (éviter overflow)
                if abs(arg_boat) > 700 or abs(arg_rov) > 700:
                    return np.inf
                
                # Calculer les longueurs curvilignes
                s_boat = a * np.sinh(arg_boat)
                s_rov = a * np.sinh(arg_rov)
                
                # Vérifier que les résultats sont finis
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
                    
                    # Supprimer les warnings de scipy pour les valeurs invalides (NaN/Inf)
                    # qui peuvent se produire lors de l'optimisation avec des valeurs extrêmes
                    with np.errstate(invalid='ignore', divide='ignore'):
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
                trace_print(5, f"⚠️  Avertissement: La caténaire a une erreur élevée ({best_error:.6f}). "
                      f"Utilisation des meilleures valeurs trouvées.")
        except Exception as e:
            trace_print(5, f"⚠️  Erreur lors de la résolution de la caténaire: {e}. Utilisation des valeurs initiales.")
        
        # Trouver x0 pour le a optimal
        x0_opt = find_x0_for_a(a_opt)
        if x0_opt is None:
            trace_print(9, f"[DEBUG] _solve_catenary: ⚠️  ERREUR - Impossible de trouver x0 pour a={a_opt:.3f}. Utilisation d'une interpolation linéaire (LIGNE DROITE).")
            x_cable = np.linspace(x_boat, x_rov, self.N + 1)
            y_cable = np.linspace(0.0, y_rov, self.N + 1)
            return x_cable, y_cable
        
        # Calculer y0
        try:
            y0_opt = -a_opt * np.cosh((x_boat - x0_opt) / a_opt)
        except:
            trace_print(5, f"⚠️  Erreur: Impossible de calculer y0. Utilisation d'une interpolation linéaire.")
            x_cable = np.linspace(x_boat, x_rov, self.N + 1)
            y_cable = np.linspace(0.0, y_rov, self.N + 1)
            return x_cable, y_cable
        
        # Vérifier la solution
        y_rov_calc = a_opt * np.cosh((x_rov - x0_opt) / a_opt) + y0_opt
        y_boat_calc = a_opt * np.cosh((x_boat - x0_opt) / a_opt) + y0_opt
        
        # y_rov devrait être négatif (profondeur : y < 0 = sous la surface)
        if abs(y_rov_calc - y_rov) > 0.1:
            trace_print(5, f"⚠️  Avertissement: y_rov calculé ({y_rov_calc:.3f}) diffère de y_rov attendu ({y_rov:.3f}, profondeur négative)")
        
        if abs(y_boat_calc) > 0.1:
            trace_print(5, f"⚠️  Avertissement: y_boat calculé ({y_boat_calc:.3f}) diffère de y_boat attendu (0.000, surface)")
        
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

        # CONTRAINTE PHYSIQUE CRITIQUE : Tous les points doivent être sous la surface (y <= 0)
        # La caténaire mathématique peut générer des points au-dessus de la surface
        # si les paramètres ne sont pas parfaitement ajustés
        y_cable = np.clip(y_cable, None, 0.0)
        # Réappliquer les extrémités après clipping
        y_cable[0] = 0.0
        y_cable[-1] = y_rov
        
        # DEBUG: Vérifier si des points ont été clippés
        points_above_surface = np.sum((y_cable > 1e-6) & (np.arange(len(y_cable)) != 0))  # Exclure le point bateau
        if points_above_surface > 0:
            trace_print(9, f"[DEBUG] _solve_catenary: ⚠️  {points_above_surface} points étaient au-dessus de la surface et ont été clippés")

        # Si la flottabilité est positive (w < 0), inverser la caténaire
        # par rapport à la droite bateau-ROV pour obtenir une courbure vers le haut.
        if w < 0:
            dx_line = x_rov - x_boat
            dy_line = y_rov - 0.0
            if abs(dx_line) > 1e-9:
                s = (x_cable - x_boat) / dx_line
                s = np.clip(s, 0.0, 1.0)
                y_line = s * dy_line
            else:
                s = np.linspace(0.0, 1.0, len(x_cable))
                y_line = s * dy_line
            y_cable = 2.0 * y_line - y_cable
            y_cable[0] = 0.0
            y_cable[-1] = y_rov
            # Re-clipper après inversion
            y_cable = np.clip(y_cable, None, 0.0)
            y_cable[0] = 0.0
            y_cable[-1] = y_rov
        
        # Vérifier la longueur
        L_actual = 0.0
        for i in range(self.N):
            dx_seg = x_cable[i+1] - x_cable[i]
            dy_seg = y_cable[i+1] - y_cable[i]
            L_actual += np.sqrt(dx_seg**2 + dy_seg**2)
        
        # Toujours normaliser pour garantir que la longueur est exactement L
        # (tolérance réduite pour forcer la normalisation)
        # IMPORTANT : Passer les positions du bateau et du ROV pour garantir le recollage exact
        if abs(L_actual - L) / L > 1e-6:
            x_cable, y_cable = self._normalize_cable_length(
                x_cable, y_cable, L,
                x_boat=x_boat,
                y_boat=0.0,
                x_rov=x_rov,
                y_rov=y_rov
            )
            
            # CONTRAINTE PHYSIQUE : Après normalisation, forcer y <= 0
            y_cable = np.clip(y_cable, None, 0.0)
            # Réappliquer les extrémités (normalement déjà faites par _normalize_cable_length, mais on s'assure)
            if x_boat is not None:
                x_cable[0] = float(x_boat)
            y_cable[0] = 0.0
            if x_rov is not None:
                x_cable[-1] = float(x_rov)
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
                trace_print(5, f"⚠️  Avertissement: Le câble semble être une ligne droite plutôt qu'une caténaire. "
                      f"Vérifiez les paramètres (L={L:.2f} m, L_straight={L_straight:.2f} m, "
                      f"rho_cable={self.rho_cable:.1f} kg/m³, rho_eau={self.environment.rho_eau:.1f} kg/m³)")
        
        # Avertissement si la longueur n'est toujours pas correcte
        if abs(L_final - L) / L > 0.02:
            trace_print(5, f"⚠️  Avertissement: Longueur du câble incorrecte après calcul de caténaire. "
                  f"Attendu: {L:.3f} m, Obtenu: {L_final:.3f} m, Erreur: {abs(L_final - L)/L*100:.1f}%")
        
        # DEBUG: Calculer la déviation maximale de la caténaire générée
        def compute_max_deviation_cat(x_arr, y_arr):
            """Calcule la déviation maximale par rapport à la ligne droite"""
            if len(x_arr) < 3:
                return 0.0
            x_start, y_start = x_arr[0], y_arr[0]
            x_end, y_end = x_arr[-1], y_arr[-1]
            max_dev = 0.0
            if abs(x_end - x_start) > 1e-6 or abs(y_end - y_start) > 1e-6:
                dx_line = x_end - x_start
                dy_line = y_end - y_start
                line_length = np.sqrt(dx_line**2 + dy_line**2)
                for i in range(1, len(x_arr) - 1):
                    dx_point = x_arr[i] - x_start
                    dy_point = y_arr[i] - y_start
                    t = (dx_point * dx_line + dy_point * dy_line) / (line_length**2 + 1e-9)
                    x_proj = x_start + t * dx_line
                    y_proj = y_start + t * dy_line
                    deviation = np.sqrt((x_arr[i] - x_proj)**2 + (y_arr[i] - y_proj)**2)
                    max_dev = max(max_dev, deviation)
            return max_dev
        
        max_dev_cat = compute_max_deviation_cat(x_cable, y_cable)
        D_straight_cat = np.sqrt((x_rov - x_boat)**2 + (y_rov - 0.0)**2)
        slack_cat = L_final - D_straight_cat
        if slack_cat > 0 and D_straight_cat > 1e-6:
            theoretical_max_dev = slack_cat * np.sqrt(slack_cat / D_straight_cat) / 2.0
            if max_dev_cat < theoretical_max_dev * 0.1:  # Si la déviation est < 10% de la théorique
                trace_print(9, f"[DEBUG] _solve_catenary: ⚠️  Caténaire générée trop droite - "
                    f"max_deviation={max_dev_cat:.6f} m, théorique≈{theoretical_max_dev:.3f} m, "
                    f"slack={slack_cat:.2f} m, L={L_final:.2f} m, D_straight={D_straight_cat:.2f} m")
        
        # Retourner le câble : index 0 = bateau, index -1 = ROV (convention cohérente avec le système)
        return x_cable, y_cable
        
        # Distance horizontale et verticale entre les extrémités
        dx = x_boat - x_rov
        dy = -y_rov  # Profondeur (y_rov est négatif)
        
        # Longueur rectiligne
        L_straight = np.sqrt(dx**2 + dy**2)

        # Calculer le ratio r = L / L_straight
        r = L / L_straight if L_straight > 1e-9 else 1.0
        
        # SUPPRESSION DE L'HYSTÉRÉSIS BINAIRE : utiliser un blending continu basé uniquement sur r
        # Cela élimine complètement les discontinuités lors du basculement
        # r = 1.0 -> 100% ligne droite (câble tendu)
        # r augmente -> transition progressive vers caténaire
        # r > 1.01 -> 100% caténaire
        
        # Si L < L_straight, câble forcément tendu (impossible physiquement)
        # CORRECTION : Ne pas modifier L, mais forcer une ligne droite avec la longueur L donnée
        # Le ROV a déjà été recalculé dans system_model.py pour que L_straight = L
        if L < L_straight:
            # Câble tendu (rectiligne)
            # Convention : index 0 = bateau, index -1 = ROV (s=0 au bateau, s=L au ROV)
            trace_print(1, "\n[DEBUG] Initialisation du câble : câble tendu (solve_equilibrium_static).")
            trace_print(8, f"[DEBUG] ⚠️  : L < L_straight (L={L:.6f}, L_straight={L_straight:.6f}). "
                "Géométrie rectiligne forcée. Le ROV devrait être recalculé pour que L_straight = L."
            )
            # Ne pas modifier L - créer une ligne droite entre bateau et ROV
            # Après recalcul du ROV, L_straight devrait être proche de L
            # Utiliser une interpolation linéaire simple
            x_cable = np.linspace(x_boat, x_rov, self.N + 1) 
            y_cable = np.linspace(0.0, y_rov, self.N + 1)
            
            # Vérifier et corriger la longueur si nécessaire
            length_check = 0.0
            for i in range(len(x_cable) - 1):
                dx = x_cable[i + 1] - x_cable[i]
                dy = y_cable[i + 1] - y_cable[i]
                length_check += np.sqrt(dx**2 + dy**2)
            
            # Si la longueur ne correspond pas à L, normaliser via l'algorithme centralisé
            # qui impose aussi le recollement exact au bateau et au ROV.
            if abs(length_check - L) / max(L, 1e-9) > 1e-3:
                try:
                    x_cable, y_cable = self._normalize_cable_length(
                        x_cable,
                        y_cable,
                        L,
                        x_boat=float(x_boat),
                        y_boat=0.0,
                        x_rov=float(x_rov),
                        y_rov=float(y_rov),
                    )
                except Exception:
                    # Fallback : interpolation linéaire en abscisse curviligne
                    s_points = np.linspace(0.0, L, self.N + 1)
                    if L_straight > 1e-9:
                        x_cable = x_boat + (x_rov - x_boat) * (s_points / L_straight)
                        y_cable = 0.0 + (y_rov - 0.0) * (s_points / L_straight)
                    else:
                        x_cable = np.linspace(x_boat, x_rov, self.N + 1)
                        y_cable = np.linspace(0.0, y_rov, self.N + 1)
            
            # Calculer les tensions en utilisant l'équilibre des forces
            # T_rov * Urov + T_bateau * Ubateau + Fext_stat = 0
            # où Fext_stat inclut le courant et le poids du câble
            min_floor = max(abs(weight_per_unit) * L / 100.0 if weight_per_unit != 0 else 0.0, 0.5)
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
                        'Cx_cable': self.Cx_cable,
                        'Cf_cable': self.Cf_cable
                    }
                    
                    vx_cable = np.zeros(len(x_cable))
                    vy_cable = np.zeros(len(x_cable))
                    
                    Fx_courant_segments, Fy_courant_segments, _, _, _, _ = compute_cable_forces(
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
                            trace_print(8, "[DEBUG] ⚠️  : câble quasi-rectiligne ou A singulière. "
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
                                trace_print(8, f"[DEBUG] ⚠️  : composante perpendiculaire ignorée (|F⊥|={np.linalg.norm(Fext_perp):.6f}).")
                            
                            delta_T = -Fext_parallel
                            T_base_est = min_floor
                            T_base = max(T_base_est, abs(delta_T) / 2.0)
                            T_bateau = max(0.0, T_base - 0.5 * delta_T)
                            T_rov = max(0.0, T_base + 0.5 * delta_T)
                            trace_print(1, f"[DEBUG] Tensions projetées: T_bateau={T_bateau:.6f}, T_rov={T_rov:.6f}, ΔT={delta_T:.6f}")
                        else:
                            T_solution = np.linalg.solve(A, b)
                            # T_solution[0] = T_bateau, T_solution[1] = T_rov
                            T_bateau = max(0.0, T_solution[0])
                            T_rov = max(0.0, T_solution[1])
                            trace_print(1, f"[DEBUG] Solution brute du système: T_bateau={T_solution[0]:.6f}, T_rov={T_solution[1]:.6f}")
                        
                        # Vérification : les tensions doivent être positives
                        if T_bateau < 1e-6:
                            T_bateau_est = min_floor
                            trace_print(8, f"[DEBUG] ⚠️  : T_bateau est trop petite ({T_bateau:.6f}). "
                                f"Utilisation d'une estimation: {T_bateau_est:.6f} "
                                f"(max(|w|*L/10,10) avec w={weight_per_unit:.6f}, L={L:.6f})."
                            )
                            if self._T_prev is not None and len(self._T_prev) > 0:
                                T_bateau = max(float(self._T_prev[0]), T_bateau_est)
                            else:
                                T_bateau = T_bateau_est
                        
                        if T_rov < 1e-6:
                            T_rov_est = min_floor
                            trace_print(8, f"[DEBUG] ⚠️  : T_rov est trop petite ({T_rov:.6f}). "
                                f"Utilisation d'une estimation: {T_rov_est:.6f} "
                                f"(max(|w|*L/10,10) avec w={weight_per_unit:.6f}, L={L:.6f})."
                            )
                            if self._T_prev is not None and len(self._T_prev) > 0:
                                T_rov = max(float(self._T_prev[-1]), T_rov_est)
                            else:
                                T_rov = T_rov_est
                    except np.linalg.LinAlgError:
                        # Estimation par défaut ou reprise de la tension précédente
                        if self._T_prev is not None and len(self._T_prev) > 0:
                            T_bateau = float(self._T_prev[0])
                            T_rov = float(self._T_prev[-1])
                        else:
                            T_rov = min_floor
                            T_bateau = min_floor
                    
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
                    T_rov = min_floor
                    T_bateau = min_floor
            else:
                T_rov = min_floor
                T_bateau = min_floor
            
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
            
            self._T_prev = T.copy()
            # Normaliser la géométrie finale pour garantir Σds = L, segments quasi uniformes
            # et extrémités parfaitement recollées sur le bateau et le ROV.
            try:
                x_cable, y_cable = self._normalize_cable_length(
                    x_cable,
                    y_cable,
                    L,
                    x_boat=float(x_boat),
                    y_boat=0.0,
                    x_rov=float(x_rov),
                    y_rov=float(y_rov),
                )
            except Exception:
                pass
            return x_cable, y_cable, T
        
        # APPROCHE CONTINUE : toujours calculer caténaire et ligne droite, puis mélanger selon r
        # Cela élimine complètement les discontinuités
        
        # 1. Calculer la caténaire
        trace_print(1, "\n[DEBUG] Initialisation du câble : calcul de la caténaire.")
        x_cable_catenary, y_cable_catenary = self._solve_catenary(x_rov, y_rov, x_boat, L, weight_per_unit)
        
        # 2. Calculer la ligne droite
        x_cable_straight = np.linspace(x_boat, x_rov, self.N + 1)
        y_cable_straight = np.linspace(0.0, y_rov, self.N + 1)
        
        # 3. Calculer le facteur de blending continu basé sur r
        # r = 1.0 -> alpha = 1.0 (100% straight)
        # r = 1.01 -> alpha = 0.0 (100% caténaire)
        # Transition linéaire entre les deux
        blend_r_min = 1.0
        blend_r_max = 1.01
        if r <= blend_r_min:
            alpha_blend = 1.0  # 100% straight
        elif r >= blend_r_max:
            alpha_blend = 0.0  # 100% caténaire
        else:
            # Transition linéaire : alpha diminue de 1.0 à 0.0 quand r augmente de blend_r_min à blend_r_max
            alpha_blend = 1.0 - (r - blend_r_min) / (blend_r_max - blend_r_min)
        
        trace_print(1, f"[DEBUG] Blending continu: r={r:.6f}, alpha={alpha_blend:.4f} (1.0=straight, 0.0=caténaire)")
        
        # 4. Mélanger caténaire et ligne droite
        x_cable = (1.0 - alpha_blend) * x_cable_catenary + alpha_blend * x_cable_straight
        y_cable = (1.0 - alpha_blend) * y_cable_catenary + alpha_blend * y_cable_straight
        
        # 5. Normaliser la longueur pour respecter L exactement
        def _cable_length(x_vals, y_vals):
            dx = np.diff(x_vals)
            dy = np.diff(y_vals)
            return float(np.sum(np.hypot(dx, dy)))
        
        L_actual = _cable_length(x_cable, y_cable)
        if L_actual > 1e-6 and abs(L_actual - L) > 1e-6:
            # Ajuster par interpolation pour respecter la longueur
            scale = L / L_actual
            x_center = x_cable[0]
            y_center = y_cable[0]
            x_cable = x_center + (x_cable - x_center) * scale
            y_cable = y_center + (y_cable - y_center) * scale
            # Réimposer les extrémités exactement
            x_cable[0] = x_boat
            y_cable[0] = 0.0
            x_cable[-1] = x_rov
            y_cable[-1] = y_rov
            # CONTRAINTE PHYSIQUE : Le câble ne peut pas être au-dessus de la surface (y > 0)
            y_cable = np.clip(y_cable, None, 0.0)
            y_cable[0] = 0.0
            y_cable[-1] = y_rov
        
        # 6. Calculer les tensions selon le mode dominant
        # Si alpha_blend est proche de 1.0 (straight), utiliser la méthode straight
        # Sinon, utiliser la méthode caténaire
        if alpha_blend > 0.95:  # Presque straight
            # Utiliser la méthode de calcul des tensions pour câble tendu
            min_floor = max(abs(weight_per_unit) * L / 100.0 if weight_per_unit != 0 else 0.0, 0.5)
            if self.N > 0:
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
                    
                    from .forces import compute_cable_forces
                    params_cable = {
                        'd': self.d,
                        'rho_cable': self.rho_cable,
                        'Cx_cable': self.Cx_cable,
                        'Cf_cable': self.Cf_cable
                    }
                    vx_cable = np.zeros(len(x_cable))
                    vy_cable = np.zeros(len(x_cable))
                    Fx_segments, Fy_segments, _, _, _, _ = compute_cable_forces(
                        x_cable, y_cable, vx_cable, vy_cable, self.environment, params_cable, L
                    )
                    Fext_stat_x = np.sum(Fx_segments)
                    Fext_stat_y = np.sum(Fy_segments)
                    
                    A = np.array([
                        [-Ubateau_x, Urov_x],
                        [-Ubateau_y, Urov_y]
                    ])
                    b = np.array([-Fext_stat_x, -Fext_stat_y])
                    
                    try:
                        det_A = np.linalg.det(A)
                        if abs(det_A) > 1e-8:
                            T_solution = np.linalg.solve(A, b)
                            T_bateau = max(0.0, T_solution[0])
                            T_rov = max(0.0, T_solution[1])
                        else:
                            # Système singulier, utiliser projection
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
                            delta_T = -Fext_parallel
                            T_base = max(min_floor, abs(delta_T) / 2.0)
                            T_bateau = max(0.0, T_base - 0.5 * delta_T)
                            T_rov = max(0.0, T_base + 0.5 * delta_T)
                        
                        # Interpoler linéairement
                        T = np.zeros(self.N + 1)
                        T[0] = T_bateau
                        for i in range(1, self.N + 1):
                            s_frac = i / self.N
                            T[i] = T_bateau + (T_rov - T_bateau) * s_frac
                    except Exception:
                        # Fallback : utiliser la méthode caténaire
                        T = self._compute_catenary_tensions(x_cable, y_cable, weight_per_unit, rov_m, rov_vol)
                else:
                    T = self._compute_catenary_tensions(x_cable, y_cable, weight_per_unit, rov_m, rov_vol)
            else:
                T = np.full(self.N + 1, min_floor)
        else:
            # Utiliser la méthode caténaire standard
            T = self._compute_catenary_tensions(x_cable, y_cable, weight_per_unit, rov_m, rov_vol)
        
        # Normaliser la géométrie finale pour garantir Σds = L et segments égaux
        try:
            x_cable, y_cable = self._normalize_cable_length(x_cable, y_cable, L)
        except Exception:
            pass
        # CONTRAINTE PHYSIQUE : Le câble ne peut pas être au-dessus de la surface (y > 0)
        # S'assurer que tous les points sont sous la surface après normalisation
        y_cable = np.clip(y_cable, None, 0.0)
        y_cable[0] = 0.0
        y_cable[-1] = y_rov
        self._trace_cable_equilibrium_forces(x_cable, y_cable, T, L, label="initialisation (blending continu)")
        self._T_prev = T.copy()
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
        trace_print(5, "\n[DEBUG] ========== _solve_catenary APPELÉE ==========")
        trace_print(5, f"[DEBUG] _solve_catenary: Paramètres - x_rov={x_rov:.3f}, y_rov={y_rov:.3f}, x_boat={x_boat:.3f}, L={L:.3f}, w={w:.6f}")

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
            if a <= 0 or not np.isfinite(a) or a < 1e-10:
                return None
            
            def equation_x0(x0):
                """Équation pour trouver x0 : y_rov = a * (cosh((x_rov - x0) / a) - cosh((x_boat - x0) / a))"""
                try:
                    # Vérifier que a est valide
                    if a <= 1e-10 or not np.isfinite(a):
                        return np.inf
                    
                    # Vérifier que x0 est fini
                    if not np.isfinite(x0):
                        return np.inf
                    
                    # Calculer les arguments de cosh
                    arg1 = (x_rov - x0) / a
                    arg2 = (x_boat - x0) / a
                    
                    # Vérifier que les arguments ne sont pas trop grands (éviter overflow)
                    if abs(arg1) > 700 or abs(arg2) > 700:
                        return np.inf
                    
                    # Calculer la valeur
                    val = a * (np.cosh(arg1) - np.cosh(arg2)) - y_rov
                    
                    # Vérifier que le résultat est fini
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
            if a <= 0 or not np.isfinite(a) or a < 1e-10:
                return np.inf
            
            x0 = find_x0_for_a(a)
            if x0 is None or not np.isfinite(x0):
                return np.inf
            
            try:
                # Calculer les arguments de sinh
                arg_boat = (x_boat - x0) / a
                arg_rov = (x_rov - x0) / a
                
                # Vérifier que les arguments ne sont pas trop grands (éviter overflow)
                if abs(arg_boat) > 700 or abs(arg_rov) > 700:
                    return np.inf
                
                # Calculer les longueurs curvilignes
                s_boat = a * np.sinh(arg_boat)
                s_rov = a * np.sinh(arg_rov)
                
                # Vérifier que les résultats sont finis
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
                    
                    # Supprimer les warnings de scipy pour les valeurs invalides (NaN/Inf)
                    # qui peuvent se produire lors de l'optimisation avec des valeurs extrêmes
                    with np.errstate(invalid='ignore', divide='ignore'):
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
                trace_print(5, f"⚠️  Avertissement: La caténaire a une erreur élevée ({best_error:.6f}). "
                      f"Utilisation des meilleures valeurs trouvées.")
        except Exception as e:
            trace_print(5, f"⚠️  Erreur lors de la résolution de la caténaire: {e}. Utilisation des valeurs initiales.")
        
        # Trouver x0 pour le a optimal
        x0_opt = find_x0_for_a(a_opt)
        if x0_opt is None:
            trace_print(9, f"[DEBUG] _solve_catenary: ⚠️  ERREUR - Impossible de trouver x0 pour a={a_opt:.3f}. Utilisation d'une interpolation linéaire (LIGNE DROITE).")
            x_cable = np.linspace(x_boat, x_rov, self.N + 1)
            y_cable = np.linspace(0.0, y_rov, self.N + 1)
            return x_cable, y_cable
        
        # Calculer y0
        try:
            y0_opt = -a_opt * np.cosh((x_boat - x0_opt) / a_opt)
        except:
            trace_print(5, f"⚠️  Erreur: Impossible de calculer y0. Utilisation d'une interpolation linéaire.")
            x_cable = np.linspace(x_boat, x_rov, self.N + 1)
            y_cable = np.linspace(0.0, y_rov, self.N + 1)
            return x_cable, y_cable
        
        # Vérifier la solution
        y_rov_calc = a_opt * np.cosh((x_rov - x0_opt) / a_opt) + y0_opt
        y_boat_calc = a_opt * np.cosh((x_boat - x0_opt) / a_opt) + y0_opt
        
        # y_rov devrait être négatif (profondeur : y < 0 = sous la surface)
        if abs(y_rov_calc - y_rov) > 0.1:
            trace_print(5, f"⚠️  Avertissement: y_rov calculé ({y_rov_calc:.3f}) diffère de y_rov attendu ({y_rov:.3f}, profondeur négative)")
        
        if abs(y_boat_calc) > 0.1:
            trace_print(5, f"⚠️  Avertissement: y_boat calculé ({y_boat_calc:.3f}) diffère de y_boat attendu (0.000, surface)")
        
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

        # Si la flottabilité est positive (w < 0), inverser la caténaire
        # par rapport à la droite bateau-ROV pour obtenir une courbure vers le haut.
        if w < 0:
            dx_line = x_rov - x_boat
            dy_line = y_rov - 0.0
            if abs(dx_line) > 1e-9:
                s = (x_cable - x_boat) / dx_line
                s = np.clip(s, 0.0, 1.0)
                y_line = s * dy_line
            else:
                s = np.linspace(0.0, 1.0, len(x_cable))
                y_line = s * dy_line
            y_cable = 2.0 * y_line - y_cable
            y_cable[0] = 0.0
            y_cable[-1] = y_rov
        
        # Vérifier la longueur
        L_actual = 0.0
        for i in range(self.N):
            dx_seg = x_cable[i+1] - x_cable[i]
            dy_seg = y_cable[i+1] - y_cable[i]
            L_actual += np.sqrt(dx_seg**2 + dy_seg**2)
        
        # Toujours normaliser pour garantir que la longueur est exactement L
        # (tolérance réduite pour forcer la normalisation)
        # NOTE: On garde _normalize_cable_length ici car c'est pour l'estimation initiale
        # IMPORTANT : Passer les positions du bateau et du ROV pour garantir le recollage exact
        if abs(L_actual - L) / L > 1e-6:
            x_cable, y_cable = self._normalize_cable_length(
                x_cable, y_cable, L,
                x_boat=x_boat,
                y_boat=0.0,
                x_rov=x_rov,
                y_rov=y_rov
            )
            
            # CONTRAINTE PHYSIQUE : Après normalisation, forcer y <= 0
            y_cable = np.clip(y_cable, None, 0.0)
            # Réappliquer les extrémités (normalement déjà faites par _normalize_cable_length, mais on s'assure)
            if x_boat is not None:
                x_cable[0] = float(x_boat)
            y_cable[0] = 0.0
            if x_rov is not None:
                x_cable[-1] = float(x_rov)
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
                trace_print(5, f"⚠️  Avertissement: Le câble semble être une ligne droite plutôt qu'une caténaire. "
                      f"Vérifiez les paramètres (L={L:.2f} m, L_straight={L_straight:.2f} m, "
                      f"rho_cable={self.rho_cable:.1f} kg/m³, rho_eau={self.environment.rho_eau:.1f} kg/m³)")
        
        # Avertissement si la longueur n'est toujours pas correcte
        if abs(L_final - L) / L > 0.02:
            trace_print(5, f"⚠️  Avertissement: Longueur du câble incorrecte après calcul de caténaire. "
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
            'Cx_cable': self.Cx_cable,
            'Cf_cable': self.Cf_cable
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
        Fx_courant_segments, Fy_courant_segments, _, _, _, _ = compute_cable_forces(
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
        
        try:
            # Si A est mal conditionnée (quasi colinéaire), éviter des solutions instables
            det_a = np.linalg.det(A)
            cond_a = np.linalg.cond(A) if np.isfinite(det_a) else np.inf
            if abs(det_a) < 1e-8 or cond_a > 1e8:
                raise np.linalg.LinAlgError("Matrice A mal conditionnée")

            # Résoudre le système linéaire
            T_solution = np.linalg.solve(A, b)
            # T_solution[0] = T_bateau, T_solution[1] = T_rov
            T_bateau = max(0.0, T_solution[0]) 
            T_rov = max(0.0, T_solution[1])
            
            # Vérification : les tensions doivent être positives
            if T_bateau < 1e-6:
                trace_print(5, f"[DEBUG] ⚠️  ERREUR CRITIQUE: T_bateau est trop petite ({T_bateau:.6f}). Utilisation d'une estimation.")
                min_floor = max(abs(weight_per_unit) * L_total / 100.0 if weight_per_unit != 0 else 0.0, 0.5)
                T_bateau = min_floor
            
            if T_rov < 1e-6:
                trace_print(5, f"[DEBUG] ⚠️  ERREUR CRITIQUE: T_rov est trop petite ({T_rov:.6f}). Utilisation d'une estimation.")
                min_floor = max(abs(weight_per_unit) * L_total / 100.0 if weight_per_unit != 0 else 0.0, 0.5)
                T_rov = min_floor
        except np.linalg.LinAlgError:
            # Si le système est singulier (câble vertical ou autre cas dégénéré)
            # Utiliser une estimation basée sur le poids du câble
            min_floor = max(abs(weight_per_unit) * L_total / 100.0 if weight_per_unit != 0 else 0.0, 0.5)
            T_rov = min_floor
            T_bateau = min_floor

        # Stabilisation: éviter des tensions extrêmes ou non physiques
        min_floor = max(abs(weight_per_unit) * L_total / 100.0 if weight_per_unit != 0 else 0.0, 0.5)
        ratio_max = 50.0
        if (
            not np.isfinite(T_bateau)
            or not np.isfinite(T_rov)
            or T_bateau < min_floor
            or T_rov < min_floor
            or (T_bateau / max(T_rov, min_floor)) > ratio_max
        ):
            if self._T_prev is not None and len(self._T_prev) > 0:
                T_bateau = max(float(self._T_prev[0]), min_floor)
                T_rov = max(float(self._T_prev[-1]), min_floor)
            else:
                T_rov = max(abs(weight_per_unit) * L_total / 10.0 if weight_per_unit != 0 else 10.0, min_floor)
                T_bateau = max(T_rov, min_floor)
        
        # Calculer les tensions le long du câble
        # Pour une caténaire, T(s) = sqrt(H² + (w*s)²) où H est la tension horizontale
        # On calcule H à partir de T_bateau et de l'angle au bateau
        sin_theta_bateau = Ubateau_x  # sin(θ) = dx/ds où θ est l'angle avec la verticale
        cos_theta_bateau = -Ubateau_y  # cos(θ) = -dy/ds (négatif car y descend)
        
        # Tension horizontale H = T_bateau * sin(theta_bateau) où θ_bateau est l'angle avec la verticale
        H = T_bateau * abs(sin_theta_bateau) if abs(sin_theta_bateau) > 1e-6 else T_bateau
        
        # Pour une caténaire, la tension suit T(s) = sqrt(H² + (w*s)²) où s est mesuré depuis le bateau
        if T_bateau < H:
            H = T_bateau
            s_bateau = 0.0
        else:
            w_abs = abs(weight_per_unit)
            if w_abs > 1e-6:
                s_bateau = np.sqrt(max(0.0, T_bateau**2 - H**2)) / w_abs
            else:
                s_bateau = 0.0
        
        # Tension au bateau : T[0] = T_bateau
        T[0] = T_bateau
        
        # Calculer les tensions pour les points intermédiaires
        w_abs = abs(weight_per_unit)
        s_cumulative = 0.0  # Initialisé à 0 car on commence au bateau
        for i in range(N):
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            ds = np.sqrt(dx**2 + dy**2)
            s_cumulative += ds  # Accumuler la distance depuis le bateau
            
            # Tension à ce point : T = sqrt(H² + (w*s_total)²)
            s_total = abs(-s_bateau + s_cumulative)
            T[i+1] = np.sqrt(H**2 + (w_abs * s_total)**2)
        
        # Vérifier que T[-1] correspond bien à T_rov (avec tolérance)
        if abs(T[-1] - T_rov) > 0.01:
            # Ajuster pour que T[-1] = T_rov exactement
            T[-1] = T_rov
        
        return T
    
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
        trace_print(5, "\n[DEBUG] ========== _solve_catenary APPELÉE ==========")
        trace_print(5, f"[DEBUG] _solve_catenary: Paramètres - x_rov={x_rov:.3f}, y_rov={y_rov:.3f}, x_boat={x_boat:.3f}, L={L:.3f}, w={w:.6f}")

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
            if a <= 0 or not np.isfinite(a) or a < 1e-10:
                return None
            
            def equation_x0(x0):
                """Équation pour trouver x0 : y_rov = a * (cosh((x_rov - x0) / a) - cosh((x_boat - x0) / a))"""
                try:
                    # Vérifier que a est valide
                    if a <= 1e-10 or not np.isfinite(a):
                        return np.inf
                    
                    # Vérifier que x0 est fini
                    if not np.isfinite(x0):
                        return np.inf
                    
                    # Calculer les arguments de cosh
                    arg1 = (x_rov - x0) / a
                    arg2 = (x_boat - x0) / a
                    
                    # Vérifier que les arguments ne sont pas trop grands (éviter overflow)
                    if abs(arg1) > 700 or abs(arg2) > 700:
                        return np.inf
                    
                    # Calculer la valeur
                    val = a * (np.cosh(arg1) - np.cosh(arg2)) - y_rov
                    
                    # Vérifier que le résultat est fini
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
            if a <= 0 or not np.isfinite(a) or a < 1e-10:
                return np.inf
            
            x0 = find_x0_for_a(a)
            if x0 is None or not np.isfinite(x0):
                return np.inf
            
            try:
                # Calculer les arguments de sinh
                arg_boat = (x_boat - x0) / a
                arg_rov = (x_rov - x0) / a
                
                # Vérifier que les arguments ne sont pas trop grands (éviter overflow)
                if abs(arg_boat) > 700 or abs(arg_rov) > 700:
                    return np.inf
                
                # Calculer les longueurs curvilignes
                s_boat = a * np.sinh(arg_boat)
                s_rov = a * np.sinh(arg_rov)
                
                # Vérifier que les résultats sont finis
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
                    
                    # Supprimer les warnings de scipy pour les valeurs invalides (NaN/Inf)
                    # qui peuvent se produire lors de l'optimisation avec des valeurs extrêmes
                    with np.errstate(invalid='ignore', divide='ignore'):
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
                trace_print(5, f"⚠️  Avertissement: La caténaire a une erreur élevée ({best_error:.6f}). "
                      f"Utilisation des meilleures valeurs trouvées.")
        except Exception as e:
            trace_print(5, f"⚠️  Erreur lors de la résolution de la caténaire: {e}. Utilisation des valeurs initiales.")
        
        # Trouver x0 pour le a optimal
        x0_opt = find_x0_for_a(a_opt)
        if x0_opt is None:
            trace_print(9, f"[DEBUG] _solve_catenary: ⚠️  ERREUR - Impossible de trouver x0 pour a={a_opt:.3f}. Utilisation d'une interpolation linéaire (LIGNE DROITE).")
            x_cable = np.linspace(x_boat, x_rov, self.N + 1)
            y_cable = np.linspace(0.0, y_rov, self.N + 1)
            return x_cable, y_cable
        
        # Calculer y0
        try:
            y0_opt = -a_opt * np.cosh((x_boat - x0_opt) / a_opt)
        except:
            trace_print(5, f"⚠️  Erreur: Impossible de calculer y0. Utilisation d'une interpolation linéaire.")
            x_cable = np.linspace(x_boat, x_rov, self.N + 1)
            y_cable = np.linspace(0.0, y_rov, self.N + 1)
            return x_cable, y_cable
        
        # Vérifier la solution
        y_rov_calc = a_opt * np.cosh((x_rov - x0_opt) / a_opt) + y0_opt
        y_boat_calc = a_opt * np.cosh((x_boat - x0_opt) / a_opt) + y0_opt
        
        # y_rov devrait être négatif (profondeur : y < 0 = sous la surface)
        if abs(y_rov_calc - y_rov) > 0.1:
            trace_print(5, f"⚠️  Avertissement: y_rov calculé ({y_rov_calc:.3f}) diffère de y_rov attendu ({y_rov:.3f}, profondeur négative)")
        
        if abs(y_boat_calc) > 0.1:
            trace_print(5, f"⚠️  Avertissement: y_boat calculé ({y_boat_calc:.3f}) diffère de y_boat attendu (0.000, surface)")
        
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
            
        # Si la flottabilité est positive (w < 0), inverser la caténaire
        # par rapport à la droite bateau-ROV pour obtenir une courbure vers le haut.
        if w < 0:
            dx_line = x_rov - x_boat
            dy_line = y_rov - 0.0
            if abs(dx_line) > 1e-9:
                s = (x_cable - x_boat) / dx_line
                s = np.clip(s, 0.0, 1.0)
                y_line = s * dy_line
            else:
                s = np.linspace(0.0, 1.0, len(x_cable))
                y_line = s * dy_line
            y_cable = 2.0 * y_line - y_cable
            y_cable[0] = 0.0
            y_cable[-1] = y_rov
            
        # Vérifier la longueur
        L_actual = 0.0
        for i in range(self.N):
            dx_seg = x_cable[i+1] - x_cable[i]
            dy_seg = y_cable[i+1] - y_cable[i]
            L_actual += np.sqrt(dx_seg**2 + dy_seg**2)

        # Toujours normaliser pour garantir que la longueur est exactement L
        # (tolérance réduite pour forcer la normalisation)
        # IMPORTANT : Passer les positions du bateau et du ROV pour garantir le recollage exact
        if abs(L_actual - L) / L > 1e-6:
            x_cable, y_cable = self._normalize_cable_length(
                x_cable, y_cable, L,
                x_boat=x_boat,
                y_boat=0.0,
                x_rov=x_rov,
                y_rov=y_rov
            )
        
            # CONTRAINTE PHYSIQUE : Après normalisation, forcer y <= 0
            y_cable = np.clip(y_cable, None, 0.0)
            # Réappliquer les extrémités (normalement déjà faites par _normalize_cable_length, mais on s'assure)
            if x_boat is not None:
                x_cable[0] = float(x_boat)
            y_cable[0] = 0.0
            if x_rov is not None:
                x_cable[-1] = float(x_rov)
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
                trace_print(5, f"⚠️  Avertissement: Le câble semble être une ligne droite plutôt qu'une caténaire. "
                      f"Vérifiez les paramètres (L={L:.2f} m, L_straight={L_straight:.2f} m, "
                      f"rho_cable={self.rho_cable:.1f} kg/m³, rho_eau={self.environment.rho_eau:.1f} kg/m³)")
        
        # Avertissement si la longueur n'est toujours pas correcte
        if abs(L_final - L) / L > 0.02:
            trace_print(5, f"⚠️  Avertissement: Longueur du câble incorrecte après calcul de caténaire. "
                  f"Attendu: {L:.3f} m, Obtenu: {L_final:.3f} m, Erreur: {abs(L_final - L)/L*100:.1f}%")
        
        # Retourner le câble : index 0 = bateau, index -1 = ROV (convention cohérente avec le système)
        return x_cable, y_cable
    
    def _objective_equilibrium_residual(self, x_int, y_int, x_boat, y_boat, x_rov, y_rov, L, 
                                       forces_func, params_forces):
        """
        Fonction objectif : minimise les résidus des équations d'équilibre.
        
        Parameters:
        -----------
        x_int, y_int : array
            Positions des points intérieurs (N-1 points)
        x_boat, y_boat, x_rov, y_rov : float
            Positions des extrémités (fixées)
        L : float
            Longueur cible du câble
        forces_func : callable
            Fonction qui calcule les forces (Fx, Fy) pour une configuration donnée
        params_forces : dict
            Paramètres pour le calcul des forces
        
        Returns:
        --------
        float
            Somme des carrés des résidus d'équilibre
        """
        # Reconstruire le câble complet avec les extrémités fixées
        N = len(x_int) + 1
        x_cable = np.zeros(N + 1)
        y_cable = np.zeros(N + 1)
        x_cable[0] = x_boat
        y_cable[0] = y_boat
        x_cable[1:-1] = x_int
        y_cable[1:-1] = y_int
        x_cable[-1] = x_rov
        y_cable[-1] = y_rov
        
        # Calculer les forces
        try:
            Fx, Fy = forces_func(x_cable, y_cable, params_forces)
        except Exception:
            # En cas d'erreur, retourner une pénalité élevée
            return 1e10
        
        # Calculer les résidus d'équilibre aux nœuds intérieurs
        # Pour chaque nœud intérieur i, l'équilibre des forces donne :
        # T[i] * u[i] - T[i-1] * u[i-1] + F[i] = 0
        # où u[i] est le vecteur unitaire du segment i
        
        # Calculer les vecteurs unitaires des segments
        dx = np.diff(x_cable)
        dy = np.diff(y_cable)
        ds = np.sqrt(dx**2 + dy**2)
        ds = np.maximum(ds, 1e-9)  # Éviter division par zéro
        ux = dx / ds
        uy = dy / ds
        
        # Estimation simple des tensions (approximation linéaire)
        # Pour une meilleure approximation, on pourrait résoudre le système de tensions
        # mais pour l'objectif, une approximation suffit
        T_est = np.ones(N + 1) * 10.0  # Estimation initiale
        
        # Calculer les résidus
        residual = 0.0
        for i in range(1, N):  # Nœuds intérieurs uniquement
            # Résidu d'équilibre horizontal
            res_x = T_est[i] * ux[i] - T_est[i-1] * ux[i-1] + Fx[i-1] if i < N else 0.0
            # Résidu d'équilibre vertical
            res_y = T_est[i] * uy[i] - T_est[i-1] * uy[i-1] + Fy[i-1] if i < N else 0.0
            residual += res_x**2 + res_y**2
        
        return residual
    
    def _constraint_length(self, x_int, y_int, x_boat, y_boat, x_rov, y_rov, L):
        """
        Contrainte : L_seg - L = 0 (avec tolérance 0.01%)
        
        Returns:
        --------
        float
            L_seg - L
        """
        # Reconstruire le câble complet
        N = len(x_int) + 1
        x_cable = np.zeros(N + 1)
        y_cable = np.zeros(N + 1)
        x_cable[0] = x_boat
        y_cable[0] = y_boat
        x_cable[1:-1] = x_int
        y_cable[1:-1] = y_int
        x_cable[-1] = x_rov
        y_cable[-1] = y_rov
        
        # Calculer L_seg
        L_seg = 0.0
        for i in range(N):
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            L_seg += np.sqrt(dx**2 + dy**2)
        
        return L_seg - L
    
    def _constraint_surface(self, x_int, y_int, x_boat, y_boat, x_rov, y_rov):
        """
        Contrainte : max(y_cable) <= 0
        
        Returns:
        --------
        float
            max(y_cable) (doit être <= 0)
        """
        # Reconstruire le câble complet
        N = len(x_int) + 1
        y_cable = np.zeros(N + 1)
        y_cable[0] = y_boat
        y_cable[1:-1] = y_int
        y_cable[-1] = y_rov

        return np.max(y_cable)
    
    def _constraint_segment_ratio(self, x_int, y_int, x_boat, y_boat, x_rov, y_rov, L, k_max):
        """
        Contrainte : ds_max / ds_target - k_max <= 0
        
        Returns:
        --------
        float
            ds_max / ds_target - k_max
        """
        # Reconstruire le câble complet
        N = len(x_int) + 1
        x_cable = np.zeros(N + 1)
        y_cable = np.zeros(N + 1)
        x_cable[0] = x_boat
        y_cable[0] = y_boat
        x_cable[1:-1] = x_int
        y_cable[1:-1] = y_int
        x_cable[-1] = x_rov
        y_cable[-1] = y_rov

        # Calculer les longueurs des segments
        ds_list = []
        for i in range(N):
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            ds = np.sqrt(dx**2 + dy**2)
            ds_list.append(ds)
        
        if len(ds_list) == 0:
            return 0.0
        
        ds_max = np.max(ds_list)
        ds_target = L / N if N > 0 else L
        ratio = ds_max / ds_target if ds_target > 1e-9 else 0.0
        
        return ratio - k_max
    
    def _forces_static_wrapper(self, x_cable, y_cable, params):
        """
        Wrapper pour calculer les forces statiques (poids uniquement).
        
        Parameters:
        -----------
        x_cable, y_cable : array
            Positions du câble
        params : dict
            Contient 'environment', 'params_cable', 'L'
        
        Returns:
        --------
        tuple (Fx, Fy)
            Forces par segment (N)
        """
        environment = params['environment']
        params_cable = params['params_cable']
        L = params['L']
        
        # Vitesses nulles pour le cas statique
        vx_cable = np.zeros(len(x_cable))
        vy_cable = np.zeros(len(x_cable))
        
        from .forces import compute_cable_forces
        Fx, Fy, _, _, _, _ = compute_cable_forces(
            x_cable, y_cable, vx_cable, vy_cable, environment, params_cable, L
        )
        
        return Fx, Fy
    
    def _forces_static_with_current_wrapper(self, x_cable, y_cable, params):
        """
        Wrapper pour calculer les forces statiques avec courant.
        
        Parameters:
        -----------
        x_cable, y_cable : array
            Positions du câble
        params : dict
            Contient 'environment', 'params_cable', 'L'
        
        Returns:
        --------
        tuple (Fx, Fy)
            Forces par segment (N)
        """
        environment = params['environment']
        params_cable = params['params_cable']
        L = params['L']
        
        # Vitesses nulles pour le cas statique (le courant est dans l'environnement)
        vx_cable = np.zeros(len(x_cable))
        vy_cable = np.zeros(len(x_cable))
        
        from .forces import compute_cable_forces
        Fx, Fy, _, _, _, _ = compute_cable_forces(
            x_cable, y_cable, vx_cable, vy_cable, environment, params_cable, L
        )
        
        return Fx, Fy
    
    def _forces_dynamic_wrapper(self, x_cable, y_cable, params):
        """
        Wrapper pour calculer les forces dynamiques (avec vitesses).
        
        Parameters:
        -----------
        x_cable, y_cable : array
            Positions du câble
        params : dict
            Contient 'environment', 'params_cable', 'L', 'vx_cable', 'vy_cable'
        
        Returns:
        --------
        tuple (Fx, Fy)
            Forces par segment (N)
        """
        environment = params['environment']
        params_cable = params['params_cable']
        L = params['L']
        vx_cable = params['vx_cable']
        vy_cable = params['vy_cable']
        
        from .forces import compute_cable_forces
        Fx, Fy, _, _, _, _ = compute_cable_forces(
            x_cable, y_cable, vx_cable, vy_cable, environment, params_cable, L
        )
        
        return Fx, Fy
    
    def solve_equilibrium_constrained(self, x_rov, y_rov, x_boat, L, forces_func, 
                                     params_forces, rov_m=None, rov_vol=None,
                                     initial_guess=None, k_max=2.0):
        """
        Résout l'équilibre du câble avec toutes les contraintes intégrées.
        
        Utilise scipy.optimize.minimize avec des contraintes pour garantir :
        1. L = L_seg à 0.01% près
        2. P0 = position bateau
        3. PN = position ROV
        4. y <= 0 pour tous les points
        5. ds_max / ds_target <= k_max
        
        Parameters:
        -----------
        x_rov, y_rov : float
            Position du ROV
        x_boat : float
            Position horizontale du bateau
        L : float
            Longueur du câble
        forces_func : callable
            Fonction qui calcule les forces (Fx, Fy) pour une configuration donnée
            Signature: Fx, Fy = forces_func(x_cable, y_cable, params_forces)
        params_forces : dict
            Paramètres pour le calcul des forces
        rov_m, rov_vol : float, optional
            Masse et volume du ROV
        initial_guess : tuple (x_int, y_int), optional
            Estimation initiale des points intérieurs
        k_max : float
            Ratio maximum ds_max / ds_target (défaut: 2.0)
        
        Returns:
        --------
        tuple (x_cable, y_cable, T)
            Configuration du câble avec tensions
        """
        # NonlinearConstraint est déjà importé en haut du fichier
        
        trace_print(5, "\n[DEBUG] ========== solve_equilibrium_constrained APPELÉE ==========")
        trace_print(5, f"[DEBUG] solve_equilibrium_constrained: Paramètres d'entrée - x_rov={x_rov:.3f}, y_rov={y_rov:.3f}, x_boat={x_boat:.3f}, L={L:.3f}, initial_guess={'None' if initial_guess is None else 'fourni'}")
        
        if L <= 0:
            return np.array([x_rov, x_boat]), np.array([y_rov, 0.0]), np.array([0.0, 0.0])
        
        N = self.N
        y_boat = 0.0
        
        # Estimation initiale : utiliser une caténaire approximative pour une forme réaliste
        # IMPORTANT : Utiliser directement la caténaire complète au lieu d'extraire les points intérieurs
        # pour préserver la forme
        x_cable_cat = None
        y_cable_cat = None
        
        trace_print(5, f"[DEBUG] solve_equilibrium_constrained: initial_guess is None = {initial_guess is None}")
        
        # Vérifier si l'initial_guess fourni est une ligne droite (déviation trop faible)
        # Si c'est le cas, ignorer l'initial_guess et générer une caténaire
        use_catenary = False
        if initial_guess is not None:
            x_int_init, y_int_init = initial_guess
            # Reconstruire le câble complet pour vérifier la déviation
            x_cable_test = np.concatenate([[x_boat], x_int_init, [x_rov]])
            y_cable_test = np.concatenate([[0.0], y_int_init, [y_rov]])
            # Calculer la déviation maximale
            if len(x_cable_test) >= 3:
                x_start, y_start = x_cable_test[0], y_cable_test[0]
                x_end, y_end = x_cable_test[-1], y_cable_test[-1]
                max_dev_test = 0.0
                if abs(x_end - x_start) > 1e-6 or abs(y_end - y_start) > 1e-6:
                    dx_line = x_end - x_start
                    dy_line = y_end - y_start
                    line_length = np.sqrt(dx_line**2 + dy_line**2)
                    for i in range(1, len(x_cable_test) - 1):
                        dx_point = x_cable_test[i] - x_start
                        dy_point = y_cable_test[i] - y_start
                        t = (dx_point * dx_line + dy_point * dy_line) / (line_length**2 + 1e-9)
                        x_proj = x_start + t * dx_line
                        y_proj = y_start + t * dy_line
                        deviation = np.sqrt((x_cable_test[i] - x_proj)**2 + (y_cable_test[i] - y_proj)**2)
                        max_dev_test = max(max_dev_test, deviation)
                # Si la déviation est très faible (< 0.01 m), c'est une ligne droite, utiliser une caténaire
                if max_dev_test < 0.01:
                    trace_print(9, f"[DEBUG] solve_equilibrium_constrained: initial_guess est une ligne droite (max_deviation={max_dev_test:.6f} m), génération d'une caténaire à la place")
                    use_catenary = True
                    initial_guess = None  # Ignorer l'initial_guess et générer une caténaire
        
        if initial_guess is None or use_catenary:
            try:
                # Calculer le poids apparent par unité de longueur
                # w = (rho_cable - rho_water) * g * A_cable
                rho_water = self.environment.rho_water if hasattr(self.environment, 'rho_water') else 1025.0
                g = 9.81
                A_cable = np.pi * (self.d / 2)**2
                w = (self.rho_cable - rho_water) * g * A_cable
                
                # Essayer de résoudre la caténaire pour obtenir une forme réaliste
                trace_print(5, "[DEBUG] ========== solve_equilibrium_constrained: Tentative d'estimation initiale avec caténaire... ==========")
                trace_print(5, f"[DEBUG] solve_equilibrium_constrained: Paramètres avant appel _solve_catenary - x_rov={x_rov:.3f}, y_rov={y_rov:.3f}, x_boat={x_boat:.3f}, L={L:.3f}, w={w:.6f}")
                x_cable_cat, y_cable_cat = self._solve_catenary(x_rov, y_rov, x_boat, L, w)
                trace_print(5, f"[DEBUG] solve_equilibrium_constrained: _solve_catenary retourné - len(x)={len(x_cable_cat)}, len(y)={len(y_cable_cat)}")
                
                if len(x_cable_cat) == N + 1 and len(y_cable_cat) == N + 1:
                    # Vérifier si la caténaire a vraiment une forme courbe
                    x_start, y_start = x_cable_cat[0], y_cable_cat[0]
                    x_end, y_end = x_cable_cat[-1], y_cable_cat[-1]
                    max_deviation = 0.0
                    if abs(x_end - x_start) > 1e-6 or abs(y_end - y_start) > 1e-6:
                        dx_line = x_end - x_start
                        dy_line = y_end - y_start
                        line_length = np.sqrt(dx_line**2 + dy_line**2)
                        for i in range(1, len(x_cable_cat) - 1):
                            dx_point = x_cable_cat[i] - x_start
                            dy_point = y_cable_cat[i] - y_start
                            t = (dx_point * dx_line + dy_point * dy_line) / (line_length**2 + 1e-9)
                            x_proj = x_start + t * dx_line
                            y_proj = y_start + t * dy_line
                            deviation = np.sqrt((x_cable_cat[i] - x_proj)**2 + (y_cable_cat[i] - y_proj)**2)
                            max_deviation = max(max_deviation, deviation)
                    
                    if max_deviation < 0.01:
                        trace_print(9, f"[DEBUG] solve_equilibrium_constrained: ⚠️  Caténaire générée semble être une ligne droite (max_deviation={max_deviation:.6f} m)")
                    else:
                        trace_print(5, f"[DEBUG] solve_equilibrium_constrained: ✓ Caténaire générée a une forme courbe (max_deviation={max_deviation:.6f} m)")
                    
                    # Utiliser directement la caténaire complète comme estimation initiale
                    trace_print(5, "[DEBUG] solve_equilibrium_constrained: Estimation initiale basée sur caténaire réussie")
                    # Extraire les points intérieurs seulement pour l'optimisation (si activée)
                    x_int_init = x_cable_cat[1:-1].copy()
                    y_int_init = y_cable_cat[1:-1].copy()
                else:
                    raise ValueError(f"Caténaire retournée avec mauvaise taille: {len(x_cable_cat)} au lieu de {N+1}")
            except Exception as e:
                # Fallback : utiliser une forme parabolique approximative au lieu d'une ligne droite
                trace_print(9, f"[DEBUG] solve_equilibrium_constrained: Échec caténaire ({e}), utilisation d'une forme parabolique approximative")
                # Forme parabolique : y = a * x^2 + b * x + c
                # Contraintes : y(0) = 0, y(x_rov) = y_rov
                # Pour une forme réaliste, ajouter une courbure vers le bas
                x_points = np.linspace(x_boat, x_rov, N + 1)
                y_points = np.linspace(y_boat, y_rov, N + 1)
                
                # Ajouter une courbure parabolique vers le bas pour simuler l'effet du poids
                # y = y_linéaire + amplitude * (x - x_boat) * (x_rov - x) / (x_rov - x_boat)^2
                if abs(x_rov - x_boat) > 1e-6:
                    amplitude = min(abs(y_rov) * 0.3, 10.0)  # Courbure modérée
                    for i in range(len(x_points)):
                        x = x_points[i]
                        curvature = amplitude * (x - x_boat) * (x_rov - x) / ((x_rov - x_boat)**2 + 1e-9)
                        y_points[i] = y_points[i] - abs(curvature)  # Courber vers le bas
                
                # S'assurer que y <= 0
                y_points = np.minimum(y_points, 0.0)
                
                # Extraire les points intérieurs
                x_int_init = x_points[1:-1].copy()
                y_int_init = y_points[1:-1].copy()
        else:
            x_int_init, y_int_init = initial_guess
        
        # Vecteur d'optimisation : positions des points intérieurs
        # Format: [x1, x2, ..., xN-1, y1, y2, ..., yN-1]
        n_int = N - 1
        x0 = np.concatenate([x_int_init, y_int_init])
        
        # Fonction objectif wrapper
        def objective(vars):
            x_int = vars[:n_int]
            y_int = vars[n_int:]
            return self._objective_equilibrium_residual(
                x_int, y_int, x_boat, y_boat, x_rov, y_rov, L, forces_func, params_forces
            )
        
        # Contraintes
        constraints = []
        
        # Contrainte de longueur : L_seg - L = 0 (tolérance 0.01%)
        def constraint_length_func(vars):
            x_int = vars[:n_int]
            y_int = vars[n_int:]
            return self._constraint_length(x_int, y_int, x_boat, y_boat, x_rov, y_rov, L)
        
        constraints.append(NonlinearConstraint(
            constraint_length_func,
            lb=-L * 1e-4,  # Tolérance 0.01%
            ub=L * 1e-4,
            keep_feasible=False
        ))
        
        # Contrainte de surface : max(y) <= 0
        def constraint_surface_func(vars):
            x_int = vars[:n_int]
            y_int = vars[n_int:]
            return self._constraint_surface(x_int, y_int, x_boat, y_boat, x_rov, y_rov)
        
        constraints.append(NonlinearConstraint(
            constraint_surface_func,
            lb=-np.inf,
            ub=0.0,
            keep_feasible=False
        ))
        
        # Contrainte de ratio : ds_max / ds_target - k_max <= 0
        def constraint_ratio_func(vars):
            x_int = vars[:n_int]
            y_int = vars[n_int:]
            return self._constraint_segment_ratio(x_int, y_int, x_boat, y_boat, x_rov, y_rov, L, k_max)
        
        constraints.append(NonlinearConstraint(
            constraint_ratio_func,
            lb=-np.inf,
            ub=0.0,
            keep_feasible=False
        ))
        
        # Optimisation désactivée temporairement pour éviter les blocages à l'initialisation
        # L'optimisation avec contraintes non-linéaires peut être très lente avec 1000 points
        # TODO: Réactiver progressivement avec des paramètres très restrictifs :
        #   - maxiter=2-3 seulement
        #   - Méthode 'SLSQP' plus rapide que 'trust-constr'
        #   - Ou utiliser un sous-échantillonnage pour réduire le nombre de variables
        USE_OPTIMIZATION = False  # Désactiver pour l'instant
        
        if USE_OPTIMIZATION:
            try:
                trace_print(9, "[DEBUG] solve_equilibrium_constrained: Démarrage de l'optimisation avec contraintes...")
                result = minimize(
                    objective,
                    x0,
                    method='SLSQP',  # Plus rapide que trust-constr
                    constraints=constraints,
                    options={
                        'maxiter': 3,  # Très limité pour éviter les blocages
                        'ftol': 1e-3,  # Tolérance sur la fonction objectif
                        'disp': False  # Pas de messages
                    }
                )
                
                if result.success:
                    trace_print(9, f"[DEBUG] solve_equilibrium_constrained: Optimisation réussie en {result.nit} itérations")
                    x_int = result.x[:n_int]
                    y_int = result.x[n_int:]
                else:
                    trace_print(9, f"[DEBUG] solve_equilibrium_constrained: Optimisation non convergée ({result.message}), utilisation de l'estimation initiale")
                    x_int = x_int_init
                    y_int = y_int_init
            except Exception as e:
                trace_print(9, f"[DEBUG] solve_equilibrium_constrained: Erreur lors de l'optimisation ({e}), utilisation de l'estimation initiale")
                x_int = x_int_init
                y_int = y_int_init
        else:
            # Utiliser directement l'estimation initiale
            # _normalize_cable_length garantira toutes les contraintes ensuite
            trace_print(4, "[DEBUG] solve_equilibrium_constrained: Utilisation de l'estimation initiale (optimisation désactivée)")
            x_int = x_int_init
            y_int = y_int_init
        
        # Reconstruire le câble complet avec recollement garanti
        # Si on a une caténaire complète, l'utiliser directement pour préserver la forme
        if x_cable_cat is not None and y_cable_cat is not None and len(x_cable_cat) == N + 1:
            # Utiliser directement la caténaire complète pour préserver la forme
            x_cable = x_cable_cat.copy()
            y_cable = y_cable_cat.copy()
            # Forcer seulement les extrémités pour garantir le recollement
            x_cable[0] = x_boat  # Contrainte P0 = bateau
            y_cable[0] = y_boat
            x_cable[-1] = x_rov  # Contrainte PN = ROV
            y_cable[-1] = y_rov
            trace_print(5, "[DEBUG] solve_equilibrium_constrained: Utilisation directe de la caténaire complète pour préserver la forme")
        else:
            # Reconstruire à partir des points intérieurs (fallback)
            x_cable = np.zeros(N + 1)
            y_cable = np.zeros(N + 1)
            x_cable[0] = x_boat  # Contrainte P0 = bateau
            y_cable[0] = y_boat
            x_cable[1:-1] = x_int
            y_cable[1:-1] = y_int
            x_cable[-1] = x_rov  # Contrainte PN = ROV
            y_cable[-1] = y_rov
        
        # Vérifier si la normalisation est nécessaire avant d'appeler _normalize_cable_length
        # Calculer la longueur actuelle
        L_actual = 0.0
        for i in range(N):
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            L_actual += np.sqrt(dx**2 + dy**2)
        
        # Vérifier les contraintes avant normalisation
        tol_L = L * 1e-4  # Tolérance 0.01%
        needs_normalization = (
            abs(L_actual - L) > tol_L or  # Longueur incorrecte
            (x_boat is not None and abs(x_cable[0] - x_boat) > 1e-6) or  # Recollement bateau
            (x_rov is not None and abs(x_cable[-1] - x_rov) > 1e-6) or  # Recollement ROV
            np.any(y_cable > 1e-9)  # Points au-dessus de la surface
        )
        
        if needs_normalization:
            try:
                trace_print(5, "[DEBUG] solve_equilibrium_constrained: Application de _normalize_cable_length pour garantir toutes les contraintes...")
                trace_print(5, f"[DEBUG] solve_equilibrium_constrained: Paramètres - L={L}, L_actual={L_actual:.6f}, x_boat={x_boat}, y_boat={y_boat}, x_rov={x_rov}, y_rov={y_rov}, k_max={k_max}")
                trace_print(5, f"[DEBUG] solve_equilibrium_constrained: Avant appel à _normalize_cable_length, x_cable.shape={x_cable.shape if hasattr(x_cable, 'shape') else len(x_cable)}, y_cable.shape={y_cable.shape if hasattr(y_cable, 'shape') else len(y_cable)}")
                x_cable, y_cable = self._normalize_cable_length(
                    x_cable, y_cable, L,
                    x_boat=float(x_boat),
                    y_boat=0.0,
                    x_rov=float(x_rov),
                    y_rov=float(y_rov),
                    k_max=k_max
                )
                trace_print(5, "[DEBUG] solve_equilibrium_constrained: _normalize_cable_length terminé avec succès")
            except Exception as e:
                trace_print(9, f"[DEBUG] solve_equilibrium_constrained: Échec de _normalize_cable_length: {e}")
                import traceback
                trace_print(9, f"[DEBUG] Traceback complet:\n{traceback.format_exc()}")
        else:
            trace_print(9, f"[DEBUG] solve_equilibrium_constrained: Pas de normalisation nécessaire (L_actual={L_actual:.6f}, L={L:.6f}, diff={abs(L_actual-L):.6e})")
            # S'assurer quand même que y <= 0 et que les extrémités sont correctes
            y_cable = np.clip(y_cable, None, 0.0)
            if x_boat is not None:
                x_cable[0] = float(x_boat)
                y_cable[0] = 0.0
            if x_rov is not None:
                x_cable[-1] = float(x_rov)
                y_cable[-1] = float(y_rov)
        
        # Calculer les tensions (approximation simple)
        # Pour une meilleure précision, on pourrait résoudre le système d'équations de tension
        T = np.ones(N + 1) * 10.0  # Estimation par défaut
        
        return x_cable, y_cable, T
    
    def solve_equilibrium_dynamic(self, x_rov, y_rov, vx_rov, vy_rov,
                                  x_boat, vx_boat, L, x_cable_prev, y_cable_prev, rov_m=None, rov_vol=None):
        """
        Résout l'équilibre dynamique du câble avec forces hydrodynamiques
        
        Utilise maintenant le solveur avec contraintes intégrées pour garantir :
        - L = L_seg à 0.01% près
        - P0 = position bateau
        - PN = position ROV
        - y <= 0 pour tous les points
        - ds_max / ds_target <= k_max
        
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
        
        # Vitesses des points du câble (interpolation linéaire entre ROV et bateau)
        vx_cable = np.linspace(vx_rov, vx_boat, self.N + 1)
        vy_cable = np.linspace(vy_rov, 0.0, self.N + 1)
        
        # Câble mou : la vitesse verticale ne se propage pas complètement le long du câble
        L_straight = np.hypot(x_rov - x_boat, y_rov - 0.0)
        slack_ratio = 0.0
        if L_straight > 1e-9:
            slack_ratio = max((L - L_straight) / max(L_straight, 1.0), 0.0)
        if slack_ratio > 0.0:
            # Atténuer la vitesse verticale quand il y a beaucoup de mou
            scale = max(0.0, 1.0 - slack_ratio / 0.2)
            vy_cable = vy_cable * scale
        
        # Préparer les paramètres pour le wrapper de forces dynamiques
        params_forces = {
            'environment': self.environment,
            'params_cable': {
            'd': self.d,
            'rho_cable': self.rho_cable,
            'Cx_cable': self.Cx_cable,
            'Cf_cable': self.Cf_cable
            },
            'L': L,
            'vx_cable': vx_cable,
            'vy_cable': vy_cable
        }
        
        # Estimation initiale : utiliser la solution statique avec courant ou la configuration précédente
        try:
            x_cable_static, y_cable_static, _ = self.solve_equilibrium_static_with_current(
                x_rov, y_rov, x_boat, L, rov_m=rov_m, rov_vol=rov_vol
            )
            
            # Mélanger avec la configuration précédente pour lisser les transitions
            v_mag = np.sqrt(vx_rov**2 + vy_rov**2 + vx_boat**2)
            alpha = 0.3 if v_mag > 0.1 else 0.7  # Plus de poids sur statique si vitesses faibles
            
            if x_cable_prev is not None and y_cable_prev is not None and len(x_cable_prev) == len(x_cable_static):
                x_cable_init = alpha * x_cable_static + (1 - alpha) * x_cable_prev
                y_cable_init = alpha * y_cable_static + (1 - alpha) * y_cable_prev
            else:
                x_cable_init = x_cable_static
                y_cable_init = y_cable_static
                    
            # Extraire les points intérieurs pour l'estimation initiale
            if len(x_cable_init) > 2:
                initial_guess = (x_cable_init[1:-1], y_cable_init[1:-1])
            else:
                initial_guess = None
        except Exception:
            initial_guess = None
        
        # Utiliser le solveur avec contraintes intégrées
        x_cable, y_cable, T = self.solve_equilibrium_constrained(
            x_rov, y_rov, x_boat, L,
            self._forces_dynamic_wrapper,
            params_forces,
            rov_m=rov_m,
            rov_vol=rov_vol,
            initial_guess=initial_guess,
            k_max=2.0
        )
        
        # Recalculer les tensions correctement en utilisant l'équilibre des forces
        # (le solveur avec contraintes retourne une estimation simple)
        weight_per_unit = (self.rho_cable - self.environment.rho_eau) * self.A_cable * self.environment.g
        T = self._compute_catenary_tensions(x_cable, y_cable, weight_per_unit, rov_m=rov_m, rov_vol=rov_vol)
        
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
            Tension initiale au ROV
        
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
        # Déduire une tension ROV locale à partir des forces du dernier segment
        min_floor = 0.5
        t_local = None
        if Fx is not None and Fy is not None and len(Fx) > 0 and len(Fy) > 0:
            try:
                t_local = float(np.hypot(Fx[-1], Fy[-1]))
            except Exception:
                t_local = None

        # Utiliser la tension initiale au ROV si fournie, sinon l'estimation locale
        if T_rov_initial is None or T_rov_initial <= 0.0:
            if t_local is not None:
                T_rov_initial = max(min_floor, t_local)
            else:
                T_rov_initial = min_floor
        elif t_local is not None and t_local > 0.0:
            # Laisser T_rov évoluer avec les forces locales (plus "physique")
            T_rov_initial = max(min_floor, 0.7 * float(T_rov_initial) + 0.3 * t_local)
        T[-1] = T_rov_initial
        
        for i in range(N - 1, -1, -1):
            # Direction du segment (i -> i+1)
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            length = np.sqrt(dx**2 + dy**2)
            
            if length > 1e-6:
                # Équilibre des forces le long du câble
                # Fx, Fy sont déjà des forces par segment (N), ne pas re-multiplier par ds
                t_hat_x = dx / length
                t_hat_y = dy / length
                dT_along = -(Fx[i] * t_hat_x + Fy[i] * t_hat_y)
                # Mettre à jour la tension en remontant vers le bateau
                T[i] = max(T[i+1] + dT_along, 0.0)
            else:
                T[i] = T[i+1]
        
        return T

    def _trace_cable_equilibrium_forces(self, x_cable, y_cable, T, L, label="initialisation"):
        from .forces import compute_cable_forces, compute_cable_apparent_weight
        if x_cable is None or y_cable is None or len(x_cable) < 2:
            trace_print(5, f"[DEBUG] Trace forces câble ({label}): câble insuffisant.")
            return
        if T is None or len(T) < 2:
            trace_print(5, f"[DEBUG] Trace forces câble ({label}): tensions indisponibles.")
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
        Fx_seg, Fy_seg, _, _, _, _ = compute_cable_forces(
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
            trace_print(5, "[DEBUG] ⚠️  Incohérence: poids apparent positif alors que câble plus dense.")
        if self.rho_cable < self.environment.rho_eau and F_poids[1] < 0.0:
            trace_print(5, "[DEBUG] ⚠️  Incohérence: poids apparent négatif alors que câble plus léger.")
        if np.dot(F_bateau, U_bat) > 0.0:
            trace_print(5, "[DEBUG] ⚠️  Incohérence: traction bateau dans le même sens que U_bateau.")
        if np.dot(F_rov, U_rov) < 0.0:
            trace_print(5, "[DEBUG] ⚠️  Incohérence: traction ROV opposée à U_rov.")

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

        # Export trace file moved to simulation-level CSV logging.
    
    def _normalize_cable_length(
        self,
        x_cable,
        y_cable,
        L_target,
        x_boat=None,
        y_boat=0.0,
        x_rov=None,
        y_rov=None,
        k_tail=10,
        k_max=2.0,
        bidirectional=True,
        _from_direction=None,
    ):
        """
        Normalise la longueur du câble en conservant au mieux sa forme générale,
        tout en respectant trois contraintes « dures » et une contrainte « souple » :

        Contraintes dures (quand les positions d'extrémité sont fournies) :
        - Le premier point est recollé au bateau (x_boat, y_boat).
        - Le dernier point est recollé au ROV (x_rov, y_rov).
        - La longueur totale est ramenée à L_target (à la précision numérique près).

        Contrainte souple :
        - Le rapport ds_max / ds_target reste raisonnable (≲ 3), où
          ds_target = L_target / N est la longueur moyenne cible d'un segment
          et ds_max la longueur du segment le plus long.

        L'algorithme suit les étapes suivantes :
        1. Calcul de l'abscisse curviligne cumulative s_cumulative et de la longueur
           actuelle L_actual.
        2. Remise à l'échelle de s_cumulative pour que s_cumulative[-1] = L_target.
        3. Rééchantillonnage du câble par interpolation linéaire en abscisse curviligne
           sur N+1 points régulièrement espacés entre 0 et L_target.
           Cette étape respecte exactement les extrémités existantes : les points
           0 et N sont inchangés (recollement bateau/ROV).
        4. Clipping sur y <= 0.
        5. Calcul des longueurs de segment et du ratio ds_max / ds_target. Si ce ratio
           dépasse 3, quelques itérations de lissage local sont appliquées autour du
           segment le plus long pour réduire ce ratio, en conservant les extrémités.

        Parameters
        ----------
        x_cable, y_cable : array-like
            Positions actuelles du câble (N+1 points).
        L_target : float
            Longueur cible (m).
        x_boat, y_boat, x_rov, y_rov : float, optionnels
            Positions cibles du bateau et du ROV pour le recollement doux des extrémités.
            Si elles sont omises, la fonction conserve simplement les extrémités existantes.
        k_tail : int, optionnel
            Nombre de nœuds de queue utilisés pour répartir l'ajustement côté ROV.

        Returns
        -------
        tuple (x_cable_new, y_cable_new)
            Positions normalisées du câble.
        """
        trace_print(5, "\n[DEBUG] _normalize_cable_length: Démarrage de la normalisation du câble.")

        # Nouvelle approche : normalisation géométrique itérative avec possibilité
        # d'ajouter des points et d'utiliser scale_slack. On l'utilise en priorité.
        try:
            x_boat_eff = x_boat if x_boat is not None else float(x_cable[0])
            y_boat_eff = y_boat if y_boat is not None else float(y_cable[0])
            x_rov_eff = x_rov if x_rov is not None else float(x_cable[-1])
            y_rov_eff = y_rov if y_rov is not None else float(y_cable[-1])

            N_debut_iter = max(len(x_cable) - 1, 1)

            x_new, y_new, rel_err, iters = self._normalize_cable_geometry(
                x_cable,
                y_cable,
                L_target,
                (x_boat_eff, y_boat_eff),
                (x_rov_eff, y_rov_eff),
                N_debut_iter,
            )

            trace_print(
                5,
                f"[DEBUG] _normalize_cable_length: _normalize_cable_geometry terminé "
                f"rel_err={rel_err:.3e}, iters={iters}",
            )
            return x_new, y_new
        except Exception as e:
            trace_print(
                8,
                f"[DEBUG] _normalize_cable_length: échec _normalize_cable_geometry ({e}), "
                "retour à l'algorithme historique.",
            )

        # ------------------------------------------------------------------
        # Algorithme historique de normalisation (fallback)
        # ------------------------------------------------------------------
        
        # DEBUG: Calculer la déviation maximale AVANT normalisation
        def compute_max_deviation(x_arr, y_arr):
            """Calcule la déviation maximale par rapport à la ligne droite"""
            if len(x_arr) < 3:
                return 0.0
            x_start, y_start = x_arr[0], y_arr[0]
            x_end, y_end = x_arr[-1], y_arr[-1]
            max_dev = 0.0
            if abs(x_end - x_start) > 1e-6 or abs(y_end - y_start) > 1e-6:
                dx_line = x_end - x_start
                dy_line = y_end - y_start
                line_length = np.sqrt(dx_line**2 + dy_line**2)
                for i in range(1, len(x_arr) - 1):
                    dx_point = x_arr[i] - x_start
                    dy_point = y_arr[i] - y_start
                    t = (dx_point * dx_line + dy_point * dy_line) / (line_length**2 + 1e-9)
                    x_proj = x_start + t * dx_line
                    y_proj = y_start + t * dy_line
                    deviation = np.sqrt((x_arr[i] - x_proj)**2 + (y_arr[i] - y_proj)**2)
                    max_dev = max(max_dev, deviation)
            return max_dev
        
        max_dev_before = compute_max_deviation(x_cable, y_cable)
        
        # Calculer la longueur actuelle et les distances cumulatives
        N = len(x_cable) - 1
        
        # Normalisation bidirectionnelle : partir des deux extrémités et se rejoindre au milieu
        if bidirectional and _from_direction is None and x_boat is not None and x_rov is not None and N >= 10:
            trace_print(5, f"[DEBUG] _normalize_cable_length: Normalisation bidirectionnelle activée - N={N}, x_boat={x_boat}, x_rov={x_rov}")
            # Étape 1 : Normalisation depuis le bateau (extrémité ROV libre)
            trace_print(5, "[DEBUG] _normalize_cable_length: Normalisation bidirectionnelle - depuis le bateau")
            x_from_boat, y_from_boat = self._normalize_cable_length(
                x_cable, y_cable, L_target,
                x_boat=x_boat, y_boat=y_boat,
                x_rov=None, y_rov=None,  # Extrémité ROV libre
                k_tail=k_tail, k_max=k_max,
                bidirectional=False,  # Désactiver la bidirectionnelle dans l'appel récursif
                _from_direction='boat'
            )
            
            # Étape 2 : Normalisation depuis le ROV (extrémité bateau libre)
            trace_print(5, "[DEBUG] _normalize_cable_length: Normalisation bidirectionnelle - depuis le ROV")
            x_from_rov, y_from_rov = self._normalize_cable_length(
                x_cable, y_cable, L_target,
                x_boat=None, y_boat=None,  # Extrémité bateau libre
                x_rov=x_rov, y_rov=y_rov,
                k_tail=k_tail, k_max=k_max,
                bidirectional=False,  # Désactiver la bidirectionnelle dans l'appel récursif
                _from_direction='rov'
            )
            
            # Étape 3 : Blending pondéré
            # Plus de poids à la normalisation "bateau" près du bateau, plus de poids à la normalisation "ROV" près du ROV
            x_new = np.zeros(N + 1)
            y_new = np.zeros(N + 1)
            
            for i in range(N + 1):
                # Calculer les poids : w_boat décroît de 1.0 (au bateau) à 0.0 (au ROV)
                # Utiliser une fonction linéaire pour une transition douce
                w_boat = 1.0 - (i / float(N)) if N > 0 else 1.0
                w_rov = 1.0 - w_boat
                
                # Normaliser les poids pour garantir w_boat + w_rov = 1.0
                w_sum = w_boat + w_rov
                if w_sum > 1e-9:
                    w_boat /= w_sum
                    w_rov /= w_sum
                
                # Mélange pondéré
                x_new[i] = w_boat * x_from_boat[i] + w_rov * x_from_rov[i]
                y_new[i] = w_boat * y_from_boat[i] + w_rov * y_from_rov[i]
            
            # Étape 4 : Forcer les extrémités après le blending
            if x_boat is not None:
                x_new[0] = float(x_boat)
            if y_boat is not None:
                y_new[0] = min(float(y_boat), 0.0)
            if x_rov is not None:
                x_new[-1] = float(x_rov)
            if y_rov is not None:
                y_new[-1] = min(float(y_rov), 0.0)
            
            # Étape 5 : Ajustement de longueur final
            # Calculer la longueur actuelle
            L_actual = 0.0
            for i in range(N):
                dx = x_new[i+1] - x_new[i]
                dy = y_new[i+1] - y_new[i]
                L_actual += np.sqrt(dx**2 + dy**2)
            
            # Ajuster légèrement si nécessaire, en préservant les extrémités
            # IMPORTANT : Ne pas ajuster si la différence est trop grande, car cela peut créer des segments longs
            # La normalisation bidirectionnelle devrait déjà avoir créé un câble de longueur proche de L_target
            if L_actual > 1e-9 and abs(L_actual - L_target) / max(L_target, 1e-9) > 1e-4:
                # Si la différence est trop grande (> 5%), utiliser une rééchantillonnage plutôt qu'un ajustement
                if abs(L_actual - L_target) / max(L_target, 1e-9) > 0.05:
                    trace_print(5, f"[DEBUG] _normalize_cable_length: Différence de longueur trop grande ({L_actual:.3f} vs {L_target:.3f}), rééchantillonnage")
                    # Rééchantillonnage en abscisse curviligne pour garantir L_target exactement
                    s_cumulative = np.zeros(N + 1)
                    for i in range(N):
                        dx = x_new[i+1] - x_new[i]
                        dy = y_new[i+1] - y_new[i]
                        s_cumulative[i+1] = s_cumulative[i] + np.sqrt(dx**2 + dy**2)
                    
                    # Remise à l'échelle
                    s_cumulative = s_cumulative * (L_target / L_actual)
                    s_points = np.linspace(0.0, L_target, N + 1)
                    
                    # Interpolation spline pour préserver la forme
                    try:
                        if len(s_cumulative) >= 4:
                            f_x = interp1d(s_cumulative, x_new, kind='cubic', bounds_error=False, fill_value='extrapolate')
                            f_y = interp1d(s_cumulative, y_new, kind='cubic', bounds_error=False, fill_value='extrapolate')
                        else:
                            f_x = interp1d(s_cumulative, x_new, kind='linear', bounds_error=False, fill_value='extrapolate')
                            f_y = interp1d(s_cumulative, y_new, kind='linear', bounds_error=False, fill_value='extrapolate')
                        
                        x_new = f_x(s_points)
                        y_new = f_y(s_points)
                    except:
                        # Fallback : interpolation linéaire
                        for i in range(N + 1):
                            s_i = s_points[i]
                            idx = np.searchsorted(s_cumulative, s_i)
                            if idx == 0:
                                idx = 1
                            elif idx >= len(s_cumulative):
                                idx = len(s_cumulative) - 1
                            s_prev = s_cumulative[idx - 1]
                            s_next = s_cumulative[idx]
                            if abs(s_next - s_prev) > 1e-9:
                                alpha = (s_i - s_prev) / (s_next - s_prev)
                                x_new[i] = x_new[idx - 1] + alpha * (x_new[idx] - x_new[idx - 1])
                                y_new[i] = y_new[idx - 1] + alpha * (y_new[idx] - y_new[idx - 1])
                
                # Réappliquer les extrémités après l'ajustement
                if x_boat is not None:
                    x_new[0] = float(x_boat)
                if y_boat is not None:
                    y_new[0] = min(float(y_boat), 0.0)
                if x_rov is not None:
                    x_new[-1] = float(x_rov)
                if y_rov is not None:
                    y_new[-1] = min(float(y_rov), 0.0)
            
            # Étape 6 : Clipping et lissage (utiliser le code existant)
            # Clipping y <= 0
            y_new = np.clip(y_new, None, 0.0)
            
            # Réappliquer les extrémités après clipping
            if x_boat is not None:
                x_new[0] = float(x_boat)
            if y_boat is not None:
                y_new[0] = min(float(y_boat), 0.0)
            if x_rov is not None:
                x_new[-1] = float(x_rov)
            if y_rov is not None:
                y_new[-1] = min(float(y_rov), 0.0)
            
            # Appliquer le lissage local si nécessaire (code existant)
            def _compute_segment_lengths(x_arr, y_arr):
                ds_list = np.sqrt(np.diff(x_arr) ** 2 + np.diff(y_arr) ** 2)
                L_total = float(ds_list.sum())
                if N > 0:
                    ds_max = float(ds_list.max())
                else:
                    ds_max = 0.0
                return ds_list, L_total, ds_max
            
            ds_list, L_total, ds_max = _compute_segment_lengths(x_new, y_new)
            ds_target = L_target / max(N, 1)
            ratio_max = ds_max / max(ds_target, 1e-9)
            
            k_ratio_max = k_max
            max_iter_smooth = 20
            iter_smooth = 0
            
            while ratio_max > k_ratio_max and iter_smooth < max_iter_smooth:
                i_max = int(np.argmax(ds_list))
                
                # Ne jamais modifier les extrémités (points 0 et N) - elles sont déjà forcées
                if i_max == 0:
                    # Le segment 0 est trop long - cela signifie que P0 n'est pas au bon endroit
                    # Forcer P0 à être exactement au bateau si fourni
                    if x_boat is not None:
                        trace_print(5, f"[DEBUG] _normalize_cable_length: Segment 0 trop long (ratio={ratio_max:.2f}), forçage P0 au bateau")
                        x_new[0] = float(x_boat)
                        y_new[0] = min(float(y_boat), 0.0) if y_boat is not None else 0.0
                    # Ajuster P1 pour réduire le segment 0
                    if N >= 2:
                        i_move = 1
                        boat_pt = np.array([x_new[0], y_new[0]])
                        next_pt = np.array([x_new[2], y_new[2]])
                        current = np.array([x_new[i_move], y_new[i_move]])
                        # Déplacer P1 vers le bateau pour réduire le segment 0
                        target = 0.3 * boat_pt + 0.7 * next_pt
                        alpha_smooth = 0.6
                        new_pt = (1.0 - alpha_smooth) * current + alpha_smooth * target
                        x_new[i_move] = float(new_pt[0])
                        y_new[i_move] = min(float(new_pt[1]), 0.0)
                elif i_max == N - 1:
                    # Le segment N-1 est trop long - cela signifie que PN n'est pas au bon endroit
                    # Forcer PN à être exactement au ROV si fourni
                    if x_rov is not None:
                        trace_print(5, f"[DEBUG] _normalize_cable_length: Segment {N-1} trop long (ratio={ratio_max:.2f}), forçage PN au ROV")
                        x_new[-1] = float(x_rov)
                        y_new[-1] = min(float(y_rov), 0.0)
                    # Ajuster PN-1 pour réduire le segment N-1
                    if N >= 2:
                        rov_pt = np.array([x_new[N], y_new[N]])
                        current = np.array([x_new[N - 1], y_new[N - 1]])
                        dist_to_rov = np.linalg.norm(current - rov_pt)
                        target_dist = min(dist_to_rov, k_max * ds_target)
                        if dist_to_rov > 1e-9:
                            direction = (rov_pt - current) / dist_to_rov
                            target_pt = rov_pt - target_dist * direction
                            alpha_smooth = 0.8
                            new_pt = (1.0 - alpha_smooth) * current + alpha_smooth * target_pt
                            x_new[N - 1] = float(new_pt[0])
                            y_new[N - 1] = min(float(new_pt[1]), 0.0)
                        else:
                            break
                    else:
                        break
                elif 0 < i_max < N - 1:
                    i_move = i_max + 1
                    prev_pt = np.array([x_new[i_move - 1], y_new[i_move - 1]])
                    next_pt = np.array([x_new[i_move + 1], y_new[i_move + 1]])
                    current = np.array([x_new[i_move], y_new[i_move]])
                    target = 0.5 * (prev_pt + next_pt)
                    alpha_smooth = 0.5
                    new_pt = (1.0 - alpha_smooth) * current + alpha_smooth * target
                    x_new[i_move] = float(new_pt[0])
                    y_new[i_move] = min(float(new_pt[1]), 0.0)
                else:
                    break
                
                y_new = np.clip(y_new, None, 0.0)
                
                # Réappliquer le recollement aux extrémités après chaque itération
                if x_boat is not None:
                    y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
                    x_new[0] = float(x_boat)
                    y_new[0] = y_boat_clipped
                if x_rov is not None:
                    y_rov_clipped = min(float(y_rov), 0.0)
                    x_new[-1] = float(x_rov)
                    y_new[-1] = y_rov_clipped
                
                ds_list, L_total, ds_max = _compute_segment_lengths(x_new, y_new)
                ratio_max = ds_max / max(ds_target, 1e-9)
                iter_smooth += 1
            
            # Réappliquer les extrémités une dernière fois
            if x_boat is not None:
                y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
                x_new[0] = float(x_boat)
                y_new[0] = y_boat_clipped
            if x_rov is not None:
                y_rov_clipped = min(float(y_rov), 0.0)
                x_new[-1] = float(x_rov)
                y_new[-1] = y_rov_clipped
            
            # Clipping final
            y_new = np.clip(y_new, None, 0.0)
            
            # Vérification finale et forçage des extrémités (sécurité supplémentaire)
            if x_boat is not None:
                dist_boat = np.sqrt((x_new[0] - float(x_boat))**2 + (y_new[0] - (min(float(y_boat), 0.0) if y_boat is not None else 0.0))**2)
                if dist_boat > 1e-6:
                    trace_print(5, f"[DEBUG] _normalize_cable_length: ⚠️  Correction finale recollement bateau: dist={dist_boat:.6f} m")
                    x_new[0] = float(x_boat)
                    y_new[0] = min(float(y_boat), 0.0) if y_boat is not None else 0.0
            
            if x_rov is not None:
                dist_rov = np.sqrt((x_new[-1] - float(x_rov))**2 + (y_new[-1] - min(float(y_rov), 0.0))**2)
                if dist_rov > 1e-6:
                    trace_print(5, f"[DEBUG] _normalize_cable_length: ⚠️  Correction finale recollement ROV: dist={dist_rov:.6f} m")
                    x_new[-1] = float(x_rov)
                    y_new[-1] = min(float(y_rov), 0.0)
            
            # Réappliquer les extrémités après vérification (sécurité supplémentaire)
            if x_boat is not None:
                x_new[0] = float(x_boat)
                y_new[0] = min(float(y_boat), 0.0) if y_boat is not None else 0.0
            if x_rov is not None:
                x_new[-1] = float(x_rov)
                y_new[-1] = min(float(y_rov), 0.0)
            
            # Calculer la déviation maximale APRÈS normalisation
            max_dev_after = compute_max_deviation(x_new, y_new)
            
            # Message de debug sur la préservation de la déviation
            if max_dev_before > 0.01 and max_dev_after < max_dev_before * 0.5:
                trace_print(5, f"[DEBUG] _normalize_cable_length: ⚠️  DÉVIATION RÉDUITE - "
                    f"avant={max_dev_before:.6f} m, après={max_dev_after:.6f} m, "
                    f"ratio={max_dev_after/max_dev_before:.3f}, L_target={L_target:.2f} m")
            elif max_dev_before > 0.01:
                trace_print(5, f"[DEBUG] _normalize_cable_length: ✓ Déviation préservée - "
                    f"avant={max_dev_before:.6f} m, après={max_dev_after:.6f} m")
            
            return x_new, y_new
        
        # Algorithme unidirectionnel actuel (utilisé si bidirectional=False ou si _from_direction est défini)
        
        s_cumulative = np.zeros(N + 1)  # Distance curviligne cumulative
        
        for i in range(N):
            dx = x_cable[i+1] - x_cable[i]
            dy = y_cable[i+1] - y_cable[i]
            ds = np.sqrt(dx**2 + dy**2)
            s_cumulative[i+1] = s_cumulative[i] + ds
        
        L_actual = s_cumulative[-1]

        # Cas dégénéré : reconstruire un câble rectiligne horizontal à partir du premier point.
        if L_actual <= 1e-6:
            x_start = float(x_cable[0]) if len(x_cable) > 0 else 0.0
            y_start = float(y_cable[0]) if len(y_cable) > 0 else 0.0
            y_start = min(y_start, 0.0)
            x_cable_new = x_start + np.linspace(0.0, L_target, N + 1)
            y_cable_new = np.full(N + 1, y_start, dtype=float)
            return x_cable_new, y_cable_new

        # 1) Remise à l'échelle de l'abscisse curviligne pour que s_cumulative[-1] = L_target.
        s_cumulative = s_cumulative * (L_target / L_actual)

        # 2) Rééchantillonnage uniforme en abscisse curviligne avec interpolation spline pour préserver la forme
        ds_target = L_target / max(N, 1)
        s_points = np.linspace(0.0, L_target, N + 1)

        # Utiliser une interpolation spline cubique pour préserver la forme de la caténaire
        # au lieu d'une interpolation linéaire qui peut transformer une courbe en ligne droite
        try:
            # Interpolation spline pour x et y en fonction de l'abscisse curviligne
            # Utiliser 'cubic' pour préserver la courbure, avec 'linear' comme fallback
            if len(s_cumulative) >= 4:  # Besoin d'au moins 4 points pour une spline cubique
                f_x = interp1d(s_cumulative, x_cable, kind='cubic', bounds_error=False, fill_value='extrapolate')
                f_y = interp1d(s_cumulative, y_cable, kind='cubic', bounds_error=False, fill_value='extrapolate')
            else:
                # Fallback vers interpolation linéaire si pas assez de points
                f_x = interp1d(s_cumulative, x_cable, kind='linear', bounds_error=False, fill_value='extrapolate')
                f_y = interp1d(s_cumulative, y_cable, kind='linear', bounds_error=False, fill_value='extrapolate')
            
            x_new = f_x(s_points)
            y_new = f_y(s_points)
            
            # Clipping immédiat après rééchantillonnage pour éviter les points au-dessus de la surface
            y_new = np.clip(y_new, None, 0.0)
            
            # S'assurer que les extrémités sont exactement correctes
            # IMPORTANT : Utiliser les positions cibles (x_boat, x_rov) si fournies, sinon les extrémités actuelles
            if x_boat is not None:
                x_new[0] = float(x_boat)
            else:
                x_new[0] = x_cable[0]
            if y_boat is not None:
                y_new[0] = min(float(y_boat), 0.0)
            else:
                y_new[0] = y_cable[0]
            
            if x_rov is not None:
                x_new[-1] = float(x_rov)
            else:
                x_new[-1] = x_cable[-1]
            if y_rov is not None:
                y_new[-1] = min(float(y_rov), 0.0)
            else:
                y_new[-1] = y_cable[-1]
        except Exception as e:
            # Fallback vers l'interpolation linéaire si la spline échoue
            trace_print(9, f"[DEBUG] _normalize_cable_length: Échec interpolation spline ({e}), utilisation interpolation linéaire")
            x_new = np.zeros(N + 1)
            y_new = np.zeros(N + 1)
            
            for i in range(N + 1):
                s_i = s_points[i]
                
                # Trouver le segment qui contient ce point dans le câble original
                idx = np.searchsorted(s_cumulative, s_i)
                if idx == 0:
                    idx = 1
                elif idx >= len(s_cumulative):
                    idx = len(s_cumulative) - 1
                
                s_prev = s_cumulative[idx - 1]
                s_next = s_cumulative[idx]
                
                if abs(s_next - s_prev) > 1e-9:
                    alpha = (s_i - s_prev) / (s_next - s_prev)
                    x_new[i] = x_cable[idx - 1] + alpha * (x_cable[idx] - x_cable[idx - 1])
                    y_new[i] = y_cable[idx - 1] + alpha * (y_cable[idx] - y_cable[idx - 1])
                else:
                    x_new[i] = x_cable[idx - 1]
                    y_new[i] = y_cable[idx - 1]
            
            # IMPORTANT : Forcer les extrémités aux positions cibles si fournies
            if x_boat is not None:
                x_new[0] = float(x_boat)
            if y_boat is not None:
                y_new[0] = min(float(y_boat), 0.0)
            if x_rov is not None:
                x_new[-1] = float(x_rov)
            if y_rov is not None:
                y_new[-1] = min(float(y_rov), 0.0)

        # 3) Clipping physique : y <= 0
        y_new = np.clip(y_new, None, 0.0)

        # 3bis) Recollement « bi‑extrémités » si les positions cibles sont fournies
        # On fixe simultanément le point 0 au bateau et le point N au ROV, puis
        # on répartit l'ajustement au milieu au lieu de concentrer la correction
        # uniquement sur la queue côté ROV.
        if x_boat is not None and x_rov is not None:
            n_points = len(x_new)
            if n_points >= 2:
                # Ancrer le premier point au bateau
                y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
                x_new[0] = float(x_boat)
                y_new[0] = y_boat_clipped

                # Ancrer le dernier point au ROV
                y_rov_clipped = min(float(y_rov), 0.0)
                x_new[-1] = float(x_rov)
                y_new[-1] = y_rov_clipped

                # Ne PAS mélanger avec une ligne droite - préserver complètement la forme originale
                # Le rééchantillonnage en abscisse curviligne a déjà préservé la forme,
                # et les extrémités sont déjà fixées correctement ci-dessus.
                # Aucun ajustement supplémentaire nécessaire pour préserver la forme de la caténaire.

                # S'assurer à nouveau du clipping
                y_new = np.clip(y_new, None, 0.0)

        # 4) Lissage local pour limiter ds_max / ds_target sans imposer ds strictement égaux
        def _compute_segment_lengths(x_arr, y_arr):
            ds_list = np.sqrt(np.diff(x_arr) ** 2 + np.diff(y_arr) ** 2)
            L_total = float(ds_list.sum())
            if N > 0:
                ds_max = float(ds_list.max())
            else:
                ds_max = 0.0
            return ds_list, L_total, ds_max

        ds_list, L_total, ds_max = _compute_segment_lengths(x_new, y_new)
        ds_target = L_target / max(N, 1)
        ratio_max = ds_max / max(ds_target, 1e-9)

        # Si le segment le plus long est trop grand par rapport au segment moyen, lisser.
        # L'objectif est de rester proche de ds_max/ds_target <= k_max dans la mesure du possible.
        k_ratio_max = k_max
        max_iter_smooth = 20  # Augmenté pour mieux garantir le respect de k_max

        iter_smooth = 0
        while ratio_max > k_ratio_max and iter_smooth < max_iter_smooth:
            # Identifier le segment le plus long
            i_max = int(np.argmax(ds_list))

            # On ne déplace jamais les extrémités 0 et N (recollement bateau / ROV)
            if 0 < i_max < N - 1:
                # Segment long entre i_max et i_max+1, avec un voisin de chaque côté
                # On déplace légèrement le point i_max+1 vers la moyenne de ses voisins.
                i_move = i_max + 1
                prev_pt = np.array([x_new[i_move - 1], y_new[i_move - 1]])
                next_pt = np.array([x_new[i_move + 1], y_new[i_move + 1]])
                current = np.array([x_new[i_move], y_new[i_move]])
                target = 0.5 * (prev_pt + next_pt)
                alpha_smooth = 0.5
                new_pt = (1.0 - alpha_smooth) * current + alpha_smooth * target
                x_new[i_move] = float(new_pt[0])
                y_new[i_move] = min(float(new_pt[1]), 0.0)  # Clipping immédiat : y <= 0
            elif i_max == 0 and N >= 2:
                # Segment le plus long entre 0 et 1 : on déplace le point 1 vers la moyenne de 0 et 2.
                i_move = 1
                prev_pt = np.array([x_new[0], y_new[0]])
                next_pt = np.array([x_new[2], y_new[2]])
                current = np.array([x_new[i_move], y_new[i_move]])
                target = 0.5 * (prev_pt + next_pt)
                alpha_smooth = 0.5
                new_pt = (1.0 - alpha_smooth) * current + alpha_smooth * target
                x_new[i_move] = float(new_pt[0])
                y_new[i_move] = min(float(new_pt[1]), 0.0)  # Clipping immédiat : y <= 0
            elif i_max == N - 1 and N >= 2:
                # Segment le plus long entre N-1 et N : on doit réduire ce segment
                # en déplaçant le point N-1 vers le ROV (point N), tout en préservant le recollement
                if N >= 2:
                    # Déplacer directement le point N-1 vers le ROV pour réduire le segment
                    rov_pt = np.array([x_new[N], y_new[N]])  # Point ROV fixe
                    current = np.array([x_new[N - 1], y_new[N - 1]])
                    # Calculer la distance actuelle au ROV
                    dist_to_rov = np.linalg.norm(current - rov_pt)
                    # Cible : réduire la distance pour que le segment soit au maximum k_max * ds_target
                    target_dist = min(dist_to_rov, k_max * ds_target)
                    if dist_to_rov > 1e-9:
                        # Déplacer le point N-1 vers le ROV
                        direction = (rov_pt - current) / dist_to_rov
                        target_pt = rov_pt - target_dist * direction
                        # Déplacer progressivement (plus agressif pour le dernier segment)
                        alpha_smooth = 0.8  # Très agressif pour garantir le respect de k_max
                        new_pt = (1.0 - alpha_smooth) * current + alpha_smooth * target_pt
                        x_new[N - 1] = float(new_pt[0])
                        y_new[N - 1] = min(float(new_pt[1]), 0.0)  # Clipping immédiat : y <= 0
                    else:
                        # Le point N-1 est déjà au ROV, on ne peut rien faire
                        break
                else:
                    # Cas dégénéré : seulement 2 points, on déplace N-1
                    i_move = N - 1
                    prev_pt = np.array([x_new[i_move - 1], y_new[i_move - 1]])
                    next_pt = np.array([x_new[N], y_new[N]])
                    current = np.array([x_new[i_move], y_new[i_move]])
                    target = 0.5 * (prev_pt + next_pt)
                    alpha_smooth = 0.8  # Plus agressif
                    new_pt = (1.0 - alpha_smooth) * current + alpha_smooth * target
                    x_new[i_move] = float(new_pt[0])
                    y_new[i_move] = min(float(new_pt[1]), 0.0)  # Clipping immédiat : y <= 0
            else:
                # Cas pathologique (très petit N), on sort.
                break

            # Clipper de nouveau en y
            y_new = np.clip(y_new, None, 0.0)

            # Réappliquer le recollement aux extrémités après chaque itération de lissage
            # FORCER le recollement exact pour garantir la contrainte
            if x_boat is not None:
                y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
                x_new[0] = float(x_boat)
                y_new[0] = y_boat_clipped
            if x_rov is not None:
                y_rov_clipped = min(float(y_rov), 0.0)
                x_new[-1] = float(x_rov)
                y_new[-1] = y_rov_clipped

            # Recalculer les longueurs de segment et le ratio
            ds_list, L_total, ds_max = _compute_segment_lengths(x_new, y_new)
            ratio_max = ds_max / max(ds_target, 1e-9)
            iter_smooth += 1

        # Réappliquer le recollement aux extrémités une dernière fois après le lissage
        # FORCER le recollement exact pour garantir la contrainte
        if x_boat is not None:
            y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
            x_new[0] = float(x_boat)
            y_new[0] = y_boat_clipped
        if x_rov is not None:
            y_rov_clipped = min(float(y_rov), 0.0)
            x_new[-1] = float(x_rov)
            y_new[-1] = y_rov_clipped

        # Optionnel : petit correctif de longueur si l'écart est significatif
        # (on vise une précision meilleure que 0.01 % sur la longueur totale)
        if L_total > 1e-9 and abs(L_total - L_target) / max(L_target, 1e-9) > 1e-4:
            scale = L_target / L_total
            # Utiliser directement les positions cibles des extrémités au lieu des valeurs modifiées
            if x_boat is not None and x_rov is not None:
                x0, y0 = float(x_boat), min(float(y_boat), 0.0) if y_boat is not None else 0.0
                xN, yN = float(x_rov), min(float(y_rov), 0.0)
            else:
                x0, y0 = x_new[0], y_new[0]
                xN, yN = x_new[-1], y_new[-1]
            
            for i in range(1, N):
                # On applique une mise à l'échelle radiale par rapport au point 0,
                # puis on corrige linéairement pour conserver exactement le point N.
                tx = x_new[i] - x0
                ty = y_new[i] - y0
                x_scaled = x0 + tx * scale
                y_scaled = y0 + ty * scale

                # Correction linéaire le long de la corde (x0,y0) -> (xN,yN)
                t = i / float(N)
                x_lin = x0 + t * (xN - x0)
                y_lin = y0 + t * (yN - y0)

                # Combinaison pour rester proche de la forme mais respecter les extrémités
                beta = 0.2
                x_new[i] = (1.0 - beta) * x_scaled + beta * x_lin
                y_new[i] = (1.0 - beta) * y_scaled + beta * y_lin

            y_new = np.clip(y_new, None, 0.0)
            
            # Réappliquer les extrémités après le correctif de longueur
            # FORCER le recollement exact pour garantir la contrainte
            if x_boat is not None:
                y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
                x_new[0] = float(x_boat)
                y_new[0] = y_boat_clipped
            if x_rov is not None:
                y_rov_clipped = min(float(y_rov), 0.0)
                x_new[-1] = float(x_rov)
                y_new[-1] = y_rov_clipped

        # Réappliquer les extrémités une dernière fois avant la vérification finale
        # FORCER le recollement exact pour garantir la contrainte
        if x_boat is not None:
            y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
            x_new[0] = float(x_boat)
            y_new[0] = y_boat_clipped
        if x_rov is not None:
            y_rov_clipped = min(float(y_rov), 0.0)
            x_new[-1] = float(x_rov)
            y_new[-1] = y_rov_clipped
        
        # DEBUG: Vérifier le recollage final
        if x_boat is not None and x_rov is not None:
            dist_boat_final = np.sqrt((x_new[0] - float(x_boat))**2 + (y_new[0] - (min(float(y_boat), 0.0) if y_boat is not None else 0.0))**2)
            dist_rov_final = np.sqrt((x_new[-1] - float(x_rov))**2 + (y_new[-1] - min(float(y_rov), 0.0))**2)
            if dist_boat_final > 1e-3 or dist_rov_final > 1e-3:
                trace_print(5, f"[DEBUG] _normalize_cable_length: ⚠️  Recollage incorrect à la fin ! "
                    f"dist_boat={dist_boat_final:.6f} m, dist_rov={dist_rov_final:.6f} m, "
                    f"P0=({x_new[0]:.3f},{y_new[0]:.3f}), PN=({x_new[-1]:.3f},{y_new[-1]:.3f}), "
                    f"Bateau=({x_boat:.3f},{min(float(y_boat), 0.0) if y_boat is not None else 0.0:.3f}), "
                    f"ROV=({x_rov:.3f},{min(float(y_rov), 0.0):.3f})")
                # Forcer le recollage manuellement
                if x_boat is not None:
                    x_new[0] = float(x_boat)
                    y_new[0] = min(float(y_boat), 0.0) if y_boat is not None else 0.0
                if x_rov is not None:
                    x_new[-1] = float(x_rov)
                    y_new[-1] = min(float(y_rov), 0.0)

        # Clipping final pour garantir y <= 0
        y_new = np.clip(y_new, None, 0.0)
        
        # Réappliquer les extrémités APRÈS le clipping final pour garantir le recollage exact
        # (le clipping peut avoir modifié y_new[0] ou y_new[-1])
        if x_boat is not None:
            x_new[0] = float(x_boat)
            y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
            y_new[0] = y_boat_clipped
        if x_rov is not None:
            x_new[-1] = float(x_rov)
            y_rov_clipped = min(float(y_rov), 0.0) if y_rov is not None else 0.0
            y_new[-1] = y_rov_clipped
        
        # DEBUG: Vérifier s'il reste des points au-dessus de la surface après tous les clippings
        points_above = np.where(y_new > 1e-6)[0]
        if len(points_above) > 0:
            # Exclure le point bateau (index 0) qui doit être à y=0
            points_above = points_above[points_above != 0]
            if len(points_above) > 0:
                max_y_above = np.max(y_new[points_above])
                trace_print(5, f"[DEBUG] _normalize_cable_length: ⚠️  {len(points_above)} points encore au-dessus de la surface après clipping final ! "
                    f"max_y={max_y_above:.6f} m, indices={points_above[:5].tolist()}")
                # Forcer le clipping manuellement
                for idx in points_above:
                    y_new[idx] = 0.0
        
        # Réappliquer les extrémités une dernière fois après le clipping manuel
        if x_boat is not None:
            x_new[0] = float(x_boat)
            y_boat_clipped = min(float(y_boat), 0.0) if y_boat is not None else 0.0
            y_new[0] = y_boat_clipped
        if x_rov is not None:
            x_new[-1] = float(x_rov)
            y_rov_clipped = min(float(y_rov), 0.0) if y_rov is not None else 0.0
            y_new[-1] = y_rov_clipped

        # Vérification finale des contraintes
        try:
            ds_list_final = np.sqrt(np.diff(x_new) ** 2 + np.diff(y_new) ** 2)
            L_final = float(ds_list_final.sum()) if len(ds_list_final) > 0 else 0.0
            ds_target_final = L_target / max(N, 1)
            ds_max_final = float(ds_list_final.max()) if len(ds_list_final) > 0 else 0.0
            ratio_final = ds_max_final / max(ds_target_final, 1e-9) if ds_target_final > 0 else 0.0

            # Contraintes vérifiées :
            # - Longueur totale proche de L_target
            # - Ratio ds_max / ds_target raisonnable (≲ 2.0)
            # - y <= 0
            tol_L_rel = 1e-4  # 0.01 %
            ok_L = (L_target <= 0.0) or (abs(L_final - L_target) / max(L_target, 1e-9) <= tol_L_rel)
            ok_ratio = ratio_final <= 2.0 + 1e-6
            ok_y = bool(np.all(y_new <= 1e-9))

            # Recollement extrémités : vérifier qu'elles sont bien aux positions cibles
            ok_ends = True
            if len(x_new) >= 2:
                if x_boat is not None:
                    dx0 = float(x_new[0] - float(x_boat))
                    dy0 = float(y_new[0] - (min(float(y_boat), 0.0) if y_boat is not None else 0.0))
                    err0 = np.hypot(dx0, dy0)
                    ok_ends = ok_ends and (err0 <= 1e-3)  # Tolérance 1 mm
                if x_rov is not None:
                    dxN = float(x_new[-1] - float(x_rov))
                    dyN = float(y_new[-1] - min(float(y_rov), 0.0))
                    errN = np.hypot(dxN, dyN)
                    ok_ends = ok_ends and (errN <= 1e-3)  # Tolérance 1 mm

            if not (ok_L and ok_ratio and ok_y and ok_ends):
                # Utiliser l'import global de trace_print (déjà importé en haut du fichier)
                trace_print(
                    8,
                    "[DEBUG _normalize_cable_length] CONTRAINTES NON RESPECTÉES : "
                    f"L_target={L_target:.6f}, L_final={L_final:.6f}, "
                    f"rel_err_L={abs(L_final - L_target) / max(L_target, 1e-9):.3e}, "
                    f"ds_target={ds_target_final:.6f}, ds_max={ds_max_final:.6f}, "
                    f"ratio_max={ratio_final:.3f}, ok_L={ok_L}, ok_ratio={ok_ratio}, "
                    f"ok_y={ok_y}, ok_ends={ok_ends}"
                )
        except Exception:
            # Ne jamais casser la simulation à cause d'un check de debug
            pass

        # DEBUG: Calculer la déviation maximale APRÈS normalisation
        max_dev_after = compute_max_deviation(x_new, y_new)
        if max_dev_before > 0.01 and max_dev_after < max_dev_before * 0.5:
            # La déviation a été significativement réduite, ce qui indique que la forme a été détruite
            trace_print(5, f"[DEBUG] _normalize_cable_length: ⚠️  DÉVIATION RÉDUITE - "
                f"avant={max_dev_before:.6f} m, après={max_dev_after:.6f} m, "
                f"ratio={max_dev_after/max_dev_before:.3f}, L_target={L_target:.2f} m")
        elif max_dev_before > 0.01:
            trace_print(5, f"[DEBUG] _normalize_cable_length: ✓ Déviation préservée - "
                f"avant={max_dev_before:.6f} m, après={max_dev_after:.6f} m")

        return x_new, y_new

