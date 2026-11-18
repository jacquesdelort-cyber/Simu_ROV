"""Système complet ROV-Câble-Bateau"""
import numpy as np
from .rov_model import ROV
from src.utils.logger import trace_print

# Borne numérique des vitesses ROV (m/s) pour forces, solveur câble et champs dérivés associés.
# L'état intégré (y[2], y[3]) n'est pas modifié ; seules les grandeurs utilisées dans compute_derivatives
# passent par vx_rov_f / vy_rov_f pour limiter les termes sensibles (réaction surface, câble, etc.).
V_MAX_ROV_VELOCITY_FORCES_MS = 500.0
# Plafond d'accélération ROV (m/s²) dans compute_derivatives — évite sauts numériques extrêmes.
A_MAX_ROV_MS2 = 45.0
# Plafond des vitesses ROV dans le vecteur d'état après intégration (m/s) — distinct des 500 m/s pour les forces.
V_MAX_ROV_STATE_IN_Y_MS = 80.0
TENSION_GUARD_BUILD_ID = "tension-continue-20261117"
from .cable_model import Cable
from .boat_model import Boat
from .environment import Environment
from ..solvers.cable_continuity import (
    continuous_tau_tension,
    rov_accel_scale,
    smooth_vertical_node_velocity,
    payout_matched_velocity,
    taut_constraint_tension,
    taut_penalty_newton,
)
from ..solvers.integrator import TimeIntegrator


class ROVSystem:
    """Système complet ROV-Câble-Bateau"""
    
    def __init__(self, params, N_segments=50):
        """
        Initialise le système complet
        
        Parameters:
        -----------
        params : dict
            Dictionnaire contenant:
            - 'rov' : paramètres du ROV
            - 'cable' : paramètres du câble
            - 'boat' : paramètres du bateau
            - 'environment' : paramètres environnementaux
        N_segments : int
            Nombre de segments pour discrétiser le câble
        """
        # Créer les composants
        self.environment = Environment(params.get('environment', {}))
        self.rov = ROV(params.get('rov', {}))
        self.boat = Boat(params.get('boat', {}))
        self.cable = Cable(params.get('cable', {}), N_segments, self.environment)
        
        self.N = N_segments
        
        # Intégrateur
        self.integrator = TimeIntegrator(method='RK45', max_step=0.1)
        
        # État précédent du câble (pour itération)
        self.x_cable_prev = None
        self.y_cable_prev = None

        # True : solveur dynamique câble ; False : équilibre statique à chaque pas (quasi-statique)
        self.dynamic_simulation = True
        # Aligné sur get_initial_state : statique avec déformation courant si True
        self.use_current_geometry_for_static = True
        # True pendant les sous-pas d'intégration : la géométrie du câble reste celle
        # de l'état, le solveur non linéaire n'est appelé qu'une fois par pas accepté.
        self._freeze_cable_geometry = False
        # Dernière commande, relue par la projection qui réécrit la tension.
        self._hold_fx = 0.0
        self._hold_fy = 0.0
        self._hold_dL_dt = 0.0
    
    def get_state_size(self):
        """Retourne la taille du vecteur d'état"""
        # ROV: 4 (x, y, vx, vy)
        # Bateau: 2 (x, vx)
        # Câble: (N+1)*2 positions + (N+1) tensions = 3*(N+1)
        # Longueur: 1
        return 6 + 3 * (self.N + 1) + 1
    
    def pack_state(self, x_rov, y_rov, vx_rov, vy_rov,
                   x_boat, vx_boat, x_cable, y_cable, T, L):
        """
        Empaquette l'état dans un vecteur
        
        Returns:
        --------
        array
            Vecteur d'état
        """
        y = np.zeros(self.get_state_size())
        y[0] = x_rov
        y[1] = y_rov
        y[2] = vx_rov
        y[3] = vy_rov
        y[4] = x_boat
        y[5] = vx_boat
        
        idx = 6
        y[idx:idx+self.N+1] = x_cable
        idx += self.N + 1
        y[idx:idx+self.N+1] = y_cable
        idx += self.N + 1
        y[idx:idx+self.N+1] = T
        idx += self.N + 1
        y[idx] = L
        
        return y
    
    def unpack_state(self, y):
        """
        Dépaquette le vecteur d'état
        
        Returns:
        --------
        tuple
            Composantes de l'état
        """
        x_rov = y[0]
        y_rov = y[1]
        vx_rov = y[2]
        vy_rov = y[3]
        x_boat = y[4]
        vx_boat = y[5]
        
        idx = 6
        x_cable = y[idx:idx+self.N+1]
        idx += self.N + 1
        y_cable = y[idx:idx+self.N+1]
        idx += self.N + 1
        T = y[idx:idx+self.N+1]
        idx += self.N + 1
        L = y[idx]
        
        return (x_rov, y_rov, vx_rov, vy_rov, x_boat, vx_boat,
                x_cable, y_cable, T, L)

    def _abs_tension_cap_n(self) -> float:
        """
        Plafond absolu (N) pour les tensions du câble dans l'ODE.
        Dérivé de ``tension_rupture`` (paramètres câble) avec bornes sûres pour le simulateur.
        """
        try:
            tr = float(self.cable.solver.params.get("tension_rupture", 4.0e4))
        except Exception:
            tr = 4.0e4
        tr = max(tr, 500.0)
        return float(min(max(tr * 2.5, 2.0e3), 1.8e5))

    def compute_derivatives(self, t, y, u):
        """
        Calcule les dérivées du système
        
        Parameters:
        -----------
        t : float
            Temps (s)
        y : array
            Vecteur d'état
        u : dict
            Vecteur de commande:
            - Fx_rov : force horizontale ROV (N)
            - Fy_rov : force verticale ROV (N)
            - vx_boat_cmd : vitesse commande bateau (m/s)
            - dL_dt : variation longueur câble (m/s)
        
        Returns:
        --------
        array
            Dérivées dy/dt
        """
        if not hasattr(self, "_tension_guard_banner_printed"):
            self._tension_guard_banner_printed = True
            trace_print(
                5,
                f"[DEBUG] compute_derivatives build marker: {TENSION_GUARD_BUILD_ID}",
            )

        # Dépaqueter l'état
        # IMPORTANT: La longueur L est extraite de l'état à chaque itération
        # Elle évolue selon dL_dt = u['dL_dt'] et est transmise au solveur
        (x_rov, y_rov, vx_rov, vy_rov, x_boat, vx_boat,
         x_cable, y_cable, T, L) = self.unpack_state(y)

        T_cap = self._abs_tension_cap_n()
        T = np.clip(np.asarray(T, dtype=float), 0.0, T_cap)

        # Vérifications de validité optimisées (seulement les valeurs critiques)
        # Vérifier seulement les premières valeurs (ROV et longueur) pour éviter le ralentissement
        if not np.isfinite(x_rov) or not np.isfinite(y_rov) or not np.isfinite(vx_rov) or not np.isfinite(vy_rov):
            raise ValueError(
                f"État ROV invalide (NaN/Inf) dans compute_derivatives à t={t:.2f} s "
                f"[source=compute_derivatives.precheck] "
                f"x_rov={x_rov}, y_rov={y_rov}, vx_rov={vx_rov}, vy_rov={vy_rov}, L={L}"
            )
        if L <= 0 or not np.isfinite(L):
            raise ValueError(f"Longueur de câble invalide: L={L} à t={t:.2f} s")
        
        # Contraintes de surface : le bateau reste à la surface (y=0), le ROV ne dépasse pas la surface (y<=0)
        # Convention : y < 0 = profondeur (sous la surface), y = 0 = surface
        # Si le ROV est au-dessus de la surface, on le ramène à la surface et on annule sa vitesse vers le haut
        if y_rov > 0.0:
            y_rov = 0.0
            if vy_rov > 0.0:
                vy_rov = 0.0

        vx_rov_f = float(
            np.clip(vx_rov, -V_MAX_ROV_VELOCITY_FORCES_MS, V_MAX_ROV_VELOCITY_FORCES_MS)
        )
        vy_rov_f = float(
            np.clip(vy_rov, -V_MAX_ROV_VELOCITY_FORCES_MS, V_MAX_ROV_VELOCITY_FORCES_MS)
        )

        x_rov_state = float(x_rov)
        y_rov_state = float(y_rov)

        # Géométrie : figée pendant les sous-pas (pas de solveur non linéaire dans f).
        # Le rafraîchissement a lieu une fois par pas accepté (refresh_cable_equilibrium_inplace).
        try:
            frozen = bool(self._freeze_cable_geometry) and len(x_cable) == self.N + 1
            if frozen:
                x_cable_new = np.asarray(x_cable, dtype=float).copy()
                y_cable_new = np.asarray(y_cable, dtype=float).copy()
                x_cable_new[0] = float(x_boat)
                y_cable_new[0] = 0.0
                x_cable_new[-1] = float(x_rov)
                y_cable_new[-1] = float(y_rov)
                weight_per_unit = (
                    (self.cable.rho_cable - self.environment.rho_eau)
                    * self.cable.A_cable
                    * self.environment.g
                )
                T_new = self.cable.solver._compute_catenary_tensions(
                    x_cable_new,
                    y_cable_new,
                    weight_per_unit,
                    rov_m=self.rov.m,
                    rov_vol=self.rov.V,
                )
            else:
                use_static_cable = (
                    not self.dynamic_simulation
                    or self.x_cable_prev is None
                    or self.y_cable_prev is None
                )
                if use_static_cable:
                    x_cable_new, y_cable_new, T_new = self.cable.solve_equilibrium(
                        x_rov, y_rov, vx_rov_f, vy_rov_f,
                        x_boat, vx_boat, L,
                        None, None,
                        rov_m=self.rov.m, rov_vol=self.rov.V,
                        force_static=True,
                        use_current_geometry=self.use_current_geometry_for_static,
                    )
                else:
                    x_cable_new, y_cable_new, T_new = self.cable.solve_equilibrium(
                        x_rov, y_rov, vx_rov_f, vy_rov_f,
                        x_boat, vx_boat, L,
                        self.x_cable_prev, self.y_cable_prev,
                        rov_m=self.rov.m, rov_vol=self.rov.V,
                    )
        except Exception as e:
            raise RuntimeError(f"Erreur lors de la résolution du câble à t={t:.2f} s: {str(e)}")
        
        # Vérifications de validité optimisées (seulement les extrémités du câble)
        # Vérifier seulement les extrémités pour éviter le ralentissement
        if len(x_cable_new) > 0 and len(y_cable_new) > 0 and len(T_new) > 0:
            if (not np.isfinite(x_cable_new[0]) or not np.isfinite(y_cable_new[0]) or not np.isfinite(T_new[0]) or
                not np.isfinite(x_cable_new[-1]) or not np.isfinite(y_cable_new[-1]) or not np.isfinite(T_new[-1])):
                raise ValueError(f"Résultats invalides du solveur de câble à t={t:.2f} s")

        # Bateau : recollage explicite. Extrémité ROV : géométrie du solveur (normalize straight peut rapprocher le ROV).
        if len(x_cable_new) > 0 and len(y_cable_new) > 0:
            x_cable_new[0] = x_boat
            y_cable_new[0] = 0.0
            x_rov = float(x_cable_new[-1])
            y_rov = float(y_cable_new[-1])

        # La surface n'est pas imposée par un clip ici : un clip change la longueur
        # au milieu du second membre. Le rappel est porté par la vitesse nodale
        # (smooth_vertical_node_velocity) ; le recollement exact est la projection
        # de fin de pas.

        if len(x_cable_new) > 0:
            # Vérifier si le câble est vertical : tous les points doivent avoir la même position x
            x_cable_range = np.max(x_cable_new) - np.min(x_cable_new)
            if x_cable_range < 1e-6:
                # Câble vertical : forcer tous les points à la même position x
                # Utiliser la moyenne pour éviter les erreurs d'arrondi
                x_avg = np.mean(x_cable_new)
                x_cable_new[:] = x_avg

        # Diagnostic de cohérence solveur <-> état ODE:
        # la géométrie retournée par le solveur (x_cable_new, y_cable_new) devrait rester
        # proche de la géométrie portée par l'état intégré (x_cable, y_cable).
        try:
            if (
                len(x_cable_new) == len(x_cable)
                and len(y_cable_new) == len(y_cable)
                and len(x_cable_new) > 2
            ):
                dx_err = np.asarray(x_cable_new, dtype=float) - np.asarray(x_cable, dtype=float)
                dy_err = np.asarray(y_cable_new, dtype=float) - np.asarray(y_cable, dtype=float)
                geom_err_rms = float(np.sqrt(np.mean(dx_err**2 + dy_err**2)))
                geom_err_max = float(np.max(np.sqrt(dx_err**2 + dy_err**2)))
                if geom_err_rms > 0.35 or geom_err_max > 0.8:
                    trace_print(
                        5,
                        (
                            "[DEBUG] compute_derivatives: écart géométrie solveur/ODE élevé "
                            f"à t={t:.2f}s (rms={geom_err_rms:.3f} m, max={geom_err_max:.3f} m)"
                        ),
                    )
        except Exception:
            pass

        T_new = np.clip(np.asarray(T_new, dtype=float), 0.0, T_cap)

        # Mettre à jour pour la prochaine itération
        self.x_cable_prev = np.asarray(x_cable_new, dtype=float).copy()
        self.y_cable_prev = np.asarray(y_cable_new, dtype=float).copy()

        # T_new[0] = tension au bateau, T_new[-1] = tension au ROV
        T_rov_target_static = float(T_new[-1]) if len(T_new) > 0 else 0.0

        Fx_drag_rov_temp, Fy_drag_rov_temp = self.rov.compute_drag_force(
            vx_rov_f, vy_rov_f, y_rov, self.environment
        )
        F_buoyancy_temp = self.rov.compute_buoyancy_force(self.environment)
        F_weight_temp = self.rov.compute_weight_force(self.environment)
        F_apparent_weight_temp = F_buoyancy_temp - F_weight_temp

        dx_straight_curr = x_rov - x_boat
        dy_straight_curr = y_rov - 0.0
        L_straight_curr = np.hypot(dx_straight_curr, dy_straight_curr)
        slack_curr = L - L_straight_curr
        slack_ratio_curr = slack_curr / max(L_straight_curr, 1.0)

        # Pénalité quadratique du mou négatif, ajoutée à la cible puis relaxée.
        # La tension appliquée au ROV reste toujours l'état T[-1].
        penalty = taut_penalty_newton(T_rov_target_static, slack_curr, L)
        T_rov_target = float(np.clip(T_rov_target_static + penalty, 0.0, T_cap))
        if len(T_new) > 0:
            T_new = np.asarray(T_new, dtype=float).copy()
            T_new[-1] = T_rov_target
        T_rov = float(T[-1]) if len(T) > 0 else T_rov_target
        T_rov = float(np.clip(T_rov, 0.0, T_cap))

        # Calculer le vecteur unitaire au niveau du ROV (direction vers le bateau)
        # CORRECTION: Après correction de _compute_catenary_tensions :
        # Les positions sont ordonnées : index 0 = bateau, index -1 = ROV
        # Direction de traction du câble sur le ROV = vers le bateau
        
        # Vérifier si le câble est vertical : tous les points doivent avoir la même position x
        is_vertical = False
        if len(x_cable_new) > 1:
            x_cable_range = np.max(x_cable_new) - np.min(x_cable_new)
            is_vertical = x_cable_range < 1e-6
        
        if is_vertical:
            # Câble vertical : pas de composante horizontale
            # La force de tension tire le ROV vers le haut (vers le bateau)
            cos_theta0 = 0.0
            sin_theta0 = 1.0  # Vers le haut (y positif, vers le bateau)
        elif len(x_cable_new) > 1:
            # Direction vers le bateau, lissée sur quelques segments proches du ROV
            # pour réduire l'injection de pics due à un coude très local.
            if len(x_cable_new) > 3:
                vx_sum = 0.0
                vy_sum = 0.0
                weights = (0.6, 0.3, 0.1)
                n_used = min(3, len(x_cable_new) - 1)
                for j in range(1, n_used + 1):
                    idx_tail = -j
                    idx_head = -j - 1
                    vx = float(x_cable_new[idx_head] - x_cable_new[idx_tail])
                    vy = float(y_cable_new[idx_head] - y_cable_new[idx_tail])
                    seg_norm = float(np.hypot(vx, vy))
                    if seg_norm > 1e-9:
                        w = weights[j - 1]
                        vx_sum += w * (vx / seg_norm)
                        vy_sum += w * (vy / seg_norm)
                if abs(vx_sum) + abs(vy_sum) > 1e-9:
                    dx_rov = vx_sum
                    dy_rov = vy_sum
                else:
                    dx_rov = float(x_cable_new[-2] - x_cable_new[-1])
                    dy_rov = float(y_cable_new[-2] - y_cable_new[-1])
            elif len(x_cable_new) > 2:
                dx_rov = x_cable_new[-2] - x_cable_new[-1]  # Vers le bateau
                dy_rov = y_cable_new[-2] - y_cable_new[-1]  # Vers le bateau
            else:
                dx_rov = x_cable_new[0] - x_cable_new[-1]  # Vers le bateau
                dy_rov = y_cable_new[0] - y_cable_new[-1]  # Vers le bateau

            ds_rov = np.sqrt(dx_rov**2 + dy_rov**2)
            if ds_rov > 1e-6:
                # Si dx_rov est très petit, forcer cos_theta0 = 0 pour éviter les erreurs numériques
                if abs(dx_rov) < 1e-6:
                    cos_theta0 = 0.0
                    sin_theta0 = 1.0 if dy_rov > 0 else -1.0
                else:
                    cos_theta0 = dx_rov / ds_rov
                    sin_theta0 = dy_rov / ds_rov
            else:
                cos_theta0 = 0.0  # Câble vertical
                sin_theta0 = 1.0  # Vers le haut
        else:
            cos_theta0 = 0.0  # Câble vertical
            sin_theta0 = 1.0  # Vers le haut
        
        # Forces sur le ROV (déjà calculées plus haut pour T_rov_target, réutiliser les valeurs)
        Fx_drag_rov = Fx_drag_rov_temp
        Fy_drag_rov = Fy_drag_rov_temp
        F_buoyancy = F_buoyancy_temp
        F_weight = F_weight_temp
        F_apparent_weight = F_apparent_weight_temp
        
        # CORRECTION: Le solveur de câble calcule une tension au ROV qui équilibre la force apparente
        # à l'équilibre statique. Quand le câble est vertical, T_rov ≈ |F_apparent_weight| et sin_theta0 ≈ 1.0,
        # donc T_rov * sin_theta0 ≈ |F_apparent_weight|, ce qui annule exactement F_apparent_weight dans
        # l'équation du mouvement, empêchant tout mouvement vertical même quand une force est appliquée.
        #
        # Solution: La tension du câble T_rov * sin_theta0 équilibre déjà le poids apparent à l'équilibre.
        # Dans l'équation du mouvement, nous ne devons pas compter deux fois le poids apparent.
        # La solution correcte est de ne pas inclure F_apparent_weight dans l'équation du mouvement
        # si la tension du câble l'équilibre déjà. La tension du câble est une force de réaction
        # qui s'adapte aux forces appliquées, donc elle équilibre le poids apparent plus les forces appliquées.
        #
        # Cependant, pour permettre le mouvement dynamique, nous devons permettre que la tension
        # s'adapte aux forces appliquées. La tension effective doit équilibrer le poids apparent
        # plus une partie des forces appliquées pour permettre l'accélération.
        
        # CORRECTION: Utiliser la tension dynamique T_rov de l'état actuel au lieu de T_rov_target du solveur.
        # La tension T_rov évolue dynamiquement selon dT_dt = (T_new - T) / tau_tension,
        # ce qui permet l'adaptation aux forces appliquées et au mouvement.
        # Le solveur calcule T_rov_target comme référence pour l'équilibre statique, mais T_rov évolue
        # selon les forces réelles, permettant ainsi le mouvement vertical.
        #
        # Avec cette correction, T[0] peut s'adapter aux forces appliquées et ne pas équilibrer
        # exactement F_apparent_weight, ce qui permet l'accélération verticale.
        
        # Équations du ROV
        # La tension T_rov évolue dynamiquement et peut s'adapter aux forces appliquées.
        # F_apparent_weight est toujours inclus car T_rov peut ne pas l'équilibrer exactement
        # dans le mouvement dynamique, permettant ainsi l'accélération.
        dvx_rov_dt = (Fx_drag_rov + T_rov * cos_theta0 + u['Fx_rov']) / self.rov.m
        
        # Calculer la force verticale totale avec la tension actuelle
        # La tension T_rov évolue dynamiquement vers T_rov_target via dT_dt
        # Si T_rov_target a été réduit (ROV remonte alors qu'il devrait descendre),
        # la tension s'adaptera progressivement, permettant le mouvement
        Fy_total = F_apparent_weight + Fy_drag_rov + T_rov * sin_theta0 + u['Fy_rov']
        dvy_rov_dt = Fy_total / self.rov.m

        # Contrainte de surface : y_rov <= 0 (profondeur négative)
        # Si le ROV est à la surface et remonte, ajouter une force de réaction du sol
        # Cette force a une origine physique : la réaction du sol/surface de l'eau
        if y_rov >= 0.0:
            if vy_rov > 0.0:
                # Force de réaction du sol : empêche le ROV de remonter au-dessus de la surface
                # Cette force est proportionnelle à la vitesse pour créer une décélération
                Fy_surface_reaction = -self.rov.m * 10.0 * vy_rov_f  # Force de réaction
                Fy_total = Fy_total + Fy_surface_reaction
                dvy_rov_dt = Fy_total / self.rov.m
            if dvy_rov_dt > 0.0:
                # Si l'accélération est encore positive, forcer à zéro (contact avec la surface)
                Fy_surface_reaction = -self.rov.m * dvy_rov_dt
                Fy_total = Fy_total + Fy_surface_reaction
                dvy_rov_dt = Fy_total / self.rov.m

        dvx_rov_dt = float(np.clip(dvx_rov_dt, -A_MAX_ROV_MS2, A_MAX_ROV_MS2))
        dvy_rov_dt = float(np.clip(dvy_rov_dt, -A_MAX_ROV_MS2, A_MAX_ROV_MS2))

        # Câble plus court que la corde : l'accélération s'éteint continûment
        # sur une bande de 2 % de L, au lieu d'un arrêt sec à slack = 0.
        accel_scale = rov_accel_scale(slack_curr, L)
        dvx_rov_dt *= accel_scale
        dvy_rov_dt *= accel_scale

        # Vérifier si le câble est vertical : tous les points doivent avoir la même position x
        is_vertical_boat = False
        if len(x_cable_new) > 1:
            x_cable_range = np.max(x_cable_new) - np.min(x_cable_new)
            is_vertical_boat = x_cable_range < 1e-6
        
        if is_vertical_boat:
            # Câble vertical : pas de composante horizontale
            cos_theta_L = 0.0
        elif len(x_cable_new) > 1:
            # Après inversion, y_cable_new[0] = 0.0 (bateau), donc on calcule depuis l'index 0
            # Direction depuis le bateau (index 0) vers le point suivant (index 1)
            dx_cable_L = x_cable_new[1] - x_cable_new[0]
            dy_cable_L = y_cable_new[1] - y_cable_new[0]
            ds_L = np.sqrt(dx_cable_L**2 + dy_cable_L**2)
            if ds_L > 1e-6:
                # Si dx_cable_L est très petit, forcer cos_theta_L = 0 pour éviter les erreurs numériques
                if abs(dx_cable_L) < 1e-6:
                    cos_theta_L = 0.0
                else:
                    cos_theta_L = dx_cable_L / ds_L
            else:
                cos_theta_L = 0.0  # Câble vertical
        else:
            cos_theta_L = 0.0  # Câble vertical
        
        F_prop_boat = self.boat.compute_propulsion_force(
            u['vx_boat_cmd'], vx_boat, self.environment
        )
        
        # Équation du bateau
        # L'effet de la tension du câble est négligeable
        # Le bateau évolue uniquement selon la commande de vitesse
        dvx_boat_dt = F_prop_boat / self.boat.m
        
        # Évolution de la longueur du câble
        # La longueur L est déterminée exclusivement par la longueur initiale et la commande dL_dt
        # dL_dt est intégré dans le vecteur d'état, donc L évolue automatiquement
        dL_dt = u['dL_dt']

        # La caténaire n'équilibre que le fil. Câble tendu : la réaction qui
        # retient le ROV (vitesse d'écartement = dL/dt) est transmise telle
        # quelle jusqu'au bateau. La projection de fin de pas réécrit T :
        # elle relit cette même commande.
        self._hold_fx = float(u['Fx_rov'])
        self._hold_fy = float(u['Fy_rov'])
        self._hold_dL_dt = float(dL_dt)
        if len(T_new) > 0:
            t_hold = float(np.clip(
                self.cable_hold_tension(
                    x_rov, y_rov, vx_rov_f, vy_rov_f, x_boat, L
                ),
                0.0,
                T_cap,
            ))
            if t_hold > float(T_new[-1]):
                T_new = np.asarray(T_new, dtype=float) + (t_hold - float(T_new[-1]))
                T_rov_target = float(T_new[-1])

        # Dérivées des positions du câble (vitesses)
        # Simplification : interpolation linéaire des vitesses
        vx_cable = np.linspace(vx_rov_f, vx_boat, self.N + 1)
        vy_cable = np.linspace(vy_rov_f, 0.0, self.N + 1)
        
        if len(y_cable_new) == len(vy_cable):
            vy_cable = smooth_vertical_node_velocity(y_cable_new, vy_cable)

        # Une seule loi de tension sur tout le domaine, y compris le mou négatif.
        # tau est une fonction lisse du mou relatif (0,05 s tendu -> 0,20 s mou).
        tau_tension = continuous_tau_tension(slack_ratio_curr)
        if len(T_new) > 0:
            T_new = np.asarray(T_new, dtype=float).copy()
            T_new[-1] = float(np.clip(T_rov_target, 0.0, T_cap))
        T_new = np.clip(np.asarray(T_new, dtype=float), 0.0, T_cap)

        dT_dt_abs_cap = max(800.0, 4.0 * T_cap / max(tau_tension, 0.02))
        dT_dt = np.clip((T_new - T) / tau_tension, -dT_dt_abs_cap, dT_dt_abs_cap)

        # Assembler les dérivées
        dydt = np.zeros_like(y)
        # Ramener progressivement la position ROV de l'état vers l'extrémité câble (solveur / straight)
        tau_rov_geom_sync = 0.05
        dydt[0] = vx_rov + (x_rov - x_rov_state) / tau_rov_geom_sync
        dydt[1] = vy_rov + (y_rov - y_rov_state) / tau_rov_geom_sync
        dydt[2] = dvx_rov_dt
        dydt[3] = dvy_rov_dt
        dydt[4] = vx_boat  # dx_boat/dt
        dydt[5] = dvx_boat_dt
        
        idx = 6
        dydt[idx:idx+self.N+1] = vx_cable  # dx_cable/dt
        idx += self.N + 1
        dydt[idx:idx+self.N+1] = vy_cable  # dy_cable/dt
        idx += self.N + 1
        dydt[idx:idx+self.N+1] = dT_dt  # dT/dt
        idx += self.N + 1
        dydt[idx] = dL_dt  # dL/dt
        
        return dydt

    def _cable_in_direction(self, x_rov, y_rov, x_boat):
        dx = float(x_rov) - float(x_boat)
        dy = float(y_rov)
        dist = float(np.hypot(dx, dy))
        if dist <= 1e-6 or not np.isfinite(dist):
            return None
        return -dx / dist, -dy / dist, dist

    def taut_rov_velocity(self, x_rov, y_rov, vx, vy, x_boat, L):
        """
        Vitesse d'état compatible avec le câble tendu.

        L'écartement est celui du dévidage. Un câble encore mou laisse la
        vitesse inchangée.
        """
        direction = self._cable_in_direction(x_rov, y_rov, x_boat)
        if direction is None:
            return float(vx), float(vy)
        cos_in, sin_in, dist = direction
        return payout_matched_velocity(
            vx,
            vy,
            cos_in,
            sin_in,
            float(self._hold_dL_dt),
            float(L) - dist,
        )

    def cable_hold_tension(self, x_rov, y_rov, vx, vy, x_boat, L) -> float:
        """
        Tension au ROV exigée par le câble inextensible déjà tendu.

        Les forces du ROV n'entrent pas dans la caténaire du fil. La traînée
        est évaluée à la vitesse imposée par le dévidage : c'est celle du
        mouvement réel une fois le câble droit, et c'est elle que la tension
        doit équilibrer.
        """
        direction = self._cable_in_direction(x_rov, y_rov, x_boat)
        if direction is None:
            return 0.0
        cos_in, sin_in, dist = direction
        slack = float(L) - dist
        vx_e, vy_e = payout_matched_velocity(
            vx, vy, cos_in, sin_in, float(self._hold_dL_dt), slack
        )
        fx_drag, fy_drag = self.rov.compute_drag_force(
            float(vx_e), float(vy_e), float(y_rov), self.environment
        )
        f_apparent = (
            self.rov.compute_buoyancy_force(self.environment)
            - self.rov.compute_weight_force(self.environment)
        )
        return taut_constraint_tension(
            float(fx_drag) + float(self._hold_fx),
            float(f_apparent) + float(fy_drag) + float(self._hold_fy),
            cos_in,
            sin_in,
            float(vx_e),
            float(vy_e),
            float(self._hold_dL_dt),
            slack,
            self.rov.m,
        )

    def refresh_cable_equilibrium_inplace(self, y):
        """
        Un seul appel au solveur non linéaire du câble.

        Écrit la géométrie dans ``y`` (extrémités collées au bateau et au ROV).
        Les tensions intégrées ne sont pas écrasées ici : la projection de fin
        de pas les recalcule sur la géométrie normalisée.
        """
        y_arr = np.asarray(y, dtype=float)
        try:
            (
                x_rov,
                y_rov,
                vx_rov,
                vy_rov,
                x_boat,
                vx_boat,
                _x_cable,
                _y_cable,
                _T,
                L,
            ) = self.unpack_state(y_arr)
        except Exception:
            return False
        if L is None or not np.isfinite(L) or float(L) <= 0.0:
            return False
        force_static = (
            not self.dynamic_simulation
            or self.x_cable_prev is None
            or self.y_cable_prev is None
        )
        try:
            x_new, y_new, _t_new = self.cable.solve_equilibrium(
                float(x_rov),
                float(y_rov),
                float(vx_rov),
                float(vy_rov),
                float(x_boat),
                float(vx_boat),
                float(L),
                None if force_static else self.x_cable_prev,
                None if force_static else self.y_cable_prev,
                rov_m=self.rov.m,
                rov_vol=self.rov.V,
                force_static=force_static,
                use_current_geometry=self.use_current_geometry_for_static,
            )
        except Exception:
            return False
        x_new = np.asarray(x_new, dtype=float).reshape(-1).copy()
        y_new = np.asarray(y_new, dtype=float).reshape(-1).copy()
        if x_new.size != self.N + 1 or y_new.size != self.N + 1:
            return False
        if not np.all(np.isfinite(x_new)) or not np.all(np.isfinite(y_new)):
            return False
        x_new[0] = float(x_boat)
        y_new[0] = 0.0
        x_new[-1] = float(x_rov)
        y_new[-1] = float(y_rov)
        idx_x = 6
        idx_y = idx_x + self.N + 1
        y_arr[idx_x:idx_y] = x_new
        y_arr[idx_y : idx_y + self.N + 1] = y_new
        self.x_cable_prev = x_new.copy()
        self.y_cable_prev = y_new.copy()
        return True

    def integrate(self, t_span, y0, u_func, dt_max=0.1):
        """
        Intègre par pas de durée au plus ``dt_max``.

        Chaque pas est un RK4 à géométrie figée, suivi d'un appel au solveur
        de câble et d'une projection de longueur. Le résultat expose les mêmes
        attributs qu'une solution ``solve_ivp`` (``success``, ``sol``, ``t``, ``y``).
        """
        from ..solvers.projected_integrator import (
            TabulatedOdeSolution,
            integrate_span_with_projection,
        )

        t0, tf = float(t_span[0]), float(t_span[1])
        y = np.asarray(y0, dtype=float).copy()
        dt_max = max(float(dt_max), 1e-4)
        self.integrator.max_step = dt_max
        times = [t0]
        states = [y.copy()]
        if tf <= t0:
            return TabulatedOdeSolution(times, states, message="intervalle vide")
        t = t0
        while t < tf - 1e-12:
            step = min(dt_max, tf - t)
            y = integrate_span_with_projection(
                self,
                (t, t + step),
                y,
                u_func,
                n_substeps=2,
                project=True,
            )
            t = t + step
            times.append(t)
            states.append(np.asarray(y, dtype=float).copy())
            if not np.all(np.isfinite(y)):
                return TabulatedOdeSolution(
                    times,
                    states,
                    success=False,
                    message="état non fini",
                )
        return TabulatedOdeSolution(
            times,
            states,
            message=(
                "géométrie de câble figée pendant le pas ; "
                "solveur et projection une fois en fin de pas"
            ),
        )

    def compute_constraint_residuals(self, y):
        """
        Métriques / résidus des contraintes géométriques câble (longueur, surface, segments).
        Voir docs/dae_cable_rov_spec.md.
        """
        from .state_projection import compute_constraint_residuals as _ccr

        return _ccr(self, np.asarray(y, dtype=float))

    def integrate_span_with_projection(
        self,
        t_span,
        y0,
        u_func,
        dt_max=0.1,
        *,
        n_substeps=4,
        k_tail=10,
    ):
        """
        Intégration half-explicit : RK4 à géométrie figée, puis un solveur de câble
        et une projection de longueur en fin d'intervalle.

        Paramètres
        ----------
        dt_max : float
            Conservé pour homogénéité avec integrate() ; le pas effectif est |t1-t0|/n_substeps.
        n_substeps : int
            Nombre de sous-pas RK4 sur l'intervalle t_span.
        k_tail : int
            Paramètre k_tail passé à la normalisation du câble.
        """
        from ..solvers.projected_integrator import integrate_span_with_projection_simple

        return integrate_span_with_projection_simple(
            self,
            tuple(t_span),
            np.asarray(y0, dtype=float),
            u_func,
            float(dt_max),
            n_substeps=int(n_substeps),
            k_tail=int(k_tail),
        )

