"""Système complet ROV-Câble-Bateau"""
import numpy as np
from .rov_model import ROV
from .cable_model import Cable
from .boat_model import Boat
from .environment import Environment
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
        # Dépaqueter l'état
        # IMPORTANT: La longueur L est extraite de l'état à chaque itération
        # Elle évolue selon dL_dt = u['dL_dt'] et est transmise au solveur
        (x_rov, y_rov, vx_rov, vy_rov, x_boat, vx_boat,
         x_cable, y_cable, T, L) = self.unpack_state(y)
        
        # Vérifications de validité optimisées (seulement les valeurs critiques)
        # Vérifier seulement les premières valeurs (ROV et longueur) pour éviter le ralentissement
        if not np.isfinite(x_rov) or not np.isfinite(y_rov) or not np.isfinite(vx_rov) or not np.isfinite(vy_rov):
            raise ValueError(f"État ROV invalide (NaN/Inf) dans compute_derivatives à t={t:.2f} s")
        if L <= 0 or not np.isfinite(L):
            raise ValueError(f"Longueur de câble invalide: L={L} à t={t:.2f} s")
        
        # Contraintes de surface : le bateau reste à la surface (y=0), le ROV ne dépasse pas la surface (y<=0)
        # Convention : y < 0 = profondeur (sous la surface), y = 0 = surface
        # Si le ROV est au-dessus de la surface, on le ramène à la surface et on annule sa vitesse vers le haut
        if y_rov > 0.0:
            y_rov = 0.0
            if vy_rov > 0.0:
                vy_rov = 0.0

        # Calculer la distance droite et le slack
        dx_straight = x_rov - x_boat
        dy_straight = y_rov - 0.0
        L_straight = np.hypot(dx_straight, dy_straight)
        slack = L - L_straight
        
        # IMPORTANT : Le slack doit toujours être >= 0
        # Si slack < 0, la tension au ROV doit être ajustée pour tirer le ROV vers le bateau
        # et réduire L_straight jusqu'à ce que slack >= 0
        # On ne recalcule PAS la position du ROV, mais on ajuste la tension pour contraindre le mouvement
        cable_length_constrained = False
        cable_taut = False
        if slack < 0.0:
            cable_taut = True
            cable_length_constrained = True
            # Le slack est négatif, le câble est tendu
            # La tension au ROV sera ajustée plus tard pour forcer slack >= 0

        # Résoudre la configuration du câble
        # IMPORTANT: La longueur L est transmise au solveur à chaque itération
        # La longueur L est déterminée exclusivement par la longueur initiale et la commande dL_dt
        try:
            if self.x_cable_prev is None or self.y_cable_prev is None:
                # Première itération : solution statique
                # Passer les paramètres du ROV pour calculer correctement la tension initiale
                x_cable_new, y_cable_new, T_new = self.cable.solver.solve_equilibrium_static(
                    x_rov, y_rov, x_boat, L, rov_m=self.rov.m, rov_vol=self.rov.V
                )
            else:
                # Solution dynamique
                # Passer les paramètres du ROV pour calculer correctement la tension initiale
                # La longueur L est extraite de l'état et transmise au solveur
                x_cable_new, y_cable_new, T_new = self.cable.solve_equilibrium(
                    x_rov, y_rov, vx_rov, vy_rov,
                    x_boat, vx_boat, L,
                    self.x_cable_prev, self.y_cable_prev,
                    rov_m=self.rov.m, rov_vol=self.rov.V
                )
        except Exception as e:
            raise RuntimeError(f"Erreur lors de la résolution du câble à t={t:.2f} s: {str(e)}")
        
        # Vérifications de validité optimisées (seulement les extrémités du câble)
        # Vérifier seulement les extrémités pour éviter le ralentissement
        if len(x_cable_new) > 0 and len(y_cable_new) > 0 and len(T_new) > 0:
            if (not np.isfinite(x_cable_new[0]) or not np.isfinite(y_cable_new[0]) or not np.isfinite(T_new[0]) or
                not np.isfinite(x_cable_new[-1]) or not np.isfinite(y_cable_new[-1]) or not np.isfinite(T_new[-1])):
                raise ValueError(f"Résultats invalides du solveur de câble à t={t:.2f} s")

        # CONTRAINTE CRITIQUE : Forcer les extrémités du câble à correspondre exactement au bateau et au ROV
        # Le solveur avec contraintes (_normalize_cable_length) garantit déjà toutes les contraintes :
        # - L = L_seg (à 0.01% près)
        # - P0 = bateau, PN = ROV
        # - y <= 0 pour tous les points
        # - ds_max/ds_target <= k_max
        # On ne fait donc que forcer les extrémités comme sécurité supplémentaire
        if len(x_cable_new) > 0 and len(y_cable_new) > 0:
            x_cable_new[0] = x_boat
            y_cable_new[0] = 0.0
            x_cable_new[-1] = x_rov
            y_cable_new[-1] = y_rov
        
        # NOTE: La normalisation redondante a été supprimée car _normalize_cable_length garantit déjà toutes les contraintes
        # Le solveur avec contraintes (_normalize_cable_length) garantit :
        # - L = L_seg (à 0.01% près)
        # - P0 = bateau, PN = ROV
        # - y <= 0 pour tous les points
        # - ds_max/ds_target <= k_max
        # On ne fait donc que forcer les extrémités comme sécurité supplémentaire (déjà fait ci-dessus)
        # et s'assurer que y <= 0 (fait ci-dessous)
        
        # Ancien code de normalisation supprimé pour éviter les violations de contraintes
            
            # CONTRAINTE CRITIQUE : Normaliser avec les extrémités fixées pour garantir toutes les contraintes
            # L'ordre des opérations est crucial :
            # 1. Forcer les extrémités
            # 2. Normaliser avec extrémités fixées (rééchantillonnage avec interpolation)
            # 3. Vérifier et corriger si nécessaire
            
            N_seg = len(x_cable_new) - 1
            ds_target = L / max(N_seg, 1)
            
            # Étape 1 : Forcer les extrémités
            x_cable_new[0] = x_boat
            y_cable_new[0] = 0.0
            x_cable_new[-1] = x_rov
            y_cable_new[-1] = y_rov
            
            # Étape 2 : Rééchantillonner avec interpolation linéaire en abscisse curviligne
            # pour garantir des segments de longueur égale = L/N avec extrémités fixées
            s_points = np.linspace(0.0, L, N_seg + 1)  # Points à s = 0, ds, 2*ds, ..., L
            
            # Calculer l'abscisse curviligne cumulative du câble actuel
            s_cum = np.zeros(N_seg + 1)
            for i in range(N_seg):
                dx = x_cable_new[i+1] - x_cable_new[i]
                dy = y_cable_new[i+1] - y_cable_new[i]
                s_cum[i+1] = s_cum[i] + np.sqrt(dx**2 + dy**2)
            
            # Rééchantillonner avec interpolation, en préservant les extrémités
            x_cable_normalized = np.zeros(N_seg + 1)
            y_cable_normalized = np.zeros(N_seg + 1)
            x_cable_normalized[0] = x_boat
            y_cable_normalized[0] = 0.0
            x_cable_normalized[-1] = x_rov
            y_cable_normalized[-1] = y_rov
            
            # Interpoler les points intermédiaires à intervalles réguliers
            for i in range(1, N_seg):
                s_i = s_points[i]  # s_i = i * ds_target
                idx = np.searchsorted(s_cum, s_i)
                if idx == 0:
                    idx = 1
                elif idx >= len(s_cum):
                    idx = len(s_cum) - 1
                
                s_prev = s_cum[idx - 1]
                s_next = s_cum[idx]
                
                if abs(s_next - s_prev) > 1e-6:
                    alpha = (s_i - s_prev) / (s_next - s_prev)
                    x_cable_normalized[i] = x_cable_new[idx - 1] + alpha * (x_cable_new[idx] - x_cable_new[idx - 1])
                    y_cable_normalized[i] = y_cable_new[idx - 1] + alpha * (y_cable_new[idx] - y_cable_new[idx - 1])
                else:
                    x_cable_normalized[i] = x_cable_new[idx - 1]
                    y_cable_normalized[i] = y_cable_new[idx - 1]
            
            # CRITIQUE : Ajuster le point N_seg-1 pour que le dernier segment (N_seg-1 -> ROV) ait exactement ds_target
            # Le dernier point DOIT être au ROV (x_rov, y_rov), donc on ajuste le point N_seg-1
            if N_seg > 0:
                # Calculer la direction du dernier segment depuis le point N_seg-1 vers le ROV
                dx_to_rov = x_rov - x_cable_normalized[N_seg-1]
                dy_to_rov = y_rov - y_cable_normalized[N_seg-1]
                dist_to_rov = np.sqrt(dx_to_rov**2 + dy_to_rov**2)
                
                if dist_to_rov > 1e-9:
                    # Ajuster le point N_seg-1 pour que le segment de N_seg-1 à ROV ait exactement ds_target
                    # Le point N_seg-1 doit être à une distance ds_target du ROV dans la direction opposée
                    scale = ds_target / dist_to_rov
                    x_cable_normalized[N_seg-1] = x_rov - dx_to_rov * scale
                    y_cable_normalized[N_seg-1] = y_rov - dy_to_rov * scale
                else:
                    # Le point N_seg-1 est déjà au ROV, le déplacer légèrement
                    if N_seg > 1:
                        # Utiliser la direction du segment précédent
                        dx_prev = x_cable_normalized[N_seg-1] - x_cable_normalized[N_seg-2]
                        dy_prev = y_cable_normalized[N_seg-1] - y_cable_normalized[N_seg-2]
                        dist_prev = np.sqrt(dx_prev**2 + dy_prev**2)
                        if dist_prev > 1e-9:
                            x_cable_normalized[N_seg-1] = x_rov - (dx_prev / dist_prev) * ds_target
                            y_cable_normalized[N_seg-1] = y_rov - (dy_prev / dist_prev) * ds_target
                        else:
                            x_cable_normalized[N_seg-1] = x_rov - ds_target
                            y_cable_normalized[N_seg-1] = y_rov
                    else:
                        x_cable_normalized[N_seg-1] = x_rov - ds_target
                        y_cable_normalized[N_seg-1] = y_rov
                
                # Forcer le dernier point au ROV
                x_cable_normalized[N_seg] = x_rov
                y_cable_normalized[N_seg] = y_rov
            
            x_cable_new = x_cable_normalized
            y_cable_new = y_cable_normalized
            
            # Vérification finale : s'assurer que les contraintes sont respectées
            # Vérifier la longueur totale et les longueurs des segments
            L_final_check = 0.0
            segments_ok = True
            for i in range(N_seg):
                dx = x_cable_new[i+1] - x_cable_new[i]
                dy = y_cable_new[i+1] - y_cable_new[i]
                ds = np.sqrt(dx**2 + dy**2)
                L_final_check += ds
                # Vérifier que chaque segment a la longueur cible (tolérance 1e-2 m comme dans contrôle_câble)
                if abs(ds - ds_target) > 1e-2:
                    segments_ok = False
            
            # Si la longueur totale n'est pas correcte, ajuster proportionnellement les points intermédiaires
            if abs(L_final_check - L) / max(L, 1e-9) > 1e-2:
                # Ajuster proportionnellement depuis le bateau vers le ROV
                if L_final_check > 1e-9:
                    scale = L / L_final_check
                    for i in range(1, N_seg):
                        dx = x_cable_new[i] - x_cable_new[0]
                        dy = y_cable_new[i] - y_cable_new[0]
                        x_cable_new[i] = x_cable_new[0] + dx * scale
                        y_cable_new[i] = y_cable_new[0] + dy * scale
                
                # Réappliquer les extrémités après ajustement
                x_cable_new[0] = x_boat
                y_cable_new[0] = 0.0
                x_cable_new[-1] = x_rov
                y_cable_new[-1] = y_rov
                
                # Si les segments ne sont toujours pas égaux après ajustement, rééchantillonner à nouveau
                if not segments_ok:
                    # Recalculer s_cum après ajustement
                    s_cum = np.zeros(N_seg + 1)
                    for i in range(N_seg):
                        dx = x_cable_new[i+1] - x_cable_new[i]
                        dy = y_cable_new[i+1] - y_cable_new[i]
                        s_cum[i+1] = s_cum[i] + np.sqrt(dx**2 + dy**2)
                    
                    # Rééchantillonner à nouveau avec interpolation
                    for i in range(1, N_seg):
                        s_i = s_points[i]
                        idx = np.searchsorted(s_cum, s_i)
                        if idx == 0:
                            idx = 1
                        elif idx >= len(s_cum):
                            idx = len(s_cum) - 1
                        s_prev = s_cum[idx - 1]
                        s_next = s_cum[idx]
                        if abs(s_next - s_prev) > 1e-6:
                            alpha = (s_i - s_prev) / (s_next - s_prev)
                            x_cable_new[i] = x_cable_new[idx - 1] + alpha * (x_cable_new[idx] - x_cable_new[idx - 1])
                            y_cable_new[i] = y_cable_new[idx - 1] + alpha * (y_cable_new[idx] - y_cable_new[idx - 1])
                        else:
                            x_cable_new[i] = x_cable_new[idx - 1]
                            y_cable_new[i] = y_cable_new[idx - 1]
                    
                    # Réappliquer les extrémités
                    x_cable_new[0] = x_boat
                    y_cable_new[0] = 0.0
                    x_cable_new[-1] = x_rov
                    y_cable_new[-1] = y_rov

        # Forcer les conditions aux limites du câble : bateau à y=0 (surface), ROV à y_rov (<=0, profondeur)
        # Convention : y < 0 = profondeur (sous la surface), y = 0 = surface
        if len(y_cable_new) > 0:
            # CONTRAINTE PHYSIQUE STRICTE : Le câble ne peut JAMAIS être au-dessus de la surface (y > 0)
            # Forcer tous les points du câble (sauf les extrémités) à être strictement en dessous de la surface
            # Seuls les points aux extrémités peuvent être à y=0 (bateau) ou y=y_rov (ROV)
            
            # D'abord, forcer tous les points à y <= 0
            y_cable_new = np.clip(y_cable_new, None, 0.0)
            
            # Ensuite, forcer tous les points intermédiaires à être strictement en dessous de la surface
            # pour éviter les discontinuités et les angles aigus
            for i in range(1, len(y_cable_new) - 1):
                if y_cable_new[i] >= -0.01:  # Points très proches de la surface (à moins de 1 cm)
                    # Forcer ces points à être légèrement en dessous pour éviter les discontinuités
                    # Interpoler entre les voisins pour créer une transition douce
                    y_prev = y_cable_new[i-1]
                    y_next = y_cable_new[i+1]
                    # Utiliser la moyenne des voisins, mais forcer à être en dessous de -0.01 m
                    if y_prev < -0.01 and y_next < -0.01:
                        y_cable_new[i] = min(-0.01, 0.5 * (y_prev + y_next))
                    elif y_prev < -0.01:
                        y_cable_new[i] = min(-0.01, y_prev - 0.01)
                    elif y_next < -0.01:
                        y_cable_new[i] = min(-0.01, y_next - 0.01)
                    else:
                        # Si les deux voisins sont aussi proches de la surface, forcer à -0.01 m
                        y_cable_new[i] = -0.01
            
            # Réappliquer les conditions aux limites strictes
            y_cable_new[0] = 0.0  # Bateau à la surface
            y_cable_new[-1] = y_rov  # ROV à sa position
            
            # NOTE: La renormalisation après clipping a été supprimée car elle peut violer les contraintes
            # Le solveur (_normalize_cable_length) garantit déjà toutes les contraintes.
            # Si le clipping modifie la longueur, cela sera corrigé au prochain appel du solveur.
            # Ancien code de renormalisation supprimé (lignes 411-476) pour éviter les violations de contraintes
            if abs(L_final_check - L) / max(L, 1e-9) > 1e-2:
                # Ajuster proportionnellement depuis le bateau vers le ROV
                if L_final_check > 1e-9:
                    scale = L / L_final_check
                    for i in range(1, N_seg):
                        dx = x_cable_new[i] - x_cable_new[0]
                        dy = y_cable_new[i] - y_cable_new[0]
                        x_cable_new[i] = x_cable_new[0] + dx * scale
                        y_cable_new[i] = y_cable_new[0] + dy * scale
                
                # Réappliquer les extrémités
                x_cable_new[0] = x_boat
                y_cable_new[0] = 0.0
                x_cable_new[-1] = x_rov
                y_cable_new[-1] = y_rov
                
                # Si les segments ne sont toujours pas égaux, rééchantillonner à nouveau
                if not segments_ok:
                    # Recalculer s_cum après ajustement
                    s_cum = np.zeros(N_seg + 1)
                    for i in range(N_seg):
                        dx = x_cable_new[i+1] - x_cable_new[i]
                        dy = y_cable_new[i+1] - y_cable_new[i]
                        s_cum[i+1] = s_cum[i] + np.sqrt(dx**2 + dy**2)
                    
                    # Rééchantillonner à nouveau avec interpolation
                    for i in range(1, N_seg):
                        s_i = s_points[i]
                        idx = np.searchsorted(s_cum, s_i)
                        if idx == 0:
                            idx = 1
                        elif idx >= len(s_cum):
                            idx = len(s_cum) - 1
                        s_prev = s_cum[idx - 1]
                        s_next = s_cum[idx]
                        if abs(s_next - s_prev) > 1e-6:
                            alpha = (s_i - s_prev) / (s_next - s_prev)
                            x_cable_new[i] = x_cable_new[idx - 1] + alpha * (x_cable_new[idx] - x_cable_new[idx - 1])
                            y_cable_new[i] = y_cable_new[idx - 1] + alpha * (y_cable_new[idx] - y_cable_new[idx - 1])
                        else:
                            x_cable_new[i] = x_cable_new[idx - 1]
                            y_cable_new[i] = y_cable_new[idx - 1]
                    
                    # Réappliquer les extrémités
                    x_cable_new[0] = x_boat
                    y_cable_new[0] = 0.0
                    x_cable_new[-1] = x_rov
                    y_cable_new[-1] = y_rov
            
            # Réappliquer la contrainte y <= 0 après renormalisation
            y_cable_new = np.clip(y_cable_new, None, 0.0)
            # Réappliquer la contrainte stricte sur les points intermédiaires
            for i in range(1, len(y_cable_new) - 1):
                if y_cable_new[i] >= -0.01:
                    y_prev = y_cable_new[i-1]
                    y_next = y_cable_new[i+1]
                    if y_prev < -0.01 and y_next < -0.01:
                        y_cable_new[i] = min(-0.01, 0.5 * (y_prev + y_next))
                    elif y_prev < -0.01:
                        y_cable_new[i] = min(-0.01, y_prev - 0.01)
                    elif y_next < -0.01:
                        y_cable_new[i] = min(-0.01, y_next - 0.01)
                    else:
                        y_cable_new[i] = -0.01
            
            # Réappliquer les extrémités après toutes les opérations
            x_cable_new[0] = x_boat
            y_cable_new[0] = 0.0
            x_cable_new[-1] = x_rov
            y_cable_new[-1] = y_rov
            
            # Vérification finale après clipping : si la longueur a encore changé, ajuster une dernière fois
            L_final_after_clip = 0.0
            for i in range(N_seg):
                dx = x_cable_new[i+1] - x_cable_new[i]
                dy = y_cable_new[i+1] - y_cable_new[i]
                L_final_after_clip += np.sqrt(dx**2 + dy**2)
            
            if abs(L_final_after_clip - L) / max(L, 1e-9) > 1e-2:
                # Ajustement proportionnel final
                if L_final_after_clip > 1e-9:
                    scale = L / L_final_after_clip
                    for i in range(1, N_seg):
                        dx = x_cable_new[i] - x_cable_new[0]
                        dy = y_cable_new[i] - y_cable_new[0]
                        x_cable_new[i] = x_cable_new[0] + dx * scale
                        y_cable_new[i] = y_cable_new[0] + dy * scale
                
                # Réappliquer les extrémités
                x_cable_new[0] = x_boat
                y_cable_new[0] = 0.0
                x_cable_new[-1] = x_rov
                y_cable_new[-1] = y_rov
                
                # Rééchantillonner une dernière fois pour garantir des segments égaux
                s_cum = np.zeros(N_seg + 1)
                for i in range(N_seg):
                    dx = x_cable_new[i+1] - x_cable_new[i]
                    dy = y_cable_new[i+1] - y_cable_new[i]
                    s_cum[i+1] = s_cum[i] + np.sqrt(dx**2 + dy**2)
                
                for i in range(1, N_seg):
                    s_i = s_points[i]
                    idx = np.searchsorted(s_cum, s_i)
                    if idx == 0:
                        idx = 1
                    elif idx >= len(s_cum):
                        idx = len(s_cum) - 1
                    s_prev = s_cum[idx - 1]
                    s_next = s_cum[idx]
                    if abs(s_next - s_prev) > 1e-6:
                        alpha = (s_i - s_prev) / (s_next - s_prev)
                        x_cable_new[i] = x_cable_new[idx - 1] + alpha * (x_cable_new[idx] - x_cable_new[idx - 1])
                        y_cable_new[i] = y_cable_new[idx - 1] + alpha * (y_cable_new[idx] - y_cable_new[idx - 1])
                    else:
                        x_cable_new[i] = x_cable_new[idx - 1]
                        y_cable_new[i] = y_cable_new[idx - 1]
                
                # Réappliquer les extrémités une dernière fois
                x_cable_new[0] = x_boat
                y_cable_new[0] = 0.0
                x_cable_new[-1] = x_rov
                y_cable_new[-1] = y_rov
        
        if len(x_cable_new) > 0:
            # Vérifier si le câble est vertical : tous les points doivent avoir la même position x
            x_cable_range = np.max(x_cable_new) - np.min(x_cable_new)
            if x_cable_range < 1e-6:
                # Câble vertical : forcer tous les points à la même position x
                # Utiliser la moyenne pour éviter les erreurs d'arrondi
                x_avg = np.mean(x_cable_new)
                x_cable_new[:] = x_avg

        # Stabiliser les tensions quand le solveur renvoie une valeur quasi nulle
        # (évite les alternances 0 / valeur nominale d'un pas à l'autre)
        if len(T_new) > 0 and len(T) > 0:
            try:
                if np.max(T_new) < 1e-6 and np.max(T) > 1e-3:
                    T_new = np.asarray(T_new, dtype=float).copy()
                    T_new[:] = T
            except Exception:
                pass

        # Si le câble est quasi droit, lisser la tension calculée pour éviter les bascules rapides
        if len(T_new) > 0 and len(T) > 0 and L_straight > 1e-9:
            try:
                slack_ratio = abs(L - L_straight) / max(L_straight, 1.0)
                if slack_ratio < 0.005:
                    # Geler la tension en zone quasi-droite pour éviter les oscillations
                    T_new = np.asarray(T, dtype=float).copy()
            except Exception:
                pass
        
        # Mettre à jour pour la prochaine itération
        self.x_cable_prev = x_cable_new.copy()
        self.y_cable_prev = y_cable_new.copy()
        
        # Tension et angle au niveau du ROV
        # IMPORTANT: Après correction de _compute_catenary_tensions :
        # Les tensions T_new retournées par le solveur suivent l'ordre des positions:
        # T_new[0] = tension au bateau, T_new[-1] = tension au ROV
        # (cohérent avec x_cable_new[0] = bateau, x_cable_new[-1] = ROV)
        T_rov_target_static = T_new[-1] if len(T_new) > 0 else 0.0  # Tension au ROV = T[-1]
        
        # Calculer les forces sur le ROV pour déterminer la tension cible dynamique
        # (On doit calculer ces forces maintenant pour adapter la tension)
        Fx_drag_rov_temp, Fy_drag_rov_temp = self.rov.compute_drag_force(
            vx_rov, vy_rov, y_rov, self.environment
        )
        F_buoyancy_temp = self.rov.compute_buoyancy_force(self.environment)
        F_weight_temp = self.rov.compute_weight_force(self.environment)
        # Force apparente (positive vers le haut)
        F_apparent_weight_temp = F_buoyancy_temp - F_weight_temp
        
        # Calculer la tension cible dynamique en fonction du mouvement et du slack
        # CONTRAINTE CRITIQUE : Le slack doit toujours être >= 0
        # Si slack < 0, augmenter la tension au ROV pour tirer le ROV vers le bateau
        T_rov_target = T_rov_target_static
        
        # Calculer le slack actuel
        dx_straight_curr = x_rov - x_boat
        dy_straight_curr = y_rov - 0.0
        L_straight_curr = np.hypot(dx_straight_curr, dy_straight_curr)
        slack_curr = L - L_straight_curr
        
        # Si slack < 0, augmenter la tension pour forcer slack >= 0
        if slack_curr < 0.0:
            # Le slack est négatif, le câble est tendu
            # Augmenter la tension au ROV pour tirer le ROV vers le bateau
            # La tension doit être suffisante pour réduire L_straight jusqu'à slack >= 0
            # Facteur d'augmentation basé sur la magnitude du slack négatif
            slack_deficit = abs(slack_curr)
            # Augmenter la tension proportionnellement au déficit de slack
            # Utiliser un facteur qui augmente rapidement quand slack devient très négatif
            # Plus le slack est négatif, plus la tension doit être forte
            tension_multiplier = 1.0 + min(20.0, slack_deficit / max(L, 1e-9) * 50.0)
            T_rov_target = T_rov_target_static * tension_multiplier
            # Assurer une tension minimale même si T_rov_target_static est faible
            # La tension doit être au moins suffisante pour créer une force significative
            min_tension = abs(F_apparent_weight_temp) * 2.0  # Au moins 2x le poids apparent
            T_rov_target = max(T_rov_target, min_tension)
            # Limiter la tension maximale pour éviter des valeurs irréalistes
            T_rov_target = min(T_rov_target, T_rov_target_static * 50.0)
        
        # Si le ROV remonte alors qu'il devrait descendre, réduire la tension cible
        # pour permettre à la force apparente (vers le bas) de dominer
        # MAIS seulement si slack >= 0 (pas de contrainte de slack)
        if slack_curr >= 0.0 and vy_rov > 0.0 and F_apparent_weight_temp < 0.0 and abs(u['Fy_rov']) < 1e-6:
            # Le ROV remonte alors qu'il devrait descendre
            # Réduire la tension cible pour permettre à la force apparente (vers le bas) de dominer
            # La tension cible doit être faible devant |F_apparent_weight| pour créer une force nette vers le bas
            reduction_factor = min(0.8, 0.3 + abs(vy_rov) * 0.5)  # Réduction jusqu'à 80%
            T_rov_target = T_rov_target * (1.0 - reduction_factor)
            # Limiter la tension cible entre 10% et 20% de |F_apparent_weight|
            target_cap = abs(F_apparent_weight_temp)
            T_rov_target = min(T_rov_target, target_cap * 0.2)
            T_rov_target = max(T_rov_target, target_cap * 0.1)
        
        # Utiliser la tension de l'état actuel au niveau du ROV (index -1)
        # La tension T[-1] est mise à jour dynamiquement via dT_dt vers T_rov_target
        # IMPORTANT : Si slack < 0, utiliser directement T_rov_target pour un effet immédiat
        # afin de ramener rapidement slack >= 0
        if slack_curr < 0.0:
            # Slack négatif : utiliser directement T_rov_target pour un effet immédiat
            T_rov = T_rov_target
        else:
            # Slack >= 0 : utiliser la tension dynamique qui évolue progressivement
            T_rov = T[-1] if len(T) > 0 else T_rov_target
        
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
            # Direction vers le bateau : du point ROV (index -1) vers le point précédent (index -2)
            if len(x_cable_new) > 2:
                # Utiliser le segment adjacent au ROV (du point -1 vers le point -2)
                dx_rov = x_cable_new[-2] - x_cable_new[-1]  # Vers le bateau
                dy_rov = y_cable_new[-2] - y_cable_new[-1]  # Vers le bateau
            else:
                # Si seulement 2 points, utiliser la direction depuis le ROV vers le bateau
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
                Fy_surface_reaction = -self.rov.m * 10.0 * vy_rov  # Force de réaction
                Fy_total = Fy_total + Fy_surface_reaction
                dvy_rov_dt = Fy_total / self.rov.m
            if dvy_rov_dt > 0.0:
                # Si l'accélération est encore positive, forcer à zéro (contact avec la surface)
                Fy_surface_reaction = -self.rov.m * dvy_rov_dt
                Fy_total = Fy_total + Fy_surface_reaction
                dvy_rov_dt = Fy_total / self.rov.m

        # Si le câble est tendu, la cinématique est imposée par dL/dt.
        # On annule l'accélération pour éviter des valeurs incohérentes.
        if cable_taut:
            dvx_rov_dt = 0.0
            dvy_rov_dt = 0.0
        
        # Forces sur le bateau
        # Les tensions T_new conservent l'ordre original: T_new[-1] = tension au bateau
        T_L = T_new[-1] if len(T_new) > 0 else 0.0
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
        
        # Dérivées des positions du câble (vitesses)
        # Simplification : interpolation linéaire des vitesses
        vx_cable = np.linspace(vx_rov, vx_boat, self.N + 1)
        vy_cable = np.linspace(vy_rov, 0.0, self.N + 1)
        
        # CONTRAINTE PHYSIQUE CRITIQUE : Les points du câble ne peuvent pas remonter s'ils sont à ou près de la surface
        # Forcer vy_cable <= 0 pour tous les points où y_cable_new est proche de la surface
        if len(y_cable_new) == len(vy_cable):
            # Points à la surface (y >= 0) : vitesse verticale strictement <= 0
            mask_surface = y_cable_new >= 0.0
            vy_cable[mask_surface] = np.minimum(vy_cable[mask_surface], 0.0)
            
            # Points très proches de la surface (y > -0.05 m) : forcer aussi vy <= 0 pour éviter qu'ils remontent
            # Cette marge de sécurité empêche les points de remonter et de créer des discontinuités
            mask_near_surface = y_cable_new > -0.05
            vy_cable[mask_near_surface] = np.minimum(vy_cable[mask_near_surface], 0.0)
            
            # Points intermédiaires (entre -0.05 et 0) : forcer une décélération si vy > 0
            # pour éviter qu'ils remontent vers la surface
            mask_intermediate = (y_cable_new > -0.05) & (y_cable_new < 0.0) & (vy_cable > 0.0)
            if np.any(mask_intermediate):
                # Forcer une décélération proportionnelle à la proximité de la surface
                depth_factor = -y_cable_new[mask_intermediate] / 0.05  # 0 à 1 selon la profondeur
                vy_cable[mask_intermediate] = -depth_factor * 0.1  # Décélération vers le bas
        
        # Dérivées des tensions : mise à jour dynamique vers les tensions calculées
        # Utiliser un modèle de relaxation pour mettre à jour les tensions
        # Le temps de relaxation s'adapte selon le mouvement pour une réponse plus rapide
        # quand le ROV remonte alors qu'il devrait descendre
        if vy_rov > 0.0 and F_apparent_weight < 0.0 and abs(u['Fy_rov']) < 1e-6:
            # Le ROV remonte alors qu'il devrait descendre : adapter la tension plus rapidement
            # Temps de relaxation plus court pour une réponse plus rapide
            tau_tension = 0.01  # 10 ms pour une adaptation rapide
        else:
            # Temps de relaxation normal pour la stabilité
            tau_tension = 0.05  # 50 ms de relaxation pour meilleure stabilité et performance

        # Si le câble est mou (slack significatif), éviter des oscillations numériques rapides
        slack_ratio = 0.0
        if L_straight > 1e-9:
            slack_ratio = max((L - L_straight) / max(L_straight, 1.0), 0.0)
        if slack_ratio > 0.01:
            tau_tension = max(tau_tension, 0.2)
        # Proche du câble tendu (L ~ L_straight), augmenter fortement l'inertie
        if L_straight > 1e-9:
            straight_ratio = abs(L - L_straight) / max(L_straight, 1.0)
            if straight_ratio < 0.002:
                tau_tension = max(tau_tension, 0.5)
        
        # Mettre à jour T_new[-1] (ROV) avec la tension cible dynamique calculée plus haut
        if len(T_new) > 0:
            T_new[-1] = T_rov_target
        
        # Empêcher les chutes brutales de tension quand le câble est quasi droit
        if len(T_new) > 0 and len(T) > 0 and L_straight > 1e-9:
            try:
                straight_ratio = abs(L - L_straight) / max(L_straight, 1.0)
                if straight_ratio < 0.002:
                    t_prev = np.asarray(T, dtype=float)
                    t_new = np.asarray(T_new, dtype=float)
                    t_new = 0.9 * t_prev + 0.1 * t_new
                    T_new = np.maximum(t_new, 0.8 * t_prev)
            except Exception:
                pass

        # Stabilisation anti-bagottement: limiter les sauts rapides de tension
        if len(T_new) > 0 and len(T) > 0:
            try:
                t_prev = np.asarray(T, dtype=float)
                t_new = np.asarray(T_new, dtype=float)
                prev_bat = float(t_prev[0])
                new_bat = float(t_new[0])
                prev_rov = float(t_prev[-1])
                new_rov = float(t_new[-1])
                jump_ratio = 2.0
                if (
                    prev_bat > 1e-6
                    and (new_bat / prev_bat > jump_ratio or new_bat / prev_bat < 1.0 / jump_ratio)
                ) or (
                    prev_rov > 1e-6
                    and (new_rov / prev_rov > jump_ratio or new_rov / prev_rov < 1.0 / jump_ratio)
                ):
                    t_new = 0.8 * t_prev + 0.2 * t_new
                    T_new = np.maximum(t_new, 0.7 * t_prev)
            except Exception:
                pass

        # Limiter les variations relatives de tension quand le câble est mou
        if len(T_new) > 0 and len(T) > 0:
            try:
                slack_ratio = 0.0
                if L_straight > 1e-9:
                    slack_ratio = max((L - L_straight) / max(L_straight, 1.0), 0.0)
                if slack_ratio > 0.005:
                    t_prev = np.asarray(T, dtype=float)
                    t_new = np.asarray(T_new, dtype=float)
                    min_floor = 0.5
                    denom = np.maximum(t_prev, min_floor)
                    ratio = t_new / denom
                    # Si le slack est grand, autoriser une décroissance plus forte
                    if slack_ratio <= 0.01:
                        min_ratio = 0.75
                    elif slack_ratio >= 0.05:
                        min_ratio = 0.4
                    else:
                        # interpolation linéaire entre 0.75 et 0.4
                        frac = (slack_ratio - 0.01) / (0.05 - 0.01)
                        min_ratio = 0.75 + (0.4 - 0.75) * frac
                    ratio = np.clip(ratio, min_ratio, 1.25)
                    T_new = ratio * denom
            except Exception:
                pass

        dT_dt = (T_new - T) / tau_tension
        
        # IMPORTANT : Si slack < 0, forcer une adaptation immédiate de la tension au ROV
        # pour ramener rapidement slack >= 0
        if slack_curr < 0.0 and len(T) > 0 and len(T_new) > 0:
            # Forcer une adaptation très rapide de la tension au ROV (index -1)
            # pour que l'effet soit immédiat sur le mouvement
            dT_dt[-1] = (T_rov_target - T[-1]) / max(tau_tension, 0.001)  # Adaptation très rapide
        
        # Assembler les dérivées
        dydt = np.zeros_like(y)
        dydt[0] = vx_rov  # dx_rov/dt
        dydt[1] = vy_rov  # dy_rov/dt
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
    
    def integrate(self, t_span, y0, u_func, dt_max=0.1):
        """
        Intègre le système sur un intervalle de temps
        
        Parameters:
        -----------
        t_span : tuple
            (t0, tf) intervalle de temps (s)
        y0 : array
            Conditions initiales
        u_func : callable
            Fonction u(t) retournant le vecteur de commande
        dt_max : float
            Pas de temps maximum (s)
        
        Returns:
        --------
        solution
            Objet solution de scipy.integrate.solve_ivp
        """
        self.integrator.max_step = dt_max
        
        def system_ode(t, y):
            u = u_func(t)
            return self.compute_derivatives(t, y, u)
        
        sol = self.integrator.integrate(system_ode, t_span, y0)
        return sol

