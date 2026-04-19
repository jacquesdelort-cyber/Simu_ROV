---
title: "Spécification DAE / contraintes câble–ROV–bateau"
project: "Simulateur ROV"
format: "Markdown"
---

# Spécification : modèle contraint (chantier DAE)

## 1. Problème posé

Le vecteur d’état `y` mélange des grandeurs **évoluant par intégration** (positions, vitesses, `L`, tensions avec dynamique de relaxation) et une **géométrie de câble** qui doit, physiquement, vérifier en permanence des **contraintes géométriques** (longueur curviligne, surface, recollement aux extrémités). L’intégration ODE classique (`dy/dt = f(t,y)`) ne garantit pas que ces contraintes restent satisfaites ; des normalisations **a posteriori** recollent le profil mais ne définissent pas une trajectoire d’un même système différentiel-algébrique.

## 2. Contraintes cibles (invariants « durs »)

Les contraintes suivantes sont considérées comme **non négociables** pour un état admissible affiché / utilisé en fin de pas :

| Id | Contrainte | Formulation |
|----|------------|-------------|
| C1 | Longueur curviligne | \(\sum_i \|P_{i+1}-P_i\| = L\) (à tolérance numérique) |
| C2 | Surface | \(y_i \leq 0\) pour tous les nœuds du câble (convention eau : \(y \leq 0\) sous la surface) |
| C3 | Extrémités | \(P_0 = (x_{\mathrm{boat}}, 0)\), \(P_N\) cohérent avec le ROV (selon mode droit / caténaire) |
| C4 | Commande moulinet | \(dL/dt = u_{\mathrm{moulinet}}(t)\) intégrée dans le scalaire `L` (déjà dans l’ODE) |

Les **tensions** discrètes sont traitées comme variables **algébriques** liées à la géométrie via `_compute_catenary_tensions` après projection (relaxation séparée dans `compute_derivatives`).

## 3. Partition différentiel / algébrique (vue cible)

**Variables différentielles (exemple, vue conceptuelle)**  
- ROV : \(x_r, y_r, v_{xr}, v_{yr}\)  
- Bateau : \(x_b, v_{xb}\)  
- Longueur : \(L\)  
- (Option) tensions avec loi de relaxation : \(T_i\)  

**Variables / relations algébriques**  
- Positions intermédiaires du câble \((x_i, y_i)_{i=1}^{N-1}\) soumises à C1–C3  
- Tensions compatibles géométrie + poids apparent après projection  

**Indice DAE** : le système complet est **mixte** ; une réduction analytique complète vers un DAE d’indice 1 explicite n’est pas encore codée. La **première étape** du chantier est une **discrétisation half-explicit** : intégration d’un pas sur les variables différentielles, puis **résolution algébrique** (projection géométrique + recalcul des tensions) pour ramener `y` sur la variété des contraintes C1–C3.

## 4. Choix d’architecture numérique (phase 1 implémentée)

- **Famille retenue pour l’instant** : **half-explicit** (schéma de type DAE semi-explicite), **sans nouvelle dépendance** (SciPy / NumPy uniquement).  
- **Pas** : sous-pas **RK4** sur \([t_k, t_{k+1}]\), avec **projection** (`_normalize_cable_length` + tensions) après **chaque** sous-pas.  
- **Évolution** possible : coordonnées minimales (ODE sur variété), ou solveur DAE dédié (SUNDIALS IDA) — voir section 6.

Forme limite visée (documentation) :

\[
M(t,y)\,\dot y = f(t,y,u), \qquad 0 = g(t,y)
\]

où \(g\) encode C1–C3 ; la projection numérique actuelle est une **approximation** de la résolution \(g(y^+)=0\) après avancement explicite.

## 5. Fichiers de référence (implémentation)

| Fichier | Rôle |
|---------|------|
| [`src/models/state_projection.py`](../src/models/state_projection.py) | Métriques de violation, projection inplace sur `y` |
| [`src/solvers/projected_integrator.py`](../src/solvers/projected_integrator.py) | `integrate_span_with_projection` (RK4 + projection) |
| [`src/models/system_model.py`](../src/models/system_model.py) | `compute_constraint_residuals`, `integrate_span_with_projection` |
| [`src/ui/pyqt_widgets/simulation_thread.py`](../src/ui/pyqt_widgets/simulation_thread.py) | Drapeau `use_constrained_integrator` |
| [`src/ui/pyqt_widgets/all_parameters_tab.py`](../src/ui/pyqt_widgets/all_parameters_tab.py) | UI : activer intégration contrainte + nombre de sous-pas |

## 6. Pistes suivantes (hors périmètre immédiat)

- Réduction d’indice et Jacobian du système algébrique (Newton par pas).  
- `solve_ivp` avec matrice de masse singulière si le modèle est reformulé en DAE lisse.  
- IDA / SUNDIALS pour contraintes implicites complètes.
