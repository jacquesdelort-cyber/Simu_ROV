# Etat systeme - Data

Ce document decrit le contenu du dictionnaire `data` alimente pendant la simulation
(`simulation_thread.py`). Ce dictionnaire contient des series temporelles qui
alimentent l'interface, les graphiques et les exports.

## Structure generale

- La plupart des champs sont des **listes** synchronisees par indice.
- Les champs `x_cable_curr`, `y_cable_curr`, `T_cable_curr` representent l'etat
  instantane du cable (tableaux remplaces a chaque pas).

## Champs (series temporelles)

- `time` : temps courant (s).
- `x_rov`, `y_rov` : position du ROV (m).
- `vx_rov`, `vy_rov` : vitesse du ROV (m/s).
- `x_boat`, `vx_boat` : position et vitesse du bateau (m, m/s).
- `L` : longueur de cable (m).
- `dl_dt_cmd` : commande dL/dt appliquee (m/s).
- `dl_dt_auto_explain` : explication textuelle de la commande auto (si auto).
- `cable_mode` : mode du cable (`catenary` ou `straight`).
- `scenario_triggers` : liste des criteres declenches au pas courant.

## Tensions

- `T_rov` : tension au ROV (N).
- `T_boat` : tension au bateau (N).
- `T_max` : tension maximale le long du cable (N).

## Forces sur le ROV

**Convention de signe pour les forces verticales** : Positif = vers le haut (surface), Négatif = vers le bas (fond).

- `Fx_drag_rov`, `Fy_drag_rov` : force de trainee ROV (N). `Fy_drag_rov` positif vers le haut, negatif vers le bas.
- `Fx_traction_rov`, `Fy_traction_rov` : traction du cable sur le ROV (N). `Fy_traction_rov` positif vers le haut, negatif vers le bas.
- `Fy_rov_app_w` : poids apparent du ROV (N). Positif si flottabilite positive (vers le haut), negatif si flottabilite negative (vers le bas).
- `F_buoyancy_net` : flottabilite nette (positif si flottabilite positive) (N). Identique a `Fy_rov_app_w`.
- `Fx_cmd_rov`, `Fy_cmd_rov` : forces de commande/propulsion du ROV (N). `Fy_cmd_rov` positif vers le haut, negatif vers le bas.
- `Fx_rov_total`, `Fy_rov_total` : somme des forces appliquees au ROV (N). `Fy_rov_total` positif vers le haut, negatif vers le bas.

## Forces sur le cable

**Convention de signe pour les forces verticales** : Négatif = vers le bas (fond), Positif = vers le haut (surface).

- `Fx_drag_cable`, `Fy_drag_cable` : composantes horizontale et verticale de la trainee totale sur le cable (N). `Fy_drag_cable` est la trainee verticale uniquement (sans le poids apparent).
- `Fy_cable_app_w` : poids apparent total du cable (N, negatif vers le bas).
- `Fx_drag_cable_longitudinal`, `Fy_drag_cable_longitudinal` : composantes de la trainee longitudinale (frottement de surface) sur le cable (N). Calculee dans le repère local puis reprojetee dans le repère global.
- `Fx_drag_cable_perpendicular`, `Fy_drag_cable_perpendicular` : composantes de la trainee perpendiculaire (trainee normale) sur le cable (N). Calculee dans le repère local puis reprojetee dans le repère global.

**Note** : La trainee totale est la somme des trainees longitudinale et perpendiculaire :
- `Fx_drag_cable = Fx_drag_cable_longitudinal + Fx_drag_cable_perpendicular`
- `Fy_drag_cable = Fy_drag_cable_longitudinal + Fy_drag_cable_perpendicular`

## Forces sur le bateau

- `Fx_traction_boat`, `Fy_traction_boat` : traction du cable sur le bateau (N).
- `F_prop_boat` : force de propulsion du bateau (N).
- `Fx_total_boat`, `Fy_total_boat` : somme des forces appliquees au bateau (N).

## Angles du cable

- `angle_rov` : angle du cable au niveau du ROV (deg).
- `angle_boat` : angle du cable au niveau du bateau (deg).

## Etat instantane du cable

- `x_cable_curr` : coordonnees x des points du cable (m).
- `y_cable_curr` : coordonnees y des points du cable (m).
- `T_cable_curr` : tensions associees aux points du cable (N).

## Exemples d'acces

Quelques exemples d'acces aux dernieres valeurs disponibles :

```python
# Derniere position et vitesse
y_rov_current = data['y_rov'][-1] if data.get('y_rov') else None
vy_rov_current = data['vy_rov'][-1] if data.get('vy_rov') else None

# Valeur precedente (si disponible)
y_rov_prev = data['y_rov'][-2] if data.get('y_rov') and len(data['y_rov']) >= 2 else None
t_prev = data['time'][-2] if data.get('time') and len(data['time']) >= 2 else None

# Mode cable et derniere explication auto
last_mode = data['cable_mode'][-1] if data.get('cable_mode') else None
last_explain = data['dl_dt_auto_explain'][-1] if data.get('dl_dt_auto_explain') else None

# Tension au bateau / ROV
t_boat = data['T_boat'][-1] if data.get('T_boat') else None
t_rov = data['T_rov'][-1] if data.get('T_rov') else None
```
