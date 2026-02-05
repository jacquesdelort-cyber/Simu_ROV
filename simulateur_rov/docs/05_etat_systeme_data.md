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

- `Fx_drag`, `Fy_drag` : force de trainee ROV (N).
- `Fx_traction`, `Fy_traction` : traction du cable sur le ROV (N).
- `F_apparent_weight` : poids apparent (positif vers le bas) (N).
- `F_buoyancy_net` : flottabilite nette (positif si flottabilite positive) (N).
- `Fx_total`, `Fy_total` : somme des forces appliquees au ROV (N).

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

`python
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
`

### Exemples complementaires

Moyenne glissante (fenetre de 20 points) :

```python
window = 20
vals = data.get('T_rov', [])
if len(vals) >= window:
    avg_T_rov = sum(vals[-window:]) / window
else:
    avg_T_rov = None
```

Detection d'evenement (seuil) :

```python
threshold = 30.0  # N
if data.get('T_rov') and data['T_rov'][-1] > threshold:
    event = 'T_rov depasse le seuil'
else:
    event = None
```

Acces aux points du cable (cote bateau / cote ROV) :

```python
x_cable = data.get('x_cable_curr') or []
y_cable = data.get('y_cable_curr') or []
if len(x_cable) >= 2 and len(y_cable) >= 2:
    # Index 0 = bateau, index -1 = ROV
    x_boat_cable, y_boat_cable = x_cable[0], y_cable[0]
    x_rov_cable, y_rov_cable = x_cable[-1], y_cable[-1]
else:
    x_boat_cable = y_boat_cable = None
    x_rov_cable = y_rov_cable = None
```
