---
title: "Tests"
project: "Simulateur ROV"
format: "Markdown"
---

# Tests

## Table des matières

1. Objectif
2. Organisation des tests
3. Tests unitaires disponibles
4. Assertions de contrôle en fin d'itération
5. Scripts de diagnostic et de validation
6. Comment lancer les tests
7. Remarques pratiques

## 1. Objectif

Ce chapitre décrit les tests actuellement implémentés dans le dépôt, leur rôle,
ainsi que les commandes à utiliser pour les exécuter.

Le projet contient deux grandes familles de tests :

- des **tests unitaires** basés sur `pytest`, regroupés dans le répertoire `tests/` ;
- des **scripts de diagnostic** placés à la racine du dépôt, utiles pour des validations manuelles ou semi-manuelles.

## 2. Organisation des tests

### 2.1 Répertoire `tests/`

Le répertoire `tests/` contient les tests unitaires automatisables :

- `tests/test_cable_solver.py`
- `tests/test_cable_normalization_edge_cases.py`
- `tests/test_utils_scale_slack.py`
- `tests/test_utils_supprimer_point.py`
- `tests/test_utils_enforce_cable_segments_nb.py`
- `tests/test_utils_deplacer_point.py`
- `tests/test_environment_current_velocity.py`
- `tests/test_rov_model.py`
- `tests/test_scenario_utils.py`

### 2.2 Scripts de test à la racine

Des scripts complémentaires existent à la racine du projet :

- `test_horizontal_movement.py`
- `test_vertical_movement.py`
- `test_horizontal_detailed.py`
- `test_pyqt.py`
- `test_pyqt_simple.py`

Le répertoire `tests/` contient aussi plusieurs scripts visuels Plotly, utiles
pour inspecter visuellement les écarts numériques :

- `tests/test_cable_normalization_visual.py`
- `tests/test_scale_slack_visual.py`
- `tests/test_deplacer_point_visual.py`
- `tests/test_supprimer_point_visual.py`
- `tests/test_enforce_cable_segments_nb_visual.py`

Ces scripts ne sont pas tous conçus comme des tests unitaires `pytest`. Certains
ouvrent l'application, lancent une simulation courte ou affichent un diagnostic
détaillé destiné à une analyse manuelle.

## 3. Tests unitaires disponibles

### 3.1 `tests/test_cable_solver.py`

Ce fichier couvre principalement le solveur de câble et la fonction de
normalisation `_normalize_cable_geometry`.

Les cas testés incluent :

- l'initialisation du modèle de câble ;
- la normalisation en imposant :
  - `P[0]` collé au bateau et `P[-1]` collé au ROV,
  - une longueur totale finale proche de `L_target` (critère `rel_err <= 1e-4` ou `max_iters = 10`) ;
- le respect de bornes locales sur la longueur des segments (entre `L_target/N_debut_iter` et `2*L_target/N_debut_iter`) ;
- le respect de la contrainte `y <= 0` ;
- l'ajout temporaire de points puis la réduction finale du nombre de segments avec `enforce_cable_segments_nb` (compatibilité avec l'état empaqueté par `system_model.pack_state`).

Les tests de normalisation utilisent volontairement des câbles synthétiques de
petite taille, avec une dizaine de segments au maximum, afin de faciliter le
diagnostic.

### 3.2 `tests/test_cable_normalization_edge_cases.py`

Ce fichier complète les tests précédents avec des cas limites de
normalisation.

Les cas testés incluent :

- des points répétés dans la géométrie d'entrée ;
- un clipping marqué au voisinage de la surface ;
- la compression d'une géométrie vers une longueur cible plus courte ;
- le cas d'une géométrie totalement dégénérée (`L_actual ≈ 0`), désormais
  reconstruite en câble rectiligne de longueur `L_target`.

Ces tests vérifient notamment :

- l'absence de `NaN` ou de valeurs infinies ;
- la longueur totale finale (tolérance relative) ;
- les bornes de segments après réduction au nombre cible ;
- le respect de `y <= 0`.

### 3.3 `tests/test_environment_current_velocity.py`

Ce fichier vérifie le comportement de `Environment.get_current_velocity()`.

Les cas couverts incluent :

- courant constant fourni sous forme de chaîne ;
- interpolation d'un profil profondeur/vitesse ;
- prise en charge des décimales avec virgule ;
- comportement de repli pour des chaînes invalides ou incomplètes ;
- utilisation du profil par défaut lorsque `v_courant` n'est pas fourni.

### 3.4 `tests/test_rov_model.py`

Ce fichier vérifie quelques invariants du modèle `ROV`.

Les cas couverts incluent :

- l'initialisation des paramètres du ROV ;
- le calcul de surface et de volume ;
- la traînée nulle pour une vitesse nulle ;
- le calcul de la poussée d'Archimède.

### 3.5 `tests/test_scenario_utils.py`

Ce fichier vérifie les fonctions de scénarios et de commande automatique.

Les cas couverts incluent :

- la validation de syntaxe des scénarios ;
- la détection de scénarios invalides ;
- la progression et le réarmement de `commande_scenario()` ;
- le traitement des paramètres de commande ;
- un test de comportement de `auto_L_1`.

### 3.6 `tests/test_utils_scale_slack.py`

Tests unitaires de la fonction géométrique `scale_slack` (cas triangulaires,
cas avec slack > 1, slack < 1 et cas alignés).

### 3.7 `tests/test_utils_supprimer_point.py`

Tests unitaires de `supprimer_point`, notamment sur les cas de micro-géométrie
et les cas où les points deviennent quasi alignés (pour éviter les retours
`None` dans des configurations réalistes).

### 3.8 `tests/test_utils_enforce_cable_segments_nb.py`

Tests unitaires de la réduction du nombre de segments (`enforce_cable_segments_nb`)
à une valeur cible, avec vérification que la géométrie et la longueur restent
compatibles.

### 3.9 `tests/test_utils_deplacer_point.py`

Tests unitaires de `deplacer_point`, avec vérification de la construction
géométrique (milieu/perpendiculaire) et des longueurs (condition sur la somme
des distances).

## 4. Assertions de contrôle en fin d'itération

En plus des tests unitaires automatisés, le code de simulation inclut des **assertions de contrôle** qui sont vérifiées automatiquement à chaque itération de la simulation. Ces assertions permettent de détecter rapidement les violations des contraintes physiques fondamentales du modèle.

### 4.1 Fonction `check_cable_invariants`

La fonction `check_cable_invariants()` est implémentée dans `src/ui/pyqt_widgets/simulation_thread.py` et vérifie plusieurs types de contraintes critiques :

1. **Cohérence longueur globale** : La longueur totale `L` (intégrée dynamiquement) doit correspondre à la somme des longueurs des segments `L_seg` calculée géométriquement. Une tolérance relative de `1e-3` est appliquée.

2. **Slack non négatif** : Le slack (`L - D_straight`, où `D_straight` est la distance en ligne droite entre le bateau et le ROV) doit être >= 0. Une tolérance de `1e-3` m est appliquée pour tenir compte des erreurs numériques.

3. **Recollement des extrémités** : 
   - Le premier point du câble doit être exactement au bateau (position `(x_boat, 0.0)`)
   - Le dernier point du câble doit être exactement au ROV (position `(x_rov, y_rov)`)
   - Une tolérance de `1e-3` m est appliquée pour la distance entre les extrémités et leurs points d'attache.

4. **Longueur maximale d’un segment** : 
   - À partir de la liste des longueurs élémentaires `ds_i`, on définit une longueur cible moyenne `ds_target = L / N` (où `N` est le nombre de segments).
   - Une contrainte locale vérifie qu’aucun segment ne dépasse un facteur `k` de cette longueur moyenne : `ds_max ≤ k * ds_target` avec `k = 2` actuellement.
   - En cas de violation, une entrée de type `segment_too_long(i=...,ratio=...)` est ajoutée à la liste des violations, et le rapport inclut le segment concerné et le ratio `ds_max / ds_target`.

### 4.2 Moments de vérification

Les assertions sont vérifiées à deux moments critiques :

- **À la fin de la phase d'initialisation** : Après le calcul de la configuration initiale du câble, pour s'assurer que l'état initial respecte toutes les contraintes.

- **À la fin de chaque itération de simulation** : Après chaque pas d'intégration temporelle, pour détecter toute dérive numérique ou violation des contraintes physiques.

### 4.3 Messages de diagnostic

En cas de violation d'une ou plusieurs contraintes, un message détaillé est généré au niveau de trace 9 (`trace_print(9, ...)`), incluant :

- La phase de simulation (`initialization` ou `iteration`)
- Le temps courant `t` et le numéro d'itération `step`
- La liste des violations détectées
- Les valeurs numériques pertinentes : `L`, `L_seg`, `D_straight`, `slack`, distances de recollement, tensions au bateau et au ROV
- Le contexte de la commande : `dL_dt_cmd`, explication de la commande automatique, mode du câble, déclencheurs de scénario

Ces messages permettent d'identifier précisément :
- À quel moment la violation s'est produite
- Quelle contrainte a été violée
- Les valeurs numériques qui ont causé le problème
- Le contexte de la simulation (commande, scénario, etc.)

### 4.4 Utilisation pour le diagnostic

Pour activer l'affichage des messages de violation, il faut configurer le niveau de trace global à 9 ou supérieur. Les violations sont signalées mais n'interrompent pas la simulation, permettant ainsi de collecter des informations de diagnostic complètes.

Ces assertions sont particulièrement utiles pour :
- Détecter les instabilités numériques
- Identifier les problèmes de synchronisation entre la géométrie du câble et l'état intégré
- Diagnostiquer les cas où le câble n'est pas correctement recollé aux extrémités
- Vérifier que le slack reste toujours non négatif

## 5. Scripts de diagnostic et de validation

### 5.1 `test_horizontal_movement.py`

Script de validation du mouvement horizontal du ROV sous l'effet d'une force
positive appliquée sur `Fx_rov`.

Ce script :

- construit un système complet ;
- lance une simulation courte ;
- vérifie que le ROV se déplace dans le bon sens ;
- retourne un code de sortie `0` si le comportement est jugé correct.

### 5.2 `test_vertical_movement.py`

Script de validation du mouvement vertical du ROV sous l'effet d'une force
sur `Fy_rov`.

Il permet de vérifier manuellement que le mouvement vertical est bien possible
et cohérent avec la force appliquée.

### 5.3 `test_horizontal_detailed.py`

Script de diagnostic plus détaillé sur le mouvement horizontal.

Il affiche notamment :

- les positions successives ;
- les composantes de force ;
- la contribution de la tension ;
- une lecture plus fine des causes d'un déplacement incohérent.

Ce script est utile pour l'analyse d'un défaut, plus que pour une validation
rapide automatisée.

### 5.4 `test_pyqt.py`

Script de vérification du démarrage de l'application PyQt.

Il teste successivement :

- l'import de `PyQt6` ;
- la création de `QApplication` ;
- l'import des modules de l'application ;
- la création de la fenêtre principale ;
- l'affichage de la fenêtre et l'entrée dans la boucle d'événements.

Ce script nécessite un environnement graphique fonctionnel.

### 5.5 `test_pyqt_simple.py`

Script minimal de lancement de l'application.

Il sert surtout à vérifier rapidement que l'import principal et le point
d'entrée PyQt fonctionnent.

### 5.6 `tests/test_cable_normalization_visual.py`

Ce script génère un rapport HTML interactif pour visualiser les tests
élémentaires de normalisation du câble.

Pour chaque cas, il affiche :

- le profil du câble avant normalisation ;
- le profil du câble après normalisation ;
- les positions bateau/ROV utilisées dans le calcul ;
- la longueur totale du câble avant normalisation ;
- la longueur minimale et maximale des segments avant normalisation ;
- la longueur totale du câble après normalisation ;
- la longueur minimale et maximale des segments après normalisation ;
- le `rel_err` final et le nombre de segments avant/après ;
- la longueur cible `L_target`.

Le rapport est généré par défaut dans :

`results/cable_normalization_visual_tests.html`

Ce script est particulièrement utile pour diagnostiquer visuellement les cas
de clipping, de compression géométrique, de points répétés ou de géométrie
dégénérée.

## 6. Comment lancer les tests

Les commandes suivantes sont à exécuter depuis la racine du dépôt.

### 6.1 PowerShell Windows

Dans l'environnement actuel, il est recommandé de désactiver le chargement
automatique des plugins `pytest` externes, car certains plugins installés
globalement peuvent perturber l'exécution des tests.

Commande pour lancer tous les tests unitaires :

```powershell
cd "C:\Users\jacqu\OneDrive\Documents\Projet ROV\simulateur_rov"
$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'
python -m pytest tests
```

Commande pour lancer seulement les tests du câble :

```powershell
cd "C:\Users\jacqu\OneDrive\Documents\Projet ROV\simulateur_rov"
$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'
python -m pytest tests/test_cable_solver.py tests/test_cable_normalization_edge_cases.py
```

Commande pour lancer un seul fichier :

```powershell
cd "C:\Users\jacqu\OneDrive\Documents\Projet ROV\simulateur_rov"
$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'
python -m pytest tests/test_scenario_utils.py
```

### 6.2 Lancement direct d'un script de diagnostic

Exemples :

```powershell
cd "C:\Users\jacqu\OneDrive\Documents\Projet ROV\simulateur_rov"
python test_horizontal_movement.py
```

```powershell
cd "C:\Users\jacqu\OneDrive\Documents\Projet ROV\simulateur_rov"
python test_vertical_movement.py
```

```powershell
cd "C:\Users\jacqu\OneDrive\Documents\Projet ROV\simulateur_rov"
python test_horizontal_detailed.py
```

### 6.3 Lancement des tests PyQt

Les scripts PyQt demandent un environnement graphique actif.

Exemples :

```powershell
cd "C:\Users\jacqu\OneDrive\Documents\Projet ROV\simulateur_rov"
python test_pyqt.py
```

```powershell
cd "C:\Users\jacqu\OneDrive\Documents\Projet ROV\simulateur_rov"
python test_pyqt_simple.py
```

### 6.4 Génération du rapport visuel de normalisation

```powershell
cd "C:\Users\jacqu\OneDrive\Documents\Projet ROV\simulateur_rov"
python tests/test_cable_normalization_visual.py
```

Le script génère ensuite le fichier :

`results/cable_normalization_visual_tests.html`

### 6.5 Lancement via l'onglet `🧪 Tests`

L'application PyQt inclut un onglet **🧪 Tests** qui :

- charge le catalogue des tests depuis `src/tests/test_catalog.py`,
- affiche les tests avec une case à cocher par entrée,
- exécute les tests cochés en séquence (via `subprocess`),
- sauvegarde l'état et le résultat dans `tests/test_status.json` (cases cochées,
  date/heure, `PASS`/`FAIL`, commentaires) ;

Les rapports HTML produits par les scripts visuels sont listés dans la colonne
"Commentaires / Rapport" lorsque disponibles.

## 7. Remarques pratiques

- Les tests `pytest` du répertoire `tests/` sont les plus adaptés à une
  validation rapide et répétable.
- Les scripts à la racine sont plutôt des outils de diagnostic ou de validation
  manuelle.
- Certains tests physiques peuvent émettre des avertissements numériques
  (`RuntimeWarning`) sans provoquer d'échec immédiat.
- Pour les tests PyQt, la réussite dépend aussi de la disponibilité de
  `PyQt6`, `QtWebEngine` et d'un affichage graphique compatible.
- Les tests de normalisation du câble ont été conçus avec des géométries
  simples et peu de segments afin de faciliter l'analyse des écarts numériques.
