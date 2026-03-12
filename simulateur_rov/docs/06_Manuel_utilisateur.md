---
title: "Manuel utilisateur"
project: "Simulateur ROV"
format: "Markdown"
---

# Manuel utilisateur - Simulateur ROV

## Table des matières

1. Introduction
2. Installation, dépendances et tests
3. Lancement de l'application
4. Interface principale
5. Onglet Simulation
6. Onglet Paramètres
7. Missions et fichiers de configuration
8. Scénarios de commande
9. Export des résultats
10. Dépannage

---

## 1. Introduction

Le simulateur ROV permet de simuler la dynamique d'un système complet **bateau–câble–ROV** dans un plan 2D. Il reproduit les phénomènes principaux : traînée hydrodynamique, poussée d'Archimède, poids apparent, tensions du câble, contraintes de surface, et influence du courant.

Ce manuel décrit l'utilisation de l'interface PyQt (application Windows native).

---

## 2. Installation, dépendances et tests

### 2.1 Prérequis

- **Python 3.10** ou supérieur
- **Windows 10/11** (recommandé)

Vérifiez la version de Python :

```bash
python --version
```

### 2.2 Installation

1. **Cloner ou télécharger** le projet dans un répertoire local.

2. **Ouvrir un terminal** (PowerShell ou CMD) dans le répertoire racine du projet (`simulateur_rov`).

3. **Créer un environnement virtuel** (recommandé) :

```bash
python -m venv venv
venv\Scripts\activate
```

4. **Installer les dépendances** :

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

Ou utiliser le script PowerShell fourni :

```bash
.\installer_dependances.ps1
```

### 2.3 Dépendances

Les principales dépendances du simulateur sont :

| Package | Usage |
|---------|-------|
| **PyQt6** | Interface graphique |
| **PyQt6-WebEngine** | Affichage des graphiques Plotly |
| **numpy** | Calculs numériques |
| **scipy** | Intégration, optimisation |
| **plotly** | Visualisation interactive |
| **pandas** | Export CSV |
| **h5py** | Export HDF5 (optionnel) |
| **pytest** | Exécution des tests |

Installation manuelle (si `requirements.txt` est absent ou en cas d'erreur) :

```bash
pip install PyQt6 PyQt6-WebEngine numpy scipy plotly pandas pytest
```

### 2.4 Tests

Le projet inclut des tests unitaires dans le répertoire `tests/`. Pour exécuter tous les tests :

```bash
python -m pytest tests/ -v
```

Pour exécuter un fichier de test spécifique :

```bash
python -m pytest tests/test_rov_model.py -v
python -m pytest tests/test_cable_solver.py -v
python -m pytest tests/test_scenario_utils.py -v
python -m pytest tests/test_environment_current_velocity.py -v
```

Tests disponibles :

- `test_rov_model.py` : Modèle ROV (traînée, flottabilité)
- `test_cable_solver.py` : Solveur de câble
- `test_scenario_utils.py` : Utilitaires de scénarios
- `test_environment_current_velocity.py` : Profil de courant

---

## 3. Lancement de l'application

### Option A : Script Python (recommandé)

```bash
python run_pyqt.py
```

### Option B : Script batch Windows

Double-cliquez sur le fichier :

```
lancer_pyqt.bat
```

### Option C : Module Python

```bash
python -m src.ui.pyqt_app
```

**Vérification** : Si l'application se lance correctement, une fenêtre s'ouvre avec le titre « 🌊 Simulateur ROV - Application Windows » et 3 onglets visibles : Simulation, Paramètres, Performances.

---

## 4. Interface principale

L'application comporte **3 onglets** :

| Onglet | Description |
|--------|-------------|
| **🎮 Simulation** | Contrôles de simulation, visualisation temps réel, métriques |
| **⚙️ Paramètres** | Configuration des missions, paramètres physiques, conditions initiales |
| **📈 Performances** | (À venir) Analyses de performances |

---

## 5. Onglet Simulation

### 5.1 Structure

L'onglet Simulation est organisé en **3 colonnes** :

- **Colonne gauche** : Contrôles (boutons, commandes, scénarios)
- **Colonne centrale** : Visualisation en temps réel (graphique Plotly interactif)
- **Colonne droite** : Métriques temps réel et graphiques secondaires

### 5.2 Contrôles de simulation

- **▶ Démarrer simulation** : Lance une nouvelle simulation avec les paramètres actuels
- **⏸ Pause** : Met en pause ou reprend la simulation
- **⏹ Arrêter** : Arrête définitivement la simulation
- **📤 Export CSV** : Exporte les données de la simulation en fichier CSV

### 5.3 Commandes en temps réel

Les commandes suivantes peuvent être ajustées pendant la simulation :

- **Fx ROV** : Force horizontale sur le ROV (N)
- **Fy ROV** : Force verticale sur le ROV (N)
- **Vx bateau** : Vitesse commandée du bateau (m/s)
- **dL/dt** : Variation de la longueur de câble (m/s) — mode manuel ou auto

### 5.4 Visualisation

Le graphique principal affiche :

- Position du ROV (point)
- Profil du câble (ligne)
- Position du bateau (point à la surface)
- Mise à jour automatique pendant la simulation

Des onglets secondaires permettent d'afficher : Métriques, Profil câble, Profil fond.

### 5.5 Métriques affichées

- Temps écoulé
- Position du ROV (x, y)
- Longueur du câble
- Tension maximale
- Vitesse du ROV

---

## 6. Onglet Paramètres

L'onglet Paramètres est organisé en **3 colonnes** :

### 6.1 Sélection de mission

En haut de l'onglet :

- **Liste déroulante Mission** : Sélectionnez une mission existante
- **Survol** : Passez la souris sur une mission pour afficher sa description
- **➕ Nouvelle mission** : Crée une nouvelle mission
- **Charger Param_mission** : Charge les paramètres de la mission sélectionnée
- **Sauvegarder Param_mission** : Sauvegarde les paramètres dans la mission
- **Charger/Sauvegarder depuis fichier** : Import/export JSON

### 6.2 Colonne 1 : Paramètres d'environnement

- **ROV** : Masse, dimensions (a, b, h), coefficients de traînée (Cx, Cy)
- **Câble** : Diamètre, masse volumique, coefficients (Cx_cable, Cf_cable)
- **Bateau** : Masse, coefficients de propulsion et traînée
- **Environnement** : Masse volumique de l'eau, gravité, profil de courant

### 6.3 Colonne 2 : Paramètres de calcul

- Méthode d'intégration (RK45, RK23, DOP853, Radau)
- Tolérances (relative, absolue)
- Pas de temps, temps final
- Nombre de segments pour la discrétisation du câble

### 6.4 Colonne 3 : Conditions initiales

- Position initiale du ROV (x, y)
- Longueur initiale du câble
- Vitesse du courant
- Options de chute libre

---

## 7. Missions et fichiers de configuration

### 7.1 Structure des missions

Chaque mission est stockée dans un sous-dossier du répertoire des missions, contenant :

- **Param_mission.json** : Paramètres de la mission (physique, calcul, conditions initiales)
- **description** : Champ texte dans le fichier JSON

### 7.2 Format JSON

Les fichiers JSON sont compatibles entre missions. Ils contiennent les sections : `parameters`, `calc_params`, `init_params`, `mission`, `description`.

### 7.3 Tooltip des missions

Lors du survol d'une entrée dans la liste déroulante des missions, la description de la mission s'affiche dans un tooltip persistant (il reste visible tant que vous survolez l'item).

---

## 8. Scénarios de commande

Un **scénario** est une liste de couples `(crit:val)` appliqués à une variable de commande (Fx ROV, Fy ROV, Vx bateau, dL/dt). Quand le critère `crit` est vrai, la valeur `val` est affectée à la variable.

### 8.1 Critères numériques

Forme : `(x>y:z)` ou `(x<y:z)` où :

- `x` ∈ `{t, l, p}` : temps (s), longueur de câble (m), profondeur (m)
- `y` et `z` : nombres (float acceptés)

Exemple : `(t>2:10) (p<-5:2.5)`

### 8.2 Événements

Un événement est un identifiant commençant par `e` en minuscule : `e0`, `e_depart`, etc.

- **Création** : `(crit:evt)` — si `crit` est vrai, l'événement est créé (valeur = temps courant)
- **Critère événement** : `(evt>x:val)` — vrai si l'événement existe et si `(t - t0) > x`
- **Raccourci** : `(evt:val)` équivalent à `(evt>0:val)`
- **Critère vide** : `(:val)` — déclenche toujours l'action

Exemple de synchronisation :

```
Scenario Fx_rov : (p<-10:e_profondeur_atteinte) (e_profondeur_atteinte>0:5)
Scenario Vx_bateau : (e_profondeur_atteinte:0.8)
```

---

## 9. Export des résultats

### 9.1 Export CSV

Depuis l'onglet Simulation, cliquez sur **📤 Export CSV** pour sauvegarder les données de la simulation (positions, vitesses, tensions, forces, etc.) dans un fichier CSV.

### 9.2 Fichiers de trace

Les simulations peuvent produire des fichiers de trace (selon la configuration) pour analyse ultérieure.

---

## 10. Dépannage

### Erreur : "No module named 'PyQt6'"

```bash
pip install PyQt6 PyQt6-WebEngine
```

### Erreur : "No module named 'PyQt6.QtWebEngineWidgets'"

```bash
pip install PyQt6-WebEngine
```

### L'application se ferme immédiatement

- Lancez depuis un terminal pour voir les erreurs : `python run_pyqt.py`
- Vérifiez les messages d'erreur affichés

### Graphiques Plotly ne s'affichent pas

1. Vérifiez que PyQt6-WebEngine est installé
2. Redémarrez l'application

### Erreur d'import : "No module named 'src'"

Assurez-vous de lancer depuis le répertoire racine du projet :

```bash
cd C:\chemin\vers\simulateur_rov
python run_pyqt.py
```

---

## Références

- **Documentation technique** : `docs/02_modelisation.md`, `docs/05_etat_systeme_data.md`
- **Guide de lancement** : `GUIDE_LANCEMENT_PYQT.md`
- **README PyQt** : `docs/README_PYQT.md`
