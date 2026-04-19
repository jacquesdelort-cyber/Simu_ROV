# Application PyQt - Simulateur ROV

## 📋 Description

Cette application PyQt est la version Windows native du simulateur ROV. Elle offre une interface de bureau native avec des performances améliorées et une expérience utilisateur plus fluide.

Documentation technique du projet (modélisation, résolution numérique, implémentation, tests) : voir la [table des matières Markdown](00_table_des_matieres.md) dans le dossier `docs/`.

## 🚀 Installation

### Prérequis

- Python 3.10 ou supérieur
- Windows 10/11

### Installation des dépendances

```bash
pip install -r requirements.txt
```

Les dépendances incluent maintenant :
- PyQt6 (interface graphique)
- PyQt6-WebEngine (pour les graphiques Plotly)
- Toutes les dépendances existantes (numpy, scipy, plotly, etc.)

## 💻 Utilisation

### Lancement de l'application

**Option 1 : Script Python**
```bash
python run_pyqt.py
```

**Option 2 : Script batch Windows**
```bash
lancer_pyqt.bat
```

L'application s'ouvrira dans une fenêtre Windows native.

### Interface

L'application comporte 4 onglets principaux :

#### 🎮 Simulation
- **Colonne gauche** : Contrôles de simulation
  - Démarrer/Arrêter/Pause/Relancer
  - Export CSV
  - Barre de progression et statut
  
- **Colonne centrale** : Visualisation en temps réel
  - Graphique Plotly interactif du système ROV-Câble-Bateau
  - Mise à jour automatique pendant la simulation
  
- **Colonne droite** : Métriques temps réel
  - Temps écoulé
  - Position du ROV
  - Longueur du câble
  - Tension maximale
  - Vitesse du ROV

#### ⚙️ Paramètres d'environnement
- Configuration des paramètres ROV (masse, dimensions, coefficients)
- Configuration du câble (diamètre, masse volumique, traînée)
- Configuration du bateau (masse, traînée)
- Paramètres environnementaux (eau, gravité, viscosité)
- Chargement/Sauvegarde depuis fichiers JSON

#### 🔧 Paramètres de calcul
- Méthode d'intégration (RK45, RK23, DOP853, Radau)
- Tolérances (relative et absolue)
- Pas de temps
- Temps final de simulation
- Nombre de segments pour la discrétisation du câble

#### 🎯 Conditions initiales
- Position initiale du ROV (x, y)
- Longueur initiale du câble
- Vitesse du courant

## Scénarios (commandes)

Un scénario est une liste de couples `(crit:val)` appliqués à une variable de commande (Fx ROV, Fy ROV, Vx bateau, dL/dt). Quand `crit` est vrai, la valeur `val` est affectée à la variable associée. Les couples sont évalués dans l'ordre d'écriture.

### Critères numériques

Forme : `(x>y:z)` ou `(x<y:z)` où :
- `x` ∈ `{t, l, p}` (temps, longueur de câble, profondeur)
- `y` et `z` sont des nombres (float acceptés)

Exemple :
```
(t>2:10) (p<-5:2.5)
```

### Événements (synchronisation)

Un événement (evt) est un identifiant commençant par `e` en minuscule, suivi de lettres, chiffres ou `_`.
Exemples valides : `e0`, `e_2`, `e_profondeur_atteinte`
Exemples invalides : `E3`, `Un_evt`, `e.2`, `e 2`

Un événement a une valeur numérique :
- Par défaut, un événement non créé vaut `None`.
- Lors de sa création, il reçoit la valeur `t` (temps courant).
- Une fois créé, sa valeur ne change plus, même si le même `evt` réapparaît dans un autre couple `(crit:evt)`.

Deux formes sont acceptées :
- Création d'événement : `(crit:evt)` si `crit` est vrai, l'événement est créé (valeur `t`).
- Critère événement : `(evt>x:val)` vrai si l'événement existe et si `(t - t0) > x`, avec `t0` la date de création.
- Raccourci : `(evt:val)` équivalent à `(evt>0:val)`.

Critère vide (toujours vrai) :
- `(:val)` déclenche toujours l'action, utile pour une consigne immédiate.
- `(:evt)` crée immédiatement un événement (valeur `t`).

Exemple combiné :
```
Scenario Fx_rov : (:e_start) (e_start>3:4.0)
Scenario Vx_bateau : (e_start:0.5)
```

Les événements sont partagés entre toutes les commandes pendant une même simulation et sont réinitialisés au démarrage.

Exemple de synchronisation :
```
(t>5:e_depart) (e_depart>2:1.2)
```

Exemple multi-commandes (même événement) :
```
Scenario Fx_rov : (p<-10:e_profondeur_atteinte) (e_profondeur_atteinte>0:5)
Scenario Vx_bateau : (e_profondeur_atteinte:0.8)
```

## 📦 Distribution

### Créer un exécutable Windows

Pour créer un fichier .exe autonome :

```bash
pip install pyinstaller

pyinstaller --onefile --windowed --name "Simulateur_ROV" run_pyqt.py
```

L'exécutable sera dans le dossier `dist/`.

## 🐛 Dépannage

### Erreur "No module named 'PyQt6'"

Installez PyQt6 :
```bash
pip install PyQt6 PyQt6-WebEngine
```

### Graphiques Plotly ne s'affichent pas

Vérifiez que PyQt6-WebEngine est installé :
```bash
pip install PyQt6-WebEngine
```

### L'application se ferme immédiatement

Vérifiez la console pour les erreurs. Les erreurs Python s'affichent dans la console Windows.

## 📝 Notes

- Les fichiers de configuration JSON sont compatibles entre les missions
- Les résultats peuvent être exportés dans les mêmes formats
