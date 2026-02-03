# Simulateur ROV-Câble-Bateau

Simulateur complet pour la modélisation dynamique du système ROV (Remotely Operated Vehicle) avec son câble ombilical et le navire de surface.

## 📋 Description

Ce simulateur permet de modéliser la dynamique complète en 2D du système ROV-Câble-Bateau, incluant :
- Les forces hydrodynamiques (traînée, poussée d'Archimède)
- La dynamique du câble ombilical (caténaire)
- Les interactions entre ROV, câble et bateau
- La visualisation en temps réel

## 🚀 Installation

### Prérequis
- Python 3.10 ou supérieur

### Installation des dépendances

```bash
pip install -r requirements.txt
```

## 💻 Utilisation

### Interface PyQt (Application Windows)

Lancez l'interface utilisateur native :

```bash
python -m src.ui.pyqt_app
```

### Script Python

Pour lancer une simulation directement depuis Python :

```bash
python main.py
```

### Utilisation programmatique

```python
from src.models.system_model import ROVSystem
from src.utils.parameters import get_default_parameters
from src.utils.initial_conditions import get_initial_state

# Initialiser le système
params = get_default_parameters()
system = ROVSystem(params, N_segments=50)

# Conditions initiales
y0 = get_initial_state(system, x_rov=0.0, y_rov=10.0, L=50.0)

# Définir les commandes
def u_func(t):
    return {
        'Fx_rov': 10.0,
        'Fy_rov': 0.0,
        'vx_boat_cmd': 0.5,
        'dL_dt': 0.0
    }

# Simuler
solution = system.integrate([0, 60], y0, u_func)
```

## 📁 Structure du Projet

```
simulateur_rov/
├── src/
│   ├── models/          # Modèles physiques (ROV, Cable, Boat, System)
│   ├── solvers/         # Solveurs numériques (intégration, câble, forces)
│   ├── utils/           # Utilitaires (paramètres, I/O, conditions initiales)
│   ├── visualization/   # Fonctions de visualisation
│   └── ui/              # Interface utilisateur (PyQt)
├── tests/               # Tests unitaires
├── data/                # Données d'entrée
├── results/             # Résultats de simulation
├── main.py              # Script principal
├── requirements.txt     # Dépendances
└── README.md           # Ce fichier
```

## 🎮 Contrôles de l'Interface

### Paramètres ROV
- **Force horizontale** : Force de propulsion horizontale (-100 à +100 N)
- **Force verticale** : Force de propulsion verticale (-100 à +100 N)

### Paramètres Bateau
- **Vitesse commande** : Vitesse horizontale de commande (-2 à +2 m/s)

### Paramètres Câble
- **Vitesse déroulement** : Vitesse de déroulement/enroulement (-1 à +1 m/s)

### Conditions Initiales
- Position et profondeur du ROV
- Longueur initiale du câble

## 📊 Visualisations

L'interface propose plusieurs visualisations :
- **Vue 2D du système** : Affichage du ROV, du câble et du bateau
- **Graphiques de position** : Évolution des positions horizontale et verticale
- **Graphiques de vitesse** : Évolution des vitesses
- **Graphique de tension** : Tension du câble au niveau du ROV

## 🔧 Paramètres Techniques

### Méthode de résolution
- Intégration temporelle : Runge-Kutta 4/5 (RK45) adaptatif
- Discrétisation du câble : Méthode des différences finies
- Nombre de segments : Configurable (10 à 100)

### Forces modélisées
- Traînée hydrodynamique (ROV et câble)
- Poussée d'Archimède
- Poids apparent du câble
- Tensions du câble

## 📝 Export des Données

Les résultats peuvent être exportés en plusieurs formats :
- **HDF5** : Format binaire efficace pour grandes quantités de données
- **CSV** : Format texte pour analyse dans Excel/autres outils
- **NPZ** : Format NumPy compressé

## 🧪 Tests

Pour exécuter les tests :

```bash
pytest tests/
```

## 📚 Documentation

Pour plus de détails sur le design et l'architecture, consultez le document `design_detaille_simulateur_ROV.html`.

## 🔬 Validation

Le simulateur peut être validé en comparant avec les données expérimentales disponibles dans le dossier `data/`.

## ⚠️ Notes

- Le simulateur est en 2D (plan x-y)
- Le modèle du câble utilise une approximation simplifiée
- Pour des simulations très longues, envisager d'optimiser avec Numba

## 📄 Licence

Ce projet est destiné à un usage académique et de recherche.

## 👤 Auteur

Simulateur développé dans le cadre du projet ROV.

## 🔄 Versions

- **v1.0.0** : Version initiale de l'interface PyQt

