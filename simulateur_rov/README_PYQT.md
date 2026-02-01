# Application PyQt - Simulateur ROV

## 📋 Description

Cette application PyQt est la version Windows native du simulateur ROV. Elle offre une interface de bureau native avec des performances améliorées et une expérience utilisateur plus fluide.

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
