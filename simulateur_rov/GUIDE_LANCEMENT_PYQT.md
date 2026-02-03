# 🚀 Guide de Lancement - Application PyQt

## Prérequis

1. **Python 3.10 ou supérieur** installé
2. Toutes les dépendances installées

## Étape 1 : Installation des dépendances

Ouvrez un terminal PowerShell ou CMD dans le répertoire du projet et exécutez :

```bash
pip install -r requirements.txt
```

Ou installez PyQt6 manuellement si nécessaire :

```bash
pip install PyQt6 PyQt6-WebEngine
```

## Étape 2 : Lancer l'application

### Option A : Script Python (Recommandé)

```bash
python run_pyqt.py
```

### Option B : Script Batch Windows

Double-cliquez sur le fichier :
```
lancer_pyqt.bat
```

### Option C : Lancer directement depuis le module

```bash
python -m src.ui.pyqt_app
```

## 📋 Vérification rapide

Si l'application se lance correctement, vous devriez voir :

1. ✅ Une fenêtre Windows s'ouvre avec le titre "🌊 Simulateur ROV - Application Windows"
2. ✅ 4 onglets visibles : Simulation, Paramètres d'environnement, Paramètres de calcul, Conditions initiales
3. ✅ Une interface avec 3 colonnes dans l'onglet Simulation :
   - Colonne gauche : Contrôles (boutons Démarrer, Arrêter, etc.)
   - Colonne centrale : Zone de visualisation (graphique Plotly)
   - Colonne droite : Métriques temps réel

## ⚠️ Résolution des problèmes courants

### Erreur : "No module named 'PyQt6'"

**Solution :**
```bash
pip install PyQt6 PyQt6-WebEngine
```

### Erreur : "No module named 'PyQt6.QtWebEngineWidgets'"

**Solution :**
```bash
pip install PyQt6-WebEngine
```

### L'application se ferme immédiatement

**Solution :**
- Ouvrez un terminal PowerShell
- Lancez l'application depuis le terminal pour voir les erreurs :
  ```bash
  python run_pyqt.py
  ```
- Vérifiez les messages d'erreur affichés dans le terminal

### Graphiques Plotly ne s'affichent pas

**Solution :**
1. Vérifiez que PyQt6-WebEngine est installé :
   ```bash
   pip install PyQt6-WebEngine
   ```
2. Si le problème persiste, redémarrez l'application

### Erreur d'import : "No module named 'src'"

**Solution :**
Assurez-vous de lancer l'application depuis le répertoire racine du projet (`simulateur_rov`) :

```bash
# Depuis le répertoire simulateur_rov
cd C:\Users\jacqu\OneDrive\Documents\Projet ROV\simulateur_rov
python run_pyqt.py
```

## 🎮 Première utilisation

1. **Configurer les paramètres** (onglet "⚙️ Paramètres d'environnement")
   - Les valeurs par défaut sont pré-remplies
   - Vous pouvez les modifier selon vos besoins

2. **Vérifier les paramètres de calcul** (onglet "🔧 Paramètres de calcul")
   - Temps final : 60 secondes par défaut
   - Nombre de segments : 50 par défaut

3. **Définir les conditions initiales** (onglet "🎯 Conditions initiales")
   - Position ROV : x=0, y=10 m par défaut
   - Longueur câble : 50 m par défaut

4. **Lancer une simulation** (onglet "🎮 Simulation")
   - Cliquez sur "▶ Démarrer simulation"
   - Observez la visualisation en temps réel
   - Les métriques se mettent à jour automatiquement

## 📝 Notes importantes

- Les fichiers JSON de paramètres sont compatibles entre les deux versions
- Les simulations peuvent être longues selon les paramètres choisis
- Utilisez les boutons "Pause" et "Arrêter" pour contrôler la simulation

## 🆘 Aide supplémentaire

Si vous rencontrez des problèmes non listés ici :

1. Vérifiez que Python est à jour : `python --version`
2. Vérifiez que toutes les dépendances sont installées : `pip list`
3. Consultez le fichier `README_PYQT.md` pour plus d'informations
4. Lancez l'application depuis un terminal pour voir les erreurs détaillées
