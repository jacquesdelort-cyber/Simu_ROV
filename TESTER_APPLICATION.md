# Guide de Test - Application Dash ROV

## 🚀 Comment tester l'application

### Option 1 : Utiliser le script de lancement

```bash
cd simulateur_rov
python run_dash.py
```

### Option 2 : Lancer directement

```bash
cd simulateur_rov
python src/ui/dash_app.py
```

### Option 3 : Depuis le répertoire simulateur_rov

```bash
cd simulateur_rov
python -m src.ui.dash_app
```

## 📋 Ce qui devrait se passer

1. **Dans le terminal** : Vous devriez voir :
   ```
   ==================================================
   Application Dash démarrée !
   Ouvrez votre navigateur à l'adresse :
   http://localhost:8050
   ==================================================
   ```

2. **Dans le navigateur** : 
   - Ouvrez http://localhost:8050 (utilisez **http** pas https)
   - Vous devriez voir :
     - Un header bleu avec "🌊 Simulateur ROV"
     - 4 onglets : Simulation, Paramètres d'environnement, Paramètres calcul, Conditions initiales
     - Les onglets "Simulation" et "Paramètres d'environnement" affichent "Interface en cours de migration..."

## ⚠️ Si vous rencontrez des erreurs

### Erreur d'import
- Vérifiez que vous êtes dans le bon répertoire (simulateur_rov)
- Vérifiez que les dépendances Dash sont installées : `pip install dash dash-bootstrap-components`

### Erreur de port
- Le port 8050 est déjà utilisé : changez le port dans `dash_app.py` (ligne 92)
- Ou arrêtez l'autre application qui utilise le port

### "Loading..." indéfiniment
- Ouvrez la console du navigateur (F12)
- Vérifiez les erreurs JavaScript
- Partagez-les avec moi

## ✅ Ce qui fonctionne actuellement

- ✅ Structure de base de l'application
- ✅ Navigation par onglets
- ✅ Layouts de base (placeholders)
- ⏳ Fonctionnalités complètes (à migrer progressivement)

## 📝 Prochaines étapes

Une fois l'application de base testée :
1. Phase 2 : Ajouter la gestion des missions
2. Phase 3 : Migrer les onglets de configuration
3. Phase 4 : Ajouter les contrôles de simulation
4. Phase 5 : Ajouter les visualisations
5. Phase 6 : Ajouter l'analyse détaillée
