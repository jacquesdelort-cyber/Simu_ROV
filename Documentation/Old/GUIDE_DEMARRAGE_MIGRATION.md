# Guide de Démarrage Rapide - Migration vers Dash

## 🚀 Démarrage en 5 minutes

### 1. Installation des dépendances

```bash
# Installer Dash et dépendances
pip install -r requirements_dash.txt
```

### 2. Tester l'exemple minimal

```bash
# Lancer l'exemple Dash
cd simulateur_rov
python src/ui/dash_app_example.py
```

L'application s'ouvrira sur `http://localhost:8050`

### 3. Comparer avec Streamlit

```bash
# En parallèle, lancer Streamlit pour comparaison
streamlit run src/ui/streamlit_app.py
```

Streamlit sera sur `http://localhost:8501`

---

## 📋 Checklist de démarrage

- [ ] Dépendances Dash installées
- [ ] Exemple Dash fonctionne (`dash_app_example.py`)
- [ ] Lire `PLAN_MIGRATION_DASH.md` pour comprendre le plan complet
- [ ] Créer branche Git pour migration : `git checkout -b migration-dash`
- [ ] Démarrer Phase 0 : Préparation

---

## 🔄 Structure recommandée

### Phase 0 : Setup (aujourd'hui)

1. **Installer Dash**
   ```bash
   pip install dash dash-bootstrap-components
   ```

2. **Tester l'exemple**
   ```bash
   python src/ui/dash_app_example.py
   ```

3. **Créer structure de fichiers**
   ```
   src/ui/
   ├── dash_app.py (futur fichier principal)
   ├── dash_app_example.py (exemple de base)
   ├── dash_components/
   │   ├── __init__.py
   │   ├── mission_controls.py
   │   ├── parameter_inputs.py
   │   ├── simulation_controls.py
   │   └── visualizations.py
   ├── dash_callbacks/
   │   ├── __init__.py
   │   ├── mission_callbacks.py
   │   ├── parameter_callbacks.py
   │   ├── simulation_callbacks.py
   │   └── data_callbacks.py
   └── dash_layouts/
       ├── __init__.py
       ├── main_layout.py
       ├── config_layout.py
       └── simulation_layout.py
   ```

---

## 📚 Ressources utiles

### Documentation Dash
- **Dash Core** : https://dash.plotly.com/
- **Dash Bootstrap Components** : https://dash-bootstrap-components.opensource.faculty.ai/
- **Dash Callbacks** : https://dash.plotly.com/basic-callbacks
- **Dash Store** : https://dash.plotly.com/dash-core-components/store

### Exemples de code
- `dash_app_example.py` : Exemple minimal fonctionnel
- `PLAN_MIGRATION_DASH.md` : Plan complet détaillé

---

## 🎯 Prochaines étapes

1. **Aujourd'hui** : Installer, tester exemple, lire plan
2. **Phase 1 (2 jours)** : Créer structure de base
3. **Phase 2-6 (11 jours)** : Migrer fonctionnalités
4. **Phase 7 (1.5 jours)** : Tests et optimisations

**Total estimé : 14.5 jours**

---

## ⚠️ Points importants

### Différences clés Streamlit vs Dash

| Aspect | Streamlit | Dash |
|--------|-----------|------|
| État | `st.session_state` | `dcc.Store` |
| Mises à jour | `st.rerun()` | Callbacks + `dcc.Interval` |
| Layout | Déclaratif linéaire | HTML/Components |
| Interactivité | Limitée | Callbacks natifs |

### Bonnes pratiques Dash

1. **Séparer layout et callbacks** : Un fichier par type
2. **Utiliser Stores** : Pour état global (équivalent session_state)
3. **Optimiser callbacks** : Memoization si nécessaire
4. **Tester progressivement** : Phase par phase

---

## 🆘 En cas de problème

### Problèmes courants

1. **Port déjà utilisé**
   ```python
   # Changer le port dans dash_app.py
   app.run(port=8051)
   ```

2. **Callbacks multiples**
   ```python
   # Ajouter dans app
   app.config.suppress_callback_exceptions = True
   ```

3. **État non mis à jour**
   - Vérifier que `allow_duplicate=True` si nécessaire
   - Vérifier `prevent_initial_call=True`

---

## 📞 Support

- Lire `PLAN_MIGRATION_DASH.md` pour détails complets
- Consulter documentation Dash officielle
- Tester `dash_app_example.py` comme référence

---

**Bonne migration ! 🚀**
