# Guide d'installation Python pour le simulateur ROV

## 📥 Étape 1 : Télécharger Python

1. Le site de téléchargement Python devrait s'ouvrir automatiquement dans votre navigateur
2. Si ce n'est pas le cas, allez sur : https://www.python.org/downloads/
3. Cliquez sur le bouton **"Download Python"** (version la plus récente, 3.12 ou 3.11)

## 🔧 Étape 2 : Installer Python

**⚠️ IMPORTANT : Pendant l'installation, cochez la case "Add Python to PATH"**

1. Double-cliquez sur le fichier téléchargé (ex: `python-3.12.x.exe`)
2. **Cochez la case "Add Python to PATH"** en bas de la fenêtre d'installation
3. Cliquez sur **"Install Now"**
4. Attendez la fin de l'installation
5. Cliquez sur **"Close"** une fois l'installation terminée

## ✅ Étape 3 : Vérifier l'installation

1. **Fermez et rouvrez PowerShell** (important pour que le PATH soit mis à jour)
2. Exécutez le script de vérification :

```powershell
.\verifier_python.ps1
```

Vous devriez voir :
- ✓ Python trouvé
- ✓ pip trouvé
- ✓ Installation Python valide

## 📦 Étape 4 : Installer les dépendances

Une fois Python vérifié, installez les dépendances du simulateur :

```powershell
python -m pip install -r requirements.txt
```

## 🚀 Étape 5 : Lancer le simulateur

Une fois tout installé, vous pouvez lancer le simulateur :

```powershell
python -m src.ui.pyqt_app
```

---

## ❓ Problèmes courants

### "python n'est pas reconnu"
- Vérifiez que vous avez coché "Add Python to PATH" lors de l'installation
- Fermez et rouvrez PowerShell
- Réinstallez Python en cochant bien la case

### "pip n'est pas reconnu"
- Utilisez `python -m pip` au lieu de `pip` seul
- Exemple : `python -m pip install -r requirements.txt`

### L'alias Windows Store s'ouvre
- Désactivez l'alias dans Paramètres > Applications > Gestionnaire d'exécution des applications
- Ou installez Python depuis python.org (recommandé)
