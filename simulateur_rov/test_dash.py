"""Script de test pour l'application Dash"""
import sys
from pathlib import Path

# Ajouter le répertoire parent au path
sys.path.insert(0, str(Path(__file__).parent))

try:
    from src.ui.dash_app import app
    print("✅ Import réussi !")
    print("✅ Application Dash créée avec succès")
    print(f"✅ Titre: {app.title}")
    print("✅ L'application est prête à démarrer")
except Exception as e:
    print(f"❌ Erreur d'import: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
