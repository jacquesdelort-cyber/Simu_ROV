"""Script pour lancer l'application Dash"""
import sys
from pathlib import Path

# Ajouter le répertoire parent au path Python
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))

if __name__ == '__main__':
    try:
        from src.ui.dash_app import app
        print("\n" + "="*50)
        print("Application Dash démarrée !")
        print("Ouvrez votre navigateur à l'adresse :")
        print("http://localhost:8050")
        print("="*50 + "\n")
        app.run(debug=True, port=8050, use_reloader=False)
    except Exception as e:
        print(f"\n❌ Erreur lors du démarrage de l'application:")
        print(f"   {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
