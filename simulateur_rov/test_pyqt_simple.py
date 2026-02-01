"""Script de test simple pour PyQt"""
import sys
import os
import traceback

def main() -> None:
    print("Test PyQt application...")

    try:
        sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
        from src.ui.pyqt_app import main as run_app
        print("Import OK, lancement de l'application...")
        run_app()
    except Exception as e:
        print(f"ERREUR: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    main()
