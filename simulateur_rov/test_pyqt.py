"""
Script de test pour vérifier le lancement de l'application PyQt
"""
import sys
import os
import traceback

def main() -> None:
    print("Début du test...")
    print(f"Python version: {sys.version}")
    print(f"Répertoire courant: {os.getcwd()}")

    try:
        print("\n1. Test import PyQt6...")
        from PyQt6.QtCore import Qt, QCoreApplication
        from PyQt6.QtWidgets import QApplication, QMainWindow
        from PyQt6 import QtWebEngineWidgets  # noqa: F401
        QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
        print("   ✓ PyQt6 importé avec succès")
    except Exception as e:
        print(f"   ✗ Erreur import PyQt6: {e}")
        traceback.print_exc()
        sys.exit(1)

    try:
        print("\n2. Création de QApplication...")
        app = QApplication(sys.argv)
        print("   ✓ QApplication créé")
    except Exception as e:
        print(f"   ✗ Erreur création QApplication: {e}")
        traceback.print_exc()
        sys.exit(1)

    try:
        print("\n3. Test import des modules de l'application...")
        sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
        from src.ui.pyqt_app import ROVSimulatorApp
        print("   ✓ Modules importés avec succès")
    except Exception as e:
        print(f"   ✗ Erreur import modules: {e}")
        traceback.print_exc()
        sys.exit(1)

    try:
        print("\n4. Création de la fenêtre principale...")
        window = ROVSimulatorApp()
        print("   ✓ Fenêtre créée")
    except Exception as e:
        print(f"   ✗ Erreur création fenêtre: {e}")
        traceback.print_exc()
        sys.exit(1)

    try:
        print("\n5. Affichage de la fenêtre...")
        window.show()
        print("   ✓ Fenêtre affichée")
        print("\n✓ Application prête ! La fenêtre devrait être visible.")
        print("  Appuyez sur Ctrl+C pour fermer l'application.\n")
    except Exception as e:
        print(f"   ✗ Erreur affichage fenêtre: {e}")
        traceback.print_exc()
        sys.exit(1)

    try:
        print("Lancement de la boucle d'événements...")
        sys.exit(app.exec())
    except KeyboardInterrupt:
        print("\nApplication fermée par l'utilisateur")
    except Exception as e:
        print(f"\n✗ Erreur dans la boucle d'événements: {e}")
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
