"""
Script pour lancer l'application PyQt du simulateur ROV
"""
import sys
import os

# Ajouter le répertoire racine au path
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from src.ui.pyqt_app import main

if __name__ == '__main__':
    main()
