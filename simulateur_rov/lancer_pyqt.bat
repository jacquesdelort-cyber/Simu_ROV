@echo off
echo ========================================
echo Lancement du Simulateur ROV (PyQt)
echo ========================================
echo.

python run_pyqt.py

if errorlevel 1 (
    echo.
    echo ERREUR: Le lancement a echoue.
    echo Verifiez que Python et les dependances sont installes.
    pause
)
