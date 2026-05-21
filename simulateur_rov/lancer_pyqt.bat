@echo off
setlocal
set "PYDIR=C:\Users\jacqu\AppData\Local\Python\pythoncore-3.14-64"
set "PATH=%PYDIR%;%PYDIR%\Scripts;%PATH%"

echo ========================================
echo Lancement du Simulateur ROV (PyQt)
echo ========================================
echo.

"%PYDIR%\python.exe" run_pyqt.py %*

if errorlevel 1 (
    echo.
    echo ERREUR: Le lancement a echoue.
    echo Verifiez que Python et les dependances sont installes.
    pause
)
