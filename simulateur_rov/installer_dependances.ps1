# Script d'installation automatique des dependances
Write-Host ""
Write-Host "=== Installation des dependances du simulateur ROV ===" -ForegroundColor Cyan
Write-Host ""

# Verifier que Python est installe
Write-Host "Verification de Python..." -ForegroundColor Yellow
try {
    $pythonVersion = python --version 2>&1
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[ERREUR] Python n'est pas installe ou n'est pas dans le PATH" -ForegroundColor Red
        Write-Host "Veuillez d'abord installer Python depuis python.org" -ForegroundColor Yellow
        Write-Host "Voir GUIDE_INSTALLATION.md pour les instructions" -ForegroundColor Yellow
        exit 1
    }
    Write-Host "[OK] Python trouve : $pythonVersion" -ForegroundColor Green
} catch {
    Write-Host "[ERREUR] Python n'est pas installe ou n'est pas dans le PATH" -ForegroundColor Red
    Write-Host "Veuillez d'abord installer Python depuis python.org" -ForegroundColor Yellow
    exit 1
}

# Verifier que les fichiers requirements existent
if (-not (Test-Path "requirements.txt")) {
    Write-Host "[ERREUR] Fichier requirements.txt introuvable" -ForegroundColor Red
    exit 1
}

# Installer les dependances principales
Write-Host ""
Write-Host "Installation des dependances principales..." -ForegroundColor Yellow
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

if ($LASTEXITCODE -eq 0) {
    Write-Host "[OK] Dependances principales installees" -ForegroundColor Green
} else {
    Write-Host "[ERREUR] Erreur lors de l'installation des dependances principales" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "=== Installation terminee ===" -ForegroundColor Cyan
Write-Host "Vous pouvez maintenant lancer le simulateur avec :" -ForegroundColor Green
Write-Host "  python -m src.ui.pyqt_app" -ForegroundColor White
Write-Host ""
