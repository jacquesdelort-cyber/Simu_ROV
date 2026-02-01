# Script de verification de l'installation Python
Write-Host ""
Write-Host "=== Verification de l'installation Python ===" -ForegroundColor Cyan
Write-Host ""

# Verifier Python
Write-Host "1. Verification de Python..." -ForegroundColor Yellow
try {
    $pythonVersion = python --version 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-Host "   [OK] Python trouve : $pythonVersion" -ForegroundColor Green
    } else {
        Write-Host "   [ERREUR] Python non trouve" -ForegroundColor Red
        Write-Host "   Verifiez que Python est installe et ajoute au PATH" -ForegroundColor Yellow
        exit 1
    }
} catch {
    Write-Host "   [ERREUR] Python non trouve" -ForegroundColor Red
    Write-Host "   Verifiez que Python est installe et ajoute au PATH" -ForegroundColor Yellow
    exit 1
}

# Verifier pip
Write-Host ""
Write-Host "2. Verification de pip..." -ForegroundColor Yellow
try {
    $pipVersion = python -m pip --version 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-Host "   [OK] pip trouve : $pipVersion" -ForegroundColor Green
    } else {
        Write-Host "   [ERREUR] pip non trouve" -ForegroundColor Red
        exit 1
    }
} catch {
    Write-Host "   [ERREUR] pip non trouve" -ForegroundColor Red
    exit 1
}

# Verifier le chemin Python
Write-Host ""
Write-Host "3. Emplacement de Python..." -ForegroundColor Yellow
$pythonPath = (Get-Command python).Source
Write-Host "   Chemin : $pythonPath" -ForegroundColor Cyan

# Verifier si c'est le stub Windows Store
if ($pythonPath -like "*WindowsApps*") {
    Write-Host "   [ATTENTION] C'est l'alias Windows Store" -ForegroundColor Yellow
    Write-Host "   Python doit etre installe depuis python.org" -ForegroundColor Yellow
} else {
    Write-Host "   [OK] Installation Python valide" -ForegroundColor Green
}

Write-Host ""
Write-Host "=== Verification terminee ===" -ForegroundColor Cyan
Write-Host ""
