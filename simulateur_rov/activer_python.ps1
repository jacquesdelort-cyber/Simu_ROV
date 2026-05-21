# Recharge le PATH Python dans le terminal courant (sans redemarrer Cursor).
$pyDir = "C:\Users\jacqu\AppData\Local\Python\pythoncore-3.14-64"
$pyScripts = Join-Path $pyDir "Scripts"
$env:Path = "$pyDir;$pyScripts;" + $env:Path
Write-Host "Python actif:" -ForegroundColor Green
& "$pyDir\python.exe" --version
