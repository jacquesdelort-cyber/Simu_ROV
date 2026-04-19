"""
Regénère les éléments dérivés de la documentation (figures PNG, etc.).

Les chapitres Markdown dans docs/ sont versionnés manuellement ; ce script
exécute les générateurs automatisés listés ci-dessous.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    scripts = [
        ROOT / "docs" / "scripts" / "gen_graphique_trainee_rov.py",
    ]
    for script in scripts:
        if not script.is_file():
            print(f"Script introuvable : {script}", file=sys.stderr)
            return 1
        print(f"Exécution : {script.relative_to(ROOT)}")
        r = subprocess.run([sys.executable, str(script)], cwd=str(ROOT))
        if r.returncode != 0:
            return int(r.returncode)
    print("Documentation dérivée : OK.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
