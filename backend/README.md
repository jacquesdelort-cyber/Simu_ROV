# Backend - Simulateur ROV

Backend Python pour le simulateur ROV-Câble-Bateau.

## Installation

1. Créer un environnement virtuel Python :
```bash
python -m venv venv
source venv/bin/activate  # Sur Windows: venv\Scripts\activate
```

2. Installer les dépendances :
```bash
pip install -r requirements.txt
```

## Lancement

```bash
python -m app.main
```

Ou avec uvicorn directement :
```bash
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

Le serveur sera accessible sur `http://localhost:8000`

## API

- **Documentation interactive** : `http://localhost:8000/docs`
- **WebSocket** : `ws://localhost:8000/ws`
- **API REST** : `http://localhost:8000/api`

## Structure

- `app/models/` : Modèles physiques (ROV, Câble, Bateau)
- `app/solvers/` : Solveurs numériques
- `app/api/` : Routes REST et WebSocket
- `app/utils/` : Utilitaires (constantes physiques, configuration)

## Tests

```bash
pytest tests/
```

