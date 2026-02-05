"""FastAPI application main entry point"""

from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import os

from .api import routes
from .api.websocket import websocket_endpoint

app = FastAPI(
    title="ROV Simulator API",
    description="API pour simulateur ROV-Câble-Bateau",
    version="1.0.0"
)

# CORS middleware pour permettre connexions depuis frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # En production, spécifier les origines autorisées
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Inclure routes REST
app.include_router(routes.router)

# WebSocket endpoint
@app.websocket("/ws")
async def websocket_route(websocket: WebSocket):
    """Route WebSocket pour simulation temps réel"""
    await websocket_endpoint(websocket)


@app.get("/")
async def root():
    """Point d'entrée"""
    return {
        "message": "ROV Simulator API",
        "version": "1.0.0",
        "endpoints": {
            "websocket": "/ws",
            "api": "/api",
            "docs": "/docs"
        }
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

