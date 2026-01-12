"""REST API routes"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Dict, Any
from ..utils.config import DEFAULT_PARAMS

router = APIRouter()


class SimulationParams(BaseModel):
    """Paramètres de simulation"""
    m_ROV: float = None
    a: float = None
    b: float = None
    h: float = None
    Cx_ROV: float = None
    Cy_ROV: float = None
    m_bateau: float = None
    d: float = None
    rho_cable: float = None
    Cx_cable: float = None
    L_initial: float = None
    N_segments: int = None
    dt: float = None
    rho_eau: float = None
    g: float = None
    V_courant_type: str = None
    V_courant_value: float = None


@router.get("/")
async def root():
    """Point d'entrée de l'API"""
    return {"message": "ROV Simulator API", "version": "1.0"}


@router.get("/api/config")
async def get_config():
    """Récupère la configuration par défaut"""
    return DEFAULT_PARAMS


@router.post("/api/config")
async def set_config(params: SimulationParams):
    """
    Configure les paramètres de simulation
    
    Note: Les paramètres sont stockés dans le SimulationManager
    """
    # Les paramètres seront appliqués via WebSocket
    config = params.dict(exclude_none=True)
    return {"status": "ok", "config": config}


@router.get("/api/health")
async def health_check():
    """Vérification de santé de l'API"""
    return {"status": "healthy"}

