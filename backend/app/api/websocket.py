"""WebSocket handlers for real-time simulation"""

import asyncio
import json
from fastapi import WebSocket, WebSocketDisconnect
from typing import Dict, Any
from ..solvers.coupled_solver import CoupledSolver
from ..utils.config import DEFAULT_PARAMS


class SimulationManager:
    """Gestionnaire de simulation"""
    
    def __init__(self):
        self.running = False
        self.params = DEFAULT_PARAMS.copy()
        self.solver = None
        self.websocket = None
        self.send_interval = 1.0 / 30.0  # 30 Hz pour affichage
        
    def configure(self, params: Dict[str, Any]):
        """Configure la simulation avec de nouveaux paramètres"""
        self.params.update(params)
        if self.solver:
            self.solver.reset(self.params)
        else:
            self.solver = CoupledSolver(self.params)
    
    async def run_simulation(self, websocket: WebSocket):
        """Boucle principale de simulation"""
        self.websocket = websocket
        self.running = True
        
        if not self.solver:
            self.solver = CoupledSolver(self.params)
        
        last_send_time = 0.0
        
        try:
            while self.running:
                # Calculer un pas de temps
                state = self.solver.step()
                
                # Envoyer état à fréquence d'affichage
                current_time = state['t']
                if current_time - last_send_time >= self.send_interval:
                    state_data = {
                        "type": "state",
                        "t": state['t'],
                        "rov": state['rov'],
                        "boat": state['boat'],
                        "cable": state['cable']
                    }
                    
                    try:
                        await websocket.send_json(state_data)
                        last_send_time = current_time
                    except Exception as e:
                        print(f"Erreur envoi WebSocket: {e}")
                        break
                
                # Petit délai pour ne pas saturer le CPU
                await asyncio.sleep(0.001)
                
        except Exception as e:
            print(f"Erreur dans boucle simulation: {e}")
        finally:
            self.running = False
    
    def set_commands(self, Fx_ROV=0.0, Fy_ROV=0.0, vx_bateau_cmd=0.0, dL_dt=0.0):
        """Met à jour les commandes"""
        if self.solver:
            self.solver.set_commands(Fx_ROV, Fy_ROV, vx_bateau_cmd, dL_dt)
    
    def stop(self):
        """Arrête la simulation"""
        self.running = False
    
    def reset(self):
        """Réinitialise la simulation"""
        if self.solver:
            self.solver.reset()
        self.running = False


# Instance globale du gestionnaire
sim_manager = SimulationManager()


async def websocket_endpoint(websocket: WebSocket):
    """Handler WebSocket principal"""
    await websocket.accept()
    
    try:
        while True:
            # Recevoir message
            data = await websocket.receive_json()
            msg_type = data.get("type")
            
            if msg_type == "config":
                # Configuration
                params = data.get("params", {})
                sim_manager.configure(params)
                await websocket.send_json({
                    "type": "config_ack",
                    "status": "configured"
                })
                
            elif msg_type == "command":
                # Commande temps réel
                sim_manager.set_commands(
                    Fx_ROV=data.get("Fx_ROV", 0.0),
                    Fy_ROV=data.get("Fy_ROV", 0.0),
                    vx_bateau_cmd=data.get("vx_bateau_cmd", 0.0),
                    dL_dt=data.get("dL_dt", 0.0)
                )
                
            elif msg_type == "start":
                # Démarrer simulation
                if not sim_manager.running:
                    # Démarrer dans une tâche asynchrone
                    asyncio.create_task(sim_manager.run_simulation(websocket))
                    await websocket.send_json({
                        "type": "start_ack",
                        "status": "started"
                    })
                else:
                    await websocket.send_json({
                        "type": "start_ack",
                        "status": "already_running"
                    })
                    
            elif msg_type == "stop":
                # Arrêter simulation
                sim_manager.stop()
                await websocket.send_json({
                    "type": "stop_ack",
                    "status": "stopped"
                })
                
            elif msg_type == "reset":
                # Réinitialiser simulation
                sim_manager.reset()
                await websocket.send_json({
                    "type": "reset_ack",
                    "status": "reset"
                })
                
            elif msg_type == "get_state":
                # Obtenir état actuel
                if sim_manager.solver:
                    state = sim_manager.solver.get_state()
                    await websocket.send_json({
                        "type": "state",
                        **state
                    })
                    
    except WebSocketDisconnect:
        sim_manager.stop()
        print("Client WebSocket déconnecté")
    except Exception as e:
        print(f"Erreur WebSocket: {e}")
        sim_manager.stop()

