/** Main application entry point */

import { WebSocketClient } from './communication/websocket.js';
import { Visualization3D } from './visualization/renderer.js';

// Configuration
const WS_URL = 'ws://localhost:8000/ws';

// Éléments DOM
let wsClient;
let visualization;
let isRunning = false;

// Initialisation
document.addEventListener('DOMContentLoaded', () => {
    initializeVisualization();
    initializeWebSocket();
    initializeControls();
});

function initializeVisualization() {
    const container = document.getElementById('canvas-container');
    visualization = new Visualization3D(container);
}

function initializeWebSocket() {
    wsClient = new WebSocketClient(WS_URL);
    
    wsClient.callbacks.onConnect = () => {
        updateConnectionStatus(true);
    };
    
    wsClient.callbacks.onDisconnect = () => {
        updateConnectionStatus(false);
    };
    
    wsClient.callbacks.onState = (data) => {
        visualization.updateState(data);
        updateTimeDisplay(data.t);
    };
    
    wsClient.callbacks.onError = (error) => {
        console.error('Erreur WebSocket:', error);
    };
    
    wsClient.connect();
}

function initializeControls() {
    // Boutons contrôle simulation
    document.getElementById('btn-start').addEventListener('click', () => {
        if (!isRunning) {
            sendConfig();
            wsClient.start();
            isRunning = true;
            updateSimulationStatus(true);
        }
    });
    
    document.getElementById('btn-stop').addEventListener('click', () => {
        wsClient.stop();
        isRunning = false;
        updateSimulationStatus(false);
    });
    
    document.getElementById('btn-reset').addEventListener('click', () => {
        wsClient.reset();
        visualization.clearTrajectory();
        isRunning = false;
        updateSimulationStatus(false);
    });
    
    // Bouton appliquer paramètres
    document.getElementById('btn-apply-params').addEventListener('click', () => {
        sendConfig();
    });
    
    // Bouton envoyer commande
    document.getElementById('btn-send-command').addEventListener('click', () => {
        sendCommand();
    });
    
    // Contrôles caméra
    document.getElementById('camera-side').addEventListener('click', () => {
        visualization.setCameraMode('side');
    });
    
    document.getElementById('camera-top').addEventListener('click', () => {
        visualization.setCameraMode('top');
    });
    
    document.getElementById('camera-follow').addEventListener('click', () => {
        visualization.setCameraMode('follow');
    });
    
    // Envoi automatique des commandes (optionnel)
    setInterval(() => {
        if (isRunning) {
            sendCommand();
        }
    }, 100); // Toutes les 100ms
}

function sendConfig() {
    const params = {
        m_ROV: parseFloat(document.getElementById('param-m-ROV').value),
        L_initial: parseFloat(document.getElementById('param-L-initial').value),
        N_segments: parseInt(document.getElementById('param-N-segments').value),
        dt: parseFloat(document.getElementById('param-dt').value),
        V_courant_value: parseFloat(document.getElementById('param-V-courant').value)
    };
    
    wsClient.sendConfig(params);
}

function sendCommand() {
    const command = {
        Fx_ROV: parseFloat(document.getElementById('cmd-Fx-ROV').value),
        Fy_ROV: parseFloat(document.getElementById('cmd-Fy-ROV').value),
        vx_bateau_cmd: parseFloat(document.getElementById('cmd-vx-bateau').value),
        dL_dt: parseFloat(document.getElementById('cmd-dL-dt').value)
    };
    
    wsClient.sendCommand(command);
}

function updateConnectionStatus(connected) {
    const statusEl = document.getElementById('connection-status');
    if (connected) {
        statusEl.textContent = 'Connecté';
        statusEl.className = 'status connected';
    } else {
        statusEl.textContent = 'Déconnecté';
        statusEl.className = 'status disconnected';
    }
}

function updateSimulationStatus(running) {
    const statusEl = document.getElementById('simulation-status');
    if (running) {
        statusEl.textContent = 'En cours';
        statusEl.className = 'status running';
    } else {
        statusEl.textContent = 'Arrêté';
        statusEl.className = 'status stopped';
    }
}

function updateTimeDisplay(t) {
    document.getElementById('time-display').textContent = `t = ${t.toFixed(2)} s`;
}

