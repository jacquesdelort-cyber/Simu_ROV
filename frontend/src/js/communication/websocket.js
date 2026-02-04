/** WebSocket client for communication with backend */

export class WebSocketClient {
    constructor(url) {
        this.url = url;
        this.ws = null;
        this.callbacks = {
            onState: null,
            onError: null,
            onConnect: null,
            onDisconnect: null
        };
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
    }
    
    connect() {
        try {
            this.ws = new WebSocket(this.url);
            
            this.ws.onopen = () => {
                console.log("WebSocket connecté");
                this.reconnectAttempts = 0;
                if (this.callbacks.onConnect) {
                    this.callbacks.onConnect();
                }
            };
            
            this.ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    
                    if (data.type === "state" && this.callbacks.onState) {
                        this.callbacks.onState(data);
                    } else if (data.type === "config_ack") {
                        console.log("Configuration appliquée");
                    } else if (data.type === "start_ack") {
                        console.log("Simulation démarrée");
                    } else if (data.type === "stop_ack") {
                        console.log("Simulation arrêtée");
                    } else if (data.type === "reset_ack") {
                        console.log("Simulation réinitialisée");
                    }
                } catch (e) {
                    console.error("Erreur parsing message:", e);
                }
            };
            
            this.ws.onerror = (error) => {
                console.error("WebSocket erreur:", error);
                if (this.callbacks.onError) {
                    this.callbacks.onError(error);
                }
            };
            
            this.ws.onclose = () => {
                console.log("WebSocket fermé");
                if (this.callbacks.onDisconnect) {
                    this.callbacks.onDisconnect();
                }
                
                // Tentative de reconnexion
                if (this.reconnectAttempts < this.maxReconnectAttempts) {
                    this.reconnectAttempts++;
                    console.log(`Tentative de reconnexion ${this.reconnectAttempts}/${this.maxReconnectAttempts}...`);
                    setTimeout(() => this.connect(), 2000);
                }
            };
        } catch (e) {
            console.error("Erreur création WebSocket:", e);
        }
    }
    
    sendCommand(command) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify({
                type: "command",
                ...command
            }));
        } else {
            console.warn("WebSocket non connecté");
        }
    }
    
    sendConfig(params) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify({
                type: "config",
                params: params
            }));
        } else {
            console.warn("WebSocket non connecté");
        }
    }
    
    start() {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify({ type: "start" }));
        } else {
            console.warn("WebSocket non connecté");
        }
    }
    
    stop() {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify({ type: "stop" }));
        } else {
            console.warn("WebSocket non connecté");
        }
    }
    
    reset() {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify({ type: "reset" }));
        } else {
            console.warn("WebSocket non connecté");
        }
    }
    
    disconnect() {
        if (this.ws) {
            this.ws.close();
        }
    }
    
    isConnected() {
        return this.ws && this.ws.readyState === WebSocket.OPEN;
    }
}

