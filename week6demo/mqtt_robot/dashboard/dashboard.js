// MQTT Dashboard for Robot Car
// Uses mqtt.js library (WebSocket connection)

let brokerConnection = null;
let isConnectedToBroker = false;

// Check if mqtt.js is loaded
window.addEventListener('load', function() {
    console.log("Dashboard loaded");
    if (typeof mqtt !== 'undefined') {
        console.log("✓ mqtt.js library loaded successfully");
    } else {
        console.error("✗ ERROR: mqtt.js library not loaded!");
        alert("Error: MQTT library not loaded. Check internet connection.");
    }
});

// Initialize MQTT connection
function connectMQTT() {
    const brokerIP = document.getElementById('brokerIP').value;
    const brokerPort = parseInt(document.getElementById('brokerPort').value);
    
    console.log("connectMQTT called with:", brokerIP, brokerPort);
    
    if (!brokerIP || !brokerPort) {
        showCommandStatus("Please enter broker IP and port", "error");
        console.error("Missing broker IP or port");
        return;
    }
    
    // Disconnect existing connection
    if (brokerConnection && brokerConnection.connected) {
        console.log("Disconnecting existing client");
        brokerConnection.end();
        brokerConnection = null;
    }
    
    // Connect to MQTT broker over WebSocket
    const webSocketUrl = `ws://${brokerIP}:${brokerPort}`;
    console.log(`Connecting to MQTT broker: ${webSocketUrl}`);
    
    updateConnectionStatus(false, "Connecting...");
    showCommandStatus("Connecting to broker...", "success");
    
    try {
        brokerConnection = mqtt.connect(webSocketUrl);
        
        // Connection success callback
        brokerConnection.on("connect", () => {
            console.log("MQTT Connected successfully");
            isConnectedToBroker = true;
            updateConnectionStatus(true, "Connected");
            document.getElementById("statusText").innerText = "Connected to MQTT Broker";
            
            // Subscribe to telemetry wildcard topic
            brokerConnection.subscribe("telemetry/#", (err) => {
                if (err) {
                    console.error("Subscription error:", err);
                } else {
                    console.log("✓ Subscribed to telemetry/#");
                    showCommandStatus("Connected and subscribed!", "success");
                    setTimeout(() => {
                        document.getElementById('commandStatus').textContent = '';
                        document.getElementById('commandStatus').className = 'command-status';
                    }, 2000);
                }
            });
        });
        
        // Connection error callback
        brokerConnection.on("error", (error) => {
            console.error("MQTT Connection error:", error);
            isConnectedToBroker = false;
            updateConnectionStatus(false, "Connection error");
            showCommandStatus("Connection error: " + error.message, "error");
        });
        
        // Connection close callback
        brokerConnection.on("close", () => {
            console.log("MQTT Connection closed");
            isConnectedToBroker = false;
            updateConnectionStatus(false, "Disconnected");
        });
        
        // Message received callback
        brokerConnection.on("message", (messageTopic, messagePayload) => {
            console.log("Received:", messageTopic, messagePayload.toString());
            
            let parsedData;
            try {
                parsedData = JSON.parse(messagePayload.toString());
            } catch (err) {
                console.error("Invalid JSON received:", messagePayload.toString());
                return;
            }
            
            // Route message based on topic
            switch (messageTopic) {
                // ------------------ Motion & IMU ------------------
                case "telemetry/speed":
                    if (parsedData.speed !== undefined) {
                        // Update speed display if element exists
                        const speedElement = document.getElementById("speedValue");
                        if (speedElement) {
                            speedElement.innerText = parsedData.speed.toFixed(2);
                        } else {
                            console.log("Speed:", parsedData.speed.toFixed(2));
                        }
                    }
                    break;

                case "telemetry/heading":
                    if (parsedData.heading !== undefined) {
                        const headingElement = document.getElementById("headingValue");
                        if (headingElement) {
                            headingElement.innerText = parsedData.heading.toFixed(1);
                        } else {
                            console.log("Heading:", parsedData.heading.toFixed(1));
                        }
                    }
                    break;

                case "telemetry/imu/raw":
                    if (parsedData.ax !== undefined) {
                        const rawAxElement = document.getElementById("raw_ax");
                        if (rawAxElement) rawAxElement.innerText = parsedData.ax.toFixed(2);
                    }
                    if (parsedData.ay !== undefined) {
                        const rawAyElement = document.getElementById("raw_ay");
                        if (rawAyElement) rawAyElement.innerText = parsedData.ay.toFixed(2);
                    }
                    if (parsedData.az !== undefined) {
                        const rawAzElement = document.getElementById("raw_az");
                        if (rawAzElement) rawAzElement.innerText = parsedData.az.toFixed(2);
                    }
                    break;

                case "telemetry/imu/filtered":
                    if (parsedData.ax !== undefined) {
                        const axElement = document.getElementById("ax");
                        if (axElement) axElement.innerText = parsedData.ax.toFixed(2);
                    }
                    if (parsedData.ay !== undefined) {
                        const ayElement = document.getElementById("ay");
                        if (ayElement) ayElement.innerText = parsedData.ay.toFixed(2);
                    }
                    if (parsedData.az !== undefined) {
                        const azElement = document.getElementById("az");
                        if (azElement) azElement.innerText = parsedData.az.toFixed(2);
                    }
                    break;

                // ------------------ Line Following + Barcode ------------------
                case "telemetry/barcode":
                    const barcodeElement = document.getElementById("barcodeValue");
                    if (barcodeElement) {
                        barcodeElement.innerText = parsedData.barcode || "--";
                    } else {
                        console.log("Barcode:", parsedData.barcode || "--");
                    }
                    break;

                case "telemetry/state":
                    const stateElement = document.getElementById("stateValue");
                    if (stateElement) {
                        stateElement.innerText = parsedData.state || "--";
                    } else {
                        console.log("State:", parsedData.state || "--");
                    }
                    break;

                case "telemetry/distance":
                    if (parsedData.distance !== undefined) {
                        const distElement = document.getElementById("distValue");
                        if (distElement) {
                            distElement.innerText = parsedData.distance.toFixed(2);
                        } else {
                            console.log("Distance:", parsedData.distance.toFixed(2));
                        }
                    }
                    break;

                // ------------------ Obstacle Detection ------------------
                case "telemetry/obstacle":
                    if (parsedData.distance !== undefined) {
                        const objDistElement = document.getElementById("objdistValue");
                        if (objDistElement) objDistElement.innerText = parsedData.distance.toFixed(2);
                    }
                    if (parsedData.clearance !== undefined) {
                        const clearElement = document.getElementById("clearValue");
                        if (clearElement) clearElement.innerText = parsedData.clearance;
                    }
                    if (parsedData.recovery_status !== undefined) {
                        const recovStatusElement = document.getElementById("recovstatusValue");
                        if (recovStatusElement) recovStatusElement.innerText = parsedData.recovery_status;
                    }
                    if (parsedData.obstacle_width !== undefined) {
                        const obstacleWidthElement = document.getElementById("ObstaclewidthValue");
                        if (obstacleWidthElement) obstacleWidthElement.innerText = parsedData.obstacle_width;
                    }
                    if (parsedData.line_event !== undefined) {
                        const lineEventElement = document.getElementById("lineEventValue");
                        if (lineEventElement) lineEventElement.innerText = parsedData.line_event;
                    }
                    break;

                default:
                    console.log("⚠️ Unhandled topic:", messageTopic);
            }
            
            // Update last update time
            updateLastUpdateTime();
        });
        
    } catch (error) {
        console.error("Connection error (catch block):", error);
        updateConnectionStatus(false, "Connection failed");
        showCommandStatus("Failed to connect: " + error.message, "error");
    }
}

// Update connection status UI
function updateConnectionStatus(isConnected, text) {
    const indicator = document.getElementById('statusIndicator');
    const statusText = document.getElementById('statusText');
    
    if (isConnected) {
        indicator.classList.add('connected');
        statusText.textContent = text || "Connected";
    } else {
        indicator.classList.remove('connected');
        statusText.textContent = text || "Disconnected";
    }
}

// Helper to update last update time
function updateLastUpdateTime() {
    const lastUpdateElement = document.getElementById('lastUpdate');
    if (lastUpdateElement) {
        lastUpdateElement.textContent = new Date().toLocaleTimeString();
    }
}

// Update motor display values
function updateMotorDisplay(motorNum, value) {
    const valueElement = document.getElementById(`motor${motorNum}Value`);
    if (valueElement) {
        valueElement.textContent = parseFloat(value).toFixed(2);
    }
}

// Send motor command
function sendMotorCommand() {
    if (!isConnectedToBroker || !brokerConnection) {
        showCommandStatus("Not connected to MQTT broker", "error");
        return;
    }
    
    const motor1Value = parseFloat(document.getElementById('motor1Slider').value);
    const motor2Value = parseFloat(document.getElementById('motor2Slider').value);
    
    const commandPayload = {
        m1: motor1Value,
        m2: motor2Value
    };
    
    // Publish to commands topic (adjust topic name if needed)
    brokerConnection.publish("robot/commands", JSON.stringify(commandPayload), { qos: 0 }, (err) => {
        if (err) {
            console.error("Publish error:", err);
            showCommandStatus("Failed to send command", "error");
        } else {
            showCommandStatus(`Command sent: m1=${motor1Value.toFixed(2)}, m2=${motor2Value.toFixed(2)}`, "success");
        }
    });
}

// Send stop command
function sendStop() {
    if (!isConnectedToBroker || !brokerConnection) {
        showCommandStatus("Not connected to MQTT broker", "error");
        return;
    }
    
    const stopCommandPayload = { action: "stop" };
    
    brokerConnection.publish("robot/commands", JSON.stringify(stopCommandPayload), { qos: 0 }, (err) => {
        if (err) {
            console.error("Publish error:", err);
            showCommandStatus("Failed to send stop command", "error");
        } else {
            // Reset sliders
            document.getElementById('motor1Slider').value = 0;
            document.getElementById('motor2Slider').value = 0;
            updateMotorDisplay(1, 0);
            updateMotorDisplay(2, 0);
            
            showCommandStatus("Emergency stop sent!", "success");
        }
    });
}

// Send zero command
function sendZero() {
    document.getElementById('motor1Slider').value = 0;
    document.getElementById('motor2Slider').value = 0;
    updateMotorDisplay(1, 0);
    updateMotorDisplay(2, 0);
    sendMotorCommand();
}

// Show command status message
function showCommandStatus(message, type) {
    const statusEl = document.getElementById('commandStatus');
    if (statusEl) {
        statusEl.textContent = message;
        statusEl.className = 'command-status ' + (type || '');
        
        if (type === 'success') {
            setTimeout(() => {
                statusEl.textContent = '';
                statusEl.className = 'command-status';
            }, 3000);
        }
    }
}

// Send motor command when slider is released (on mouseup/touchend)
document.addEventListener('DOMContentLoaded', function() {
    const motor1Slider = document.getElementById('motor1Slider');
    const motor2Slider = document.getElementById('motor2Slider');
    
    if (motor1Slider) {
        motor1Slider.addEventListener('mouseup', sendMotorCommand);
        motor1Slider.addEventListener('touchend', sendMotorCommand);
    }
    
    if (motor2Slider) {
        motor2Slider.addEventListener('mouseup', sendMotorCommand);
        motor2Slider.addEventListener('touchend', sendMotorCommand);
    }
});

