// MQTT Dashboard for Robot Car
// Uses Paho MQTT JavaScript Client

let mqttClient = null;
let connected = false;

// MQTT Topics
const TOPIC_TELEMETRY = "robot/telemetry";
const TOPIC_COMMANDS = "robot/commands";

// Initialize MQTT connection
function connectMQTT() {
    const brokerIP = document.getElementById('brokerIP').value;
    const brokerPort = parseInt(document.getElementById('brokerPort').value);
    
    if (!brokerIP || !brokerPort) {
        showCommandStatus("Please enter broker IP and port", "error");
        return;
    }
    
    const broker = `${brokerIP}:${brokerPort}`;
    
    // Disconnect existing connection
    if (mqttClient && mqttClient.isConnected()) {
        mqttClient.disconnect();
    }
    
    // Create new client
    const clientId = "dashboard_" + Math.random().toString(16).substr(2, 8);
    
    mqttClient = new Paho.MQTT.Client(broker, clientId);
    
    // Set callback handlers
    mqttClient.onConnectionLost = onConnectionLost;
    mqttClient.onMessageArrived = onMessageArrived;
    
    // Connect options
    const options = {
        onSuccess: onConnect,
        onFailure: onConnectFailure,
        useSSL: false,
        timeout: 10,
        keepAliveInterval: 30
    };
    
    updateConnectionStatus(false, "Connecting...");
    
    try {
        mqttClient.connect(options);
    } catch (error) {
        console.error("Connection error:", error);
        updateConnectionStatus(false, "Connection failed");
        showCommandStatus("Failed to connect: " + error.message, "error");
    }
}

// Connection success callback
function onConnect() {
    console.log("MQTT Connected");
    connected = true;
    updateConnectionStatus(true, "Connected");
    
    // Subscribe to telemetry topic
    mqttClient.subscribe(TOPIC_TELEMETRY, {
        onSuccess: function() {
            console.log("Subscribed to", TOPIC_TELEMETRY);
        },
        onFailure: function(error) {
            console.error("Subscription failed:", error);
        }
    });
    
    showCommandStatus("Connected successfully!", "success");
    setTimeout(() => {
        document.getElementById('commandStatus').textContent = '';
        document.getElementById('commandStatus').className = 'command-status';
    }, 2000);
}

// Connection failure callback
function onConnectFailure(error) {
    console.error("MQTT Connection failed:", error);
    connected = false;
    updateConnectionStatus(false, "Connection failed");
    showCommandStatus("Failed to connect: " + error.errorMessage, "error");
}

// Connection lost callback
function onConnectionLost(responseObject) {
    if (responseObject.errorCode !== 0) {
        console.error("MQTT Connection lost:", responseObject.errorMessage);
        connected = false;
        updateConnectionStatus(false, "Disconnected");
    }
}

// Message received callback
function onMessageArrived(message) {
    console.log("Message received:", message.destinationName, message.payloadString);
    
    if (message.destinationName === TOPIC_TELEMETRY) {
        try {
            const data = JSON.parse(message.payloadString);
            updateTelemetry(data);
        } catch (error) {
            console.error("Failed to parse telemetry:", error);
        }
    }
}

// Update telemetry display
function updateTelemetry(data) {
    if (data.enc1 !== undefined) document.getElementById('enc1').textContent = data.enc1;
    if (data.enc2 !== undefined) document.getElementById('enc2').textContent = data.enc2;
    if (data.m1 !== undefined) document.getElementById('m1').textContent = data.m1.toFixed(2);
    if (data.m2 !== undefined) document.getElementById('m2').textContent = data.m2.toFixed(2);
    if (data.ts !== undefined) {
        const date = new Date(data.ts);
        document.getElementById('timestamp').textContent = date.toLocaleTimeString();
    }
    
    document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();
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

// Update motor display values
function updateMotorDisplay(motorNum, value) {
    const valueElement = document.getElementById(`motor${motorNum}Value`);
    valueElement.textContent = parseFloat(value).toFixed(2);
}

// Send motor command
function sendMotorCommand() {
    if (!connected || !mqttClient) {
        showCommandStatus("Not connected to MQTT broker", "error");
        return;
    }
    
    const m1 = parseFloat(document.getElementById('motor1Slider').value);
    const m2 = parseFloat(document.getElementById('motor2Slider').value);
    
    const command = {
        m1: m1,
        m2: m2
    };
    
    const message = new Paho.MQTT.Message(JSON.stringify(command));
    message.destinationName = TOPIC_COMMANDS;
    message.qos = 0;
    
    mqttClient.send(message);
    
    showCommandStatus(`Command sent: m1=${m1.toFixed(2)}, m2=${m2.toFixed(2)}`, "success");
}

// Send stop command
function sendStop() {
    if (!connected || !mqttClient) {
        showCommandStatus("Not connected to MQTT broker", "error");
        return;
    }
    
    const command = { action: "stop" };
    const message = new Paho.MQTT.Message(JSON.stringify(command));
    message.destinationName = TOPIC_COMMANDS;
    message.qos = 0;
    
    mqttClient.send(message);
    
    // Reset sliders
    document.getElementById('motor1Slider').value = 0;
    document.getElementById('motor2Slider').value = 0;
    updateMotorDisplay(1, 0);
    updateMotorDisplay(2, 0);
    
    showCommandStatus("Emergency stop sent!", "success");
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
    statusEl.textContent = message;
    statusEl.className = 'command-status ' + (type || '');
    
    if (type === 'success') {
        setTimeout(() => {
            statusEl.textContent = '';
            statusEl.className = 'command-status';
        }, 3000);
    }
}

// Send motor command when slider is released (on mouseup/touchend)
document.getElementById('motor1Slider').addEventListener('mouseup', sendMotorCommand);
document.getElementById('motor1Slider').addEventListener('touchend', sendMotorCommand);
document.getElementById('motor2Slider').addEventListener('mouseup', sendMotorCommand);
document.getElementById('motor2Slider').addEventListener('touchend', sendMotorCommand);

// Auto-connect on page load (optional - comment out if you want manual connect)
// window.addEventListener('load', function() {
//     setTimeout(connectMQTT, 1000);
// });

