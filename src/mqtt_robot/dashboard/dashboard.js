// MQTT Dashboard for Robot Car
// Uses mqtt.js library (WebSocket connection)

let brokerConnection = null;
let isConnectedToBroker = false;

// Check if mqtt.js is loaded
window.addEventListener('load', function() {
    console.log("Dashboard loaded");
    if (typeof mqtt !== 'undefined') {
        console.log("mqtt.js library loaded successfully");
    } else {
        console.error("ERROR: mqtt.js library not loaded!");
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
            
            // Subscribe to robot topics
            brokerConnection.subscribe("robot/telemetry", (err) => {
                if (err) {
                    console.error("Subscription error:", err);
                } else {
                    console.log("✓ Subscribed to robot/telemetry");
                }
            });
            
            brokerConnection.subscribe("robot/line_sensor", (err) => {
                if (err) {
                    console.error("Subscription error:", err);
                } else {
                    console.log("✓ Subscribed to robot/line_sensor");
                }
            });
            
            brokerConnection.subscribe("robot/barcode", (err) => {
                if (err) {
                    console.error("Subscription error:", err);
                } else {
                    console.log("✓ Subscribed to robot/barcode");
                }
            });
            
            brokerConnection.subscribe("robot/ultrasonic", (err) => {
                if (err) {
                    console.error("Subscription error:", err);
                } else {
                    console.log("✓ Subscribed to robot/ultrasonic");
                }
            });
            
            brokerConnection.subscribe("robot/imu", (err) => {
                if (err) {
                    console.error("Subscription error:", err);
                } else {
                    console.log("✓ Subscribed to robot/imu");
                }
            });
            
            brokerConnection.subscribe("robot/state", (err) => {
                if (err) {
                    console.error("Subscription error:", err);
                } else {
                    console.log("✓ Subscribed to robot/state");
                }
            });
            
            brokerConnection.subscribe("robot/obstacle", (err) => {
                if (err) {
                    console.error("Subscription error:", err);
                } else {
                    console.log("✓ Subscribed to robot/obstacle");
                }
            });
            
            brokerConnection.subscribe("robot/line_events", (err) => {
                if (err) {
                    console.error("Subscription error:", err);
                } else {
                    console.log("✓ Subscribed to robot/line_events");
                }
            });
            
            showCommandStatus("Connected and subscribed!", "success");
            setTimeout(() => {
                document.getElementById('commandStatus').textContent = '';
                document.getElementById('commandStatus').className = 'command-status';
            }, 2000);
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
                // ------------------ Robot Telemetry ------------------
                case "robot/telemetry":
                    // Update encoder and motor values
                    if (parsedData.enc1 !== undefined) {
                        const enc1Element = document.getElementById("enc1");
                        if (enc1Element) enc1Element.innerText = parsedData.enc1;
                    }
                    if (parsedData.enc2 !== undefined) {
                        const enc2Element = document.getElementById("enc2");
                        if (enc2Element) enc2Element.innerText = parsedData.enc2;
                    }
                    if (parsedData.m1 !== undefined) {
                        const m1Element = document.getElementById("m1");
                        if (m1Element) m1Element.innerText = parsedData.m1.toFixed(2);
                    }
                    if (parsedData.m2 !== undefined) {
                        const m2Element = document.getElementById("m2");
                        if (m2Element) m2Element.innerText = parsedData.m2.toFixed(2);
                    }
                    if (parsedData.ts !== undefined) {
                        const tsElement = document.getElementById("timestamp");
                        if (tsElement) tsElement.innerText = parsedData.ts;
                    }
                    // Update speed data
                    if (parsedData.speed_kmh !== undefined) {
                        const speedElement = document.getElementById("speedKmh");
                        if (speedElement) speedElement.innerText = parsedData.speed_kmh.toFixed(2);
                    }
                    if (parsedData.speed_left !== undefined) {
                        const speedLeftElement = document.getElementById("speedLeft");
                        if (speedLeftElement) speedLeftElement.innerText = parsedData.speed_left.toFixed(2);
                    }
                    if (parsedData.speed_right !== undefined) {
                        const speedRightElement = document.getElementById("speedRight");
                        if (speedRightElement) speedRightElement.innerText = parsedData.speed_right.toFixed(2);
                    }
                    // Update distance traveled
                    if (parsedData.distance_m !== undefined) {
                        const distanceElement = document.getElementById("distanceTraveled");
                        if (distanceElement) distanceElement.innerText = parsedData.distance_m.toFixed(3);
                    }
                    console.log("Robot telemetry:", parsedData);
                    updateLastUpdateTime();
                    break;
                    
                // ------------------ Line Sensor Data ------------------
                case "robot/line_sensor":
                    // Update ADC value (always show, even if 0)
                    if (parsedData.adc !== undefined) {
                        const adcElement = document.getElementById('lineADC');
                        if (adcElement) {
                            adcElement.textContent = parsedData.adc;
                        }
                    }
                    
                    // Update On Line status (handle both boolean and integer 0/1)
                    if (parsedData.on_line !== undefined) {
                        const onLineEl = document.getElementById('onLine');
                        if (onLineEl) {
                            // Handle both boolean and integer (0/1) values
                            const isOnLine = parsedData.on_line === true || parsedData.on_line === 1 || parsedData.on_line === "1";
                            onLineEl.textContent = isOnLine ? "Yes" : "No";
                            onLineEl.style.color = isOnLine ? "#10b981" : "#ef4444";
                        }
                    }
                    
                    // Update Error value
                    if (parsedData.error !== undefined) {
                        const errorElement = document.getElementById('lineError');
                        if (errorElement) {
                            errorElement.textContent = parseFloat(parsedData.error).toFixed(3);
                        }
                    }
                    
                    // Update Correction value
                    if (parsedData.correction !== undefined) {
                        const correctionElement = document.getElementById('lineCorrection');
                        if (correctionElement) {
                            correctionElement.textContent = parseFloat(parsedData.correction).toFixed(3);
                        }
                    }
                    
                    console.log("Line sensor data:", parsedData);
                    updateLastUpdateTime();
                    break;
                    
                // ------------------ Barcode Data ------------------
                case "robot/barcode":
                    if (parsedData.message !== undefined) {
                        document.getElementById('barcodeMessage').textContent = parsedData.message;
                        document.getElementById('barcodeMessage').style.color = "#10b981";
                    }
                    if (parsedData.length !== undefined) {
                        document.getElementById('barcodeLength').textContent = parsedData.length;
                    }
                    console.log("Barcode decoded:", parsedData);
                    break;

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

                // ------------------ Ultrasonic / Obstacle Detection ------------------
                case "robot/ultrasonic":
                    if (parsedData.dist_cm !== undefined) {
                        const objDistElement = document.getElementById("obstacleDistance");
                        if (objDistElement) objDistElement.innerText = parsedData.dist_cm.toFixed(1) + " cm";
                    }
                    break;
                    
                case "robot/obstacle":
                    if (parsedData.dist_cm !== undefined) {
                        const objDistElement = document.getElementById("obstacleDistance");
                        if (objDistElement) objDistElement.innerText = parsedData.dist_cm.toFixed(1) + " cm";
                    }
                    if (parsedData.left_width !== undefined) {
                        const leftWidthElement = document.getElementById("obstacleLeftWidth");
                        if (leftWidthElement) leftWidthElement.innerText = parsedData.left_width.toFixed(1) + " cm";
                    }
                    if (parsedData.right_width !== undefined) {
                        const rightWidthElement = document.getElementById("obstacleRightWidth");
                        if (rightWidthElement) rightWidthElement.innerText = parsedData.right_width.toFixed(1) + " cm";
                    }
                    if (parsedData.chosen_path !== undefined) {
                        const chosenPathElement = document.getElementById("obstacleChosenPath");
                        if (chosenPathElement) chosenPathElement.innerText = parsedData.chosen_path;
                    }
                    if (parsedData.recovery_status !== undefined) {
                        const recovStatusElement = document.getElementById("obstacleRecoveryStatus");
                        if (recovStatusElement) {
                            recovStatusElement.innerText = parsedData.recovery_status;
                            // Color code recovery status
                            if (parsedData.recovery_status === "REJOINED") {
                                recovStatusElement.style.color = "#10b981";
                            } else if (parsedData.recovery_status === "FAILED") {
                                recovStatusElement.style.color = "#ef4444";
                            } else {
                                recovStatusElement.style.color = "#f59e0b";
                            }
                        }
                        // Add to obstacle log
                        addObstacleLog(parsedData);
                    }
                    break;
                    
                // ------------------ System State ------------------
                case "robot/state":
                    console.log("System state received:", parsedData);
                    if (parsedData.state !== undefined && parsedData.state !== null) {
                        const stateElement = document.getElementById("systemState");
                        if (stateElement) {
                            stateElement.innerText = parsedData.state;
                            console.log("Updated system state to:", parsedData.state);
                            
                            // Update mode status in line following panel
                            const modeStatusElement = document.getElementById("modeStatus");
                            if (modeStatusElement) {
                                modeStatusElement.textContent = "Mode: " + parsedData.state;
                            }
                        } else {
                            console.error("systemState element not found in DOM!");
                        }
                    } else {
                        console.warn("System state data missing 'state' field:", parsedData);
                    }
                    updateLastUpdateTime();
                    break;
                    
                // ------------------ IMU Data ------------------
                case "robot/imu":
                    if (parsedData.heading_raw !== undefined) {
                        const headingRawElement = document.getElementById("headingRaw");
                        if (headingRawElement) headingRawElement.innerText = parsedData.heading_raw.toFixed(1);
                    }
                    if (parsedData.heading_filtered !== undefined) {
                        const headingFilteredElement = document.getElementById("headingFiltered");
                        if (headingFilteredElement) headingFilteredElement.innerText = parsedData.heading_filtered.toFixed(1);
                    }
                    if (parsedData.distance_m !== undefined) {
                        const distanceElement = document.getElementById("distanceTraveled");
                        if (distanceElement) distanceElement.innerText = parsedData.distance_m.toFixed(3);
                    }
                    if (parsedData.correction !== undefined) {
                        const imuCorrectionElement = document.getElementById("imuCorrection");
                        if (imuCorrectionElement) imuCorrectionElement.innerText = parsedData.correction.toFixed(3);
                    }
                    if (parsedData.ax !== undefined) {
                        const accelXElement = document.getElementById("accelX");
                        if (accelXElement) accelXElement.innerText = parsedData.ax.toFixed(3);
                    }
                    if (parsedData.ay !== undefined) {
                        const accelYElement = document.getElementById("accelY");
                        if (accelYElement) accelYElement.innerText = parsedData.ay.toFixed(3);
                    }
                    if (parsedData.az !== undefined) {
                        const accelZElement = document.getElementById("accelZ");
                        if (accelZElement) accelZElement.innerText = parsedData.az.toFixed(3);
                    }
                    break;
                    
                // ------------------ Line Events ------------------
                case "robot/line_events":
                    if (parsedData.event !== undefined) {
                        const lineEventElement = document.getElementById("lineEventDisplay");
                        if (lineEventElement) {
                            const eventText = parsedData.event === "ON_LINE" ? "ON LINE" : "OFF LINE";
                            lineEventElement.innerText = eventText;
                            lineEventElement.style.color = parsedData.event === "ON_LINE" ? "#10b981" : "#ef4444";
                        }
                        
                        // Display ADC value (1 for on line, 0 for off line)
                        if (parsedData.adc !== undefined) {
                            const lineEventADCElement = document.getElementById("lineEventADC");
                            if (lineEventADCElement) {
                                lineEventADCElement.innerText = parsedData.adc;
                                lineEventADCElement.style.color = parsedData.adc === 1 ? "#10b981" : "#ef4444";
                            }
                        }
                        
                        // Add timestamp (handle both milliseconds and seconds)
                        if (parsedData.ts !== undefined) {
                            const lineEventTimeElement = document.getElementById("lineEventTime");
                            if (lineEventTimeElement) {
                                // Timestamp from Pico is in milliseconds since boot
                                // Convert to Date object (will show relative time, but readable)
                                const timestamp = parseInt(parsedData.ts);
                                const date = new Date(timestamp);
                                // If timestamp is too small (likely milliseconds since boot, not epoch), show as relative time
                                if (timestamp < 1000000000000) {
                                    // It's milliseconds since boot, show as time string
                                    const hours = Math.floor(timestamp / 3600000) % 24;
                                    const minutes = Math.floor((timestamp % 3600000) / 60000);
                                    const seconds = Math.floor((timestamp % 60000) / 1000);
                                    lineEventTimeElement.innerText = `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
                                } else {
                                    // It's a proper epoch timestamp
                                    lineEventTimeElement.innerText = date.toLocaleTimeString();
                                }
                            }
                        }
                    }
                    console.log("Line event:", parsedData);
                    break;

                default:
                    // Try to handle robot/telemetry even if topic doesn't match exactly
                    if (messageTopic.includes("robot/telemetry") || 
                        (messageTopic.includes("telemetry") && parsedData.enc1 !== undefined)) {
                        console.log("Handling telemetry via default case:", messageTopic);
                        // Update encoder and motor values
                        if (parsedData.enc1 !== undefined) {
                            const enc1Element = document.getElementById("enc1");
                            if (enc1Element) enc1Element.innerText = parsedData.enc1;
                        }
                        if (parsedData.enc2 !== undefined) {
                            const enc2Element = document.getElementById("enc2");
                            if (enc2Element) enc2Element.innerText = parsedData.enc2;
                        }
                        if (parsedData.m1 !== undefined) {
                            const m1Element = document.getElementById("m1");
                            if (m1Element) m1Element.innerText = parsedData.m1.toFixed(2);
                        }
                        if (parsedData.m2 !== undefined) {
                            const m2Element = document.getElementById("m2");
                            if (m2Element) m2Element.innerText = parsedData.m2.toFixed(2);
                        }
                        if (parsedData.ts !== undefined) {
                            const tsElement = document.getElementById("timestamp");
                            if (tsElement) tsElement.innerText = parsedData.ts;
                        }
                        updateLastUpdateTime();
                        console.log("Robot telemetry (fallback):", parsedData);
                    } else {
                        console.log("⚠️ Unhandled topic:", messageTopic);
                    }
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

// Send directional command
function sendDirection(direction, event) {
    // Check connection first - same as emergency stop
    if (!isConnectedToBroker || !brokerConnection) {
        showCommandStatus("Not connected to MQTT broker", "error");
        console.error("Cannot send directional command: Not connected to MQTT broker");
        return;
    }
    
    let m1 = 0.0;
    let m2 = 0.0;
    let directionName = "";
    
    // Motor calibration for straight movement
    // Adjust these values if the car drifts left/right when going "straight"
    // If car drifts RIGHT: increase m1 (left motor) or decrease m2 (right motor)
    // If car drifts LEFT: decrease m1 (left motor) or increase m2 (right motor)
    const STRAIGHT_M1 = 0.80;  // Left motor (increased for more power)
    const STRAIGHT_M2 = 0.80;  // Right motor (increased for more power)
    
    // Set motor values based on direction
    switch(direction) {
        case 'straight':
            m1 = STRAIGHT_M1;
            m2 = STRAIGHT_M2;
            directionName = "Straight";
            break;
        case 'backward':
            m1 = -STRAIGHT_M1;  // Use same calibration for backward
            m2 = -STRAIGHT_M2;
            directionName = "Backward";
            break;
        case 'clockwise':
            m1 = 0.8;  // Increased rotation speed
            m2 = -0.8;
            directionName = "Clockwise";
            break;
        case 'anticlockwise':
            m1 = -0.8;  // Increased rotation speed
            m2 = 0.8;
            directionName = "Anticlockwise";
            break;
        default:
            showCommandStatus("Unknown direction", "error");
            return;
    }
    
    // Update slider displays
    document.getElementById('motor1Slider').value = m1;
    document.getElementById('motor2Slider').value = m2;
    updateMotorDisplay(1, m1);
    updateMotorDisplay(2, m2);
    
    // Show immediate feedback - sending command
    showCommandStatus(`Sending ${directionName} command...`, "success");
    
    // Find and activate the pressed button
    const buttons = document.querySelectorAll('.btn-direction');
    buttons.forEach(btn => btn.classList.remove('active', 'pulsing'));
    
    // Get the button that was clicked
    const clickedButton = event?.target || document.querySelector(`.btn-${direction}`);
    if (clickedButton && clickedButton.classList.contains('btn-direction')) {
        clickedButton.classList.add('active', 'pulsing');
        
        // Remove pulsing after animation
        setTimeout(() => {
            clickedButton.classList.remove('pulsing');
        }, 600);
        
        // Remove active state after 2 seconds
        setTimeout(() => {
            clickedButton.classList.remove('active');
        }, 2000);
    }
    
    // Send command
    const commandPayload = {
        m1: m1,
        m2: m2
    };
    
    brokerConnection.publish("robot/commands", JSON.stringify(commandPayload), { qos: 0 }, (err) => {
        if (err) {
            console.error("Publish error:", err);
            showCommandStatus(`Failed to send ${directionName} command`, "error");
            // Remove active state on error
            if (clickedButton) {
                clickedButton.classList.remove('active', 'pulsing');
            }
        } else {
            showCommandStatus(`✅ ${directionName} command sent successfully! (m1=${m1.toFixed(2)}, m2=${m2.toFixed(2)})`, "success");
        }
    });
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
        } else if (type === 'error') {
            // Error messages persist longer (5 seconds) so user can see them
            setTimeout(() => {
                statusEl.textContent = '';
                statusEl.className = 'command-status';
            }, 5000);
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

// ============================================================
// Line Following Functions
// ============================================================
function enableLineFollowing() {
    if (!isConnectedToBroker || !brokerConnection) {
        showCommandStatus("Not connected to MQTT broker", "error");
        return;
    }
    
    const command = JSON.stringify({ mode: "line_follow" });
    brokerConnection.publish("robot/commands", command, { qos: 0 }, (err) => {
        if (err) {
            showCommandStatus("Failed to enable line following", "error");
        } else {
            showCommandStatus("✅ Line following mode enabled", "success");
            document.getElementById('modeStatus').textContent = "Mode: Line Following";
            document.getElementById('lineFollowBtn').classList.add('active');
            document.getElementById('imuStraightBtn').classList.remove('active');
            document.getElementById('manualBtn').classList.remove('active');
        }
    });
}

function enableIMUStraight() {
    if (!isConnectedToBroker || !brokerConnection) {
        showCommandStatus("Not connected to MQTT broker", "error");
        return;
    }
    
    const command = JSON.stringify({ mode: "imu_straight" });
    brokerConnection.publish("robot/commands", command, { qos: 0 }, (err) => {
        if (err) {
            showCommandStatus("Failed to enable IMU straight mode", "error");
        } else {
            showCommandStatus("✅ IMU Straight-Line mode enabled", "success");
            document.getElementById('modeStatus').textContent = "Mode: IMU Straight-Line";
            document.getElementById('imuStraightBtn').classList.add('active');
            document.getElementById('lineFollowBtn').classList.remove('active');
            document.getElementById('manualBtn').classList.remove('active');
        }
    });
}

function enableManualMode() {
    if (!isConnectedToBroker || !brokerConnection) {
        showCommandStatus("Not connected to MQTT broker", "error");
        return;
    }
    
    const command = JSON.stringify({ mode: "manual" });
    brokerConnection.publish("robot/commands", command, { qos: 0 }, (err) => {
        if (err) {
            showCommandStatus("Failed to enable manual mode", "error");
        } else {
            showCommandStatus("✅ Manual control mode enabled", "success");
            document.getElementById('modeStatus').textContent = "Mode: Manual";
            document.getElementById('manualBtn').classList.add('active');
            document.getElementById('lineFollowBtn').classList.remove('active');
        }
    });
}

// ============================================================
// Barcode Functions
// ============================================================
function enableBarcode() {
    if (!isConnectedToBroker || !brokerConnection) {
        showCommandStatus("Not connected to MQTT broker", "error");
        return;
    }
    
    const command = JSON.stringify({ mode: "barcode", barcode: "enable" });
    brokerConnection.publish("robot/commands", command, { qos: 0 }, (err) => {
        if (err) {
            showCommandStatus("Failed to enable barcode", "error");
        } else {
            showCommandStatus("✅ Barcode tracking enabled", "success");
            document.getElementById('barcodeEnableBtn').classList.add('active');
            document.getElementById('barcodeDisableBtn').classList.remove('active');
        }
    });
}

function disableBarcode() {
    if (!isConnectedToBroker || !brokerConnection) {
        showCommandStatus("Not connected to MQTT broker", "error");
        return;
    }
    
    const command = JSON.stringify({ mode: "barcode", barcode: "disable" });
    brokerConnection.publish("robot/commands", command, { qos: 0 }, (err) => {
        if (err) {
            showCommandStatus("Failed to disable barcode", "error");
        } else {
            showCommandStatus("Barcode tracking disabled", "success");
            document.getElementById('barcodeDisableBtn').classList.add('active');
            document.getElementById('barcodeEnableBtn').classList.remove('active');
            document.getElementById('barcodeMessage').textContent = "--";
            document.getElementById('barcodeLength').textContent = "0";
        }
    });
}

// Add obstacle encounter to log
function addObstacleLog(data) {
    const logContainer = document.getElementById('obstacleLogContainer');
    if (!logContainer) return;
    
    const logEntry = document.createElement('div');
    logEntry.className = 'obstacle-log-entry';
    
    const timestamp = data.ts ? new Date(data.ts).toLocaleTimeString() : new Date().toLocaleTimeString();
    let logText = `[${timestamp}] `;
    
    if (data.dist_cm !== undefined) {
        logText += `Distance: ${data.dist_cm.toFixed(1)}cm `;
    }
    if (data.left_width !== undefined && data.right_width !== undefined) {
        logText += `Widths: L=${data.left_width.toFixed(1)}cm R=${data.right_width.toFixed(1)}cm `;
    }
    if (data.chosen_path) {
        logText += `Path: ${data.chosen_path} `;
    }
    if (data.recovery_status) {
        logText += `Status: ${data.recovery_status}`;
    }
    
    logEntry.textContent = logText;
    
    // Color code by recovery status
    if (data.recovery_status === "REJOINED") {
        logEntry.style.color = "#10b981";
    } else if (data.recovery_status === "FAILED") {
        logEntry.style.color = "#ef4444";
    } else {
        logEntry.style.color = "#f59e0b";
    }
    
    // Add to top of log
    logContainer.insertBefore(logEntry, logContainer.firstChild);
    
    // Keep only last 10 entries
    while (logContainer.children.length > 10) {
        logContainer.removeChild(logContainer.lastChild);
    }
}

