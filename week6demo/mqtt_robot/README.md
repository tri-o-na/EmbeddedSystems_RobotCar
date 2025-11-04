# MQTT Robot Car - Phase 1

This project implements MQTT communication for the Pico W robot car. **Phase 1** uses mock/dummy data to test MQTT connectivity independently from hardware components. **Phase 2** will integrate real motor and encoder hardware.

## Project Structure

```
week6demo/mqtt_robot/
├── src/
│   ├── main.c              # Main program (Phase 1: mock data)
│   └── mqtt_pico.c         # MQTT client implementation
├── include/
│   ├── mqtt_client.h       # MQTT client API
│   ├── mqtt_config.h       # Configuration (WiFi, broker, topics)
│   └── lwipopts.h          # lwIP stack configuration
├── dashboard/
│   ├── dashboard.html      # HTML dashboard interface
│   ├── dashboard.css       # Dashboard styling
│   └── dashboard.js        # Dashboard MQTT client
└── README.md               # This file
```

## Prerequisites

1. **Mosquitto MQTT Broker** - Install and run on your computer or network
   - Windows: Download from https://mosquitto.org/download/
   - macOS: `brew install mosquitto`
   - Linux: `sudo apt-get install mosquitto mosquitto-clients`

2. **Network** - Pico W and your computer must be on the same network

## Configuration

### 1. WiFi Configuration

Edit `include/mqtt_config.h`:

```c
#define WIFI_SSID "YourWiFiNetwork"
#define WIFI_PASS "YourPassword"
```

### 2. MQTT Broker Configuration

Edit `include/mqtt_config.h`:

```c
#define MQTT_BROKER_IP "192.168.1.100"  // Your broker IP address
#define MQTT_BROKER_PORT 1883
```

**To find your broker IP:**
- Windows: Run `ipconfig` in CMD, look for "IPv4 Address"
- macOS/Linux: Run `ifconfig` or `ip addr`, look for your network interface IP

### 3. Dashboard Configuration

Open `dashboard/dashboard.js` and update if needed (or use the UI in the dashboard):

```javascript
const brokerIP = document.getElementById('brokerIP').value;  // Default: 192.168.1.100
const brokerPort = 1883;
```

## Building and Flashing

1. **Build the project:**
   ```bash
   mkdir -p build
   cd build
   cmake ..
   cmake --build . -j
   ```

2. **Flash to Pico W:**
   - Hold BOOTSEL button on Pico W
   - Connect via USB
   - Copy `build/mqtt_robot.uf2` to the `RPI-RP2` drive

3. **Monitor output:**
   - Connect via serial monitor (115200 baud)
   - Or use USB serial to see debug messages

## Running Mosquitto Broker

### Start Mosquitto

**macOS/Linux:**
```bash
mosquitto -v
```

**Windows:**
```bash
mosquitto.exe -v
```

The `-v` flag enables verbose output for debugging.

### Test Broker (Optional)

In a separate terminal:

```bash
# Subscribe to telemetry (listen for messages)
mosquitto_sub -h localhost -t "robot/telemetry"

# Publish a test command
mosquitto_pub -h localhost -t "robot/commands" -m '{"m1":0.5,"m2":0.5}'
```

## Using the Dashboard

1. **Open the dashboard:**
   - Simply open `dashboard/dashboard.html` in a web browser
   - Or use a local web server (recommended):
     ```bash
     # Python 3
     cd dashboard
     python3 -m http.server 8000
     # Then open http://localhost:8000/dashboard.html
     ```

2. **Connect to MQTT broker:**
   - Enter your broker IP address (e.g., `192.168.1.100`)
   - Enter port (default: `1883`)
   - Click "Connect"

3. **View telemetry:**
   - Telemetry data will appear automatically when connected
   - Data updates every second (1000ms)

4. **Control motors:**
   - Use sliders to set motor speeds (-1.0 to 1.0)
   - Release slider to send command
   - Click "Emergency Stop" to send stop command
   - Click "Set to Zero" to set both motors to 0

## MQTT Topics

- **`robot/telemetry`** (Publish) - Robot publishes sensor/motor data
  - Format: `{"enc1":1234,"enc2":5678,"m1":0.5,"m2":0.5,"ts":123456}`
  
- **`robot/commands`** (Subscribe) - Robot subscribes to receive commands
  - Format: `{"m1":0.7,"m2":0.6}` or `{"action":"stop"}`

## Phase 1 vs Phase 2

### Phase 1 (Current Implementation)
- ✅ MQTT connection and communication
- ✅ WiFi connectivity
- ✅ Mock/dummy telemetry data
- ✅ Command reception and parsing (logged only)
- ❌ No actual motor control
- ❌ No real encoder readings

### Phase 2 (Future)
- Integrate `motor_encoder.c` from `week6demo/Integration/`
- Replace mock data with real encoder readings
- Replace command logging with actual `motor_set()` calls
- Add safety limits and emergency stop functionality

## Troubleshooting

### WiFi Connection Fails
- Check SSID and password in `mqtt_config.h`
- Ensure network is 2.4 GHz (Pico W doesn't support 5 GHz)
- Check signal strength

### MQTT Connection Fails
- Verify Mosquitto is running: `mosquitto -v`
- Check broker IP address is correct
- Ensure firewall allows port 1883
- Verify Pico W and broker are on same network

### No Telemetry in Dashboard
- Check MQTT connection status (should show "Connected")
- Verify topics match: `robot/telemetry`
- Check browser console for errors (F12)
- Verify Mosquitto is running

### Commands Not Received
- Check broker IP in dashboard configuration
- Verify subscription to `robot/commands` topic
- Check serial output for error messages
- Test with `mosquitto_pub` command-line tool

## Message Format

### Telemetry (Published by Robot)
```json
{
  "enc1": 1234,      // Left encoder pulse width (microseconds)
  "enc2": 5678,      // Right encoder pulse width (microseconds)
  "m1": 0.5,         // Motor 1 speed (-1.0 to 1.0)
  "m2": 0.6,         // Motor 2 speed (-1.0 to 1.0)
  "ts": 123456       // Timestamp (milliseconds since boot)
}
```

### Commands (Received by Robot)
```json
{
  "m1": 0.7,         // Motor 1 speed (-1.0 to 1.0)
  "m2": 0.6          // Motor 2 speed (-1.0 to 1.0)
}
```

Or:
```json
{
  "action": "stop"   // Emergency stop
}
```

## Next Steps (Phase 2)

1. Copy `motor_encoder.c` and `motor_encoder.h` from `week6demo/Integration/`
2. Include motor initialization in `main.c`
3. Replace mock telemetry with real encoder readings
4. Replace command logging with `motor_set()` calls
5. Add safety features (max speed limits, watchdog)

## Notes

- The MQTT client implementation in `mqtt_pico.c` is a basic implementation. If you have a working `mqtt_pico.c` from your example project, you can replace this file.
- This implementation uses QoS 0 (at most once delivery) for simplicity
- Connection reconnection logic is included but may need tuning
- Keep-alive is set to 60 seconds

