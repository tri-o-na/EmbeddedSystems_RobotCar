# EmbeddedSystems_RobotCar

### Robotic Car Group Project – AY25/26

---

## 🔍 Overview
This project showcases an **intelligent autonomous line-following robot** built using the **Raspberry Pi Pico W**. The car integrates multiple sensors and subsystems to perform real-time navigation, obstacle avoidance, barcode decoding, and telemetry streaming via MQTT.

---

## 🔧 Key Features
1. **Line Following (PID-Controlled)**
   - Single IR sensor used for line detection.
   - PID algorithm ensures smooth and accurate tracking.

2. **Barcode Navigation**
   - Code39 barcodes embedded on the track instruct the robot (LEFT / RIGHT / STOP / U-TURN).
   - Barcode decoding handled by a separate IR sensor.

3. **Obstacle Detection and Avoidance**
   - Ultrasonic sensor mounted on a servo performs environmental scanning.
   - Calculates obstacle width and determines the optimal path (left or right).
   - Automatically rejoins the line after detouring.

4. **MQTT Telemetry Dashboard**
   - Live data streaming via WiFi and MQTT.
   - Dashboard visualizes: speed, distance, line position, heading, obstacle width, and state transitions.

---

## 🛠️ Hardware Components
- **Raspberry Pi Pico W (RP2040)**
- **Motor Driver** (TB6612FNG / L298N)
- **DC Motors** with wheel encoders
- **IR Line Sensor(s)**
- **Ultrasonic Sensor (HC-SR04)** with **Servo (SG90)**
- **IMU**
- **Power supply and common ground**

---

## 🚀 System Workflow
1. Initialize all peripherals (motors, line sensors, ultrasonic, IMU, WiFi/MQTT).
2. Enter main loop:
   - Read line sensor and compute PID correction.
   - Adjust motor speeds to maintain line tracking.
   - Detect and decode barcodes to perform route changes.
   - Continuously check for obstacles and execute avoidance.
   - Publish live telemetry (speed, heading, obstacle, barcode) to MQTT.
3. Repeat until the course is complete.

---

## 📊 MQTT Dashboard Telemetry
### Displayed Data
- **Speed & Distance** (from encoders)
- **Line position & barcode events**
- **IMU heading / acceleration**
- **Obstacle detection and path choice**
- **Recovery & movement state**
- **Final log summary:** speed, distance, IMU, line events, barcode, obstacle encounters

---

## 🔌 Building & Flashing
```bash
mkdir build && cd build
cmake ..
make
```
Then copy the generated `.uf2` file to the Pico W (BOOTSEL mode).

---

## 🔑 Usage
1. Place the robot on a black-line track.
2. Power it on and ensure it connects to WiFi and the MQTT broker.
3. Open the dashboard to view real-time telemetry.
4. Observe live line-following, barcode commands, and obstacle avoidance.

---

## 👥 Contributors
Developed by **Team 12** for the *Embedded Systems (AY25/26)* course.

---

