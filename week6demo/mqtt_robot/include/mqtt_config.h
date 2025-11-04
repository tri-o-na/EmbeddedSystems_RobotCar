#pragma once

// MQTT Configuration
#define MQTT_BROKER_IP "172.20.10.4"  // Change to your MQTT broker IP
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "pico_robot_car"

// MQTT Topics
#define MQTT_TOPIC_TELEMETRY "robot/telemetry"
#define MQTT_TOPIC_COMMANDS "robot/commands"
#define MQTT_TOPIC_BARCODE "robot/barcode"
#define MQTT_TOPIC_LINE_SENSOR "robot/line_sensor"

// Telemetry publish interval (milliseconds)
#define TELEMETRY_INTERVAL_MS 1000

