#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

// WiFi credentials - UPDATE THESE
#define WIFI_SSID "GWiPhone"
#define WIFI_PASS "b5rw1m9rrbox3"

// MQTT broker settings - USE PUBLIC BROKER
#define MQTT_BROKER_IP "172.20.10.3"  // Must match your Mosquitto IP
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "robot_car_obstacle_12345"  // Make this unique

// MQTT topics
#define MQTT_TOPIC_TELEMETRY "telemetry/obstacle"
#define MQTT_TOPIC_COMMANDS "robot/commands"

// Telemetry interval
#define TELEMETRY_INTERVAL_MS 1000

// Increase MQTT timeout values
#define MQTT_PUBLISH_TIMEOUT_MS 5000  // Increase from default
#define MQTT_KEEPALIVE_INTERVAL 60    // Increase keepalive

#endif