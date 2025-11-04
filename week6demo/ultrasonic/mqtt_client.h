#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <stdbool.h>

// MQTT API functions
void robot_mqtt_init_and_connect(const char *broker_ip, int port);
void robot_mqtt_process_events(void);
void robot_mqtt_ensure_connected(void);
bool robot_mqtt_check_connection(void);
void robot_set_command_handler(void (*callback)(const char *topic, const char *payload, int len));

// Helper function for publishing with backpressure control
bool mqtt_publish_message_safe(const char *topic, const char *payload);

// Telemetry function
void robot_send_telemetry_data(void);

// In mqtt_config.h, add these topics:
#define MQTT_TOPIC_TELEMETRY "telemetry/obstacle"
#define MQTT_TOPIC_DISTANCE "telemetry/distance"
#define MQTT_TOPIC_SCAN "telemetry/scan"
#define MQTT_TOPIC_COMMANDS "robot/commands"

// In mqtt_config.h, use your laptop's IP:
#define MQTT_BROKER_IP "172.20.10.3"  // Your actual laptop IP
#define MQTT_BROKER_PORT 1883

#endif