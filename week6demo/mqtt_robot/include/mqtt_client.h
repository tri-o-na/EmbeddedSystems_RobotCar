#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <stdbool.h>

void robot_mqtt_init_and_connect(const char *broker_ip, int port);
void robot_send_telemetry_data(void);
void robot_mqtt_process_events(void);
void robot_mqtt_ensure_connected(void);
bool robot_mqtt_check_connection(void);
void robot_set_command_handler(void (*callback)(const char *topic, const char *payload, int len));

#endif


#ifndef WIFI_CREDENTIALS_H
#define WIFI_CREDENTIALS_H

#define WIFI_SSID "Kai"
#define WIFI_PASS "12345678900"

#endif

