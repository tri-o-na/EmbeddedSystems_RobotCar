// MQTT Client Implementation for Pico W using lwIP MQTT Client API
// Uses built-in lwIP MQTT client for reliability

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/timeouts.h"
#include "mqtt_config.h"
#include "pico/time.h"

// External reference to cyw43_state (defined in cyw43_arch)
extern cyw43_t cyw43_state;

// ============================================================
// lwIP Poll Helper
// ============================================================
void robot_mqtt_process_events(void) {
    sys_check_timeouts();
    cyw43_arch_poll();
}

// ============================================================
// Manual struct definition (needed for Pico SDK lwIP)
// ============================================================
struct mqtt_connect_client_info_custom {
    const char *client_id;
    const char *client_user;
    const char *client_pass;
    uint16_t    keep_alive;
    const char *will_topic;
    const char *will_msg;
    uint8_t     will_qos;
    uint8_t     will_retain;
};

// ============================================================
// Global MQTT state
// ============================================================
static mqtt_client_t *mqtt_client_instance = NULL;
static ip_addr_t mqtt_broker_ip;
static int mqtt_broker_port_num;
static struct mqtt_connect_client_info_custom mqtt_client_config;
static bool mqtt_has_ever_connected = false;

// Callback function pointer for received commands
static void (*mqtt_cmd_callback)(const char *topic, const char *payload, int len) = NULL;

// ============================================================
// Backpressure Control State
// ============================================================
static bool mqtt_publish_in_progress = false;
static absolute_time_t mqtt_last_publish_time = 0;  // Initialize to 0 (nil_time equivalent)
static const uint32_t MQTT_MIN_PUBLISH_INTERVAL_MS = 800;  // Minimum 800ms between publishes
static const uint32_t MQTT_PUBLISH_DRAIN_TIMEOUT_MS = 1500; // Max time to wait for queue to drain

// ============================================================
// MQTT Connection Status Callback
// ============================================================
static void mqtt_conn_status_callback(mqtt_client_t *client,
                                       void *arg,
                                       mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        mqtt_has_ever_connected = true;
        printf("[MQTT] Connection accepted by broker\n");
        
        // Subscribe to commands topic after successful connection
        printf("[MQTT] Subscribing to topic: %s\n", MQTT_TOPIC_COMMANDS);
        err_t sub_err = mqtt_sub_unsub(client, MQTT_TOPIC_COMMANDS, 0, NULL, NULL, 0);
        if (sub_err == ERR_OK) {
            printf("[MQTT] Subscription request sent successfully\n");
        } else {
            printf("[MQTT] ERROR: Subscription failed with error: %d\n", sub_err);
        }
    } else {
        printf("[MQTT] Connection failed (status %d)\n", status);
    }
}

// Topic storage for data callback
static char incoming_topic_buffer[64] = {0};

// ============================================================
// MQTT Incoming Publish Callback (called when publish arrives)
// ============================================================
static void mqtt_incoming_publish_callback(void *arg,
                                            const char *topic,
                                            u32_t tot_len) {
    printf("[MQTT] Incoming publish on topic '%s' (%lu bytes)\n", topic, (unsigned long)tot_len);
    // Store topic for data callback
    strncpy(incoming_topic_buffer, topic, sizeof(incoming_topic_buffer) - 1);
    incoming_topic_buffer[sizeof(incoming_topic_buffer) - 1] = '\0';
}

// ============================================================
// MQTT Incoming Data Callback (called with actual payload data)
// ============================================================
static void mqtt_incoming_data_callback(void *arg,
                                         const u8_t *data,
                                         u16_t len,
                                         u8_t flags) {
    static char payload_buffer[512] = {0};
    static u32_t total_received = 0;
    
    // Accumulate data chunks
    if (total_received + len < sizeof(payload_buffer)) {
        memcpy(&payload_buffer[total_received], data, len);
        total_received += len;
    }
    
    // Check if this is the last chunk (MQTT_DATA_FLAG_LAST)
    if (flags & MQTT_DATA_FLAG_LAST) {
        payload_buffer[total_received] = '\0';
        
        if (mqtt_cmd_callback && total_received > 0 && incoming_topic_buffer[0] != '\0') {
            printf("[MQTT] Complete message on '%s': %.*s\n", 
                   incoming_topic_buffer, (int)total_received, payload_buffer);
            mqtt_cmd_callback(incoming_topic_buffer, payload_buffer, total_received);
        }
        
        // Reset for next message
        total_received = 0;
        incoming_topic_buffer[0] = '\0';
    }
}

// ============================================================
// Backpressure Check: Can we publish now?
// ============================================================
static bool mqtt_can_publish_now(void) {
    absolute_time_t now = get_absolute_time();
    
    // If no publish has happened yet, we can publish
    if (is_nil_time(mqtt_last_publish_time)) {
        return true;
    }
    
    // Check minimum interval between publishes (rate limiting)
    int64_t time_since_last_publish = absolute_time_diff_us(mqtt_last_publish_time, now) / 1000; // Convert to ms
    
    if (time_since_last_publish < (int64_t)MQTT_MIN_PUBLISH_INTERVAL_MS) {
        // Too soon - haven't waited long enough
        return false;
    }
    
    // Check if previous publish is still in progress
    if (mqtt_publish_in_progress) {
        // If publish has been in progress for too long, assume it failed or queue is stuck
        if (time_since_last_publish > (int64_t)MQTT_PUBLISH_DRAIN_TIMEOUT_MS) {
            printf("[MQTT] Previous publish timeout - resetting state\n");
            mqtt_publish_in_progress = false;
            return true; // Allow retry
        }
        return false; // Still waiting for previous publish
    }
    
    // Safe to publish
    return true;
}

// ============================================================
// Publish Message Helper with WiFi Check and Backpressure
// ============================================================
bool mqtt_publish_message_safe(const char *topic, const char *payload) {
    // BACKPRESSURE CHECK: Only publish if queue has drained
    if (!mqtt_can_publish_now()) {
        // Silently skip - this is normal flow control
        return false;
    }
    
    // Check WiFi link first
    int wifi_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    
    if (wifi_status != CYW43_LINK_UP) {
        printf("[MQTT] WiFi disconnected (status=%d) — cannot publish to %s\n", wifi_status, topic);
        return false;
    }
    
    // Check if MQTT session is alive
    if (mqtt_client_is_connected(mqtt_client_instance) == 0) {
        printf("[MQTT] MQTT not connected — skipping publish to %s\n", topic);
        return false;
    }
    
    // Mark publish as in progress BEFORE attempting
    mqtt_publish_in_progress = true;
    mqtt_last_publish_time = get_absolute_time();
    
    // Drain queue before publishing by processing events multiple times
    for (int i = 0; i < 3; i++) {
        robot_mqtt_process_events();
        sleep_ms(10);
    }
    
    // Attempt publish
    err_t pub_err = mqtt_publish(mqtt_client_instance, topic, payload,
                                 strlen(payload), 0, 0, NULL, NULL);
    
    if (pub_err == ERR_OK) {
        printf("[MQTT] Published to %s: %s\n", topic, payload);
        
        // Drain queue after publish to help send immediately
        for (int i = 0; i < 5; i++) {
            robot_mqtt_process_events();
            sleep_ms(5);
        }
        
        // Mark publish as complete after a short delay (to allow TCP to queue)
        // The actual completion will be detected by the minimum interval check
        // Reset flag after a conservative time to allow queue to drain
        // We'll reset it on next successful can_publish_now check
        
    } else {
        printf("[MQTT] Publish failed to %s (error %d)\n", topic, pub_err);
        mqtt_publish_in_progress = false; // Reset immediately on error
        
        // Recheck WiFi
        int recheck = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        if (recheck != CYW43_LINK_UP) {
            printf("[MQTT] WiFi link lost, will reconnect on next cycle\n");
        }
        return false;
    }
    
    // Reset in_progress flag after minimum interval (ensuring queue has time to drain)
    // This will be checked on next call to can_publish_now()
    // We keep it set to prevent rapid re-publishing
    
    return true;
}

// ============================================================
// Public API: Publish Telemetry (with backpressure control)
// ============================================================
void robot_send_telemetry_data(void) {
    if (mqtt_client_instance == NULL || mqtt_client_is_connected(mqtt_client_instance) == 0) {
        return;
    }
    
    static uint32_t telemetry_counter = 0;
    char telemetry_payload[256];
    
    // Mock telemetry data (Phase 1)
    uint32_t enc1 = 1000 + (telemetry_counter % 500);
    uint32_t enc2 = 1200 + (telemetry_counter % 500);
    float m1 = 0.5f + (telemetry_counter % 100) * 0.01f;
    if (m1 > 1.0f) m1 = 1.0f;
    float m2 = 0.4f + (telemetry_counter % 100) * 0.01f;
    if (m2 > 1.0f) m2 = 1.0f;
    
    snprintf(telemetry_payload, sizeof(telemetry_payload), 
             "{\"enc1\":%lu,\"enc2\":%lu,\"m1\":%.2f,\"m2\":%.2f,\"ts\":%lu}",
             (unsigned long)enc1, (unsigned long)enc2, m1, m2,
             (unsigned long)to_ms_since_boot(get_absolute_time()));
    
    // Attempt publish with backpressure checking
    // This will return false if queue is full or too soon since last publish
    if (mqtt_publish_message_safe(MQTT_TOPIC_TELEMETRY, telemetry_payload)) {
        telemetry_counter++;
    } else {
        // Backpressure: skip this publish cycle
        // Counter NOT incremented so we'll try same data next time
    }
    
    // Update publish_in_progress flag: reset if enough time has passed
    if (mqtt_publish_in_progress && !is_nil_time(mqtt_last_publish_time)) {
        absolute_time_t now = get_absolute_time();
        int64_t time_elapsed = absolute_time_diff_us(mqtt_last_publish_time, now) / 1000;
        
        // After minimum interval + small buffer, reset the flag to allow next publish
        if (time_elapsed > (int64_t)(MQTT_MIN_PUBLISH_INTERVAL_MS + 100)) {
            mqtt_publish_in_progress = false;
        }
    }
}

// ============================================================
// Public API: Set Command Callback
// ============================================================
void robot_set_command_handler(void (*callback)(const char *topic, const char *payload, int len)) {
    mqtt_cmd_callback = callback;
}

// ============================================================
// Public API: Start MQTT Client
// ============================================================
void robot_mqtt_init_and_connect(const char *broker_ip, int port) {
    printf("[MQTT] Starting MQTT client...\n");
    printf("[MQTT] Resolving broker IP: %s\n", broker_ip);
    
    if (!ip4addr_aton(broker_ip, &mqtt_broker_ip)) {
        printf("[MQTT] ERROR: Invalid broker IP format: %s\n", broker_ip);
        return;
    }
    
    mqtt_broker_port_num = port;
    
    // Create MQTT client instance
    mqtt_client_instance = mqtt_client_new();
    if (!mqtt_client_instance) {
        printf("[MQTT] ERROR: Failed to create MQTT client\n");
        return;
    }
    
    // Set up client configuration
    mqtt_client_config.client_id   = MQTT_CLIENT_ID;
    mqtt_client_config.client_user = NULL;
    mqtt_client_config.client_pass = NULL;
    mqtt_client_config.keep_alive  = 60;
    mqtt_client_config.will_topic  = NULL;
    mqtt_client_config.will_msg    = NULL;
    mqtt_client_config.will_qos    = 0;
    mqtt_client_config.will_retain  = 0;
    
    // Set up incoming message callbacks
    // Note: third parameter (arg) will be passed to publish callback, 
    // then that arg is passed to data callback
    mqtt_set_inpub_callback(mqtt_client_instance, 
                            mqtt_incoming_publish_callback,
                            mqtt_incoming_data_callback,
                            incoming_topic_buffer);
    
    printf("[MQTT] Connecting to broker %s:%d...\n", broker_ip, port);
    err_t conn_err = mqtt_client_connect(mqtt_client_instance, 
                                         &mqtt_broker_ip, 
                                         mqtt_broker_port_num,
                                         mqtt_conn_status_callback,
                                         NULL,
                                         (const struct mqtt_connect_client_info_t *)&mqtt_client_config);
    
    if (conn_err == ERR_OK) {
        printf("[MQTT] Connection request sent, waiting for broker response...\n");
    } else {
        printf("[MQTT] ERROR: mqtt_client_connect() failed immediately (error %d)\n", conn_err);
    }
}

// ============================================================
// Public API: Reconnect if Needed
// ============================================================
static bool mqtt_reconnect_in_progress = false;
static absolute_time_t mqtt_last_reconnect_time;

void robot_mqtt_ensure_connected(void) {
    if (mqtt_client_instance == NULL || mqtt_reconnect_in_progress) {
        return;
    }
    
    // Don't reconnect until we've ever connected successfully
    if (!mqtt_has_ever_connected) {
        return;
    }
    
    // Retry every 5 seconds at most
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(mqtt_last_reconnect_time, now) < 5 * 1000000) {
        return;
    }
    
    mqtt_last_reconnect_time = now;
    
    if (mqtt_client_is_connected(mqtt_client_instance) == 0) {
        mqtt_reconnect_in_progress = true;
        printf("[MQTT] Disconnected. Attempting reconnect...\n");
        
        // Properly disconnect
        mqtt_disconnect(mqtt_client_instance);
        for (int i = 0; i < 5; i++) {
            robot_mqtt_process_events();
            sleep_ms(100);
        }
        
        // Free old client
        mqtt_client_free(mqtt_client_instance);
        
        // Create new client
        mqtt_client_instance = mqtt_client_new();
        if (!mqtt_client_instance) {
            printf("[MQTT] ERROR: Failed to recreate MQTT client\n");
            mqtt_reconnect_in_progress = false;
            sleep_ms(2000);
            return;
        }
        
        // Re-resolve broker IP (in case it changed)
        char broker_ip_str[16];
        snprintf(broker_ip_str, sizeof(broker_ip_str), MQTT_BROKER_IP);
        if (!ip4addr_aton(broker_ip_str, &mqtt_broker_ip)) {
            printf("[MQTT] ERROR: Invalid broker IP on reconnect\n");
            mqtt_reconnect_in_progress = false;
            return;
        }
        
        // Set up callbacks again
        mqtt_set_inpub_callback(mqtt_client_instance,
                                mqtt_incoming_publish_callback,
                                mqtt_incoming_data_callback,
                                NULL);
        
        printf("[MQTT] Reconnecting to broker %s:%d...\n", broker_ip_str, mqtt_broker_port_num);
        err_t recon_err = mqtt_client_connect(mqtt_client_instance,
                                              &mqtt_broker_ip,
                                              mqtt_broker_port_num,
                                              mqtt_conn_status_callback,
                                              NULL,
                                              (const struct mqtt_connect_client_info_t *)&mqtt_client_config);
        
        if (recon_err == ERR_OK) {
            printf("[MQTT] Reconnect attempt started (waiting for broker)...\n");
        } else {
            printf("[MQTT] ERROR: Reconnect connect() failed (error %d)\n", recon_err);
        }
        
        mqtt_reconnect_in_progress = false;
    }
}

// ============================================================
// Public API: Check Connection Status
// ============================================================
bool robot_mqtt_check_connection(void) {
    if (mqtt_client_instance == NULL) {
        return false;
    }
    return mqtt_client_is_connected(mqtt_client_instance) != 0;
}