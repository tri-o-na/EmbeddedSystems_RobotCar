// MQTT Robot Car - Phase 2: Real Motor/Encoder Integration
// This version uses real encoder data and motor control

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "mqtt_client.h"  // Also includes WiFi credentials
#include "mqtt_config.h"
#include "motor_encoder.h"  // Motor and encoder functions

// External reference to cyw43_state (defined in cyw43_arch)
extern cyw43_t cyw43_state;

// Track current motor speeds for telemetry
static float current_m1 = 0.0f;
static float current_m2 = 0.0f;

// Function to get current motor speeds (for telemetry)
void get_current_motor_speeds(float *m1, float *m2) {
    *m1 = current_m1;
    *m2 = current_m2;
}

// Command handler - processes incoming MQTT commands
void process_robot_commands(const char *topic, const char *payload, int len) {
    printf("MQTT Command received on topic '%s': %.*s\n", topic, len, payload);
    
    // Parse JSON and call motor_set() with received values
    if (strstr(payload, "\"m1\"") != NULL || strstr(payload, "\"m2\"") != NULL) {
        // Extract motor values
        float m1 = 0.0f, m2 = 0.0f;
        char *m1_pos = strstr(payload, "\"m1\"");
        char *m2_pos = strstr(payload, "\"m2\"");
        
        if (m1_pos) {
            m1_pos = strchr(m1_pos, ':');
            if (m1_pos) {
                m1 = strtof(m1_pos + 1, NULL);
            }
        }
        if (m2_pos) {
            m2_pos = strchr(m2_pos, ':');
            if (m2_pos) {
                m2 = strtof(m2_pos + 1, NULL);
            }
        }
        
        // Clamp values to safe range
        if (m1 > 1.0f) m1 = 1.0f;
        if (m1 < -1.0f) m1 = -1.0f;
        if (m2 > 1.0f) m2 = 1.0f;
        if (m2 < -1.0f) m2 = -1.0f;
        
        printf("  Parsed command: m1=%.2f, m2=%.2f\n", m1, m2);
        
        // Actually control the motors!
        motor_set(m1, m2);
        current_m1 = m1;
        current_m2 = m2;
        
        printf("  Motors set: m1=%.2f, m2=%.2f\n", m1, m2);
    } else if (strstr(payload, "\"action\"") != NULL) {
        if (strstr(payload, "stop") != NULL) {
            printf("  Emergency stop command received\n");
            motors_stop();
            current_m1 = 0.0f;
            current_m2 = 0.0f;
            printf("  Motors stopped\n");
        }
    }
}

int main() {
    // Initialize stdio FIRST with immediate output
    stdio_init_all();
    printf("START\n");
    fflush(stdout);
    
    // Small delay to let USB initialize
    sleep_ms(1500);
    
    printf("\n=== MQTT Robot Car - Phase 2 ===\n");
    printf("Step 1: Initializing motors and encoders...\n");
    
    // Initialize motors and encoders FIRST (before WiFi)
    motors_and_encoders_init();
    motors_stop();  // Start with motors stopped
    
    printf("Step 2: Testing LED...\n");
    
    // Test LED IMMEDIATELY - before WiFi
    if (cyw43_arch_init()) {
        printf("ERROR: Failed to initialize WiFi hardware\n");
        while(1) { 
            // Flash LED rapidly to show error
            sleep_ms(100);
        }
        return -1;
    }
    
    printf("Step 3: LED test - turning ON for 2 seconds...\n");
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(2000);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    printf("Step 4: LED test complete\n");
    
    printf("Step 5: Enabling station mode...\n");
    cyw43_arch_enable_sta_mode();
    printf("Station mode enabled\n");
    
    printf("\n=== WiFi Connection Details ===\n");
    printf("SSID: '%s'\n", WIFI_SSID);
    printf("Password length: %d characters\n", (int)strlen(WIFI_PASS));
    printf("Attempting connection (this may take 30 seconds)...\n\n");
    
    int wifi_result = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, 
                                                          CYW43_AUTH_WPA2_AES_PSK, 30000);
    
    if (wifi_result) {
        printf("\n=== WiFi Connection Failed ===\n");
        printf("Error code: %d\n", wifi_result);
        
        // Provide helpful error messages based on error code
        if (wifi_result == -2 || wifi_result == -3) {
            printf("Error -2/-3: Connection timeout or authentication failed\n");
            printf("Troubleshooting steps:\n");
            printf("  1. Verify WiFi password is correct: '%s'\n", WIFI_PASS);
            printf("  2. Ensure network '%s' is in range and broadcasting\n", WIFI_SSID);
            printf("  3. Check that network supports 2.4GHz (Pico W only supports 2.4GHz)\n");
            printf("  4. Try moving Pico W closer to router\n");
            printf("  5. Verify router isn't blocking the device (MAC filtering)\n");
        } else {
            printf("Unknown error code. Common causes:\n");
            printf("  - Wrong WiFi password\n");
            printf("  - Network not available (5GHz only, out of range)\n");
            printf("  - Router security settings\n");
        }
        
        // Flash LED to show WiFi failure
        while(1) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(500);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            sleep_ms(500);
        }
        return -1;
    }
    
    printf("\n=== WiFi Connected! ===\n");
    const ip4_addr_t *ip = netif_ip4_addr(netif_default);
    printf("IP address: %s\n", ip4addr_ntoa(ip));
    
    // Set up MQTT
    printf("Setting up MQTT...\n");
    robot_set_command_handler(process_robot_commands);
    
    printf("Starting MQTT client...\n");
    robot_mqtt_init_and_connect(MQTT_BROKER_IP, MQTT_BROKER_PORT);
    
    sleep_ms(2000);
    
    printf("\n=== Main Loop Started ===\n");
    printf("Robot is ready! Send commands via MQTT to control motors.\n");
    uint32_t last_telemetry = 0;
    uint32_t last_blink = 0;
    absolute_time_t last_connection_check = get_absolute_time();
    
    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Blink LED every second
        if (now - last_blink > 1000) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(50);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            last_blink = now;
        }
        
        // Poll MQTT and network MORE FREQUENTLY to drain queue faster
        // This is critical for preventing TCP send queue overflow
        for (int i = 0; i < 3; i++) {
            robot_mqtt_process_events();
        }
        robot_mqtt_ensure_connected();
        
        // ============================================================
        // Connection Monitoring Logic (from reference code)
        // Check every 5 seconds to avoid spamming
        // ============================================================
        if (absolute_time_diff_us(last_connection_check, get_absolute_time()) > 5 * 1000000) {
            if (!robot_mqtt_check_connection()) {
                int wifi_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
                
                if (wifi_status == CYW43_LINK_UP) {
                    // Wi-Fi is fine → broker likely down
                    printf("⚠️ MQTT broker unreachable, Wi-Fi is OK. Trying to reconnect to broker...\n");
                    robot_mqtt_ensure_connected();
                } else {
                    // Wi-Fi is actually down → attempt reconnect
                    printf("⚠️ Wi-Fi connection lost (status=%d). Attempting Wi-Fi reconnect...\n", wifi_status);
                    int r = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS,
                                                               CYW43_AUTH_WPA2_AES_PSK, 10000);
                    if (r == 0) {
                        printf("✅ Wi-Fi reconnected successfully.\n");
                        const ip4_addr_t *ip = netif_ip4_addr(netif_default);
                        printf("New IP address: %s\n", ip4addr_ntoa(ip));
                    } else {
                        printf("❌ Wi-Fi reconnection failed (err %d).\n", r);
                    }
                }
            }
            last_connection_check = get_absolute_time();
        }
        
        // Publish telemetry periodically (backpressure checking is done inside)
        if (robot_mqtt_check_connection() && (now - last_telemetry > TELEMETRY_INTERVAL_MS)) {
            robot_send_telemetry_data();
            last_telemetry = now;
        }
        
        // Shorter sleep to process events more frequently (helps drain TCP queue)
        sleep_ms(25);
    }
    
    return 0;
}

