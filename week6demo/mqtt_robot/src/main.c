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
#include "barcode.h"
#include "line_follow.h"
#include "pid.h"
#include "imu_lis3dh.h"

// External reference to cyw43_state (defined in cyw43_arch)
extern cyw43_t cyw43_state;

// Track current motor speeds for telemetry
static float current_m1 = 0.0f;
static float current_m2 = 0.0f;

// Speed calculation constants
#define WHEEL_DIAMETER_MM 65.0f        // Wheel diameter in mm (adjust to your wheel size)
#define ENCODER_TICKS_PER_REV 20.0f    // Encoder ticks per wheel revolution
#define WHEEL_CIRCUMFERENCE_M ((WHEEL_DIAMETER_MM * 3.14159f) / 1000.0f)  // in meters

// Speed tracking variables
static uint32_t last_encoder_left = 0;
static uint32_t last_encoder_right = 0;
static uint64_t last_speed_calc_time = 0;
float speed_kmh_left = 0.0f;      // Non-static so mqtt_pico.c can access
float speed_kmh_right = 0.0f;     // Non-static so mqtt_pico.c can access
float speed_kmh_avg = 0.0f;       // Non-static so mqtt_pico.c can access

// Line following mode state
static bool line_following_enabled = false;
static PID line_follow_pid;
static uint64_t last_seen_black_time = 0;
static int last_turn_dir = 0; // -1 = left, +1 = right, 0 = straight
static const float LINE_FOLLOW_BASE_SPEED = 0.30f;

// Barcode tracking state
static bool barcode_tracking_enabled = false;

// IMU straight-line navigation mode
static bool imu_straight_mode = false;
static PID heading_pid;
static const float IMU_BASE_SPEED = 0.45f;
static float heading_setpoint = 0.0f;
static float heading_smooth = 0.0f;
static float distance_traveled_m = 0.0f;

// Function to calculate speed in km/h from encoder readings
void calculate_speed_kmh(void) {
    uint64_t now = time_us_64();
    uint32_t enc_left = encoder_get_count(1);
    uint32_t enc_right = encoder_get_count(2);
    
    // Calculate speed every 200ms minimum
    if (last_speed_calc_time == 0) {
        last_speed_calc_time = now;
        last_encoder_left = enc_left;
        last_encoder_right = enc_right;
        return;
    }
    
    float time_seconds = (now - last_speed_calc_time) / 1000000.0f;
    if (time_seconds < 0.2f) return;  // Update every 200ms
    
    // Calculate ticks difference
    uint32_t delta_left = enc_left - last_encoder_left;
    uint32_t delta_right = enc_right - last_encoder_right;
    
    // Calculate revolutions per second
    float rps_left = delta_left / ENCODER_TICKS_PER_REV / time_seconds;
    float rps_right = delta_right / ENCODER_TICKS_PER_REV / time_seconds;
    
    // Calculate speed: RPS × circumference (m) × 3600 (sec/hr) / 1000 (m/km)
    speed_kmh_left = rps_left * WHEEL_CIRCUMFERENCE_M * 3.6f;
    speed_kmh_right = rps_right * WHEEL_CIRCUMFERENCE_M * 3.6f;
    speed_kmh_avg = (speed_kmh_left + speed_kmh_right) / 2.0f;
    
    // Update tracking variables
    last_encoder_left = enc_left;
    last_encoder_right = enc_right;
    last_speed_calc_time = now;
}

// Function to get current motor speeds (for telemetry)
void get_current_motor_speeds(float *m1, float *m2) {
    *m1 = current_m1;
    *m2 = current_m2;
}

// Forward declaration for MQTT publish
extern bool mqtt_publish_message_safe(const char *topic, const char *payload);

// External reference to barcode decoded message
extern char decoded_message[];
extern int message_length;

// Command handler - processes incoming MQTT commands
void process_robot_commands(const char *topic, const char *payload, int len) {
    printf("MQTT Command received on topic '%s': %.*s\n", topic, len, payload);
    
    // Check for mode commands first
    if (strstr(payload, "\"mode\"") != NULL) {
        if (strstr(payload, "\"line_follow\"") != NULL || strstr(payload, "\"line_following\"") != NULL) {
            line_following_enabled = true;
            imu_straight_mode = false;
            printf("  Line following mode ENABLED\n");
            pid_init(&line_follow_pid, 0.08f, 0.00f, 0.002f, -0.25f, 0.25f);
            line_follow_pid.setpoint = 1.0f; // want to always see black (1.0)
            last_seen_black_time = time_us_64();
            last_turn_dir = 0;
        } else if (strstr(payload, "\"imu_straight\"") != NULL) {
            imu_straight_mode = true;
            line_following_enabled = false;
            printf("  IMU Straight-line mode ENABLED\n");
            pid_init(&heading_pid, 0.5f, 0.0f, 0.02f, -0.15f, 0.15f);
            heading_pid.setpoint = 0.0f;  // Target heading (will be set to current heading)
            heading_smooth = 0.0f;
            distance_traveled_m = 0.0f;
            encoder_reset_counts();
        } else if (strstr(payload, "\"manual\"") != NULL) {
            line_following_enabled = false;
            imu_straight_mode = false;
            printf("  Manual control mode ENABLED\n");
            motors_stop();
            current_m1 = 0.0f;
            current_m2 = 0.0f;
        }
        if (strstr(payload, "\"barcode\"") != NULL) {
            if (strstr(payload, "\"enable\"") != NULL || strstr(payload, "true") != NULL) {
                barcode_tracking_enabled = true;
                reset_message();
                printf("  Barcode tracking ENABLED\n");
            } else {
                barcode_tracking_enabled = false;
                printf("  Barcode tracking DISABLED\n");
            }
        }
        return; // Don't process motor commands when changing mode
    }
    
    // Emergency stop ALWAYS works, regardless of mode
    if (strstr(payload, "\"action\"") != NULL && strstr(payload, "stop") != NULL) {
        printf("  Emergency stop command received\n");
        line_following_enabled = false;  // Disable all auto modes
        imu_straight_mode = false;
        motors_stop();
        current_m1 = 0.0f;
        current_m2 = 0.0f;
        printf("  Motors stopped\n");
        return;
    }
    
    // Only process motor commands if not in auto mode
    if (!line_following_enabled && !imu_straight_mode) {
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
    
    // Ensure line following is disabled on startup
    line_following_enabled = false;
    current_m1 = 0.0f;
    current_m2 = 0.0f;
    
    // Initialize line following sensor
    printf("Step 1a: Initializing line following sensor...\n");
    lf_init();
    
    // Initialize IMU sensor
    printf("Step 1b: Initializing IMU sensor...\n");
    if (imu_init()) {
        printf("IMU initialized successfully\n");
    } else {
        printf("WARNING: IMU initialization failed\n");
    }
    
    // Initialize barcode sensor (GPIO already configured in barcode.c)
    printf("Step 1c: Barcode sensor ready (GPIO %d)\n", BARCODE_IR_SENSOR_PIN);
    
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
        
        // Safety check: Only stop motors if they're supposed to be at zero
        // This check runs every loop but only stops if values are actually zero
        if (!line_following_enabled && !imu_straight_mode && current_m1 == 0.0f && current_m2 == 0.0f) {
            // Only call motors_stop once to avoid excessive GPIO writes
            static bool last_was_stopped = false;
            if (!last_was_stopped) {
                motors_stop();
                last_was_stopped = true;
            }
        } else {
            static bool last_was_stopped = false;
            last_was_stopped = false;  // Reset flag when motors are active
        }
        
        // Line following control loop (if enabled)
        if (line_following_enabled) {
            line_sample_t ls;
            lf_read(&ls);
            uint64_t now_us = time_us_64();
            
            // Convert sensor to analog-like input for PID
            float sensor_val = ls.left_on_line ? 1.0f : 0.0f;
            float error = line_follow_pid.setpoint - sensor_val;
            float correction = pid_update(&line_follow_pid, error);
            
            float left_speed = 0.0f, right_speed = 0.0f;
            
            if (ls.left_on_line) {
                // On black line → go straight, apply PID correction
                left_speed = LINE_FOLLOW_BASE_SPEED - correction;
                right_speed = LINE_FOLLOW_BASE_SPEED + correction;
                last_seen_black_time = now_us;
                
                // Track last turn direction
                if (correction > 0.01f)
                    last_turn_dir = +1;   // drifting right → needs left correction
                else if (correction < -0.01f)
                    last_turn_dir = -1;   // drifting left → needs right correction
            } else {
                // Off the line - recovery logic
                uint64_t lost_duration = now_us - last_seen_black_time;
                
                if (lost_duration < 300000) {  // 300ms
                    // Short-term drift → steer using last known direction
                    if (last_turn_dir == -1) {
                        left_speed = LINE_FOLLOW_BASE_SPEED * 0.25f;
                        right_speed = LINE_FOLLOW_BASE_SPEED * 1.1f;
                    } else {
                        left_speed = LINE_FOLLOW_BASE_SPEED * 1.0f;
                        right_speed = LINE_FOLLOW_BASE_SPEED * 0.5f;
                    }
                } else {
                    // Lost line - try to recover
                    bool found = false;
                    // Try right turn
                    for (int i = 0; i < 15; i++) {
                        motor_set(0.3f, -0.3f);
                        sleep_ms(25);
                        lf_read(&ls);
                        if (ls.left_on_line) { found = true; break; }
                    }
                    // Try left turn if not found
                    if (!found) {
                        for (int i = 0; i < 25; i++) {
                            motor_set(-0.3f, 0.3f);
                            sleep_ms(25);
                            lf_read(&ls);
                            if (ls.left_on_line) { found = true; break; }
                        }
                    }
                    if (!found) {
                        // Reverse briefly
                        motor_set(-0.25f, -0.25f);
                        sleep_ms(200);
                    }
                    continue; // Skip motor_set below, already handled in recovery
                }
            }
            
            // Apply computed motor speeds
            motor_set(left_speed, right_speed);
            current_m1 = left_speed;
            current_m2 = right_speed;
            
            // Publish line sensor data (more frequently than telemetry)
            if (robot_mqtt_check_connection() && (now - last_telemetry > 200)) {
                char line_payload[128];
                snprintf(line_payload, sizeof(line_payload),
                         "{\"adc\":%u,\"on_line\":%d,\"error\":%.3f,\"correction\":%.3f,\"ts\":%lu}",
                         ls.adc_left, ls.left_on_line ? 1 : 0, error, correction,
                         (unsigned long)now);
                mqtt_publish_message_safe(MQTT_TOPIC_LINE_SENSOR, line_payload);
            }
        }
        
        // IMU straight-line navigation (if enabled)
        if (imu_straight_mode) {
            static uint64_t last_imu_update = 0;
            uint64_t now_us = time_us_64();
            
            // Run control loop at 20Hz (every 50ms)
            if ((now_us - last_imu_update) > 50000) {
                last_imu_update = now_us;
                
                imu_state_t imu;
                bool imu_ok = imu_read(&imu);
                
                if (imu_ok) {
                    // Apply low-pass filtering to heading
                    heading_smooth = 0.9f * heading_smooth + 0.1f * imu.heading;
                    
                    // Calculate distance from encoders
                    uint32_t total_ticks = (encoder_get_count(1) + encoder_get_count(2)) / 2;
                    distance_traveled_m = (total_ticks / ENCODER_TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_M;
                    
                    // PID correction based on filtered heading
                    float correction = pid_update(&heading_pid, heading_smooth);
                    
                    // Apply correction to motor speeds
                    float left_speed = IMU_BASE_SPEED + correction;
                    float right_speed = IMU_BASE_SPEED - correction;
                    
                    // Clamp speeds
                    if (left_speed > 1.0f) left_speed = 1.0f;
                    if (left_speed < 0.2f) left_speed = 0.2f;
                    if (right_speed > 1.0f) right_speed = 1.0f;
                    if (right_speed < 0.2f) right_speed = 0.2f;
                    
                    // Apply motor speeds (remember: motor_set uses swapped order)
                    motor_set(right_speed, left_speed);
                    current_m1 = right_speed;
                    current_m2 = left_speed;
                    
                    // Publish IMU telemetry (raw and filtered)
                    if (robot_mqtt_check_connection() && (now - last_telemetry > 200)) {
                        char imu_payload[256];
                        snprintf(imu_payload, sizeof(imu_payload),
                                 "{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"heading_raw\":%.2f,\"heading_filtered\":%.2f,\"distance_m\":%.3f,\"correction\":%.3f,\"ts\":%lu}",
                                 imu.ax, imu.ay, imu.az,
                                 imu.heading, heading_smooth, distance_traveled_m, correction,
                                 (unsigned long)now);
                        mqtt_publish_message_safe(MQTT_TOPIC_IMU, imu_payload);
                    }
                } else {
                    // IMU read failed - coast forward with last known speeds
                    printf("IMU read failed - coasting\n");
                }
            }
        }
        
        // Barcode tracking (if enabled) - publish decoded messages
        if (barcode_tracking_enabled) {
            static int last_message_length = 0;
            if (message_length > 0 && message_length != last_message_length) {
                // New message decoded - publish it
                char barcode_payload[128];
                snprintf(barcode_payload, sizeof(barcode_payload),
                         "{\"message\":\"%s\",\"length\":%d,\"ts\":%lu}",
                         decoded_message, message_length,
                         (unsigned long)now);
                mqtt_publish_message_safe(MQTT_TOPIC_BARCODE, barcode_payload);
                last_message_length = message_length;
                printf("[BARCODE] Published: %s\n", decoded_message);
            }
        }
        
        // Calculate speed in km/h from encoders
        calculate_speed_kmh();
        
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

