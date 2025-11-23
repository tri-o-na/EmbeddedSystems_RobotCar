// Line Following with Single IR Sensor + PID + Smart Recovery + MQTT Telemetry
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "motor_encoder.h"
#include "pid.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <line_follow.h>
#include <barcode.h>
#include "ultrasonic.h"
#include "imu_lis3dh.h"
#include "mqtt_client.h"
#include "mqtt_config.h"

// External reference to cyw43_state
extern cyw43_t cyw43_state;

// WiFi scan tracking
static int scan_network_count = 0;
static bool target_network_found = false;

// WiFi scan result callback
static int wifi_scan_result(void *env, const cyw43_ev_scan_result_t *result) {
    if (result) {
        scan_network_count++;
        printf("  [%d] Found: %-32s (RSSI: %d, Channel: %d", 
               scan_network_count, result->ssid, result->rssi, result->channel);
        if (result->auth_mode == CYW43_AUTH_WPA2_AES_PSK) {
            printf(", WPA2 AES");
        } else if (result->auth_mode == CYW43_AUTH_WPA_TKIP_PSK) {
            printf(", WPA TKIP");
        } else if (result->auth_mode == CYW43_AUTH_OPEN) {
            printf(", OPEN");
        }
        printf(")\n");
        
        // Check if this is our target network
        if (strcmp((char*)result->ssid, WIFI_SSID) == 0) {
            target_network_found = true;
            printf("  *** MATCH FOUND: This is the target network '%s' ***\n", WIFI_SSID);
            if (result->channel > 14) {
                printf("  WARNING: Channel %d suggests 5GHz - Pico W only supports 2.4GHz!\n", result->channel);
            } else {
                printf("  Channel %d is 2.4GHz - should work with Pico W\n", result->channel);
            }
            printf("  Signal strength: %d dBm (closer to 0 is better, -80 or better recommended)\n", result->rssi);
        }
    } else {
        printf("  [Callback called with NULL result]\n");
    }
    return 0;
}

// Forward declaration for MQTT publish
extern bool mqtt_publish_message_safe(const char *topic, const char *payload);

// External reference to barcode decoded message
extern char decoded_message[];
extern int message_length;

// System state tracking
typedef enum {
    STATE_IDLE,
    STATE_LINE_FOLLOWING,
    STATE_TURNING_LEFT,
    STATE_TURNING_RIGHT,
    STATE_AVOIDING_OBSTACLE,
    STATE_SCANNING,
    STATE_RECOVERING,
    STATE_MANUAL
} system_state_t;

static system_state_t current_state = STATE_IDLE;
static bool line_following_enabled = true;  // Default to line following
static bool manual_mode = false;
static float current_m1 = 0.0f;
static float current_m2 = 0.0f;

// Distance traveled tracking
static uint32_t initial_encoder_left = 0;
static uint32_t initial_encoder_right = 0;
static bool distance_tracking_initialized = false;

// Line event tracking
static bool last_line_state = false;
static uint32_t last_line_event_time = 0;

// Obstacle encounter logging
#define MAX_OBSTACLE_LOGS 10
typedef struct {
    float distance_cm;
    float left_width;
    float right_width;
    char chosen_path[8];
    char recovery_status[16];
    uint32_t timestamp;
} obstacle_log_t;
static obstacle_log_t obstacle_logs[MAX_OBSTACLE_LOGS];
static int obstacle_log_count = 0;

// Speed calculation constants
#define WHEEL_DIAMETER_MM 65.0f
#define ENCODER_TICKS_PER_REV 20.0f
#define WHEEL_CIRCUMFERENCE_M ((WHEEL_DIAMETER_MM * 3.14159f) / 1000.0f)

// Speed tracking variables
static uint32_t last_encoder_left = 0;
static uint32_t last_encoder_right = 0;
static uint64_t last_speed_calc_time = 0;
static float speed_kmh_left = 0.0f;
static float speed_kmh_right = 0.0f;
static float speed_kmh_avg = 0.0f;

// ---- DISTANCE SETTINGS ----
#define STOP_DISTANCE_CM 20.0f 
#define DETECTION_RANGE_CM 30.0f
#define CLEAR_RANGE_CM 40.0f 

// ---- SERVO SETTINGS ----
#define SERVO_PIN 15 
#define TRIG_PIN 0   
#define ECHO_PIN 1   

#define SERVO_CENTER_PULSE 1500
#define SERVO_LEFT_PULSE 2000
#define SERVO_RIGHT_PULSE 1000

// 45-degree scan
#define SERVO_SCAN_DEGREES 45.0f
#define SERVO_US_PER_DEG 5.556f 
#define SERVO_STEP_US 10 
#define SERVO_SAMPLE_DELAY_MS 350 

// BEAM WIDTH CORRECTION
#define BEAM_CORRECTION_DEG 12.0f 

// Helper function to clamp a float value
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Get system state string
static const char* get_state_string(system_state_t state) {
    switch(state) {
        case STATE_IDLE: return "IDLE";
        case STATE_LINE_FOLLOWING: return "Following line";
        case STATE_TURNING_LEFT: return "Turning left";
        case STATE_TURNING_RIGHT: return "Turning right";
        case STATE_AVOIDING_OBSTACLE: return "Avoiding obstacle";
        case STATE_SCANNING: return "Scanning";
        case STATE_RECOVERING: return "Recovering";
        case STATE_MANUAL: return "Manual";
        default: return "UNKNOWN";
    }
}

// Calculate distance traveled in meters
static float calculate_distance_traveled(void) {
    if (!distance_tracking_initialized) {
        initial_encoder_left = encoder_get_count(1);
        initial_encoder_right = encoder_get_count(2);
        distance_tracking_initialized = true;
        return 0.0f;
    }
    
    uint32_t enc_left = encoder_get_count(1);
    uint32_t enc_right = encoder_get_count(2);
    
    uint32_t delta_left = enc_left - initial_encoder_left;
    uint32_t delta_right = enc_right - initial_encoder_right;
    uint32_t avg_ticks = (delta_left + delta_right) / 2;
    
    return (avg_ticks / ENCODER_TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_M;
}

// Command handler - processes incoming MQTT commands
void process_robot_commands(const char *topic, const char *payload, int len) {
    printf("MQTT Command received on topic '%s': %.*s\n", topic, len, payload);
    
    // Check for mode commands first
    if (strstr(payload, "\"mode\"") != NULL) {
        if (strstr(payload, "\"line_follow\"") != NULL || strstr(payload, "\"line_following\"") != NULL) {
            line_following_enabled = true;
            manual_mode = false;
            current_state = STATE_LINE_FOLLOWING;
            printf("  Line following mode ENABLED\n");
        } else if (strstr(payload, "\"manual\"") != NULL) {
            line_following_enabled = false;
            manual_mode = true;
            current_state = STATE_MANUAL;
            printf("  Manual control mode ENABLED\n");
            motor_set(0, 0);
            current_m1 = 0.0f;
            current_m2 = 0.0f;
        }
        if (strstr(payload, "\"barcode\"") != NULL) {
            // Barcode enable/disable handled in barcode module
            printf("  Barcode command received\n");
        }
        return; // Don't process motor commands when changing mode
    }
    
    // Emergency stop ALWAYS works, regardless of mode
    if (strstr(payload, "\"action\"") != NULL && strstr(payload, "stop") != NULL) {
        printf("  Emergency stop command received\n");
        line_following_enabled = false;
        manual_mode = true;
        current_state = STATE_IDLE;
        motor_set(0, 0);
        current_m1 = 0.0f;
        current_m2 = 0.0f;
        printf("  Motors stopped\n");
        return;
    }
    
    // Only process motor commands if in manual mode
    if (manual_mode) {
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
            motor_set(m2, m1);  // Note: motor_set takes (right, left)
            current_m1 = m1;
            current_m2 = m2;
            
            printf("  Motors set: m1=%.2f, m2=%.2f\n", m1, m2);
        }
    }
}

// ---- Motor Calibration Bias ----
#define CALIBRATION_BIAS 0.18f 

// Function to calculate speed in km/h from encoder readings
void calculate_speed_kmh(void) {
    uint64_t now = time_us_64();
    uint32_t enc_left = encoder_get_count(1);
    uint32_t enc_right = encoder_get_count(2);
    
    if (last_speed_calc_time == 0) {
        last_speed_calc_time = now;
        last_encoder_left = enc_left;
        last_encoder_right = enc_right;
        return;
    }
    
    float time_seconds = (now - last_speed_calc_time) / 1000000.0f;
    if (time_seconds < 0.2f) return;
    
    uint32_t delta_left = enc_left - last_encoder_left;
    uint32_t delta_right = enc_right - last_encoder_right;
    
    float rps_left = delta_left / ENCODER_TICKS_PER_REV / time_seconds;
    float rps_right = delta_right / ENCODER_TICKS_PER_REV / time_seconds;
    
    speed_kmh_left = rps_left * WHEEL_CIRCUMFERENCE_M * 3.6f;
    speed_kmh_right = rps_right * WHEEL_CIRCUMFERENCE_M * 3.6f;
    speed_kmh_avg = (speed_kmh_left + speed_kmh_right) / 2.0f;
    
    last_encoder_left = enc_left;
    last_encoder_right = enc_right;
    last_speed_calc_time = now;
}

// --- HARDWARE INIT FUNCTIONS ---
void servo_init_hw() {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 64.0f);
    pwm_init(slice_num, &config, true);
    pwm_set_wrap(slice_num, 39062);
}

void servo_set_pulse(uint16_t pulse_us) {
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_set_gpio_level(SERVO_PIN, (uint16_t)(pulse_us * 1.953f)); 
}

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("=== Combined: Final PID Follow + Barcode + Ultrasonic Scan + MQTT ===\n");

    // Initialize hardware
    motors_and_encoders_init();
    lf_init();
    barcode_init();
    servo_init_hw();
    setupUltrasonicPins(TRIG_PIN, ECHO_PIN);
    imu_init();
    servo_set_pulse(SERVO_CENTER_PULSE);

    // Initialize WiFi with proper reset sequence
    printf("Initializing WiFi...\n");
    
    // Add initial delay to ensure chip is powered/reset
    sleep_ms(1000);
    
    if (cyw43_arch_init()) {
        printf("ERROR: Failed to initialize WiFi hardware\n");
        while(1) {
            sleep_ms(100);
        }
        return -1;
    }
    
    printf("Enabling station mode...\n");
    cyw43_arch_enable_sta_mode();
    
    // Disconnect any existing WiFi connection first (important for reflashing)
    printf("Resetting WiFi connection state...\n");
    cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
    sleep_ms(2000);  // Wait for disconnect to complete
    
    printf("Station mode enabled, waiting for WiFi chip to fully reset...\n");
    sleep_ms(3000);  // Give WiFi chip time to reset
    
    // Poll to ensure chip is ready before scanning
    printf("Polling WiFi chip to ensure readiness...\n");
    for (int i = 0; i < 20; i++) {
        cyw43_arch_poll();
        sleep_ms(50);
    }
    printf("WiFi chip ready for scanning.\n");
    
    // Scan for available networks to debug
    printf("\n=== Scanning for WiFi Networks ===\n");
    printf("This may take 10-15 seconds...\n");
    scan_network_count = 0;
    target_network_found = false;
    
    cyw43_wifi_scan_options_t scan_options = {0};
    int scan_result = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, wifi_scan_result);
    if (scan_result == 0) {
        printf("Scan initiated successfully, polling for results...\n");
        // Process scan results - need to poll (increased iterations for better results)
        for (int i = 0; i < 100; i++) {
            cyw43_arch_poll();
            if (i % 10 == 0) {
                printf("  Polling... (%d/100)\n", i);
            }
            sleep_ms(100);
        }
        printf("\nScan complete. Found %d network(s).\n", scan_network_count);
        
        if (!target_network_found) {
            printf("\n*** WARNING: Target network '%s' NOT FOUND in scan! ***\n", WIFI_SSID);
            printf("Possible reasons:\n");
            printf("  1. Network is out of range (move Pico W closer to phone)\n");
            printf("  2. Phone hotspot not broadcasting (connect another device first)\n");
            printf("  3. Network is 5GHz only (Pico W only supports 2.4GHz)\n");
            printf("  4. SSID doesn't match exactly (case-sensitive, check for spaces)\n");
            printf("  5. Phone hotspot is idle (keep screen on, ensure hotspot is active)\n");
            printf("\nWill still attempt connection, but likely to fail...\n");
        } else {
            printf("*** Target network '%s' was found in scan - connection should work! ***\n", WIFI_SSID);
        }
    } else {
        printf("Scan failed to initiate (error %d), continuing anyway...\n", scan_result);
        printf("This might indicate WiFi chip initialization issue.\n");
    }
    
    printf("\n=== WiFi Connection Details ===\n");
    printf("SSID: '%s' (length: %d chars)\n", WIFI_SSID, (int)strlen(WIFI_SSID));
    printf("Password: '%s' (length: %d chars)\n", WIFI_PASS, (int)strlen(WIFI_PASS));
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
    
    // Initialize MQTT
    printf("Connecting to MQTT broker %s:%d...\n", MQTT_BROKER_IP, MQTT_BROKER_PORT);
    robot_set_command_handler(process_robot_commands);
    robot_mqtt_init_and_connect(MQTT_BROKER_IP, MQTT_BROKER_PORT);
    sleep_ms(2000);
    printf("MQTT ready!\n");
    
    // Initialize state
    current_state = STATE_LINE_FOLLOWING;
    line_following_enabled = true;
    manual_mode = false;

    // Base speed maintained at 0.20f for reliable barcode detection
    const float base_speed = 0.15f; 
    float left_speed = 0.0f, right_speed = 0.0f;

    // --- Line Following PID setup ---
    PID linePID;
    pid_init(&linePID, 0.06f, 0.00f, 0.005f, -0.25f, 0.25f);
    linePID.setpoint = 1.0f;
    
    uint64_t last_seen_black_time = 0;
    int last_turn_dir = 0;
    uint64_t now;
    uint32_t last_telemetry = 0;
    uint32_t last_state_publish = 0;
    uint32_t last_line_sensor_publish = 50;   // Offset by 50ms to stagger publishes
    uint32_t last_imu_publish = 100;          // Offset by 100ms to stagger publishes
    uint32_t last_ultrasonic_publish = 150;   // Offset by 150ms to stagger publishes
    uint32_t last_blink = 0;
    absolute_time_t last_connection_check = get_absolute_time();
    float error = 0.0f;
    float correction = 0.0f;

    printf("Starting control loop...\n");

    while (true) {
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        
        // Blink LED every second
        if (now_ms - last_blink > 1000) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(50);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            last_blink = now_ms;
        }
        
        // Poll MQTT
        for (int i = 0; i < 3; i++) {
            robot_mqtt_process_events();
        }
        robot_mqtt_ensure_connected();
        
        // Connection monitoring
        if (absolute_time_diff_us(last_connection_check, get_absolute_time()) > 5 * 1000000) {
            if (!robot_mqtt_check_connection()) {
                int wifi_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
                if (wifi_status == CYW43_LINK_UP) {
                    robot_mqtt_ensure_connected();
                } else {
                    cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS,
                                                       CYW43_AUTH_WPA2_AES_PSK, 10000);
                }
            }
            last_connection_check = get_absolute_time();
        }
        
        line_sample_t ls;
        lf_read(&ls);
        now = time_us_64();

        // -------------------------------------------------------------------
        // 0. ULTRASONIC OBSTACLE DETECTION & SCAN
        // -------------------------------------------------------------------
        float dist = get_distance_cm(TRIG_PIN, ECHO_PIN);
        
        // Debug: Print sensor reading every second
        static uint32_t last_debug_print = 0;
        if (now_ms - last_debug_print > 1000) {
            printf("[DEBUG] Ultrasonic reading: %.1f cm\n", dist);
            last_debug_print = now_ms;
        }
        
        // Publish ultrasonic telemetry (only when value changes significantly, every 500ms)
        static float last_published_dist = -999.0f;
        if (robot_mqtt_check_connection() && (now_ms - last_ultrasonic_publish > 500)) {
            // Only publish if value changed significantly (>1cm) or if it's the first reading
            if (fabsf(dist - last_published_dist) > 1.0f || last_published_dist < -900.0f) {
                char us_payload[64];
                snprintf(us_payload, sizeof(us_payload), "{\"dist_cm\":%.1f}", dist);
                if (mqtt_publish_message_safe("robot/ultrasonic", us_payload)) {
                    printf("[ULTRASONIC] Published: %.1f cm\n", dist);
                    last_published_dist = dist;
                    last_ultrasonic_publish = now_ms;  // Only update timestamp on successful publish
                } else {
                    printf("[ULTRASONIC] Failed to publish (backpressure or connection issue)\n");
                }
            } else {
                // Value hasn't changed enough, skip this publish cycle
                last_ultrasonic_publish = now_ms;  // Update timestamp to avoid constant checking
            }
        }
        
        if (line_following_enabled && dist > 0.1f && dist < STOP_DISTANCE_CM && dist != -1.0f) {
            current_state = STATE_AVOIDING_OBSTACLE;
            printf("Obstacle at %.1f cm! Stopping for width scan.\n", dist);
            motor_set(0, 0);
            sleep_ms(500);
            
            // Publish obstacle detection
            current_state = STATE_SCANNING;
            if (robot_mqtt_check_connection()) {
                char obstacle_payload[128];
                snprintf(obstacle_payload, sizeof(obstacle_payload),
                         "{\"dist_cm\":%.1f,\"recovery_status\":\"SCANNING\",\"ts\":%lu}",
                         dist, (unsigned long)now_ms);
                mqtt_publish_message_safe("robot/obstacle", obstacle_payload);
            }

            int scan_delta = (int)(SERVO_SCAN_DEGREES * SERVO_US_PER_DEG); 
            
            int min_pulse_seen = SERVO_CENTER_PULSE;
            int max_pulse_seen = SERVO_CENTER_PULSE;
            float min_dist_seen = 1000.0f;

            // 1. Scan RIGHT (Center -> Right)
            printf("Scanning Right...\n");
            for (int p = SERVO_CENTER_PULSE; p >= (SERVO_CENTER_PULSE - scan_delta); p -= SERVO_STEP_US) {
                servo_set_pulse(p);
                sleep_ms(SERVO_SAMPLE_DELAY_MS);
                
                float scan_dist = get_distance_cm(TRIG_PIN, ECHO_PIN);
                printf("  [RIGHT] Pulse: %d, Dist: %.1f cm\n", p, scan_dist);

                if (scan_dist > 0.1f && scan_dist < DETECTION_RANGE_CM) {
                    if (p < min_pulse_seen) min_pulse_seen = p;
                    if (scan_dist < min_dist_seen) min_dist_seen = scan_dist;
                }
            }
            servo_set_pulse(SERVO_CENTER_PULSE);
            sleep_ms(500);

            // 2. Scan LEFT (Center -> Left)
            printf("Scanning Left...\n");
            for (int p = SERVO_CENTER_PULSE; p <= (SERVO_CENTER_PULSE + scan_delta); p += SERVO_STEP_US) {
                servo_set_pulse(p);
                sleep_ms(SERVO_SAMPLE_DELAY_MS);
                
                float scan_dist = get_distance_cm(TRIG_PIN, ECHO_PIN);
                printf("  [LEFT]  Pulse: %d, Dist: %.1f cm\n", p, scan_dist);

                if (scan_dist > 0.1f && scan_dist < DETECTION_RANGE_CM) {
                    if (p > max_pulse_seen) max_pulse_seen = p;
                    if (scan_dist < min_dist_seen) min_dist_seen = scan_dist;
                }
            }
            servo_set_pulse(SERVO_CENTER_PULSE);
            sleep_ms(500);

            // --- CALCULATE WIDTHS ---
            float left_span_us = (float)(max_pulse_seen - SERVO_CENTER_PULSE);
            float right_span_us = (float)(SERVO_CENTER_PULSE - min_pulse_seen);
            float left_deg = left_span_us / SERVO_US_PER_DEG;
            float right_deg = right_span_us / SERVO_US_PER_DEG;

            printf("  [DEBUG] Raw Angles -> L: %.1f deg, R: %.1f deg. Min Dist: %.1f\n", left_deg, right_deg, min_dist_seen);

            left_deg = (left_deg > BEAM_CORRECTION_DEG) ? (left_deg - BEAM_CORRECTION_DEG) : 0.0f;
            right_deg = (right_deg > BEAM_CORRECTION_DEG) ? (right_deg - BEAM_CORRECTION_DEG) : 0.0f;

            float left_obs_width = min_dist_seen * (left_deg * 3.14159f / 180.0f);
            float right_obs_width = min_dist_seen * (right_deg * 3.14159f / 180.0f);

            printf("Widths (Corrected) -> L: %.1f cm, R: %.1f cm. Total: %.1f cm\n", left_obs_width, right_obs_width, left_obs_width + right_obs_width);

            // Publish obstacle details
            const char* chosen_path;
            if (left_obs_width < right_obs_width) {
                chosen_path = "LEFT";
            } else {
                chosen_path = "RIGHT";
            }
            
            if (robot_mqtt_check_connection()) {
                char obstacle_payload[256];
                snprintf(obstacle_payload, sizeof(obstacle_payload),
                         "{\"dist_cm\":%.1f,\"left_width\":%.1f,\"right_width\":%.1f,\"chosen_path\":\"%s\",\"recovery_status\":\"TURNING\",\"ts\":%lu}",
                         min_dist_seen, left_obs_width, right_obs_width, chosen_path, (unsigned long)now_ms);
                mqtt_publish_message_safe("robot/obstacle", obstacle_payload);
                
                // Log obstacle encounter
                if (obstacle_log_count < MAX_OBSTACLE_LOGS) {
                    obstacle_logs[obstacle_log_count].distance_cm = min_dist_seen;
                    obstacle_logs[obstacle_log_count].left_width = left_obs_width;
                    obstacle_logs[obstacle_log_count].right_width = right_obs_width;
                    strncpy(obstacle_logs[obstacle_log_count].chosen_path, chosen_path, 7);
                    strncpy(obstacle_logs[obstacle_log_count].recovery_status, "TURNING", 15);
                    obstacle_logs[obstacle_log_count].timestamp = now_ms;
                    obstacle_log_count++;
                }
            }

            // 3. Decision: Go with left/right with lesser cm (lesser width)
            if (left_obs_width < right_obs_width) {
                printf("Turning LEFT (Lesser width)\n");
                current_state = STATE_TURNING_LEFT;
                
                motor_set(0.35f, -0.35f); 
                sleep_ms(300); 

                motor_set(0.25f, 0.25f); 
                sleep_ms(2000); 

                printf("Turning back RIGHT 60 deg...\n");
                motor_set(-0.35f, 0.35f); 
                sleep_ms(600);

                printf("Moving forward to find line...\n");
                motor_set(0.20f, 0.20f);
                current_state = STATE_RECOVERING;
                
                if (robot_mqtt_check_connection()) {
                    char obstacle_payload[128];
                    snprintf(obstacle_payload, sizeof(obstacle_payload),
                             "{\"recovery_status\":\"SEARCHING\",\"ts\":%lu}",
                             (unsigned long)now_ms);
                    mqtt_publish_message_safe("robot/obstacle", obstacle_payload);
                }
                
                uint64_t search_start = time_us_64();
                bool line_found = false;
                while (true) {
                    line_sample_t ls_check;
                    lf_read(&ls_check);
                    if (ls_check.right_on_line) {
                        printf("Line found!\n");
                        line_found = true;
                        break;
                    }
                    if (time_us_64() - search_start > 5000000) break;
                    sleep_ms(10);
                }
                
                if (robot_mqtt_check_connection()) {
                    char obstacle_payload[128];
                    snprintf(obstacle_payload, sizeof(obstacle_payload),
                             "{\"recovery_status\":\"%s\",\"ts\":%lu}",
                             line_found ? "REJOINED" : "FAILED", (unsigned long)now_ms);
                    mqtt_publish_message_safe("robot/obstacle", obstacle_payload);
                    
                    if (obstacle_log_count > 0) {
                        strncpy(obstacle_logs[obstacle_log_count-1].recovery_status,
                               line_found ? "REJOINED" : "FAILED", 15);
                    }
                }

            } else {
                printf("Turning RIGHT (Lesser width)\n");
                current_state = STATE_TURNING_RIGHT;
                
                motor_set(-0.35f, 0.35f); 
                sleep_ms(300); 

                motor_set(0.25f, 0.25f); 
                sleep_ms(2000); 

                printf("Turning back LEFT 60 deg...\n");
                motor_set(0.35f, -0.35f); 
                sleep_ms(600);

                printf("Moving forward to find line...\n");
                motor_set(0.20f, 0.20f);
                current_state = STATE_RECOVERING;
                
                if (robot_mqtt_check_connection()) {
                    char obstacle_payload[128];
                    snprintf(obstacle_payload, sizeof(obstacle_payload),
                             "{\"recovery_status\":\"SEARCHING\",\"ts\":%lu}",
                             (unsigned long)now_ms);
                    mqtt_publish_message_safe("robot/obstacle", obstacle_payload);
                }
                
                uint64_t search_start = time_us_64();
                bool line_found = false;
                while (true) {
                    line_sample_t ls_check;
                    lf_read(&ls_check);
                    if (ls_check.right_on_line) {
                        printf("Line found!\n");
                        line_found = true;
                        break;
                    }
                    if (time_us_64() - search_start > 5000000) break;
                    sleep_ms(10);
                }
                
                if (robot_mqtt_check_connection()) {
                    char obstacle_payload[128];
                    snprintf(obstacle_payload, sizeof(obstacle_payload),
                             "{\"recovery_status\":\"%s\",\"ts\":%lu}",
                             line_found ? "REJOINED" : "FAILED", (unsigned long)now_ms);
                    mqtt_publish_message_safe("robot/obstacle", obstacle_payload);
                    
                    if (obstacle_log_count > 0) {
                        strncpy(obstacle_logs[obstacle_log_count-1].recovery_status,
                               line_found ? "REJOINED" : "FAILED", 15);
                    }
                }
            }
            
            motor_set(0,0);
            sleep_ms(200);
            current_state = STATE_LINE_FOLLOWING;
            continue; 
        }

        // -------------------------------------------------------------------
        // Track line events (on/off transitions) - ALWAYS track, not just in line following mode
        // -------------------------------------------------------------------
        if (ls.right_on_line != last_line_state) {
            last_line_state = ls.right_on_line;
            if (robot_mqtt_check_connection() && (now_ms - last_line_event_time > 100)) {
                char line_event_payload[128];
                snprintf(line_event_payload, sizeof(line_event_payload),
                         "{\"event\":\"%s\",\"adc\":%d,\"ts\":%lu}",
                         ls.right_on_line ? "ON_LINE" : "OFF_LINE", 
                         ls.right_on_line ? 1 : 0, 
                         (unsigned long)now_ms);
                mqtt_publish_message_safe("robot/line_events", line_event_payload);
                last_line_event_time = now_ms;
            }
        }

        // -------------------------------------------------------------------
        // 1. PID LINE FOLLOWING
        // -------------------------------------------------------------------
        if (line_following_enabled) {
            current_state = STATE_LINE_FOLLOWING;
            float sensor_val = ls.right_on_line ? 1.0f : 0.0f;
            error      = linePID.setpoint - sensor_val;
            correction = pid_update(&linePID, error);

            float min_drive = 0.20f;

            if (ls.right_on_line) {
                left_speed  = base_speed - correction + CALIBRATION_BIAS;
                right_speed = base_speed + correction;

                last_seen_black_time = now;

                if (correction > 0.01f) {
                    last_turn_dir = +1;
                    current_state = STATE_TURNING_RIGHT;
                } else if (correction < -0.01f) {
                    last_turn_dir = -1;
                    current_state = STATE_TURNING_LEFT;
                } else {
                    current_state = STATE_LINE_FOLLOWING;
                }
            } 
            else {
                uint64_t lost_duration = now - last_seen_black_time;
                current_state = STATE_RECOVERING;

                if (lost_duration < 300000) {
                    if (last_turn_dir == -1) {
                        left_speed  = base_speed * 1.0f;
                        right_speed = base_speed * 0.5f;
                        current_state = STATE_TURNING_LEFT;
                    } else { 
                        left_speed  = base_speed * 0.25f;
                        right_speed = base_speed * 1.1f;
                        current_state = STATE_TURNING_RIGHT;
                    }
                } 
                else {
                    printf("Line lost — probing both sides...\n");
                    bool found = false;

                    for (int i = 0; i < 15; i++) {
                        motor_set(0.30f, -0.30f);
                        sleep_ms(25);
                        lf_read(&ls);
                        if (ls.right_on_line) { found = true; break; }
                    }

                    if (!found) {
                        for (int i = 0; i < 25; i++) {
                            motor_set(-0.30f, 0.30f);
                            sleep_ms(25);
                            lf_read(&ls);
                            if (ls.right_on_line) { found = true; break; }
                        }
                    }

                    motor_set(0, 0);
                    if (found) {
                        printf("Reacquired black line! Resuming PID.\n");
                        current_state = STATE_LINE_FOLLOWING;
                    } else {
                        printf("Still lost — reversing briefly...\n");
                        motor_set(-0.25f, -0.25f);
                        sleep_ms(200);
                    }

                    left_speed  = 0.0f;
                    right_speed = 0.0f;
                }
            }

            // -------------------------------------------------------------------
            // 2. APPLY MOTOR SPEEDS
            // -------------------------------------------------------------------
            if (left_speed != 0.0f || right_speed != 0.0f) {
                if (left_speed > 0 && left_speed < min_drive) left_speed = min_drive;
                if (right_speed > 0 && right_speed < min_drive) right_speed = min_drive;
                
                left_speed  = clampf(left_speed,  -1.0f, 1.0f);
                right_speed = clampf(right_speed, -1.0f, 1.0f);
                
                motor_set(right_speed, left_speed);
                current_m1 = left_speed;
                current_m2 = right_speed;
            } else {
                current_m1 = 0.0f;
                current_m2 = 0.0f;
            }
        } else {
            // Manual mode or idle
            current_m1 = 0.0f;
            current_m2 = 0.0f;
            // Reset error and correction when not in line following mode
            error = 0.0f;
            correction = 0.0f;
        }

        // -------------------------------------------------------------------
        // 3. NON-BLOCKING BARCODE SCAN (LEFT SENSOR)
        // -------------------------------------------------------------------
        barcode_nonblocking_update();
        
        // Publish barcode if new message decoded
        static int last_message_length = 0;
        if (message_length > 0 && message_length != last_message_length) {
            char barcode_payload[128];
            snprintf(barcode_payload, sizeof(barcode_payload),
                     "{\"message\":\"%s\",\"length\":%d,\"ts\":%lu}",
                     decoded_message, message_length, (unsigned long)now_ms);
            mqtt_publish_message_safe(MQTT_TOPIC_BARCODE, barcode_payload);
            last_message_length = message_length;
            printf("[BARCODE] Published: %s\n", decoded_message);
        }

        // Calculate and publish telemetry
        calculate_speed_kmh();
        float distance_m = calculate_distance_traveled();
        
        // Publish main telemetry with all data (every 300ms)
        if (robot_mqtt_check_connection() && (now_ms - last_telemetry > 300)) {
            uint32_t enc1 = encoder_pulse_width_us(1);
            uint32_t enc2 = encoder_pulse_width_us(2);
            char telemetry_payload[256];
            snprintf(telemetry_payload, sizeof(telemetry_payload),
                     "{\"enc1\":%lu,\"enc2\":%lu,\"m1\":%.2f,\"m2\":%.2f,\"speed_kmh\":%.2f,\"speed_left\":%.2f,\"speed_right\":%.2f,\"distance_m\":%.3f,\"ts\":%lu}",
                     (unsigned long)enc1, (unsigned long)enc2, current_m1, current_m2,
                     speed_kmh_avg, speed_kmh_left, speed_kmh_right, distance_m,
                     (unsigned long)now_ms);
            mqtt_publish_message_safe(MQTT_TOPIC_TELEMETRY, telemetry_payload);
            last_telemetry = now_ms;  // Update timestamp regardless to prevent blocking
        }
        
        // Read and publish IMU data (every 500ms)
        if (robot_mqtt_check_connection() && (now_ms - last_imu_publish > 500)) {
            imu_state_t imu;
            bool imu_ok = imu_read(&imu);
            
            if (imu_ok) {
                char imu_payload[256];
                snprintf(imu_payload, sizeof(imu_payload),
                         "{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"heading_raw\":%.2f,\"heading_filtered\":%.2f,\"distance_m\":%.3f,\"correction\":%.3f,\"ts\":%lu}",
                         imu.ax, imu.ay, imu.az,
                         imu.heading, imu.heading,  // Using same value for raw and filtered
                         distance_m, 0.0f,  // distance_m already calculated, correction=0 for now
                         (unsigned long)now_ms);
                
                if (mqtt_publish_message_safe(MQTT_TOPIC_IMU, imu_payload)) {
                    printf("[IMU] Published: ax=%.3f, ay=%.3f, az=%.3f, heading=%.2f\n", 
                           imu.ax, imu.ay, imu.az, imu.heading);
                }
                last_imu_publish = now_ms;  // Update timestamp regardless to prevent blocking
            }
        }
        
        // Publish line sensor data separately (every 400ms)
        // This allows dashboard to see sensor readings even in manual mode
        if (robot_mqtt_check_connection() && (now_ms - last_line_sensor_publish > 400)) {
            // Read fresh line sensor data right before publishing
            line_sample_t ls_fresh;
            lf_read(&ls_fresh);
            char line_payload[128];
            snprintf(line_payload, sizeof(line_payload),
                     "{\"adc\":%u,\"on_line\":%d,\"error\":%.3f,\"correction\":%.3f,\"ts\":%lu}",
                     ls_fresh.adc_right, ls_fresh.right_on_line ? 1 : 0, error, correction, (unsigned long)now_ms);
            if (mqtt_publish_message_safe(MQTT_TOPIC_LINE_SENSOR, line_payload)) {
                printf("[LINE] Published: ADC=%u, on_line=%d, error=%.3f, correction=%.3f\n",
                       ls_fresh.adc_right, ls_fresh.right_on_line ? 1 : 0, error, correction);
            }
            last_line_sensor_publish = now_ms;  // Update timestamp regardless to prevent blocking
        }
        
        // Publish system state (every 1.5 seconds - less frequent to reduce backpressure)
        if (robot_mqtt_check_connection() && (now_ms - last_state_publish > 1500)) {
            char state_payload[128];
            const char* state_str = get_state_string(current_state);
            snprintf(state_payload, sizeof(state_payload),
                     "{\"state\":\"%s\",\"ts\":%lu}",
                     state_str, (unsigned long)now_ms);
            if (mqtt_publish_message_safe("robot/state", state_payload)) {
                printf("[STATE] Published: %s\n", state_str);
                last_state_publish = now_ms;  // Only update timestamp on successful publish
            } else {
                printf("[STATE] Failed to publish: %s (backpressure?)\n", state_str);
            }
        }

        sleep_ms(60);
    }
}
