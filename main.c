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
    servo_set_pulse(SERVO_CENTER_PULSE);

    // Initialize WiFi
    printf("Initializing WiFi...\n");
    if (cyw43_arch_init()) {
        printf("ERROR: Failed to initialize WiFi hardware\n");
        while(1) {
            sleep_ms(100);
        }
        return -1;
    }
    
    printf("Enabling station mode...\n");
    cyw43_arch_enable_sta_mode();
    printf("Station mode enabled, waiting for WiFi chip to initialize...\n");
    sleep_ms(5000);  // Increased delay - Give WiFi chip more time to initialize
    
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
    robot_mqtt_init_and_connect(MQTT_BROKER_IP, MQTT_BROKER_PORT);
    sleep_ms(2000);
    printf("MQTT ready!\n");

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
    uint32_t last_blink = 0;
    absolute_time_t last_connection_check = get_absolute_time();

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
        
        // Publish ultrasonic telemetry
        if (robot_mqtt_check_connection() && (now_ms % 500 < 50)) {
            char us_payload[64];
            snprintf(us_payload, sizeof(us_payload), "{\"dist_cm\":%.1f}", dist);
            mqtt_publish_message_safe("robot/ultrasonic", us_payload);
        }
        
        if (dist > 0.1f && dist < STOP_DISTANCE_CM) {
            printf("Obstacle at %.1f cm! Stopping for width scan.\n", dist);
            motor_set(0, 0);
            sleep_ms(500);

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

            // 3. Decision: Go with left/right with lesser cm (lesser width)
            if (left_obs_width < right_obs_width) {
                printf("Turning LEFT (Lesser width)\n");
                
                motor_set(0.35f, -0.35f); 
                sleep_ms(300); 

                motor_set(0.25f, 0.25f); 
                sleep_ms(2000); 

                printf("Turning back RIGHT 60 deg...\n");
                motor_set(-0.35f, 0.35f); 
                sleep_ms(600);

                printf("Moving forward to find line...\n");
                motor_set(0.20f, 0.20f); 
                
                uint64_t search_start = time_us_64();
                while (true) {
                    line_sample_t ls_check;
                    lf_read(&ls_check);
                    if (ls_check.right_on_line) {
                        printf("Line found!\n");
                        break;
                    }
                    if (time_us_64() - search_start > 5000000) break;
                    sleep_ms(10);
                }

            } else {
                printf("Turning RIGHT (Lesser width)\n");
                
                motor_set(-0.35f, 0.35f); 
                sleep_ms(300); 

                motor_set(0.25f, 0.25f); 
                sleep_ms(2000); 

                printf("Turning back LEFT 60 deg...\n");
                motor_set(0.35f, -0.35f); 
                sleep_ms(600);

                printf("Moving forward to find line...\n");
                motor_set(0.20f, 0.20f); 
                
                uint64_t search_start = time_us_64();
                while (true) {
                    line_sample_t ls_check;
                    lf_read(&ls_check);
                    if (ls_check.right_on_line) {
                        printf("Line found!\n");
                        break;
                    }
                    if (time_us_64() - search_start > 5000000) break;
                    sleep_ms(10);
                }
            }
            
            motor_set(0,0);
            sleep_ms(200);
            continue; 
        }

        // -------------------------------------------------------------------
        // 1. PID LINE FOLLOWING
        // -------------------------------------------------------------------
        float sensor_val = ls.right_on_line ? 1.0f : 0.0f;
        float error      = linePID.setpoint - sensor_val;
        float correction = pid_update(&linePID, error);

        float min_drive = 0.20f;

        if (ls.right_on_line) {
            left_speed  = base_speed - correction + CALIBRATION_BIAS;
            right_speed = base_speed + correction;

            last_seen_black_time = now;

            if (correction > 0.01f)
                last_turn_dir = +1;
            else if (correction < -0.01f)
                last_turn_dir = -1;
        } 
        else {
            uint64_t lost_duration = now - last_seen_black_time;

            if (lost_duration < 300000) {
                if (last_turn_dir == -1) {
                    left_speed  = base_speed * 1.0f;
                    right_speed = base_speed * 0.5f;
                } else { 
                    left_speed  = base_speed * 0.25f;
                    right_speed = base_speed * 1.1f;
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
        
        if (robot_mqtt_check_connection() && (now_ms - last_telemetry > TELEMETRY_INTERVAL_MS)) {
            // Publish line sensor data
            char line_payload[128];
            snprintf(line_payload, sizeof(line_payload),
                     "{\"adc\":%u,\"on_line\":%d,\"error\":%.3f,\"correction\":%.3f,\"ts\":%lu}",
                     ls.adc_right, ls.right_on_line ? 1 : 0, error, correction, (unsigned long)now_ms);
            mqtt_publish_message_safe(MQTT_TOPIC_LINE_SENSOR, line_payload);
            
            // Publish encoder speeds
            char speed_payload[128];
            snprintf(speed_payload, sizeof(speed_payload),
                     "{\"speed_left\":%.2f,\"speed_right\":%.2f,\"speed_avg\":%.2f,\"ts\":%lu}",
                     speed_kmh_left, speed_kmh_right, speed_kmh_avg, (unsigned long)now_ms);
            mqtt_publish_message_safe("robot/speed", speed_payload);
            
            last_telemetry = now_ms;
        }

        sleep_ms(60);
    }
}
