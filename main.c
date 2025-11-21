// Line Following with Single IR Sensor + PID + Smart Recovery
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "motor_encoder.h"
#include "pid.h"
#include <stdio.h>
#include <math.h>
#include <line_follow.h>
#include <barcode.h>
#include "ultrasonic.h" 

// ---- DISTANCE SETTINGS ----
#define STOP_DISTANCE_CM 20.0f 
#define DETECTION_RANGE_CM 30.0f // Increased slightly to ensure we catch the edges
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
// HC-SR04 has a beam width of ~30 degrees. We subtract this to fix the "smearing".
// We subtract about 10-12 degrees from EACH side's detected span.
#define BEAM_CORRECTION_DEG 12.0f 

// Helper function to clamp a float value
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ---- Motor Calibration Bias ----
// User confirmed 0.15f is the straightest bias.
#define CALIBRATION_BIAS 0.18f 

// --- HARDWARE INIT FUNCTIONS ---
void servo_init_hw() {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 64.0f); // 125MHz / 64 = ~1.95MHz
    pwm_init(slice_num, &config, true);
    pwm_set_wrap(slice_num, 39062); // 50Hz frequency
}

void servo_set_pulse(uint16_t pulse_us) {
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    // Conversion for 50Hz @ div 64
    pwm_set_gpio_level(SERVO_PIN, (uint16_t)(pulse_us * 1.953f)); 
}

// REMOVED: ultrasonic_init_hw (using setupUltrasonicPins from ultrasonic.c)
// REMOVED: get_distance_cm (using get_distance_cm from ultrasonic.c)

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("=== Combined: Final PID Follow + Barcode + Ultrasonic Scan ===\n");

    motors_and_encoders_init();
    lf_init();      // Initializes Right Sensor (GPIO26)
    barcode_init(); // Initializes Left Sensor (GPIO2)
    
    // Initialize New Hardware
    servo_init_hw();
    setupUltrasonicPins(TRIG_PIN, ECHO_PIN); // Use function from ultrasonic.c
    servo_set_pulse(SERVO_CENTER_PULSE);

    // Base speed maintained at 0.20f for reliable barcode detection
    const float base_speed = 0.15f; 
    float left_speed = 0.0f, right_speed = 0.0f;

    // --- Line Following PID setup ---
    PID linePID;
    pid_init(&linePID, 0.06f, 0.00f, 0.005f, -0.25f, 0.25f);
    linePID.setpoint = 1.0f; // want to always see black (1.0)
    
    uint64_t last_seen_black_time = 0;
    int last_turn_dir = 0; // -1 = left, +1 = right, 0 = straight
    uint64_t now;

    printf("Starting control loop...\n");

    while (true) {
        line_sample_t ls;
        lf_read(&ls);
        now = time_us_64();

        // -------------------------------------------------------------------
        // 0. ULTRASONIC OBSTACLE DETECTION & SCAN
        // -------------------------------------------------------------------
        // Pass pins to the function from ultrasonic.c
        float dist = get_distance_cm(TRIG_PIN, ECHO_PIN); 
        
        if (dist > 0.1f && dist < STOP_DISTANCE_CM) {
            printf("Obstacle at %.1f cm! Stopping for width scan.\n", dist);
            motor_set(0, 0);
            sleep_ms(500);

            int scan_delta = (int)(SERVO_SCAN_DEGREES * SERVO_US_PER_DEG); 
            
            // Variables to track the object's extent
            int min_pulse_seen = SERVO_CENTER_PULSE; // Furthest RIGHT pulse seen
            int max_pulse_seen = SERVO_CENTER_PULSE; // Furthest LEFT pulse seen
            float min_dist_seen = 1000.0f;           // Closest distance to object

            // 1. Scan RIGHT (Center -> Right)
            printf("Scanning Right...\n");
            for (int p = SERVO_CENTER_PULSE; p >= (SERVO_CENTER_PULSE - scan_delta); p -= SERVO_STEP_US) {
                servo_set_pulse(p);
                sleep_ms(SERVO_SAMPLE_DELAY_MS);
                
                float scan_dist = get_distance_cm(TRIG_PIN, ECHO_PIN);
                printf("  [RIGHT] Pulse: %d, Dist: %.1f cm\n", p, scan_dist);

                if (scan_dist > 0.1f && scan_dist < DETECTION_RANGE_CM) {
                    // Update extent
                    if (p < min_pulse_seen) min_pulse_seen = p;
                    // Update closest distance
                    if (scan_dist < min_dist_seen) min_dist_seen = scan_dist;
                }
            }
            // Return to center
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
                    // Update extent
                    if (p > max_pulse_seen) max_pulse_seen = p;
                    // Update closest distance
                    if (scan_dist < min_dist_seen) min_dist_seen = scan_dist;
                }
            }
            // Return to center
            servo_set_pulse(SERVO_CENTER_PULSE);
            sleep_ms(500);

            // --- CALCULATE WIDTHS ---
            // Calculate raw span in microseconds
            float left_span_us = (float)(max_pulse_seen - SERVO_CENTER_PULSE);
            float right_span_us = (float)(SERVO_CENTER_PULSE - min_pulse_seen);

            // Convert to degrees
            float left_deg = left_span_us / SERVO_US_PER_DEG;
            float right_deg = right_span_us / SERVO_US_PER_DEG;

            printf("  [DEBUG] Raw Angles -> L: %.1f deg, R: %.1f deg. Min Dist: %.1f\n", left_deg, right_deg, min_dist_seen);

            // Apply Beam Width Correction
            // Subtract the beam width from the detected angle to reduce "smearing"
            left_deg = (left_deg > BEAM_CORRECTION_DEG) ? (left_deg - BEAM_CORRECTION_DEG) : 0.0f;
            right_deg = (right_deg > BEAM_CORRECTION_DEG) ? (right_deg - BEAM_CORRECTION_DEG) : 0.0f;

            // Calculate Width = Radius * Angle(rad)
            float left_obs_width = min_dist_seen * (left_deg * 3.14159f / 180.0f);
            float right_obs_width = min_dist_seen * (right_deg * 3.14159f / 180.0f);

            printf("Widths (Corrected) -> L: %.1f cm, R: %.1f cm. Total: %.1f cm\n", left_obs_width, right_obs_width, left_obs_width + right_obs_width);

            // 3. Decision: Go with left/right with lesser cm (lesser width)
            if (left_obs_width < right_obs_width) {
                printf("Turning LEFT (Lesser width)\n");
                
                // Step 1: Turn ~30 degrees LEFT
                motor_set(0.35f, -0.35f); 
                sleep_ms(300); 

                // Step 2: Go Forward for 2 seconds
                motor_set(0.25f, 0.25f); 
                sleep_ms(2000); 

                // Step 3: Turn RIGHT (opposite) ~60 degrees
                // (Double the time of the 30 deg turn)
                printf("Turning back RIGHT 60 deg...\n");
                motor_set(-0.35f, 0.35f); 
                sleep_ms(600);

                // Step 4: Move Forward to find the line
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
                    // Timeout after 5 seconds to prevent infinite run
                    if (time_us_64() - search_start > 5000000) break;
                    sleep_ms(10);
                }

            } else {
                printf("Turning RIGHT (Lesser width)\n");
                
                // Step 1: Turn ~30 degrees RIGHT
                motor_set(-0.35f, 0.35f); 
                sleep_ms(300); 

                // Step 2: Go Forward for 2 seconds
                motor_set(0.25f, 0.25f); 
                sleep_ms(2000); 

                // Step 3: Turn LEFT (opposite) ~60 degrees
                printf("Turning back LEFT 60 deg...\n");
                motor_set(0.35f, -0.35f); 
                sleep_ms(600);

                // Step 4: Move Forward to find the line
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
                    // Timeout after 5 seconds to prevent infinite run
                    if (time_us_64() - search_start > 5000000) break;
                    sleep_ms(10);
                }
            }
            
            // Stop and resume loop (PID will take over)
            motor_set(0,0);
            sleep_ms(200); // Stabilize before next sensor reading
            
            // The loop will restart, read 'dist' again, and ONLY enter this block
            // if 'dist < STOP_DISTANCE_CM' (20cm). If path is clear, it skips.
            continue; 
        }

        // -------------------------------------------------------------------
        // 1. PID LINE FOLLOWING  (restored from original Demo 2 behaviour)
        // -------------------------------------------------------------------
        float sensor_val = ls.right_on_line ? 1.0f : 0.0f;  // black = 1.0, white = 0.0
        float error      = linePID.setpoint - sensor_val;
        float correction = pid_update(&linePID, error);

        // Minimum forward drive for small positive speeds
        float min_drive = 0.20f;

        if (ls.right_on_line) {
            // On black line -> go straight with PID correction + calibration bias
            left_speed  = base_speed - correction + CALIBRATION_BIAS;
            right_speed = base_speed + correction;

            last_seen_black_time = now;

            // Same semantics as your original code:
            // +corr -> drifting right (need to steer left)
            // -corr -> drifting left (need to steer right)
            if (correction > 0.01f)
                last_turn_dir = +1;   // drifting right → last turn was LEFT
            else if (correction < -0.01f)
                last_turn_dir = -1;   // drifting left → last turn was RIGHT
            // else keep last_turn_dir as-is
        } 
        else {
            // --- Off the line ---
            uint64_t lost_duration = now - last_seen_black_time;

            if (lost_duration < 300000) {
                // Short-term drift → follow last known direction
                if (last_turn_dir == -1) {
                    // last time we corrected RIGHT → so we were drifting left
                    // bias more to RIGHT wheel (turn right)
                    left_speed  = base_speed * 1.0f;
                    right_speed = base_speed * 0.5f;
                } else { 
                    // last time we corrected LEFT → we were drifting right
                    // bias more to LEFT wheel (turn left)
                    left_speed  = base_speed * 0.25f;
                    right_speed = base_speed * 1.1f;
                }
            } 
            else {
                // Lost line for a while → active search
                printf("Line lost — probing both sides...\n");
                bool found = false;

                // First spin LEFT a bit
                for (int i = 0; i < 15; i++) {
                    motor_set(0.30f, -0.30f);  // left spin
                    sleep_ms(25);
                    lf_read(&ls);
                    if (ls.right_on_line) { found = true; break; }
                }

                if (!found) {
                    // Then spin RIGHT if not found
                    for (int i = 0; i < 25; i++) {
                        motor_set(-0.30f, 0.30f);  // right spin
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

        sleep_ms(60); // Loop delay
    }
}