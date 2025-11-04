#include "pico/stdlib.h"
#include "motor_encoder.h"
#include "ultrasonic.h"
#include "servo.h"
#include "imu_lis3dh.h"
#include <stdio.h>
#include <math.h>

#define SERVO_PIN 15
#define TRIG_PIN 0
#define ECHO_PIN 1

#define DETECTION_RANGE_CM 45.0f      // Stop and scan when obstacle within 40cm
#define OBJECT_EDGE_THRESHOLD 55.0f   // Consider "clear" if reading ≥55cm
#define REVERSE_TIME_MS 500
#define BYPASS_TIME_MS 6000
#define SMALL_TURN_TIME_MS 100        // ms, for ~15 degree turn

// Speed profile
const float base_speed = -0.05f;

#define APPROACH_MIN_SPEED   (base_speed * 0.6f)
#define TURN_SPEED           (base_speed * 0.8f)
#define REVERSE_SPEED        (base_speed * 0.8f)

#define SLOWDOWN_START_CM    60.0f
#define EMERGENCY_STOP_CM    12.0f
#define ULTRASONIC_CHECK_DELAY_MS 10

// IMU heading control
#define HEADING_TOLERANCE 2.0f

static float get_distance_reading() {
    return get_stable_distance_cm(TRIG_PIN, ECHO_PIN);
}

// Struct to hold scan results
typedef struct {
    float max_distance;
    int minus_one_count;
} ScanResult;

// Scan full range and return both maximum clearance and -1.0 count
static ScanResult scan_for_max_clearance(bool scan_left) {
    const int step = 40;
    const uint32_t delay_ms = 250;
    int pos;
    float max_distance = -1.0f;
    int minus_one_count = 0;

    printf("Scanning %s: ", scan_left ? "LEFT" : "RIGHT");

    if (scan_left) {
        for (pos = 1500; pos <= 2000; pos += step) {
            servo_set_pulse_us((uint16_t)pos);
            sleep_ms(delay_ms);
            float d = get_distance_reading();
            printf("%.1f ", d);
            if (d == -1.0f) minus_one_count++;
            if (d > max_distance) max_distance = d;
            if (d >= OBJECT_EDGE_THRESHOLD) break;
        }
    } else {
        for (pos = 1500; pos >= 1000; pos -= step) {
            servo_set_pulse_us((uint16_t)pos);
            sleep_ms(delay_ms);
            float d = get_distance_reading();
            printf("%.1f ", d);
            if (d == -1.0f) minus_one_count++;
            if (d > max_distance) max_distance = d;
            if (d >= OBJECT_EDGE_THRESHOLD) break;  // Add this line!
        }
    }

    printf("\n");
    ScanResult result = {max_distance, minus_one_count};
    return result;
}

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float approach_speed(float dist_cm) {
    if (dist_cm <= 0) return base_speed;
    if (dist_cm <= DETECTION_RANGE_CM) return APPROACH_MIN_SPEED;
    if (dist_cm >= SLOWDOWN_START_CM) return base_speed;

    float t = (dist_cm - DETECTION_RANGE_CM) / (SLOWDOWN_START_CM - DETECTION_RANGE_CM);
    t = clampf(t, 0.0f, 1.0f);
    return APPROACH_MIN_SPEED + t * (base_speed - APPROACH_MIN_SPEED);
}

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("=== Obstacle Avoidance with IMU Heading Control ===\n");

    motors_and_encoders_init();
    servo_init(SERVO_PIN);
    setupUltrasonicPins(TRIG_PIN, ECHO_PIN);
    imu_init();
    
    imu_state_t imu = {0};
    // Initialize heading properly ONCE
    imu_read(&imu);
    float target_heading = imu.heading;
    float heading_smooth = imu.heading;

    printf("IMU initialized. Starting obstacle avoidance...\n");

    while (true) {
        bool imu_ok = imu_read(&imu);
        if (imu_ok) {
            heading_smooth = 0.85f * heading_smooth + 0.15f * imu.heading;
        }

        float center_distance = get_distance_reading();
        printf("DBG: dist=%.1f cm, heading=%.1f°\n", center_distance, heading_smooth);

        // Emergency stop (brake then brief reverse)
        if (center_distance > 0 && center_distance <= EMERGENCY_STOP_CM) {
            motors_stop();
            motor_set(-REVERSE_SPEED * 0.6f, -REVERSE_SPEED * 0.6f);
            sleep_ms(150);
            motors_stop();
        }

        if (center_distance > 0 && center_distance <= DETECTION_RANGE_CM) {
            // Stop and scan
            motors_stop();
            sleep_ms(500);
            printf("\n=== OBSTACLE DETECTED at %.1f cm ===\n", center_distance);

            encoder_release_pin_for_servo();
            servo_init(SERVO_PIN);
            sleep_ms(100);

            // --- Scan LEFT side ---
            int left_detections = 0;
            int left_clear_count = 0;
            float left_avg_obstacle_dist = 0;
            float left_max_clear_dist = -1.0f;
            float left_width = -1.0f;
            int left_first_pos = -1, left_last_pos = -1;
            float left_first_dist = -1.0f, left_last_dist = -1.0f;
            {
                const int step = 40;
                printf("LEFT SCAN:\n");
                float prev_dist = -1.0f;
                
                for (int pos = 1500; pos <= 2000; pos += step) {
                    servo_set_pulse_us((uint16_t)pos);
                    sleep_ms(300);
                    float d = get_distance_reading();
                    
                    // Smooth out sudden jumps > 5cm
                    if (prev_dist > 0 && d > 0 && fabsf(d - prev_dist) > 5.0f) {
                        d = (prev_dist + d) / 2.0f;
                    }
                    
                    printf("  pos=%d, dist=%.1f cm", pos, d);
                    
                    if (d > 0 && d < OBJECT_EDGE_THRESHOLD) {
                        // Obstacle detected
                        if (left_first_pos == -1) {
                            left_first_pos = pos;
                            left_first_dist = d;
                        }
                        left_last_pos = pos;
                        left_last_dist = d;
                        left_detections++;
                        left_avg_obstacle_dist += d;
                        printf(" [OBSTACLE]\n");
                    } else if (d >= OBJECT_EDGE_THRESHOLD || d == -1.0f) {
                        // Clear path
                        left_clear_count++;
                        if (d > left_max_clear_dist) left_max_clear_dist = d;
                        printf(" [CLEAR]\n");
                    } else {
                        printf("\n");
                    }
                    
                    prev_dist = d;
                }
                
                if (left_detections > 0) {
                    left_avg_obstacle_dist /= left_detections;
                }
                
                // Calculate width using polar to Cartesian conversion
                if (left_first_pos != -1 && left_last_pos != -1 && 
                    left_first_pos != left_last_pos && 
                    left_first_dist > 0 && left_last_dist > 0) {
                    
                    float angle1_deg = ((float)(left_first_pos - 1500) / 500.0f) * 90.0f;
                    float angle2_deg = ((float)(left_last_pos - 1500) / 500.0f) * 90.0f;
                    
                    float angle1_rad = angle1_deg * M_PI / 180.0f;
                    float angle2_rad = angle2_deg * M_PI / 180.0f;
                    
                    float x1 = left_first_dist * sinf(angle1_rad);
                    float y1 = left_first_dist * cosf(angle1_rad);
                    float x2 = left_last_dist * sinf(angle2_rad);
                    float y2 = left_last_dist * cosf(angle2_rad);
                    
                    left_width = sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
                    
                    printf("  LEFT width = %.1f cm\n", left_width);
                }
                
                printf("  LEFT Summary: %d detections (avg dist=%.1f cm), %d clear, width=%.1f cm\n", 
                       left_detections, left_avg_obstacle_dist, left_clear_count, left_width);
            }
            servo_move_to_center();
            sleep_ms(500);

            // --- Scan RIGHT side ---
            int right_detections = 0;
            int right_clear_count = 0;
            float right_avg_obstacle_dist = 0;
            float right_max_clear_dist = -1.0f;
            float right_width = -1.0f;
            int right_first_pos = -1, right_last_pos = -1;
            float right_first_dist = -1.0f, right_last_dist = -1.0f;
            {
                const int step = 40;
                printf("RIGHT SCAN:\n");
                float prev_dist = -1.0f;
                
                for (int pos = 1500; pos >= 1000; pos -= step) {
                    servo_set_pulse_us((uint16_t)pos);
                    sleep_ms(300);
                    float d = get_distance_reading();
                    
                    // Smooth out sudden jumps > 5cm
                    if (prev_dist > 0 && d > 0 && fabsf(d - prev_dist) > 5.0f) {
                        d = (prev_dist + d) / 2.0f;
                    }
                    
                    printf("  pos=%d, dist=%.1f cm", pos, d);
                    
                    if (d > 0 && d < OBJECT_EDGE_THRESHOLD) {
                        // Obstacle detected
                        if (right_first_pos == -1) {
                            right_first_pos = pos;
                            right_first_dist = d;
                        }
                        right_last_pos = pos;
                        right_last_dist = d;
                        right_detections++;
                        right_avg_obstacle_dist += d;
                        printf(" [OBSTACLE]\n");
                    } else if (d >= OBJECT_EDGE_THRESHOLD || d == -1.0f) {
                        // Clear path
                        right_clear_count++;
                        if (d > right_max_clear_dist) right_max_clear_dist = d;
                        printf(" [CLEAR]\n");
                    } else {
                        printf("\n");
                    }
                    
                    prev_dist = d;
                }
                
                if (right_detections > 0) {
                    right_avg_obstacle_dist /= right_detections;
                }
                
                // Calculate width using polar to Cartesian conversion
                if (right_first_pos != -1 && right_last_pos != -1 && 
                    right_first_pos != right_last_pos && 
                    right_first_dist > 0 && right_last_dist > 0) {
                    
                    float angle1_deg = ((float)(right_first_pos - 1500) / 500.0f) * 90.0f;
                    float angle2_deg = ((float)(right_last_pos - 1500) / 500.0f) * 90.0f;
                    
                    float angle1_rad = angle1_deg * M_PI / 180.0f;
                    float angle2_rad = angle2_deg * M_PI / 180.0f;
                    
                    float x1 = right_first_dist * sinf(angle1_rad);
                    float y1 = right_first_dist * cosf(angle1_rad);
                    float x2 = right_last_dist * sinf(angle2_rad);
                    float y2 = right_last_dist * cosf(angle2_rad);
                    
                    right_width = sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
                    
                    printf("  RIGHT width = %.1f cm\n", right_width);
                }
                
                printf("  RIGHT Summary: %d detections (avg dist=%.1f cm), %d clear, width=%.1f cm\n", 
                       right_detections, right_avg_obstacle_dist, right_clear_count, right_width);
            }
            servo_move_to_center();
            sleep_ms(300);

            motors_stop();
            encoder_reclaim_pin_from_servo();

            // --- Decision Logic: Use width + other factors ---
            bool go_left;
            int left_score = 0;
            int right_score = 0;
            
            printf("\n=== DECISION ANALYSIS ===\n");
            
            // Factor 1: Smaller width is better (easier to go around)
            if (left_width > 0 && right_width > 0) {
                if (left_width < right_width) {
                    left_score += 5;
                    printf("LEFT has smaller width (%.1f vs %.1f cm) +5\n", left_width, right_width);
                } else if (right_width < left_width) {
                    right_score += 5;
                    printf("RIGHT has smaller width (%.1f vs %.1f cm) +5\n", right_width, left_width);
                } else {
                    printf("Equal width (%.1f cm each)\n", left_width);
                }
            } else if (left_width > 0) {
                left_score += 3;
                printf("Only LEFT has valid width (%.1f cm) +3\n", left_width);
            } else if (right_width > 0) {
                right_score += 3;
                printf("Only RIGHT has valid width (%.1f cm) +3\n", right_width);
            }
            
            // Factor 2: More clear positions is better
            if (left_clear_count > right_clear_count) {
                left_score += 2;
                printf("LEFT has more clear positions (%d vs %d) +2\n", left_clear_count, right_clear_count);
            } else if (right_clear_count > left_clear_count) {
                right_score += 2;
                printf("RIGHT has more clear positions (%d vs %d) +2\n", right_clear_count, left_clear_count);
            }
            
            // Factor 3: Greater obstacle distance is better
            if (left_detections > 0 && right_detections > 0) {
                if (left_avg_obstacle_dist > right_avg_obstacle_dist) {
                    left_score += 1;
                    printf("LEFT obstacle is farther (%.1f vs %.1f cm) +1\n", left_avg_obstacle_dist, right_avg_obstacle_dist);
                } else if (right_avg_obstacle_dist > left_avg_obstacle_dist) {
                    right_score += 1;
                    printf("RIGHT obstacle is farther (%.1f vs %.1f cm) +1\n", right_avg_obstacle_dist, left_avg_obstacle_dist);
                }
            }
            
            printf("\nFinal Score: LEFT=%d, RIGHT=%d\n", left_score, right_score);
            printf("LEFT width: %.1f cm, RIGHT width: %.1f cm\n", left_width, right_width);
            
            // Make decision
            if (left_score > right_score) {
                go_left = true;
                printf("Decision: Bypass LEFT (better score)\n");
            } else if (right_score > left_score) {
                go_left = false;
                printf("Decision: Bypass RIGHT (better score)\n");
            } else {
                // Tie-breaker: prefer smaller width
                if (left_width > 0 && right_width > 0) {
                    go_left = left_width <= right_width;
                } else {
                    go_left = left_clear_count >= right_clear_count;
                }
                printf("Decision: Bypass %s (tie-breaker)\n", go_left ? "LEFT" : "RIGHT");
            }

            printf("\n=== DECISION: Bypass to %s ===\n\n", go_left ? "LEFT" : "RIGHT");

            // Execute bypass maneuver with your motor speeds
            printf("Step 1: Reversing...\n");
            motor_set(0.6f, 0.6f);
            sleep_ms(REVERSE_TIME_MS);
            motors_stop();
            sleep_ms(200);

            if (go_left) {
                printf("Step 2: Turning LEFT...\n");
                motor_set(-0.6f, 0.6f);
                sleep_ms(SMALL_TURN_TIME_MS);
                motors_stop();
                sleep_ms(200);

                printf("Step 3: Going straight (bypass)...\n");
                motor_set(-0.6f, -0.6f);
                sleep_ms(BYPASS_TIME_MS);
                motors_stop();
                sleep_ms(200);

                printf("Step 4: Turning RIGHT (resume heading)...\n");
                motor_set(0.6f, -0.6f);
                sleep_ms(SMALL_TURN_TIME_MS);
                motors_stop();
                sleep_ms(200);
            } else {
                printf("Step 2: Turning RIGHT...\n");
                motor_set(0.6f, -0.6f);
                sleep_ms(SMALL_TURN_TIME_MS);
                motors_stop();
                sleep_ms(200);

                printf("Step 3: Going straight (bypass)...\n");
                motor_set(-0.6f, -0.6f);
                sleep_ms(BYPASS_TIME_MS);
                motors_stop();
                sleep_ms(200);

                printf("Step 4: Turning LEFT (resume heading)...\n");
                motor_set(-0.6f, 0.6f);
                sleep_ms(SMALL_TURN_TIME_MS);
                motors_stop();
                sleep_ms(200);
            }

            printf("Bypass complete. Waiting for path to clear...\n");
            // Wait until the path ahead is clear before resuming
            int wait_count = 0;
            while (get_distance_reading() > 0 && get_distance_reading() <= OBJECT_EDGE_THRESHOLD) {
                motors_stop();
                if (wait_count % 10 == 0) {  // Print every second
                    printf("Waiting for path to clear... (%.1f cm)\n", get_distance_reading());
                }
                wait_count++;
                sleep_ms(100);
                if (wait_count >= 50) { printf("Timeout - resuming\n"); break; }
            }
            printf("Path clear! Resuming normal operation.\n\n");

            // Update target heading after bypass
            if (imu_read(&imu)) {
                target_heading = imu.heading;
                heading_smooth = imu.heading;
                printf("New target heading: %.1f°\n", target_heading);
            }
        } else {
            // Forward with heading correction
            float sp = approach_speed(center_distance);
            
            float left_speed = sp;
            float right_speed = sp;

            // Apply trim to both motors (both wheels will be equal speed)
            motor_set(left_speed, right_speed);
             
            printf("  sp=%.2f L=%.2f R=%.2f\n",
                   sp, left_speed, right_speed);
        }

        sleep_ms(ULTRASONIC_CHECK_DELAY_MS);
    }
}