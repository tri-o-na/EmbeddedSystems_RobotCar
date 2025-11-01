#include "pico/stdlib.h"
#include "imu_lis3dh.h"
#include "motor_encoder.h"
#include "pid.h"
#include "ultrasonic.h"
#include "servo.h"
#include "servo.h" // <-- Add this include

#include "hardware/pwm.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define SERVO_PIN 15
#define TRIG_PIN 0
#define ECHO_PIN 1

#define DETECTION_RANGE_CM 17.0   // Stop and scan at this distance
#define OBJECT_EDGE_THRESHOLD 40.0   // cm
#define REVERSE_TIME_MS 500    // ms
#define BYPASS_TIME_MS 6000    // ms
#define SMALL_TURN_TIME_MS 360 // ms, for ~15 degree turn

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
            if (d >= OBJECT_EDGE_THRESHOLD) break;
        }
    }

    printf("\n");
    ScanResult result = {max_distance, minus_one_count};
    return result;
}

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("=== Demo: IMU+PID Forward + Servo Obstacle Avoidance (FIXED v2) ===\n");

    imu_init();
    motors_and_encoders_init();
    servo_init(SERVO_PIN); // <-- Use servo_init for servo
    setupUltrasonicPins(TRIG_PIN, ECHO_PIN);

    PID headingPID;
    pid_init(&headingPID, 0.5f, 0.0f, 0.02f, -0.15f, 0.15f);
    headingPID.setpoint = 0.0f;

    while (true) {
        float center_distance = get_distance_reading();
        printf("DBG: center_distance=%.1f cm\n", center_distance);

        if (center_distance > 0 && center_distance <= DETECTION_RANGE_CM) {
            // Stop the car and handle obstacle
            motors_stop();
            sleep_ms(500);

            printf("\n=== OBSTACLE DETECTED at %.1f cm ===\n", center_distance);

            // --- Obstacle handling (scan and bypass) ---
            encoder_release_pin_for_servo();
            servo_init(SERVO_PIN);

            // --- Scan LEFT side and calculate width ---
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
                    
                    // Convert servo pulse to angle (adjust to match YOUR servo!)
                    // Standard: 1000µs=-90°, 1500µs=0°, 2000µs=+90°
                    float angle1_deg = ((float)(left_first_pos - 1500) / 500.0f) * 90.0f;
                    float angle2_deg = ((float)(left_last_pos - 1500) / 500.0f) * 90.0f;
                    
                    // Convert to radians
                    float angle1_rad = angle1_deg * M_PI / 180.0f;
                    float angle2_rad = angle2_deg * M_PI / 180.0f;
                    
                    // Convert polar (angle, distance) to Cartesian (x, y)
                    float x1 = left_first_dist * sinf(angle1_rad);  // sin because 0° is forward
                    float y1 = left_first_dist * cosf(angle1_rad);
                    float x2 = left_last_dist * sinf(angle2_rad);
                    float y2 = left_last_dist * cosf(angle2_rad);
                    
                    // Calculate Euclidean distance between the two points
                    left_width = sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
                    
                    printf("  LEFT width calculation:\n");
                    printf("    First edge: pos=%d, angle=%.1f°, dist=%.1f cm -> (%.1f, %.1f)\n", 
                           left_first_pos, angle1_deg, left_first_dist, x1, y1);
                    printf("    Last edge:  pos=%d, angle=%.1f°, dist=%.1f cm -> (%.1f, %.1f)\n", 
                           left_last_pos, angle2_deg, left_last_dist, x2, y2);
                    printf("    WIDTH = %.1f cm\n", left_width);
                }
                
                printf("  LEFT Summary: %d detections (avg dist=%.1f cm), %d clear, width=%.1f cm\n", 
                       left_detections, left_avg_obstacle_dist, left_clear_count, left_width);
            }
            servo_move_to_center();
            sleep_ms(500);

            // --- Scan RIGHT side and calculate width ---
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
                    
                    printf("  RIGHT width calculation:\n");
                    printf("    First edge: pos=%d, angle=%.1f°, dist=%.1f cm -> (%.1f, %.1f)\n", 
                           right_first_pos, angle1_deg, right_first_dist, x1, y1);
                    printf("    Last edge:  pos=%d, angle=%.1f°, dist=%.1f cm -> (%.1f, %.1f)\n", 
                           right_last_pos, angle2_deg, right_last_dist, x2, y2);
                    printf("    WIDTH = %.1f cm\n", right_width);
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

            // Execute bypass maneuver
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
            }
            printf("Path clear! Resuming normal operation.\n\n");
        } else {
            // Move forward at normal speed
            motor_set(-0.6f, -0.6f);
        }
        sleep_ms(20);
    }
}