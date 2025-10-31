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
            servo_init(SERVO_PIN); // <-- Use servo_init for servo

            // Scan left and get results
            ScanResult left_result = scan_for_max_clearance(true);
            printf("LEFT max clearance: %.1f cm, -1.0 count: %d\n", left_result.max_distance, left_result.minus_one_count);

            // Return to center between scans
            servo_move_to_center();
            sleep_ms(500);

            // Scan right and get results
            ScanResult right_result = scan_for_max_clearance(false);
            printf("RIGHT max clearance: %.1f cm, -1.0 count: %d\n", right_result.max_distance, right_result.minus_one_count);

            // Return to center
            servo_move_to_center();
            sleep_ms(300);

            motors_stop();
            encoder_reclaim_pin_from_servo();

            // Decision logic: go to side with more -1.0 readings (more open positions)
            bool go_left;
            if (left_result.minus_one_count > right_result.minus_one_count) {
                go_left = true;
                printf("More open space detected on LEFT (more -1.0 readings).\n");
            } else if (right_result.minus_one_count > left_result.minus_one_count) {
                go_left = false;
                printf("More open space detected on RIGHT (more -1.0 readings).\n");
            } else {
                // If equal, default to LEFT
                go_left = true;
                printf("Equal open space, defaulting to LEFT.\n");
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