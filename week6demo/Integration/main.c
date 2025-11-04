#include "pico/stdlib.h"
#include "imu_lis3dh.h"
#include "motor_encoder.h"
#include "pid.h"
#include <stdio.h>
#include <math.h>

// Test function to verify motor speed control
void test_motor_speeds(void) {
    printf("\n=== MOTOR SPEED TEST ===\n");
    
    printf("Testing LEFT motor at different speeds:\n");
    for (float speed = 0.2f; speed <= 1.0f; speed += 0.2f) {
        printf("Setting left=%.1f, right=0.0 - Watch if motor speed changes!\n", speed);
        motor_set(speed, 0.0f);
        sleep_ms(2000);  // Run for 2 seconds
    }
    
    motors_stop();
    sleep_ms(1000);
    
    printf("Testing RIGHT motor at different speeds:\n");
    for (float speed = 0.2f; speed <= 1.0f; speed += 0.2f) {
        printf("Setting left=0.0, right=%.1f - Watch if motor speed changes!\n", speed);
        motor_set(0.0f, speed);
        sleep_ms(2000);
    }
    
    motors_stop();
    printf("=== TEST COMPLETE ===\n\n");
}

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("=== Demo 1: Basic Motion & Sensing Integration ===\n");

    // --- Initialize sensors and motors ---
    imu_init();
    motors_and_encoders_init();

    // UNCOMMENT THIS LINE TO TEST MOTOR SPEED CONTROL
    //test_motor_speeds();
    //return 0;  // Stop here after test

    // --- PID controller for drift correction ---
    PID driftPID;
    // Stronger gains to actively reduce drift
    pid_init(&driftPID, 0.008f, 0.00003f, 0.001f, -0.25f, 0.25f);
    driftPID.setpoint = 0.0f;

    // --- Base speeds with mechanical bias ---
    const float BASE_LEFT = 0.20f;   // Left motor needs boost
    const float BASE_RIGHT = 0.12f;  // Right motor is naturally faster
    
    // --- Tracking variables ---
    static uint32_t left_total = 0;
    static uint32_t right_total = 0;
    uint32_t last_left_ticks = 0;
    uint32_t last_right_ticks = 0;

    float distance_traveled = 0.0f;
    const float TICKS_PER_CM = 10.0f;

    imu_state_t imu = {0};

    printf("Starting control loop...\n");
    printf("Adjusted for left motor mechanical difference\n\n");

    while (true) {
        // --- Read IMU ---
        if (!imu_read(&imu)) {
            printf("IMU read failed!\n");
            continue;
        }

        // --- Read encoders ---
        uint32_t left_ticks = encoder_get_count(1);
        uint32_t right_ticks = encoder_get_count(2);

        uint32_t left_delta = left_ticks - last_left_ticks;
        uint32_t right_delta = right_ticks - last_right_ticks;
        
        left_total += left_delta;
        right_total += right_delta;
        
        distance_traveled = ((left_total + right_total) / 2.0f) / TICKS_PER_CM;
        
        last_left_ticks = left_ticks;
        last_right_ticks = right_ticks;

        // --- Calculate drift error ---
        float drift_error = (float)((int32_t)right_total - (int32_t)left_total);

        // --- PID correction ---
        float correction = pid_update(&driftPID, drift_error);
        
        if (fabsf(correction) < 0.005f) correction = 0;

        // --- Apply correction with bias ---
        float left_speed  = BASE_LEFT + correction;
        float right_speed = BASE_RIGHT - correction;

        // --- Clamp to [0, 1] ---
        if (left_speed > 1.0f) left_speed = 1.0f;
        if (left_speed < 0.0f) left_speed = 0.0f;
        if (right_speed > 1.0f) right_speed = 1.0f;
        if (right_speed < 0.0f) right_speed = 0.0f;

        // --- Apply to motors ---
        motor_set(left_speed, right_speed);

        // --- Telemetry ---
        printf("=== TELEMETRY ===\n");
        printf("Speed: L=%.3f R=%.3f | ", left_speed, right_speed);
        printf("Distance: %.1f cm | ", distance_traveled);
        printf("Heading: %.2f°\n", imu.heading);
        
        printf("IMU (filtered): ax=%.3f ay=%.3f az=%.3f\n", 
               imu.ax, imu.ay, imu.az);
        
        printf("Encoders: L=%lu R=%lu (delta: L=%lu R=%lu)\n",
               (unsigned long)left_total, (unsigned long)right_total,
               (unsigned long)left_delta, (unsigned long)right_delta);
        
        printf("PID: Drift=%ld Correction=%.4f\n",
               (long)drift_error, correction);
        
        printf("\n");

        sleep_ms(50);
    }
}