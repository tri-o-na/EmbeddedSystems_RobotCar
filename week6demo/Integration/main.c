#include "pico/stdlib.h"
#include "imu_lis3dh.h"
#include "motor_encoder.h"
#include "pid.h"
#include <stdio.h>
#include <math.h>

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("=== Encoder-Based Straight Driving ===\n");

    // --- Initialize sensors and motors ---
    imu_init();
    motors_and_encoders_init();

    // --- PID controller for drift correction ---
    PID driftPID;
    // Kp: how aggressively to correct drift (start low, increase if needed)
    // Ki: helps eliminate steady-state drift (start at 0, add if car still drifts)
    // Kd: reduces oscillation (helps smooth corrections)
    pid_init(&driftPID, 0.00008f, 0.0000002f, 0.00001f, -0.15f, 0.15f);
    driftPID.setpoint = 0.0f;  // target: equal encoder counts (no drift)

    // --- Base speeds (tune these first to get roughly straight) ---
    const float BASE_SPEED = 0.62f;  // Overall forward speed
    
    // --- Tracking variables ---
    static uint32_t left_total = 0;
    static uint32_t right_total = 0;
    uint32_t last_left_ticks = 0;
    uint32_t last_right_ticks = 0;

    imu_state_t imu = {0};

    printf("Starting control loop...\n");
    printf("The car will try to drive straight by keeping encoder counts equal.\n\n");

    while (true) {
        // --- Read encoders ---
        uint32_t left_ticks = encoder_get_count(1);
        uint32_t right_ticks = encoder_get_count(2);

        // Calculate incremental ticks since last reading
        uint32_t left_delta = left_ticks - last_left_ticks;
        uint32_t right_delta = right_ticks - last_right_ticks;
        
        // Accumulate total distance traveled by each wheel
        left_total += left_delta;
        right_total += right_delta;
        
        // Update last tick counts
        last_left_ticks = left_ticks;
        last_right_ticks = right_ticks;

        // --- Calculate drift error ---
        // Positive error = right wheel has traveled further (car turning left)
        // Negative error = left wheel has traveled further (car turning right)
        float drift_error = (float)((int32_t)right_total - (int32_t)left_total);

        // --- PID correction ---
        float correction = pid_update(&driftPID, drift_error);
        
        // Small deadband to ignore minor noise
        if (fabsf(correction) < 0.01f) correction = 0;

        // --- Apply correction to motors ---
        // If right wheel is ahead (drift_error > 0), slow down right, speed up left
        // If left wheel is ahead (drift_error < 0), slow down left, speed up right
        float left_speed  = BASE_SPEED - correction;
        float right_speed = BASE_SPEED + correction;

        // --- Clamp to safe range [0, 1] ---
        if (left_speed > 1.0f) left_speed = 1.0f;
        if (left_speed < 0.0f) left_speed = 0.0f;
        if (right_speed > 1.0f) right_speed = 1.0f;
        if (right_speed < 0.0f) right_speed = 0.0f;

        // --- Apply to motors ---
        motor_set(left_speed, right_speed);

        // --- Optional: Read IMU for monitoring (not used for control) ---
        imu_read(&imu);

        // --- Telemetry ---
        printf("Drift=%ld Corr=%.4f | L=%.3f R=%.3f | EncL=%lu EncR=%lu | Total L=%lu R=%lu\n",
               (long)drift_error, correction,
               left_speed, right_speed,
               (unsigned long)left_delta, (unsigned long)right_delta,
               (unsigned long)left_total, (unsigned long)right_total);

        sleep_ms(50);
    }
}