#include "pico/stdlib.h"
#include "imu_lis3dh.h"
#include "motor_encoder.h"
#include "pid.h"
#include <stdio.h>
#include <math.h>

// === CONFIG ===
#define BASE_SPEED 0.6f       // main forward speed (0.0–1.0)
#define CORR_LIMIT 0.25f      // limit PID correction output
#define LOOP_DELAY_MS 50      // update interval

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("=== Demo 1: IMU + PID + Encoder Integration ===\n");

    // --- Initialize systems ---
    imu_init();
    motors_and_encoders_init();

    // --- PID controller setup ---
    PID headingPID;
    pid_init(&headingPID, 0.8f, 0.0f, 0.05f, -CORR_LIMIT, CORR_LIMIT);
    headingPID.setpoint = 0.0f;  // want flat / straight heading

    imu_state_t imu = {0};
    uint32_t last_print = 0;

    printf("Starting control loop...\n");

    while (true) {
        // === Read IMU ===
        if (!imu_read(&imu)) continue;  // skip if failed

        // === Calculate correction ===
        float correction = pid_update(&headingPID, imu.heading);
        if (fabsf(correction) < 0.02f) correction = 0; // deadzone for noise

        // === Apply correction ===
        float left_speed  = BASE_SPEED + correction;
        float right_speed = BASE_SPEED - correction;

        // Clamp motor commands
        if (left_speed > 1.0f) left_speed = 1.0f;
        if (left_speed < 0.0f) left_speed = 0.0f;
        if (right_speed > 1.0f) right_speed = 1.0f;
        if (right_speed < 0.0f) right_speed = 0.0f;

        // Drive motors
        motor_set(left_speed, right_speed);

        // === Optional Encoder Feedback (for monitoring) ===
        uint32_t encL = encoder_pulse_width_us(1);
        uint32_t encR = encoder_pulse_width_us(2);

        // Print telemetry every 200ms
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_print > 200) {
            last_print = now;
            printf("Tilt=%.2f Corr=%.3f | L=%.2f R=%.2f | EncL=%lu EncR=%lu\n",
                   imu.heading, correction, left_speed, right_speed,
                   (unsigned long)encL, (unsigned long)encR);
        }

        sleep_ms(LOOP_DELAY_MS);
    }
}
