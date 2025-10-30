#include <stdio.h>
#include "pico/stdlib.h"
#include "imu_lis3dh.h"
#include "motor_encoder.h"
#include "pid.h"

// Target heading (degrees)
#define TARGET_HEADING 0.0f

// PID tuning (adjust to your setup)
#define KP 1.8f
#define KI 0.0f
#define KD 0.3f

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("\n=== Robot Car Integration Demo ===\n");

    // --- Initialize IMU ---
    if (!imu_init()) {
        printf("IMU init failed! Check wiring.\n");
        while (1) sleep_ms(1000);
    }
    printf("IMU OK.\n");

    // --- Initialize motors + encoders ---
    motors_and_encoders_init();

    // --- PID Controller for heading ---
    PID headingPID;
    pid_init(&headingPID, KP, KI, KD, -0.4f, 0.4f);  // limit correction ±40%

    imu_state_t imu = {0};

    const float base_speed = 0.45f;  // base motor speed fraction
    absolute_time_t last_print = get_absolute_time();

    while (true) {
        // Read IMU
        if (!imu_read(&imu)) {
            printf("IMU read failed\n");
            sleep_ms(10);
            continue;
        }

        // Compute heading correction
        float error = TARGET_HEADING - imu.heading;
        if (error > 180.0f) error -= 360.0f;
        if (error < -180.0f) error += 360.0f;

        float corr = pid_update(&headingPID, error);

        // Adjust motor speeds based on correction
        float left_speed  = base_speed + corr;
        float right_speed = base_speed - corr;

        // Clamp to [-1,1]
        if (left_speed > 1.0f) left_speed = 1.0f;
        if (left_speed < -1.0f) left_speed = -1.0f;
        if (right_speed > 1.0f) right_speed = 1.0f;
        if (right_speed < -1.0f) right_speed = -1.0f;

        // Send to motors
        motor_set(left_speed, right_speed);

        // Print telemetry every 200 ms
        if (absolute_time_diff_us(last_print, get_absolute_time()) > 200000) {
            last_print = get_absolute_time();
            printf("H=%.2f° err=%.2f corr=%.3f  L=%.2f R=%.2f\n",
                imu.heading, error, corr, left_speed, right_speed);
        }

        sleep_ms(20);
    }
}
