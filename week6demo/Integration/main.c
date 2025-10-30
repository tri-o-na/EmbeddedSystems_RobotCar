#include <stdio.h>
#include "pico/stdlib.h"
#include "imu_mpu6050.h"
#include "motor_encoder.h"
#include "pid.h"

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("=== Demo 1: PID + IMU heading control ===\n");

    imu_init();
    motors_and_encoders_init();

    PID headingPID;
    pid_init(&headingPID, 1.0, 0.1, 0.05, -0.5, 0.5, 0.01);
    headingPID.setpoint = 0.0f;   // desired heading = 0°

    const float base_speed = 0.4f;

    while (true) {
        imu_state_t imu;
        if (!imu_read(&imu)) {
            printf("IMU read error\n");
            sleep_ms(100);
            continue;
        }

        float heading = imu.heading;           // degrees
        float correction = pid_update(&headingPID, heading);

        float left = base_speed + correction;
        float right = base_speed - correction;

        // Clamp to safe range
        if (left > 1) left = 1;
        if (left < -1) left = -1;
        if (right > 1) right = 1;
        if (right < -1) right = -1;

        motor_set(left, right);

        printf("Heading=%.2f Corr=%.2f L=%.2f R=%.2f\n",
               heading, correction, left, right);
        sleep_ms(10);
    }
}
