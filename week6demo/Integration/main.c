#include "pico/stdlib.h"
#include "imu_lis3dh.h"
#include "motor_encoder.h"
#include "pid.h"
#include <stdio.h>
#include <math.h>

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("=== Demo 10: LIS3DH + PID Motor Control ===\n");

    imu_init();
    motors_and_encoders_init();

    PID headingPID;
    pid_init(&headingPID, 0.5f, 0.0f, 0.02f, -0.15f, 0.15f);
    headingPID.setpoint = 0.0f;

    const float base_speed = 0.55f;  // more torque
    imu_state_t imu = {0};
    static float heading_smooth = 0;

    printf("Starting control loop...\n");

    while (true) {
        bool ok = imu_read(&imu);
        if (!ok) {
            printf("IMU read failed — using last heading\n");
        }

        heading_smooth = 0.9f * heading_smooth + 0.1f * imu.heading;

        float correction = pid_update(&headingPID, heading_smooth);
        if (fabsf(correction) < 0.05f) correction = 0;

        float left_speed = base_speed + correction;
        float right_speed = base_speed - correction;

        if (left_speed < 0.2f) left_speed = 0.2f;
        if (right_speed < 0.2f) right_speed = 0.2f;
        if (left_speed > 1.0f) left_speed = 1.0f;
        if (right_speed > 1.0f) right_speed = 1.0f;

        motor_set(left_speed, right_speed);

        printf("Heading=%.2f Corr=%.2f L=%.2f R=%.2f | EncL=%lu EncR=%lu\n",
               heading_smooth, correction, left_speed, right_speed,
               (unsigned long)encoder_pulse_width_us(1),
               (unsigned long)encoder_pulse_width_us(2));

        sleep_ms(50);
    }
}
