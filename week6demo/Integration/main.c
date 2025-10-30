#include "pico/stdlib.h"
#include "imu_mpu6050.h"
#include "motor_encoder.h"
#include "pid.h"
#include <stdio.h>

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("Demo1: PID + IMU\n");

    imu_init();
    motors_and_encoders_init();

    PID headingPID;
    pid_init(&headingPID, 1.0, 0.1, 0.05, -0.5, 0.5);
    headingPID.setpoint = 0;

    const float base_speed = 0.4;
    imu_state_t imu = {0};

    while (true) {
        imu_read(&imu);
        float correction = pid_update(&headingPID, imu.heading);
        float left = base_speed + correction;
        float right = base_speed - correction;
        motor_set(left, right);
        printf("H=%.2f Corr=%.2f L=%.2f R=%.2f\n", imu.heading, correction, left, right);
        sleep_ms(10);
    }
}
