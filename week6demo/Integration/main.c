// #include "pico/stdlib.h"
// #include "imu_lis3dh.h"
// #include "motor_encoder.h"
// #include "pid.h"
// #include <stdio.h>
// #include <math.h>

// int main() {
//     stdio_init_all();
//     sleep_ms(500);
//     printf("=== Demo 10: LIS3DH + PID Motor Control ===\n");

//     imu_init();
//     motors_and_encoders_init();

//     PID headingPID;
//     pid_init(&headingPID, 0.5f, 0.0f, 0.02f, -0.15f, 0.15f);
//     headingPID.setpoint = 0.0f;

//     const float base_speed = 0.55f;  // more torque
//     imu_state_t imu = {0};
//     static float heading_smooth = 0;

//     printf("Starting control loop...\n");

//     while (true) {
//         bool ok = imu_read(&imu);
//         if (!ok) {
//             printf("IMU read failed — using last heading\n");
//         }

//         heading_smooth = 0.9f * heading_smooth + 0.1f * imu.heading;

//         float correction = pid_update(&headingPID, heading_smooth);
//         if (fabsf(correction) < 0.05f) correction = 0;

//         float left_speed = base_speed + correction;
//         float right_speed = base_speed - correction;

//         if (left_speed < 0.2f) left_speed = 0.2f;
//         if (right_speed < 0.2f) right_speed = 0.2f;
//         if (left_speed > 1.0f) left_speed = 1.0f;
//         if (right_speed > 1.0f) right_speed = 1.0f;

//         motor_set(left_speed, right_speed);

//         printf("Heading=%.2f Corr=%.2f L=%.2f R=%.2f | EncL=%lu EncR=%lu\n",
//                heading_smooth, correction, left_speed, right_speed,
//                (unsigned long)encoder_pulse_width_us(1),
//                (unsigned long)encoder_pulse_width_us(2));

//         sleep_ms(50);
//     }
// }

// #include "pico/stdlib.h"
// #include "imu_lis3dh.h"
// #include "motor_encoder.h"
// #include "pid.h"
// #include <stdio.h>
// #include <math.h>

// int main() {
//     stdio_init_all();
//     sleep_ms(500);
//     printf("=== Demo 10: LIS3DH + PID Motor Control ===\n");

//     imu_init();
//     motors_and_encoders_init();

//     PID headingPID;
//     pid_init(&headingPID, 0.5f, 0.0f, 0.02f, -0.15f, 0.15f);
//     headingPID.setpoint = 0.0f;

//     const float base_speed = 0.55f;  // more torque
//     imu_state_t imu = {0};
//     static float heading_smooth = 0;

//     printf("Starting control loop...\n");

//     while (true) {
//         bool ok = imu_read(&imu);
//         if (!ok) {
//             printf("IMU read failed — using last heading\n");
//         }

//         heading_smooth = 0.9f * heading_smooth + 0.1f * imu.heading;

//         float correction = pid_update(&headingPID, heading_smooth);
//         if (fabsf(correction) < 0.05f) correction = 0;

//         float left_speed = base_speed + correction;
//         float right_speed = base_speed - correction;

//         if (left_speed < 0.2f) left_speed = 0.2f;
//         if (right_speed < 0.2f) right_speed = 0.2f;
//         if (left_speed > 1.0f) left_speed = 1.0f;
//         if (right_speed > 1.0f) right_speed = 1.0f;

//         motor_set(left_speed, right_speed);

//         printf("Heading=%.2f Corr=%.2f L=%.2f R=%.2f | EncL=%lu EncR=%lu\n",
//                heading_smooth, correction, left_speed, right_speed,
//                (unsigned long)encoder_pulse_width_us(1),
//                (unsigned long)encoder_pulse_width_us(2));

//         sleep_ms(50);
//     }
// }


#include "pico/stdlib.h"
#include "imu_lis3dh.h"
#include "motor_encoder.h"
#include "pid.h"
#include <stdio.h>
#include <math.h>
#include <line_follow.h>
#include <barcode.h>

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("=== Demo 2: Single IR + PID + Smart Recovery (Left Turn Fix) ===\n");

    imu_init();
    motors_and_encoders_init();
    lf_init();

    const float base_speed = 0.30f;
    float left_speed = 0.0f, right_speed = 0.0f;

    // PID setup
    PID pid;
    pid_init(&pid, 0.08f, 0.00f, 0.002f, -0.25f, 0.25f);
    
    pid.setpoint = 1.0f; // want to always see black (1.0)

    uint64_t last_seen_black_time = 0;
    int last_turn_dir = 0; // -1 = left, +1 = right, 0 = straight
    uint64_t now;

    printf("Starting control loop...\n");

    while (true) {
        line_sample_t ls;
        lf_read(&ls);
        now = time_us_64();

        // Convert sensor to analog-like input for PID
        float sensor_val = ls.left_on_line ? 1.0f : 0.0f;
        float error = pid.setpoint - sensor_val;
        float correction = pid_update(&pid, error);

        if (ls.left_on_line) {
            // On black line → go straight, apply small PID bias
            left_speed  = base_speed - correction;
            right_speed = base_speed + correction;
            last_seen_black_time = now;

            // Stabilize last turn direction (±0.01 threshold to avoid noise)
            if (correction > 0.01f)
                last_turn_dir = +1;   // drifting right → needs left correction
            else if (correction < -0.01f)
                last_turn_dir = -1;   // drifting left → needs right correction
        } 
        else {
            // --- Off the line ---
            uint64_t lost_duration = now - last_seen_black_time;

            if (lost_duration < 300000) {
                // Short-term drift → steer gently using last known direction
                if (last_turn_dir == -1) {
                    // stronger left bias
                    left_speed  = base_speed * 0.25f;
                    right_speed = base_speed * 1.1f;
                } else {
                    left_speed  = base_speed * 1.0f;
                    right_speed = base_speed * 0.5f;
                }
            } 
            else {
                // Lost line
                uint64_t lost_duration = now - last_seen_black_time;

                if (lost_duration < 300000) {
                    // short loss — keep bias
                    if (last_turn_dir == -1)
                        motor_set(0.2f, 0.45f); // turn left
                    else
                        motor_set(0.45f, 0.2f); // turn right
                } else {
                    printf("Line lost — probing both sides...\n");
                    bool found = false;
                    // 1) Try right
                    for (int i = 0; i < 15; i++) {
                        motor_set(0.3f, -0.3f);
                        sleep_ms(25);
                        lf_read(&ls);
                        if (ls.left_on_line) { found = true; break; }
                    }
                    // 2) Try left if not found
                    if (!found) {
                        for (int i = 0; i < 25; i++) {
                            motor_set(-0.3f, 0.3f);
                            sleep_ms(25);
                            lf_read(&ls);
                            if (ls.left_on_line) { found = true; break; }
                        }
                    }

                    motor_set(0, 0);
                    if (found) {
                        printf("Reacquired black line!\n");
                    } else {
                        printf("Still lost — reversing briefly...\n");
                        motor_set(-0.25f, -0.25f);
                        sleep_ms(200);
                    }
                }
            }

        }

        // Apply computed motor speeds
        motor_set(right_speed, left_speed);

        printf("ADC=%4u | onLine=%d | err=%.2f | corr=%.3f | L=%.2f R=%.2f | dir=%d\n",
               ls.adc_left, ls.left_on_line, error, correction,
               left_speed, right_speed, last_turn_dir);

        sleep_ms(60);
    }
}
