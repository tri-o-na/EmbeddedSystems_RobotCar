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
    printf("=== Demo 2: Single IR + Barcode (Fixed) ===\n");

    imu_init();
    motors_and_encoders_init();
    lf_init();

    barcode_cfg_t bc;
    barcode_init(&bc,
                 120000,   // short barcode ≤120ms = LEFT
                 250000,   // long barcode ≥250ms = RIGHT
                 200000);  // quiet gap before next barcode

    const float base_speed = 0.5f;

    printf("Starting control loop...\n");

    while (true) {
        line_sample_t ls;
        lf_read(&ls);

        // === Line Following ===
        float left_speed, right_speed;
        if (ls.left_on_line) {
            // On black line → go straight
            left_speed = base_speed;
            right_speed = base_speed;
            motor_set(left_speed, right_speed);
        } else {
            // Off line → nudge left to find black
            left_speed = base_speed;
            right_speed = base_speed * 0.4f;
            motor_set(left_speed, right_speed);
        }

        // === Barcode Detection ===
        barcode_cmd_t cmd = barcode_feed(&bc, ls.black_us_left);

        if (cmd == BC_LEFT) {
            printf("Barcode: LEFT turn detected\n");
            // TODO: turn_using_imu(-90);
        } else if (cmd == BC_RIGHT) {
            printf("Barcode: RIGHT turn detected\n");
            // TODO: turn_using_imu(90);
        } else if (cmd == BC_STOP) {
            printf("Barcode: STOP detected\n");
            motors_stop();
        }

        // === Telemetry ===
        printf("ADC=%4u [THRESH=2000] %s | L=%.2f R=%.2f | Dur=%6lu us | BC=%s\n",
               ls.adc_left,
               ls.left_on_line ? "ON_BLACK " : "ON_WHITE ",
               left_speed, right_speed,
               (unsigned long)ls.black_us_left,
               barcode_cmd_name(cmd));

        sleep_ms(50);
    }
}
