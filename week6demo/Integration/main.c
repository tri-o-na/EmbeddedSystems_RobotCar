// // Line Following with Single IR Sensor + PID + Smart Recovery
// #include "pico/stdlib.h"
// #include "imu_lis3dh.h"
// #include "motor_encoder.h"
// #include "pid.h"
// #include <stdio.h>
// #include <math.h>
// #include <line_follow.h>
// #include <barcode.h>

// int main() {
//     stdio_init_all();
//     sleep_ms(500);
//     printf("=== Demo 2: Single IR + PID + Smart Recovery (Left Turn Fix) ===\n");

//     imu_init();
//     motors_and_encoders_init();
//     lf_init();

//     const float base_speed = 0.30f;
//     float left_speed = 0.0f, right_speed = 0.0f;

//     // PID setup
//     PID pid;
//     // Kp, Ki, Kd
//     pid_init(&pid, 0.08f, 0.00f, 0.002f, -0.25f, 0.25f);
    
//     pid.setpoint = 1.0f; // want to always see black (1.0)

//     uint64_t last_seen_black_time = 0;
//     int last_turn_dir = 0; // -1 = left, +1 = right, 0 = straight
//     uint64_t now;

//     printf("Starting control loop...\n");

//     while (true) {
//         line_sample_t ls;
//         lf_read(&ls);
//         now = time_us_64();

//         // Convert sensor to analog-like input for PID
//         float sensor_val = ls.left_on_line ? 1.0f : 0.0f; // sees black = 1.0, white = 0.0
//         float error = pid.setpoint - sensor_val; // desired - actual
//         float correction = pid_update(&pid, error);
//          // runs the PID algorithm, which outputs a number (correction) that represents how strongly the 
//          // robot should steer to get back on the line.
//          // + -> robot should steer left (too far right)
//          // - -> robot should steer right (too far left)
//          // 0 -> On the center, go straight 

//         if (ls.left_on_line) { // check if the sensor sees the black line
//             // On black line → go straight, apply small PID bias
//             left_speed  = base_speed - correction;
//             right_speed = base_speed + correction;
//             last_seen_black_time = now;

//             // Stabilize last turn direction (±0.01 threshold to avoid noise)
//             if (correction > 0.01f)
//                 last_turn_dir = +1;   // drifting right → needs left correction
//             else if (correction < -0.01f)
//                 last_turn_dir = -1;   // drifting left → needs right correction
//         } 
//         else {
//             // --- Off the line ---
//             uint64_t lost_duration = now - last_seen_black_time;

//             if (lost_duration < 300000) {
//                 // Short-term drift → steer gently using last known direction
//                 if (last_turn_dir == -1) {
//                     // stronger left bias
//                     left_speed  = base_speed * 0.25f;
//                     right_speed = base_speed * 1.1f;
//                 } else {
//                     left_speed  = base_speed * 1.0f;
//                     right_speed = base_speed * 0.5f;
//                 }
//             } 
//             else {
//                 // Lost line
//                 uint64_t lost_duration = now - last_seen_black_time;

//                 if (lost_duration < 300000) {
//                     // short loss — keep bias
//                     if (last_turn_dir == -1)
//                         motor_set(0.2f, 0.45f); // turn left
//                     else
//                         motor_set(0.45f, 0.2f); // turn right
//                 } else {
//                     printf("Line lost — probing both sides...\n");
//                     bool found = false;
                    
//                     for (int i = 0; i < 15; i++) {
//                         motor_set(0.3f, -0.3f); // spin left
//                         sleep_ms(25);
//                         lf_read(&ls);
//                         if (ls.left_on_line) { found = true; break; }
//                     }
                    
//                     if (!found) {
//                         for (int i = 0; i < 25; i++) {
//                             motor_set(-0.3f, 0.3f); // spin right
//                             sleep_ms(25);
//                             lf_read(&ls);
//                             if (ls.left_on_line) { found = true; break; }
//                         }
//                     }
//                     // once found the line, stop and resume normal operation
//                     motor_set(0, 0);
//                     if (found) {
//                         printf("Reacquired black line!\n");
//                     } else {
//                         printf("Still lost — reversing briefly...\n");
//                         motor_set(-0.25f, -0.25f);
//                         sleep_ms(200);
//                     }
//                 }
//             }

//         }

//         // Apply computed motor speeds
//         motor_set(right_speed, left_speed);

//         printf("ADC=%4u | onLine=%d | err=%.2f | corr=%.3f | L=%.2f R=%.2f | dir=%d\n",
//                ls.adc_left, ls.left_on_line, error, correction,
//                left_speed, right_speed, last_turn_dir);

//         sleep_ms(60);
//     }
// }


// Barcode
#include "pico/stdlib.h"
#include <stdio.h>
#include "motor_encoder.h"
#include "barcode.h"

#define CONTROL_INTERVAL_MS 50
#define TARGET_PULSE_US 6000   // slower = better readings
#define KP 0.00008f

float left_pwm = 0.20f;
float right_pwm = 0.20f;

static inline float clamp(float v) {
    if (v < 0) return 0;
    if (v > 1) return 1;
    return v;
}

bool barcode_black_detected() {
    return gpio_get(3) == 1;  // 1 = black
}

int main() {
    stdio_init_all();
    sleep_ms(500);

    motors_and_encoders_init();
    barcode_init();

    absolute_time_t last_speed_update = get_absolute_time();

    while (1) {

        // --- MOTOR CONTROL ---
        motor_set(left_pwm, right_pwm);

        if (absolute_time_diff_us(last_speed_update, get_absolute_time()) 
            >= CONTROL_INTERVAL_MS * 1000) {

            last_speed_update = get_absolute_time();

            uint32_t L = encoder_pulse_width_us(1);
            uint32_t R = encoder_pulse_width_us(2);

            if (L > 0 && R > 0) {
                float errL = TARGET_PULSE_US - L;
                float errR = TARGET_PULSE_US - R;

                left_pwm  = clamp(left_pwm  + KP * errL);
                right_pwm = clamp(right_pwm + KP * errR);
            }
        }


        // --- BARCODE SCANNING (TRIGGERED ONLY) ---
        if (barcode_black_detected()) {

            printf("\n=== CHARACTER START ===\n");

            barcode_scan_t scan;
            barcode_scan(&scan);
            barcode_decode_full(&scan);

            printf("=== CHARACTER END ===\n");

            // Prevent detecting same character twice
            sleep_ms(200);
        }

        sleep_ms(5);
    }
}









