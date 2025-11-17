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


#include <stdio.h>
#include "pico/stdlib.h"

#include "line_follow.h"
#include "motor_encoder.h"
#include "pid.h"
#include "barcode.h"
#include <math.h>

// ====== PID ======
PID pid;

// ---- TUNE THESE IF NEEDED ----
// Based on your comment: white ~200, black ~4000
#define HYST_LOW   800      // must be BELOW this = white
#define HYST_HIGH  1400     // must be ABOVE this = black

#define ADC_WHITE_EST   150.0f
#define ADC_BLACK_EST  3500.0f

// ========= SPEEDS =========
#define BASE_SPEED      0.35f
#define BOOST_SPEED     0.28f
#define TURN_GAIN       0.50f

#define REVERSE_SPEED  -0.20f
#define TURN_CORRECT   0.30f


#define FWD_SPEED   0.30f
#define REV_SPEED  -0.25f

float HYSTERESIS_LOW  = 800;   // if lower → definitely white
float HYSTERESIS_HIGH = 1400;  // if higher → definitely black

// ******** MISSING FUNCTION FIX ********
// add clampf to your main.c
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// void follow_line(void)
// {
//     line_sample_t s;
//     lf_read(&s);

//     float raw_adc = (float)s.adc_left;

//     // ----- Smooth the ADC value -----
//     static float adc_filtered = 0.0f;
//     adc_filtered = (adc_filtered * 0.85f) + (raw_adc * 0.15f);
//     float adc = adc_filtered;

//     // Normalize ADC to 0..1 range
//     float norm = (adc - ADC_WHITE_EST) / (ADC_BLACK_EST - ADC_WHITE_EST);
//     norm = clampf(norm, 0.0f, 1.0f);

//     // Speed boost on black
//     float speed = (norm > 0.7f) ? BOOST_SPEED : BASE_SPEED;

//     // Steering
//     float error = norm - 0.5f;
//     float turn  = 0.35f * error;

//     float left  = speed + turn;
//     float right = speed - turn;

//     // ------- FIX A: Minimum drive to stop LEFT stall -------
//     float MIN_DRIVE = 0.18f;

//     if (left > 0 && left < MIN_DRIVE) left = MIN_DRIVE;
//     if (right > 0 && right < MIN_DRIVE) right = MIN_DRIVE;

//     if (left < 0 && left > -MIN_DRIVE) left = -MIN_DRIVE;
//     if (right < 0 && right > -MIN_DRIVE) right = -MIN_DRIVE;
//     // --------------------------------------------------------

//     // ------- FIX B: Slight boost for left-turn control -------
//     // If turning left (error is negative), give right motor 5% boost
//     if (error < -0.05f) {
//         right *= 1.05f;
//     }
//     // ----------------------------------------------------------

//     left  = clampf(left,  -1.0f, 1.0f);
//     right = clampf(right, -1.0f, 1.0f);

//     motor_set(left, right);
// }

void follow_line(void)
{
    line_sample_t s;
    lf_read(&s);

    float adc = (float)s.adc_left;

    // Smooth
    static float filtered = 0;
    filtered = filtered * 0.80f + adc * 0.20f;

    bool on_black = (filtered > 1100);
    bool on_white = !on_black;

    //-----------------------------------------
    // LOST LINE DETECTION
    //-----------------------------------------
    static int white_count = 0;
    static bool reversing = false;
    static absolute_time_t reverse_end;

    if (on_white)
        white_count++;
    else
        white_count = 0;

    //-----------------------------------------
    // Reverse ONLY when extremely lost
    //-----------------------------------------
    if (!reversing && white_count > 80)      // ~400ms lost
    {
        reversing = true;
        reverse_end = make_timeout_time_ms(350);
    }

    //-----------------------------------------
    // REVERSE MODE
    //-----------------------------------------
    if (reversing)
    {
        if (time_reached(reverse_end))
            reversing = false;
        else
        {
            motor_set(-0.22f, -0.22f);       // smooth reverse
            return;
        }
    }

    //-----------------------------------------
    // NORMAL LINE FOLLOWING
    //-----------------------------------------

    float FWD  = 0.28f;   // your forward speed
    float TURN = 0.30f;   // strong turning force

    float left, right;

    if (on_black)
    {
        // BLACK = turn RIGHT
        left  = FWD - TURN;
        right = FWD + TURN;
    }
    else
    {
        // WHITE = turn LEFT
        left  = FWD + TURN;
        right = FWD - TURN;
    }

    //-----------------------------------------
    // MINIMUM DRIVE FIX — prevents stalling
    //-----------------------------------------
    const float MIN = 0.17f;  // motor dead-zone

    if (left > 0 && left < MIN) left = MIN;
    if (right > 0 && right < MIN) right = MIN;

    motor_set(left, right);
}









int main()
{
    stdio_init_all();
    lf_init();
    motors_and_encoders_init();
    barcode_init_nonblocking();

    

    while (1)
    {
        barcode_update();  // non-blocking barcode scanner  
        follow_line();     // smooth line following
        sleep_ms(5);


    }
}
