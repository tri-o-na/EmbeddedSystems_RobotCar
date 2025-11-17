#include "pico/stdlib.h"
#include <stdio.h>
#include <stdint.h>
#include "motor_encoder.h"
#include "pid.h"
#include "odometry.h"

// ========================
// Configuration
// ========================
#define CONTROL_LOOP_MS 50      // 50ms loop interval (20Hz control frequency)
#define DT (float)CONTROL_LOOP_MS / 1000.0f

// Target speeds in encoder ticks per second (tps)
// M1 (Left) is faster, so we give it a LOWER setpoint to slow it down.
// M2 (Right) is slower, so we give it a HIGHER setpoint.
#define BASE_SPEED_TPS   20.0f   
#define M1_SPEED_OFFSET  -0.0f   // Example: M1 target is 20 - 2 = 18.0 tps
#define M2_SPEED_OFFSET  +0.0f   // Example: M2 target is 20 + 0 = 20.0 tps

#define TARGET_SPEED_M1 (BASE_SPEED_TPS + M1_SPEED_OFFSET)
#define TARGET_SPEED_M2 (BASE_SPEED_TPS + M2_SPEED_OFFSET)

// PID Constants (Use the stabilized, low KI values)
#define KP 0.02f
#define KI 0.0001f  
#define KD 0.00f


// ========================
// Global Variables
// ========================
PID left_pid;
PID right_pid;
Pose2D pose; 
static uint32_t last_left_ticks = 0;
static uint32_t last_right_ticks = 0;


void setup() {
    stdio_init_all();
    
    // Initialize hardware
    motors_and_encoders_init();
    odom_init(&pose);
    
    // 1. Initialize Left Motor PID (M1)
    pid_init(&left_pid, KP, KI, KD, -1.0f, 1.0f);
    left_pid.setpoint = TARGET_SPEED_M1; // M1 setpoint

    // 2. Initialize Right Motor PID (M2)
    pid_init(&right_pid, KP, KI, KD, -1.0f, 1.0f);
    right_pid.setpoint = TARGET_SPEED_M2; // M2 setpoint
    
    // Get initial encoder counts to properly start tracking changes
    encoder_get_counts(&last_left_ticks, &last_right_ticks);
    printf("Setup complete. Driving straight with M1=%.1f tps and M2=%.1f tps.\n", 
           TARGET_SPEED_M1, TARGET_SPEED_M2);
}


void loop() {
    uint32_t current_left_ticks, current_right_ticks;
    
    // --- 1. Read Encoders and Calculate Speed ---
    encoder_get_counts(&current_left_ticks, &current_right_ticks);

    // Calculate change in ticks since last loop
    int32_t dleft_ticks  = (int32_t)(current_left_ticks - last_left_ticks);
    int32_t dright_ticks = (int32_t)(current_right_ticks - last_right_ticks);

    last_left_ticks = current_left_ticks;
    last_right_ticks = current_right_ticks;

    // Calculate instantaneous speed (ticks/second)
    float left_speed_tps  = (float)dleft_ticks / DT;
    float right_speed_tps = (float)dright_ticks / DT;
    
    
    // --- 2. Update PID Controllers ---
    float left_output  = pid_update(&left_pid, left_speed_tps);
    float right_output = pid_update(&right_pid, right_speed_tps);


    // --- 3. Set Motor Power ---
    motor_set(left_output, right_output);
    
    
    // --- 4. Odometry Update (Optional) ---
    odom_update(&pose, dleft_ticks, dright_ticks);


    // Debugging print (optional)
    printf("L(M1): SP=%.1f, Meas=%.1f, Out=%.2f | R(M2): SP=%.1f, Meas=%.1f, Out=%.2f\n", 
           left_pid.setpoint, left_speed_tps, left_output,
           right_pid.setpoint, right_speed_tps, right_output);

    // Sleep for the remainder of the loop time
    sleep_ms(CONTROL_LOOP_MS);
}


int main() {
    setup();
    while (1) {
        loop();
    }
    return 0;
}