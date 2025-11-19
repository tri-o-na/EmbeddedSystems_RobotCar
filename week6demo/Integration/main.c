// Line Following with Single IR Sensor + PID + Smart Recovery
#include "pico/stdlib.h"
// IMU logic removed
#include "motor_encoder.h"
#include "pid.h"
#include <stdio.h>
#include <math.h>
#include <line_follow.h>
#include <barcode.h>

// Helper function to clamp a float value
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ---- Motor Calibration Bias ----
// User confirmed 0.15f is the straightest bias.
#define CALIBRATION_BIAS 0.18f 

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("=== Combined: Final PID Follow + Barcode Scan (Slowed) + User-Optimized Bias ===\n");

    // imu_init(); // IMU removed
    motors_and_encoders_init();
    lf_init();      // Initializes Right Sensor (GPIO26)
    barcode_init(); // Initializes Left Sensor (GPIO2)

    // Base speed maintained at 0.20f for reliable barcode detection
    const float base_speed = 0.15f; 
    float left_speed = 0.0f, right_speed = 0.0f;

    // --- Line Following PID setup ---
    PID linePID;
    // 🔥 MODIFIED: Kp reduced to 0.06f to reduce overshoot/oscillation. Kd increased to 0.005f for better damping.
    pid_init(&linePID, 0.06f, 0.00f, 0.005f, -0.25f, 0.25f);
    linePID.setpoint = 1.0f; // want to always see black (1.0)
    
    uint64_t last_seen_black_time = 0;
    int last_turn_dir = 0; // -1 = left, +1 = right, 0 = straight
    uint64_t now;

    printf("Starting control loop...\n");

    while (true) {
        line_sample_t ls;
        lf_read(&ls);
        now = time_us_64();

        // -------------------------------------------------------------------
        // 1. PID LINE FOLLOWING (RIGHT SENSOR - GPIO26)
        // -------------------------------------------------------------------
        float sensor_val = ls.right_on_line ? 1.0f : 0.0f; 
        float correction = pid_update(&linePID, linePID.setpoint - sensor_val);
        
        float min_drive = 0.20f; 

        if (ls.right_on_line) { // check if the RIGHT sensor sees the black line
            
            // FIX: Apply Deadband to correction when on the line (error=0.0f)
            // This prevents random turns caused by noisy D-term output.
            if (fabsf(correction) < 0.02f) { 
                correction = 0.0f;
            }
            
            // On black line → Go straight with dampened PID correction
            // PID Polarity is correct for Right Sensor: +correction -> Turn Left (R > L)
            // Bias is added to Left side to fight persistent Left drift.
            left_speed  = base_speed - correction + CALIBRATION_BIAS; 
            right_speed = base_speed + correction;
            
            last_seen_black_time = now;

            // Update last turn direction based on correction
            if (correction > 0.01f)
                last_turn_dir = -1;   // drifting right -> corrected with left turn (-1)
            else if (correction < -0.01f)
                last_turn_dir = +1;   // drifting left -> corrected with right turn (+1)
            else
                last_turn_dir = 0;    // Driving straight (due to deadband)
        } 
        else {
            // --- Off the line (Recovery) ---
            uint64_t lost_duration = now - last_seen_black_time;

            if (lost_duration < 300000) {
                // Short-term drift: Explicit aggressive turn based on last direction
                // Speeds from user's requested logic (0.45f fast, 0.20f slow)
                if (last_turn_dir == -1) {
                    // Turn LEFT (Right motor faster: 0.45f)
                    left_speed  = 0.20f; 
                    right_speed = 0.45f; 
                } else {
                    // Turn RIGHT (Left motor faster: 0.45f)
                    left_speed  = 0.45f; 
                    right_speed = 0.20f; 
                }
            } 
            else {
                // Long-term loss: Blocking search maneuvers
                
                printf("Line lost — probing both sides...\n");
                bool found = false;
                
                // Spin LEFT search (R fwd, L bwd)
                // Using user's requested spin speed (0.3f) and count (15)
                for (int i = 0; i < 15; i++) {
                    motor_set(0.30f, -0.30f); // spin left (R fwd, L bwd)
                    sleep_ms(25);
                    lf_read(&ls);
                    // NOTE: Switched from user's provided ls.left_on_line to ls.right_on_line for consistency with PID logic.
                    if (ls.right_on_line) { found = true; break; } 
                }
                
                if (!found) {
                    // Spin RIGHT search (R bwd, L fwd)
                    // Using user's requested spin speed (0.3f) and count (25)
                    for (int i = 0; i < 25; i++) {
                        motor_set(-0.30f, 0.30f); // spin right (R bwd, L fwd)
                        sleep_ms(25);
                        lf_read(&ls);
                        // NOTE: Switched from user's provided ls.left_on_line to ls.right_on_line for consistency with PID logic.
                        if (ls.right_on_line) { found = true; break; }
                    }
                }
                
                // Final recovery action
                motor_set(0, 0);
                if (found) {
                    printf("Reacquired black line! Resuming PID.\n");
                } else {
                    printf("Still lost — reversing briefly...\n");
                    // Reverse (R bwd, L bwd)
                    // Using user's requested reverse speed and duration.
                    motor_set(-0.25f, -0.25f); 
                    sleep_ms(200); 
                }
                
                // Clear speeds as motor_set was handled in the blocking loop above
                left_speed = 0.0f;
                right_speed = 0.0f;
            }
        }
        
        // -------------------------------------------------------------------
        // 2. APPLY MOTOR SPEEDS
        // -------------------------------------------------------------------
        if (left_speed != 0.0f || right_speed != 0.0f) {
            // Apply minimum drive fix
            if (left_speed > 0 && left_speed < min_drive) left_speed = min_drive;
            if (right_speed > 0 && right_speed < min_drive) right_speed = min_drive;
            
            // Clamp speeds
            left_speed  = clampf(left_speed,  -1.0f, 1.0f);
            right_speed = clampf(right_speed, -1.0f, 1.0f);
            
            // motor_set(right_speed, left_speed_to_send)
            motor_set(right_speed, left_speed);
        }

        // -------------------------------------------------------------------
        // 3. NON-BLOCKING BARCODE SCAN (LEFT SENSOR)
        // -------------------------------------------------------------------
        barcode_nonblocking_update(); 

        sleep_ms(60); // Loop delay
    }
}