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
// Bias maintained at 0.05f to match the slow base speed.
#define CALIBRATION_BIAS 0.05f 

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("=== Combined: Final PID Follow + Barcode Scan (Slowed) + Rebalanced Bias ===\n");

    // imu_init(); // IMU removed
    motors_and_encoders_init();
    lf_init();      // Initializes Right Sensor (GPIO26)
    barcode_init(); // Initializes Left Sensor (GPIO2)

    // 🔥 MODIFIED: Base speed increased to 0.18f (was 0.15f) for reliable motor starting.
    const float base_speed = 0.18f; 
    float left_speed = 0.0f, right_speed = 0.0f;

    // --- Line Following PID setup ---
    PID linePID;
    // PID tuning remains the same: Kp=0.05f, Kd=0.003f, Limits=±0.15f
    pid_init(&linePID, 0.05f, 0.00f, 0.003f, -0.15f, 0.15f);
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
        
        // 🔥 MODIFIED: min_drive increased to 0.15f (was 0.10f) to ensure motors start reliably.
        float min_drive = 0.15f; 

        if (ls.right_on_line) { // check if the RIGHT sensor sees the black line
            
            // FIX: Apply Deadband to correction when on the line (error=0.0f)
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
            // --- Off the line (Smart Recovery) ---
            uint64_t lost_duration = now - last_seen_black_time;

            if (lost_duration < 300000) {
                // Short-term drift
                // 🔥 MODIFIED: Adjusted recovery multipliers for new base_speed (0.18f)
                if (last_turn_dir == -1) {
                    // Turn LEFT (stronger right bias + calibration)
                    left_speed  = base_speed * 0.5f + CALIBRATION_BIAS; // L_slow
                    right_speed = base_speed * 1.5f; // R_fast
                } else {
                    // Turn RIGHT (stronger left bias + calibration)
                    left_speed  = base_speed * 1.5f + CALIBRATION_BIAS; // L_fast
                    right_speed = base_speed * 0.5f; // R_slow
                }
            } 
            else {
                // Long-term loss: Engaging blocking search maneuvers
                
                printf("Line lost — probing both sides...\n");
                bool found = false;
                
                // Spin LEFT search (R fwd, L bwd)
                for (int i = 0; i < 15; i++) {
                    motor_set(0.20f, -0.20f); 
                    sleep_ms(25);
                    lf_read(&ls);
                    if (ls.right_on_line) { found = true; break; }
                }
                
                if (!found) {
                    // Spin RIGHT search (R bwd, L fwd)
                    for (int i = 0; i < 25; i++) {
                        motor_set(-0.20f, 0.20f); 
                        sleep_ms(25);
                        lf_read(&ls);
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
                    motor_set(-0.18f, -0.18f); // Use base_speed for gentle reverse
                    sleep_ms(150);
                }
                
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