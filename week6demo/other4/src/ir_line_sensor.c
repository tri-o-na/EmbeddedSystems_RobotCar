#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

// --- 1. CONFIGURATION ---
#define ADC_PIN_A 26 // GPIO26 for ADC0 (Sensor A)
#define ADC_PIN_B 27 // GPIO27 for ADC1 (Sensor B)
#define IR_EMITTER_PIN 15 // Example GPIO pin for controlling IR LED (optional)
#define BAUD_RATE 115200

// ADC Thresholds (These should be tuned for your specific environment)
// Based on your log: Black (4000+) vs. White (100-200). 
// Threshold is set where readings *below* it mean "ON WHITE LINE".
#define THRESHOLD_A 500  
#define THRESHOLD_B 50 

// --- 2. GLOBAL VARIABLES ---
// Variables to hold the current time when the sensor was active/inactive
uint64_t last_active_micros_A = 0;
uint64_t last_active_micros_B = 0;


// --- 3. HELPER FUNCTION TO PRINT DATA ---
void print_sensor_data(const char* label, uint16_t adc, const char* do_state, uint64_t black_time) {
    printf("%s: ADC=%4d %s DO=%s black_time=%10llu us  ", 
           label, adc, (adc > THRESHOLD_A || adc > THRESHOLD_B) ? "BLACK" : "WHITE", do_state, black_time);
}


// --- 4. INITIALIZATION FUNCTION ---
void setup() {
    // Initialize USB serial communication
    stdio_init_all();
    sleep_ms(2000); // Wait for terminal to connect
    printf("---- Opened the serial port COM10 ----\n");

    // Initialize ADC hardware
    adc_init();

    // Configure ADC GPIO pins
    adc_gpio_init(ADC_PIN_A);
    adc_gpio_init(ADC_PIN_B);

    // Optional: Initialize IR Emitter Pin
    // gpio_init(IR_EMITTER_PIN);
    // gpio_set_dir(IR_EMITTER_PIN, GPIO_OUT);
    // gpio_put(IR_EMITTER_PIN, 1); // Turn IR emitter ON
}


// --- 5. MAIN LOOP ---
int main() {
    setup();

    while (1) {
        // --- 6. READ ADC VALUES ---
        adc_select_input(0); // Select ADC0 (Sensor A)
        uint16_t adc_A = adc_read();

        adc_select_input(1); // Select ADC1 (Sensor B)
        uint16_t adc_B = adc_read();

        // --- 7. DIGITAL STATE LOGIC (DO) ---
        // Logic: Low ADC (High Reflectance) = Sensor on WHITE LINE (Active)
        bool is_active_A = (adc_A < THRESHOLD_A); 
        bool is_active_B = (adc_B < THRESHOLD_B); 

        // Set the state string to be printed in the log (DO=BLACK means on the line)
        const char* do_state_A = is_active_A ? "BLACK" : "WHITE"; 
        const char* do_state_B = is_active_B ? "BLACK" : "WHITE"; 

        // --- 8. CALCULATE BARCODE TIME (Time spent on BLACK segment) ---
        uint64_t current_micros = time_us_64();
        
        // DECLARATION: Fixes the 'undeclared' error from your previous compile log
        uint64_t log_black_time_A; 
        uint64_t log_black_time_B;

        // Logic for Sensor A: Timer runs when NOT active (i.e., on the BLACK segment)
        if (!is_active_A) { // <-- Timer starts/runs when over Black Floor (Barcode Segment)
            if (last_active_micros_A == 0) {
                last_active_micros_A = current_micros;
                log_black_time_A = 0; 
            } else {
                log_black_time_A = current_micros - last_active_micros_A;
            }
        } else { // Timer resets when sensor is over the White Line
            last_active_micros_A = 0; 
            log_black_time_A = 0; 
        }

        // Logic for Sensor B: Timer runs when NOT active (i.e., on the BLACK segment)
        if (!is_active_B) { // <-- Timer starts/runs when over Black Floor (Barcode Segment)
            if (last_active_micros_B == 0) {
                last_active_micros_B = current_micros; 
                log_black_time_B = 0;
            } else {
                log_black_time_B = current_micros - last_active_micros_B;
            }
        } else { // Timer resets when sensor is over the White Line
            last_active_micros_B = 0; 
            log_black_time_B = 0;
        }

        // --- 9. PRINT RESULTS ---
        print_sensor_data("A", adc_A, do_state_A, log_black_time_A);
        printf("  |  ");
        print_sensor_data("B", adc_B, do_state_B, log_black_time_B);
        printf("\n");

        sleep_ms(50); // Control the print speed
    }

    return 0; // Should not be reached
}