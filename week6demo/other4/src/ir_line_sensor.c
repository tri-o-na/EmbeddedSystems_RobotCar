#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include "hardware/gpio.h"

// ================== Pin map ==================
// Sensor A
#define A_ADC_INPUT  0      // ADC0 -> GP26
#define A_PIN_DO     27     // DO on GP27

// Sensor B
#define B_PIN_DO     3      // DO on GP3
#define B_USE_ADC    1      // we ARE using analog on B
#define B_ADC_INPUT  2      // ADC2 -> GP28

// Print period (ms)
#define LOOP_MS      50

// ============== Globals for BLACK duration tracking (FLIPPED from WHITE) ==============
// Track when we entered BLACK and current duration
static volatile uint32_t a_black_start_us = 0;  // When we entered BLACK
static volatile uint32_t b_black_start_us = 0;
static volatile bool a_currently_black = false; // Current state
static volatile bool b_currently_black = false;

// ============== IRQ handler for both DO lines (active-low) ==============
static void do_irq(uint gpio, uint32_t events) {
    uint32_t now = to_us_since_boot(get_absolute_time());

    if (gpio == A_PIN_DO) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            // Went LOW (BLACK detected) - START timing
            if (!a_currently_black) {
                a_black_start_us = now;
                a_currently_black = true;
            }
        } else if (events & GPIO_IRQ_EDGE_RISE) {
            // Went HIGH (WHITE detected) - STOP timing
            a_currently_black = false;
            a_black_start_us = 0;
        }
    } else if (gpio == B_PIN_DO) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            // Went LOW (BLACK detected) - START timing
            if (!b_currently_black) {
                b_black_start_us = now;
                b_currently_black = true;
            }
        } else if (events & GPIO_IRQ_EDGE_RISE) {
            // Went HIGH (WHITE detected) - STOP timing
            b_currently_black = false;
            b_black_start_us = 0;
        }
    }
}

// Function to get current BLACK duration
uint32_t get_current_black_duration_us(bool currently_black, uint32_t start_time) {
    if (!currently_black || start_time == 0) {
        return 0;  // Not on black or no valid start time
    }
    return to_us_since_boot(get_absolute_time()) - start_time;
}

int main() {
    stdio_init_all();
    sleep_ms(300);

    printf("\nIR line sensor demo — TWO sensors (Continuous BLACK duration tracking)\n");

    // -------- ADC init --------
    adc_init();
    adc_gpio_init(26 + A_ADC_INPUT);   // A analog (GP26/27/28)
#if B_USE_ADC
    adc_gpio_init(26 + B_ADC_INPUT);   // B analog (GP26/27/28)
#endif

    // -------- Digital pins init (DO) --------
    gpio_init(A_PIN_DO);
    gpio_set_dir(A_PIN_DO, GPIO_IN);
    gpio_pull_up(A_PIN_DO);            // open-collector DO (idle HIGH, active LOW)

    gpio_init(B_PIN_DO);
    gpio_set_dir(B_PIN_DO, GPIO_IN);
    gpio_pull_up(B_PIN_DO);

    // IRQs for both edges on both DOs
    gpio_set_irq_enabled_with_callback(A_PIN_DO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &do_irq);
    gpio_set_irq_enabled(B_PIN_DO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // -------- Baseline/thresholds from ambient (FLIPPED LOGIC) --------
    // Sensor A
    adc_select_input(A_ADC_INPUT);
    uint32_t sumA = 0;
    for (int i = 0; i < 50; i++) { sumA += adc_read(); sleep_ms(5); }
    uint16_t avgA = sumA / 50;
    int16_t threshA = (int16_t)avgA - 200;  // FLIPPED: below ambient => WHITE
    if (threshA < 0) threshA = 0;

    // Sensor B (since we enabled analog)
#if B_USE_ADC
    adc_select_input(B_ADC_INPUT);
    uint32_t sumB = 0;
    for (int i = 0; i < 50; i++) { sumB += adc_read(); sleep_ms(5); }
    uint16_t avgB = sumB / 50;
    int16_t threshB = (int16_t)avgB - 200;  // FLIPPED: below ambient => WHITE
    if (threshB < 0) threshB = 0;
#endif

    printf("A: ADC on GP%u, DO on GP%u | ambient=%u, thresh_white=%d\n",
           26 + A_ADC_INPUT, A_PIN_DO, avgA, threshA);
#if B_USE_ADC
    printf("B: ADC on GP%u, DO on GP%u | ambient=%u, thresh_white=%d\n",
           26 + B_ADC_INPUT, B_PIN_DO, avgB, threshB);
#else
    printf("B: DO on GP%u only (no analog)\n", B_PIN_DO);
#endif

    printf("\nContinuous BLACK duration tracking (updates while on black):\n");
    
    // **FIX: Initialize state based on current pin level at boot**
    // 0 = Active/BLACK, 1 = Idle/WHITE
    if (gpio_get(A_PIN_DO) == 0) { 
        a_black_start_us = to_us_since_boot(get_absolute_time());
        a_currently_black = true;
    }
    if (gpio_get(B_PIN_DO) == 0) { 
        b_black_start_us = to_us_since_boot(get_absolute_time());
        b_currently_black = true;
    }

    while (true) {
        // -------- Sensor A (ADC + DO + continuous BLACK duration) --------
        adc_select_input(A_ADC_INPUT);
        uint16_t a_adc = adc_read();                // 0..4095
        bool a_white_adc = ((int)a_adc < threshA);  // FLIPPED: below threshold => WHITE
        int a_do_raw = gpio_get(A_PIN_DO);          // 0 = active (BLACK), 1 = idle (WHITE)
        bool a_black_do = (a_do_raw == 0);          // 0 = BLACK, 1 = WHITE
        
        // Get current BLACK duration (continuously updating while on black)
        uint32_t a_black_duration = get_current_black_duration_us(a_currently_black, a_black_start_us);

        // -------- Sensor B (ADC + DO + continuous BLACK duration) --------
#if B_USE_ADC
        adc_select_input(B_ADC_INPUT);
        uint16_t b_adc = adc_read();
        bool b_white_adc = ((int)b_adc < threshB);  // FLIPPED: below threshold => WHITE
#else
        uint16_t b_adc = 0;
        bool b_white_adc = false;
#endif
        int b_do_raw = gpio_get(B_PIN_DO);
        bool b_black_do = (b_do_raw == 0);          // 0 = BLACK, 1 = WHITE
        
        // Get current BLACK duration (continuously updating while on black)
        uint32_t b_black_duration = get_current_black_duration_us(b_currently_black, b_black_start_us);

        // -------- Print status with continuously updating BLACK time --------
#if B_USE_ADC
        printf("A: ADC=%4u %-5s  DO=%s  black_time=%6lu us   |   B: ADC=%4u %-5s  DO=%s  black_time=%6lu us\n",
               a_adc, a_white_adc ? "WHITE" : "BLACK",
               a_black_do ? "BLACK" : "WHITE", (unsigned long)a_black_duration,
               b_adc, b_white_adc ? "WHITE" : "BLACK",
               b_black_do ? "BLACK" : "WHITE", (unsigned long)b_black_duration);
#else
        printf("A: ADC=%4u %-5s  DO=%s  black_time=%6lu us   |   B: DO=%s  black_time=%6lu us\n",
               a_adc, a_white_adc ? "WHITE" : "BLACK",
               a_black_do ? "BLACK" : "WHITE", (unsigned long)a_black_duration,
               b_black_do ? "BLACK" : "WHITE", (unsigned long)b_black_duration);
#endif

        sleep_ms(LOOP_MS);
    }
}