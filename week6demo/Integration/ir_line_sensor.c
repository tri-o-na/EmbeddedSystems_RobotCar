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

// ============== Globals for LOW pulse width (active-low) ==============
// Measure LOW-time: timestamp FALL, compute on RISE
static volatile uint32_t a_last_fall_us = 0, b_last_fall_us = 0;
static volatile uint32_t a_low_pw_us = 0,   b_low_pw_us = 0;

// ============== IRQ handler for both DO lines (active-low) ==============
static void do_irq(uint gpio, uint32_t events) {
    uint32_t now = to_us_since_boot(get_absolute_time());

    if (gpio == A_PIN_DO) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            a_last_fall_us = now;                  // went LOW (BLACK active)
        } else if ((events & GPIO_IRQ_EDGE_RISE) && a_last_fall_us) {
            a_low_pw_us = now - a_last_fall_us;    // LOW pulse finished
        }
    } else if (gpio == B_PIN_DO) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            b_last_fall_us = now;
        } else if ((events & GPIO_IRQ_EDGE_RISE) && b_last_fall_us) {
            b_low_pw_us = now - b_last_fall_us;
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(300);

    printf("\nIR line sensor demo — TWO sensors (ADC contrast + DO LOW pulse width)\n");

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

    // IRQs for LOW pulse width on both DOs (need both edges)
    gpio_set_irq_enabled_with_callback(A_PIN_DO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &do_irq);
    gpio_set_irq_enabled(B_PIN_DO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // -------- Baseline/thresholds from ambient --------
    // We’ll classify BLACK when ADC is significantly ABOVE ambient average.
    // (Many breakout circuits produce higher ADC codes on dark/black.)
    // If your wiring yields the inverse, flip the comparison below.
    // Sensor A
    adc_select_input(A_ADC_INPUT);
    uint32_t sumA = 0;
    for (int i = 0; i < 50; i++) { sumA += adc_read(); sleep_ms(5); }
    uint16_t avgA = sumA / 50;
    int16_t threshA = (int16_t)avgA + 200;  // above ambient => BLACK
    if (threshA > 4095) threshA = 4095;

    // Sensor B (since we enabled analog)
#if B_USE_ADC
    adc_select_input(B_ADC_INPUT);
    uint32_t sumB = 0;
    for (int i = 0; i < 50; i++) { sumB += adc_read(); sleep_ms(5); }
    uint16_t avgB = sumB / 50;
    int16_t threshB = (int16_t)avgB + 200;  // above ambient => BLACK
    if (threshB > 4095) threshB = 4095;
#endif

    printf("A: ADC on GP%u, DO on GP%u | ambient=%u, thresh_black=%d\n",
           26 + A_ADC_INPUT, A_PIN_DO, avgA, threshA);
#if B_USE_ADC
    printf("B: ADC on GP%u, DO on GP%u | ambient=%u, thresh_black=%d\n",
           26 + B_ADC_INPUT, B_PIN_DO, avgB, threshB);
#else
    printf("B: DO on GP%u only (no analog)\n", B_PIN_DO);
#endif

    while (true) {
        // -------- Sensor A (ADC + DO + LOW pulse width) --------
        adc_select_input(A_ADC_INPUT);
        uint16_t a_adc = adc_read();                // 0..4095
        bool a_black_adc = ((int)a_adc > threshA);  // above threshold => BLACK
        int a_do_raw = gpio_get(A_PIN_DO);          // 0 = active (BLACK), 1 = idle (WHITE)
        bool a_black_do = (a_do_raw == 1);
        uint32_t a_pw = a_low_pw_us;                // LOW-time in microseconds

        // -------- Sensor B (ADC + DO + LOW pulse width) --------
#if B_USE_ADC
        adc_select_input(B_ADC_INPUT);
        uint16_t b_adc = adc_read();
        bool b_black_adc = ((int)b_adc > threshB);
#else
        uint16_t b_adc = 0;
        bool b_black_adc = false;
#endif
        int b_do_raw = gpio_get(B_PIN_DO);
        bool b_black_do = (b_do_raw == 1);
        uint32_t b_pw = b_low_pw_us;

        // -------- Print a compact, human-friendly status line --------
#if B_USE_ADC
        printf("A: ADC=%4u %-5s  DO=%s  low_pw=%6lu us   |   B: ADC=%4u %-5s  DO=%s  low_pw=%6lu us\n",
               a_adc, a_black_adc ? "BLACK" : "WHITE",
               a_black_do ? "BLACK" : "WHITE", (unsigned long)a_pw,
               b_adc, b_black_adc ? "BLACK" : "WHITE",
               b_black_do ? "BLACK" : "WHITE", (unsigned long)b_pw);
#else
        printf("A: ADC=%4u %-5s  DO=%s  low_pw=%6lu us   |   B: DO=%s  low_pw=%6lu us\n",
               a_adc, a_black_adc ? "BLACK" : "WHITE",
               a_black_do ? "BLACK" : "WHITE", (unsigned long)a_pw,
               b_black_do ? "BLACK" : "WHITE", (unsigned long)b_pw);
#endif

        sleep_ms(LOOP_MS);
    }
}
