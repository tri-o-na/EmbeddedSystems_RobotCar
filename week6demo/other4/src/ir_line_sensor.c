#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include "hardware/gpio.h"

// ================== Pin map ==================
// Sensor A (your first sensor)
#define A_ADC_INPUT     0      // 0->GP26, 1->GP27, 2->GP28
#define A_PIN_DO        27     // DO on GP27

// Sensor B (your second sensor)
// NOTE: Its AO is on GP2 (not an ADC pin). We read only DO on GP3.
// If you later move its AO to GP28, set B_USE_ADC = 1 and B_ADC_INPUT = 2.
#define B_PIN_DO        3      // DO on GP3
#define B_USE_ADC       0      // 0 = no analog for sensor B (AO on GP2 can't be used)
#define B_ADC_INPUT     2      // if B_USE_ADC==1, 2->GP28

// Print period (ms)
#define LOOP_MS         50

// ============== Globals for pulse width ==============
static volatile uint32_t a_last_rise_us = 0, b_last_rise_us = 0;
static volatile uint32_t a_pulse_us = 0,   b_pulse_us = 0;

// ============== IRQ handler for both DO lines ==============
static void do_irq(uint gpio, uint32_t events) {
    uint32_t now = to_us_since_boot(get_absolute_time());
    if (gpio == A_PIN_DO) {
        if (events & GPIO_IRQ_EDGE_RISE)      a_last_rise_us = now;
        else if ((events & GPIO_IRQ_EDGE_FALL) && a_last_rise_us) a_pulse_us = now - a_last_rise_us;
    } else if (gpio == B_PIN_DO) {
        if (events & GPIO_IRQ_EDGE_RISE)      b_last_rise_us = now;
        else if ((events & GPIO_IRQ_EDGE_FALL) && b_last_rise_us) b_pulse_us = now - b_last_rise_us;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(300);

    printf("\nIR line sensor demo — dual sensors\n");

    // -------- ADC init --------
    adc_init();
    // Sensor A analog (GP26/27/28 depending on A_ADC_INPUT)
    adc_gpio_init(26 + A_ADC_INPUT);
    // Sensor B analog (optional, only if rewired to ADC pin and B_USE_ADC=1)
#if B_USE_ADC
    adc_gpio_init(26 + B_ADC_INPUT);
#endif
    // We'll switch adc_select_input() before each read below.

    // -------- Digital pins init (DO) --------
    gpio_init(A_PIN_DO);
    gpio_set_dir(A_PIN_DO, GPIO_IN);
    gpio_pull_up(A_PIN_DO);  // most DO pins are open-collector active-low

    gpio_init(B_PIN_DO);
    gpio_set_dir(B_PIN_DO, GPIO_IN);
    gpio_pull_up(B_PIN_DO);

    // IRQs for pulse width on both DOs
    gpio_set_irq_enabled_with_callback(A_PIN_DO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &do_irq);
    gpio_set_irq_enabled(B_PIN_DO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // -------- Quick baseline for Sensor A threshold --------
    adc_select_input(A_ADC_INPUT);
    uint32_t sumA = 0;
    for (int i = 0; i < 50; i++) { sumA += adc_read(); sleep_ms(5); }
    uint16_t avgA = sumA / 50;
    int16_t threshA = (int16_t)avgA - 200;    // tune in the field
    if (threshA < 0) threshA = 0;

#if B_USE_ADC
    // Optional baseline for Sensor B if its AO is rewired to an ADC pin
    adc_select_input(B_ADC_INPUT);
    uint32_t sumB = 0;
    for (int i = 0; i < 50; i++) { sumB += adc_read(); sleep_ms(5); }
    uint16_t avgB = sumB / 50;
    int16_t threshB = (int16_t)avgB - 200;
    if (threshB < 0) threshB = 0;
#endif

    printf("Sensor A: ADC on GP%u, DO on GP%u | avg=%u, thresh=%d\n",
           26 + A_ADC_INPUT, A_PIN_DO, avgA, threshA);
#if B_USE_ADC
    printf("Sensor B: ADC on GP%u, DO on GP%u | avg=%u, thresh=%d\n",
           26 + B_ADC_INPUT, B_PIN_DO, avgB, threshB);
#else
    printf("Sensor B: DO on GP%u (no analog; AO currently on GP2 which is not ADC)\n", B_PIN_DO);
#endif

    while (true) {
        // -------- Sensor A reads --------
        adc_select_input(A_ADC_INPUT);
        uint16_t a_adc = adc_read();                 // 0..4095
        bool a_black = ((int)a_adc < threshA);
        int a_do = gpio_get(A_PIN_DO);

        // -------- Sensor B reads --------
#if B_USE_ADC
        adc_select_input(B_ADC_INPUT);
        uint16_t b_adc = adc_read();
        bool b_black = ((int)b_adc < threshB);
#else
        uint16_t b_adc = 0;       // not used
        bool b_black = (gpio_get(B_PIN_DO) == 0); // assume active-low DO: 0=BLACK
#endif
        int b_do = gpio_get(B_PIN_DO);

        // -------- Print status --------
        // A: show ADC+label+DO+pulse
        printf("A: ADC=%u %s  DO=%d pulse_us=%lu   ",
               a_adc, a_black ? "BLACK" : "WHITE", a_do, (unsigned long)a_pulse_us);

        // B: show either ADC+label or DO-only depending on wiring
#if B_USE_ADC
        printf("B: ADC=%u %s  DO=%d pulse_us=%lu\n",
               b_adc, b_black ? "BLACK" : "WHITE", b_do, (unsigned long)b_pulse_us);
#else
        printf("B: DO=%d %s  pulse_us=%lu\n",
               b_do, (b_do == 0 ? "BLACK" : "WHITE"), (unsigned long)b_pulse_us);
#endif

        sleep_ms(LOOP_MS);
    }
}
