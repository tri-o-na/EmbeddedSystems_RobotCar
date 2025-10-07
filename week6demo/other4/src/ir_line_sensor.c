#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/time.h"

// --- Pins (adjust) ---
#define ADC_INPUT   0     // 0->GP26, 1->GP27, 2->GP28
#define PIN_DIGITAL 15    // optional digital output from sensor (to measure pulse width)

// Globals
static volatile uint32_t last_rise_us = 0;
static volatile uint32_t pulse_us = 0;

static void digital_irq(uint gpio, uint32_t events) {
    uint32_t now = to_us_since_boot(get_absolute_time());
    if (events & GPIO_IRQ_EDGE_RISE) {
        last_rise_us = now;
    } else if (events & GPIO_IRQ_EDGE_FALL) {
        if (last_rise_us) pulse_us = now - last_rise_us;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(300);

    // ADC init
    adc_init();
    // Enable on-board temp sensor? (no) — we use external sensor on ADC_INPUT
    adc_gpio_init(26 + ADC_INPUT);
    adc_select_input(ADC_INPUT);

    // Optional digital pin for pulse width
    gpio_init(PIN_DIGITAL);
    gpio_set_dir(PIN_DIGITAL, GPIO_IN);
    gpio_pull_down(PIN_DIGITAL);
    gpio_set_irq_enabled_with_callback(PIN_DIGITAL,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &digital_irq);

    // Quick baseline for threshold suggestion
    uint32_t sum = 0;
    for (int i=0;i<50;i++) { sum += adc_read(); sleep_ms(5); }
    uint16_t avg = sum/50;
    int16_t threshold = (int16_t)avg - 200; // tweak in the field
    if (threshold < 0) threshold = 0;

    printf("IR line sensor demo. Avg=%u, thresh=%d\n", avg, threshold);

    while (true) {
        uint16_t v = adc_read(); // 12-bit (0..4095)
        bool is_black = (int)v < threshold;
        printf("ADC=%u %s  digital=%d pulse_us=%lu\n",
               v, is_black ? "BLACK" : "WHITE", gpio_get(PIN_DIGITAL),
               (unsigned long)pulse_us);
        sleep_ms(50);
    }
}
