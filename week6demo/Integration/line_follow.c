#include "line_follow.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <string.h>

#define ADC_LINE 0          // GPIO26
#define THRESH_LINE 2000    // midpoint between white (~200) and black (~4000)

static uint64_t last_black_start = 0;
static uint64_t black_duration = 0;

void lf_init(void) {
    adc_init();
    adc_gpio_init(26);
}

static inline uint16_t read_adc(int ch) {
    adc_select_input(ch);
    sleep_us(5);
    return adc_read();
}

void lf_read(line_sample_t *s) {
    memset(s, 0, sizeof(*s));
    s->adc_left = read_adc(ADC_LINE);

    // 👇 FIX: black gives high ADC, so onLine = 1 when adc > threshold
    s->left_on_line = (s->adc_left > THRESH_LINE);

    uint64_t now = time_us_64();

    if (s->left_on_line) {
        // Sensor sees black
        if (!last_black_start) last_black_start = now;
        s->black_us_left = now - last_black_start;
    } else {
        // Sensor sees white — reset timer
        if (last_black_start) black_duration = now - last_black_start;
        s->black_us_left = black_duration;
        last_black_start = 0;
    }
}
