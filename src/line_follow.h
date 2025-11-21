#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float line_error;      // normalized [-1..1], 0=center
    uint16_t adc_left;
    uint16_t adc_right;
    uint64_t black_us_left;
    uint64_t black_us_right;
    bool left_on_line;
    bool right_on_line;
} line_sample_t;

void lf_init(void);
void lf_read(line_sample_t *out);

#endif
