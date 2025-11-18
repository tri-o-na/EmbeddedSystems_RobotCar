#ifndef BARCODE_H
#define BARCODE_H

#include "pico/stdlib.h"
#include <stdint.h>

#define MAX_BARS 256
#define MIN_BAR_US 80
#define MAX_BAR_US 3000000
#define STABLE_READS 6

typedef struct {
    uint32_t duration_us[MAX_BARS];
    uint8_t  color[MAX_BARS]; // 1 = black, 0 = white
    int      count;
} barcode_scan_t;

void barcode_init(void);
void barcode_scan(barcode_scan_t *scan); // BLOCKING - AVOID USE IN MAIN LOOP
void barcode_decode_full(barcode_scan_t *scan);
void barcode_nonblocking_update(void); // NON-BLOCKING - USE IN MAIN LOOP

#endif