#ifndef BARCODE_H
#define BARCODE_H

#include <stdint.h>

typedef enum { BC_NONE=0, BC_LEFT, BC_RIGHT, BC_STOP } barcode_cmd_t;

typedef struct {
    uint32_t short_max_us;
    uint32_t long_min_us;
    uint32_t quiet_gap_us;
    uint64_t last_emit_us;
} barcode_cfg_t;

void barcode_init(barcode_cfg_t *cfg, uint32_t short_max, uint32_t long_min, uint32_t gap);
barcode_cmd_t barcode_feed(barcode_cfg_t *cfg, uint64_t black_us);
const char* barcode_cmd_name(barcode_cmd_t c);

#endif
