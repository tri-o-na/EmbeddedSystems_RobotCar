#include "barcode.h"
#include "pico/stdlib.h"

void barcode_init(barcode_cfg_t *cfg, uint32_t smax, uint32_t lmin, uint32_t gap) {
    cfg->short_max_us = smax;
    cfg->long_min_us  = lmin;
    cfg->quiet_gap_us = gap;
    cfg->last_emit_us = 0;
}

barcode_cmd_t barcode_feed(barcode_cfg_t *cfg, uint64_t black_us) {
    uint64_t now = time_us_64();
    if (cfg->last_emit_us && (now - cfg->last_emit_us) < cfg->quiet_gap_us) return BC_NONE;

    static uint64_t last_long_time = 0;

    if (black_us >= cfg->long_min_us) {
        if (last_long_time && (now - last_long_time) < 600000) {
            last_long_time = 0;
            cfg->last_emit_us = now;
            return BC_STOP;
        }
        last_long_time = now;
        cfg->last_emit_us = now;
        return BC_RIGHT;
    } else if (black_us > 0 && black_us <= cfg->short_max_us) {
        cfg->last_emit_us = now;
        return BC_LEFT;
    }
    return BC_NONE;
}

const char* barcode_cmd_name(barcode_cmd_t c){
    switch(c){
        case BC_LEFT: return "LEFT";
        case BC_RIGHT: return "RIGHT";
        case BC_STOP: return "STOP";
        default: return "NONE";
    }
}
