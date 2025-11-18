#include "barcode.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define BARCODE_PIN 3
#define TIMEOUT_US 4000000

// ------------------------------------------------------
// Code39 dictionary
// ------------------------------------------------------
typedef struct {
    const char *bits; // 9 chars '0' or '1'
    char ch;
} code39_t;

static const code39_t CODE39_TABLE[] = {
    {"000110100",'0'}, {"100100001",'1'}, {"001100001",'2'}, {"101100000",'3'},
    {"000110001",'4'}, {"100110000",'5'}, {"001110000",'6'}, {"000100101",'7'},
    {"100100100",'8'}, {"001100100",'9'}, {"100001001",'A'}, {"001001001",'B'},
    {"101001000",'C'}, {"000011001",'D'}, {"100011000",'E'}, {"001011000",'F'},
    {"000001101",'G'}, {"100001100",'H'}, {"001001100",'I'}, {"000011100",'J'},
    {"100000011",'K'}, {"001000011",'L'}, {"101000010",'M'}, {"000010011",'N'},
    {"100010010",'O'}, {"001010010",'P'}, {"000000111",'Q'}, {"100000110",'R'},
    {"001000110",'S'}, {"000010110",'T'}, {"110000001",'U'}, {"011000001",'V'},
    {"111000000",'W'}, {"010010001",'X'}, {"110010000",'Y'}, {"011010000",'Z'},
    {"010000101",'-'}, {"110000100",'.'}, {"011000100",' '}, {"010010100",'*'}
};

// ------------------------------------------------------
// Init
// ------------------------------------------------------
void barcode_init(void) {
    gpio_init(BARCODE_PIN);
    gpio_set_dir(BARCODE_PIN, GPIO_IN);
}

// ------------------------------------------------------
// Wait for first black
// ------------------------------------------------------
static void wait_for_first_black() {
    printf("Waiting for first BLACK bar...\n");

    while (true) {
        int v = gpio_get(BARCODE_PIN);

        int stable = 0;
        for (int i = 0; i < STABLE_READS; i++) {
            sleep_us(80);
            if (gpio_get(BARCODE_PIN) == v) stable++;
        }

        if (stable == STABLE_READS && v == 1) {
            printf("Detected START (BLACK)\n");
            return;
        }
    }
}

// ------------------------------------------------------
// Non-blocking state machine variables
// ------------------------------------------------------
static barcode_scan_t current_scan;
static int prev_val = -1;
static absolute_time_t last_edge_time;
static absolute_time_t last_scan_start;
static bool scanning_active = false;

// ------------------------------------------------------
// Non-Blocking Update Function
// ------------------------------------------------------
void barcode_nonblocking_update(void) {
    
    // Check for a long timeout to end the scan
    if (scanning_active && absolute_time_diff_us(last_edge_time, get_absolute_time()) > TIMEOUT_US) {
        scanning_active = false;
        printf("Scan ended by timeout. Captured %d bars.\n", current_scan.count);
        
        // Only decode if we captured enough data
        if (current_scan.count >= 9) {
            barcode_decode_full(&current_scan);
        }
        return;
    }

    int v = gpio_get(BARCODE_PIN);

    // State initialization: Wait for the first transition (black bar start)
    if (!scanning_active) {
        if (v == 1) { // Wait for BLACK (high)
            current_scan.count = 0;
            scanning_active = true;
            prev_val = 1;
            last_edge_time = get_absolute_time();
            last_scan_start = last_edge_time;
            printf("Starting non-blocking scan (Left Sensor)...\n");
        }
        return; // Wait for BLACK
    }
    
    // Edge detection logic
    if (v != prev_val) {
        
        // Debouncing/Stability check
        int stable = 0;
        for (int i = 0; i < STABLE_READS; i++) {
            sleep_us(80);
            if (gpio_get(BARCODE_PIN) == v) stable++;
        }
        if (stable < STABLE_READS) return; // Not stable, ignore edge
        
        // Edge is confirmed
        absolute_time_t now = get_absolute_time();
        uint32_t dur = absolute_time_diff_us(last_scan_start, now);
        
        // Record bar if duration is valid
        if (dur >= MIN_BAR_US && dur <= MAX_BAR_US && current_scan.count < MAX_BARS) {
            current_scan.duration_us[current_scan.count] = dur;
            current_scan.color[current_scan.count] = prev_val;
            current_scan.count++;

            // printf("[%02d] %s %u us\n",
            //        current_scan.count,
            //        prev_val ? "Black" : "White",
            //        dur);
        } else if (dur > MAX_BAR_US) {
             // Treat as end of scan if bar is too long
             scanning_active = false;
             printf("Scan ended by too long bar. Captured %d bars.\n", current_scan.count);
             if (current_scan.count >= 9) {
                 barcode_decode_full(&current_scan);
             }
             return;
        }

        // Update state for next transition
        prev_val = v;
        last_scan_start = now;
        last_edge_time = now;
    }
}

// ------------------------------------------------------
// Scan bars
// ------------------------------------------------------
void barcode_scan(barcode_scan_t *scan) {
    memset(scan, 0, sizeof(*scan));
    wait_for_first_black();

    int prev = 1;
    absolute_time_t last_edge = get_absolute_time();
    absolute_time_t start = last_edge;

    printf("=== SCANNING ===\n");

    while (true) {
        int v = gpio_get(BARCODE_PIN);

        int stable = 0;
        for (int i = 0; i < STABLE_READS; i++) {
            sleep_us(80);
            if (gpio_get(BARCODE_PIN) == v) stable++;
        }
        if (stable < STABLE_READS) continue;

        if (v != prev) {
            absolute_time_t now = get_absolute_time();
            uint32_t dur = absolute_time_diff_us(start, now);

            if (dur >= MIN_BAR_US && dur <= MAX_BAR_US) {
                scan->duration_us[scan->count] = dur;
                scan->color[scan->count] = prev;
                scan->count++;

                printf("[%02d] %s %u us\n",
                       scan->count,
                       prev ? "Black" : "White",
                       dur);
            }

            prev = v;
            start = now;
            last_edge = now;
        }

        if (absolute_time_diff_us(last_edge, get_absolute_time()) > TIMEOUT_US)
            break;
    }

    printf("Captured %d bars\n", scan->count);
}

// ------------------------------------------------------
// Classify 9 bars into bits ("0"=narrow, "1"=wide)
// ------------------------------------------------------
static void classify_9(const uint32_t *dur, char *out) {

    uint32_t min = dur[0], max = dur[0];
    for (int i = 1; i < 9; i++) {
        if (dur[i] < min) min = dur[i];
        if (dur[i] > max) max = dur[i];
    }

    // adaptive threshold between narrow and wide
    float threshold = (min + max) * 0.5f;

    for (int i = 0; i < 9; i++) {
        out[i] = (dur[i] >= threshold ? '1' : '0');
    }

    out[9] = '\0';
}


// ------------------------------------------------------
// Convert 9-bit pattern to character
// ------------------------------------------------------
static char decode_9bit(const char *bits) {
    for (int i = 0; i < (int)(sizeof(CODE39_TABLE)/sizeof(code39_t)); i++) {
        if (strcmp(bits, CODE39_TABLE[i].bits) == 0)
            return CODE39_TABLE[i].ch;
    }
    return '?';
}

// ------------------------------------------------------
// FULL CHARACTER-BASED DECODER
// ------------------------------------------------------
void barcode_decode_full(barcode_scan_t *scan) {
    if (scan->count < 9) {
        printf("Not enough bars.\n");
        return;
    }

    printf("\n=== DECODING MULTI-CHAR (RIGID 10-BAR STEP) ===\n"); // MODIFIED title

    char output[64];
    int out_i = 0;

    int s = 0;
    while (s + 9 <= scan->count) {

        // 9 bars window (0-8, 10-18, etc.)
        uint32_t win[9];
        for (int i = 0; i < 9; i++)
            win[i] = scan->duration_us[s + i];

        char bits[10];
        classify_9(win, bits);

        char ch = decode_9bit(bits);

        printf("Bars %d–%d → %s → %c\n", s, s+8, bits, ch);

        if (ch != '?') {
            // valid character found
            output[out_i++] = ch;

            // Enforce rigid skip: 9 data bars + 1 ICG = 10 bars
            s += 10; 
        } else {
            // invalid → Still skip 10 bars to enforce synchronization 
            // (9 data bars + 1 separator bar)
            s += 10; // <-- MODIFIED: Removed sliding window (s+=1)
        }
    }

    if (out_i == 0) {
        printf("No valid characters decoded using rigid structure.\n"); // MODIFIED message
        return;
    }

    output[out_i] = '\0';

    printf("FINAL STRING: %s\n", output);
    printf("=========================\n");
}

