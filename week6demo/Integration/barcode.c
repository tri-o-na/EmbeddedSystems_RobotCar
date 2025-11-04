#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "barcode.h"

// === Code39 Character Dictionaries ===
const Code39Mapping CODE39_DICT[] = {
    {"000110100", '0'}, {"100100001", '1'}, {"001100001", '2'}, {"101100000", '3'},
    {"000110001", '4'}, {"100110000", '5'}, {"001110000", '6'}, {"000100101", '7'},
    {"100100100", '8'}, {"001100100", '9'}, {"100001001", 'A'}, {"001001001", 'B'},
    {"101001000", 'C'}, {"000011001", 'D'}, {"100011000", 'E'}, {"001011000", 'F'},
    {"000001101", 'G'}, {"100001100", 'H'}, {"001001100", 'I'}, {"000011100", 'J'},
    {"100000011", 'K'}, {"001000011", 'L'}, {"101000010", 'M'}, {"000010011", 'N'},
    {"100010010", 'O'}, {"001010010", 'P'}, {"000000111", 'Q'}, {"100000110", 'R'},
    {"001000110", 'S'}, {"000010110", 'T'}, {"110000001", 'U'}, {"011000001", 'V'},
    {"111000000", 'W'}, {"010010001", 'X'}, {"110010000", 'Y'}, {"011010000", 'Z'},
    {"010000101", '-'}, {"110000100", '.'}, {"011000100", ' '}, {"010010100", '*'}
};

const Code39Mapping CODE39_DICT_REVERSE[] = {
    {"001011000", '0'}, {"100001001", '1'}, {"100001100", '2'}, {"000001101", '3'},
    {"100011000", '4'}, {"000011001", '5'}, {"000011100", '6'}, {"101001000", '7'},
    {"001001001", '8'}, {"001001100", '9'}, {"100100001", 'A'}, {"100100100", 'B'},
    {"000100101", 'C'}, {"100110000", 'D'}, {"000110001", 'E'}, {"000110100", 'F'},
    {"101110000", 'G'}, {"001100001", 'H'}, {"001110100", 'I'}, {"001110000", 'J'},
    {"110000001", 'K'}, {"110000100", 'L'}, {"010000101", 'M'}, {"110010000", 'N'},
    {"010010001", 'O'}, {"010010100", 'P'}, {"111000000", 'Q'}, {"011000001", 'R'},
    {"011000100", 'S'}, {"011010000", 'T'}, {"100000011", 'U'}, {"100000110", 'V'},
    {"000000111", 'W'}, {"100010010", 'X'}, {"000010011", 'Y'}, {"000010110", 'Z'},
    {"101000010", '-'}, {"001000011", '.'}, {"001000110", ' '}, {"001010010", '*'}
};

// === Globals ===
char decoded_message[MAX_MESSAGE_LENGTH];
int message_length = 0;
bool use_reverse_dict = false;

// === Adaptive IR Reading ===
int get_denoised_state(void)
{
    int sum = 0;
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        sum += gpio_get(BARCODE_IR_SENSOR_PIN);
        sleep_us(100); // fast sampling
    }
    float avg = (float)sum / SAMPLE_SIZE;

    static float low_avg = 0.3f, high_avg = 0.7f;
    if (avg > high_avg) high_avg = (high_avg * 0.9f) + (avg * 0.1f);
    if (avg < low_avg)  low_avg  = (low_avg  * 0.9f) + (avg * 0.1f);

    float dynamic_threshold = (high_avg + low_avg) / 2.0f;

    // Inverted logic: HIGH = black, LOW = white
    return (avg > dynamic_threshold) ? 0 : 1;
}

// === Utility Functions ===
void sort_bars(BarInfo *bars, int count)
{
    for (int i = 0; i < count - 1; i++)
        for (int j = 0; j < count - i - 1; j++)
            if (bars[j].width < bars[j + 1].width) {
                BarInfo t = bars[j];
                bars[j] = bars[j + 1];
                bars[j + 1] = t;
            }
}

int count_wide_bars(const char *pattern)
{
    int count = 0;
    for (int i = 0; i < BARS_PER_CHAR; i++)
        if (pattern[i] == '1') count++;
    return count;
}

char decode_pattern(const char *pattern)
{
    const Code39Mapping *dict = use_reverse_dict ? CODE39_DICT_REVERSE : CODE39_DICT;
    size_t size = use_reverse_dict ? sizeof(CODE39_DICT_REVERSE) / sizeof(CODE39_DICT_REVERSE[0])
                                   : sizeof(CODE39_DICT) / sizeof(CODE39_DICT[0]);

    for (size_t i = 0; i < size; i++)
        if (strcmp(pattern, dict[i].pattern) == 0)
            return dict[i].character;
    return '?';
}

void add_to_message(char decoded_char)
{
    if (message_length < MAX_MESSAGE_LENGTH - 1 && decoded_char != '*') {
        decoded_message[message_length++] = decoded_char;
        decoded_message[message_length] = '\0';
        printf("Decoded so far: %s\n", decoded_message);
    }
}

void reset_message(void)
{
    message_length = 0;
    decoded_message[0] = '\0';
    use_reverse_dict = false;
    printf("Message buffer reset\n");
}

// === Pattern Processing ===
void process_bars(BarInfo *bars, int char_num)
{
    char pattern[BARS_PER_CHAR + 1];
    BarInfo sorted[BARS_PER_CHAR];
    memcpy(sorted, bars, sizeof(BarInfo) * BARS_PER_CHAR);
    sort_bars(sorted, BARS_PER_CHAR);

    float threshold = sorted[2].width - 0.1f;

    for (int i = 0; i < BARS_PER_CHAR; i++)
        pattern[i] = (bars[i].width > threshold) ? '1' : '0';
    pattern[BARS_PER_CHAR] = '\0';

    int wide_count = count_wide_bars(pattern);

    printf("\n=== Character %d ===\n", char_num);
    printf("Pattern: %s (%d wide)\n", pattern, wide_count);

    if (wide_count == WIDE_BARS_PER_CHAR) {
        char decoded = decode_pattern(pattern);
        printf("→ Decoded: %c\n", decoded);
        add_to_message(decoded);
    } else {
        printf("⚠ Invalid pattern (expected %d wide)\n", WIDE_BARS_PER_CHAR);
    }
}

// === Tracking Bars ===
void track_bars(void)
{
    BarInfo bars[BARS_PER_CHAR];
    int current_state = get_denoised_state();
    int bar_count = 0, char_count = 0;
    absolute_time_t bar_start = get_absolute_time();
    bool started = false;

    gpio_init(RESET_BUTTON_PIN);
    gpio_set_dir(RESET_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(RESET_BUTTON_PIN);

    printf("\nReady to scan — waiting for first black bar...\n");
    printf("Press reset button (GP20) anytime to restart.\n");

    while (true)
    {
        // RESET BUTTON HANDLER
        if (gpio_get(RESET_BUTTON_PIN) == 0) {
            printf("\n=== RESET TRIGGERED ===\n");
            reset_message();
            bar_count = 0;
            char_count = 0;
            started = false;
            sleep_ms(300);
            continue;
        }

        int new_state = get_denoised_state();

        // Only begin once first BLACK is seen
        if (!started) {
            if (new_state == 0) {
                started = true;
                bar_start = get_absolute_time();
                current_state = new_state;
                printf("First black detected — start decoding...\n");
            }
            continue;
        }

        // On state change
        if (new_state != current_state)
        {
            absolute_time_t now = get_absolute_time();
            float width_ms = absolute_time_diff_us(bar_start, now) / 1000.0f;

            if (width_ms < 0.3f)
                continue; // ignore noise flicker

            bars[bar_count].width = width_ms;
            bars[bar_count].state = current_state;

            printf("Char %d - Bar %d: %s (%.2f ms)\n",
                   char_count, bar_count, current_state ? "WHITE" : "BLACK", width_ms);

            bar_count++;

            if (bar_count == BARS_PER_CHAR) {
                process_bars(bars, char_count);
                bar_count = 0;
                char_count++;
            }

            bar_start = now;
            current_state = new_state;
        }

        sleep_us(100);
    }
}

// === Entry Task (Decoding Only, No Motor Control) ===
void barcodeTask(void *pvParameters)
{
    stdio_init_all();

    printf("=== Barcode Reader Active ===\n");
    printf("Sensor pin: GPIO %d\n", BARCODE_IR_SENSOR_PIN);
    printf("Reset button: GPIO %d\n", RESET_BUTTON_PIN);
    printf("Waiting for barcode...\n");

    track_bars(); // start scanning and decoding
}
