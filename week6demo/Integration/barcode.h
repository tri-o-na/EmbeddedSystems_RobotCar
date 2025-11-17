#ifndef BARCODE_H
#define BARCODE_H
#include <stdbool.h>

#define SAMPLE_SIZE 5
#define THRESHOLD 0.5f
#define BARS_PER_CHAR 9
#define WIDE_BARS_PER_CHAR 3
#define OUTLIER_RATIO 5.0f
#define MAX_MESSAGE_LENGTH 50

#define BARCODE_IR_SENSOR_PIN 2
#define RESET_BUTTON_PIN 20

typedef struct {
    float width;
    int state;
} BarInfo;

typedef struct {
    const char *pattern;
    char character;
} Code39Mapping;

int  get_denoised_state(void);
void sort_bars(BarInfo *bars, int count);
int  count_wide_bars(const char *pattern);
char decode_pattern(const char *pattern);
void process_bars(BarInfo *bars, int character_count);
void add_to_message(char decoded_char);
void reset_message(void);
void track_bars(void);
void barcodeTask(void *pvParameters);
void barcode_update(void);
void barcode_init_nonblocking(void);



#endif
