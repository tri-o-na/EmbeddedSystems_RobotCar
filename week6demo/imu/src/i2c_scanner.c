#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define SDA_PIN 4   // try also (16,17)
#define SCL_PIN 5

int main() {
    stdio_init_all();
    sleep_ms(2000);

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    printf("I2C Scanner starting...\n");

    int found = 0;
    for (int addr = 1; addr < 127; addr++) {
        uint8_t rxdata; 
        int ret = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);
        if (ret >= 0) {
            printf("✅ Found device at 0x%02x\n", addr);
            found = 1;
        }
    }

    if (!found) {
        printf("⚠️ No I2C devices found on SDA=%d SCL=%d\n", SDA_PIN, SCL_PIN);
    }

    while (1) {
        sleep_ms(1000);
    }
}
