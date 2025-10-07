#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define SDA_PIN 16   // SDA must be GPIO16
#define SCL_PIN 17   // SCL must be GPIO17

int main() {
    stdio_init_all();
    sleep_ms(2000); // give USB serial some time

    // Init I2C at 100kHz
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    printf("I2C Scanner starting...\n");

    while (true) {
        for (int addr = 1; addr < 127; addr++) {
            uint8_t rxdata;
            int ret = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);
            if (ret >= 0) {
                printf("Found device at address 0x%02X\n", addr);
            }
        }
        printf("Scan complete.\n\n");
        sleep_ms(2000);
    }
}
