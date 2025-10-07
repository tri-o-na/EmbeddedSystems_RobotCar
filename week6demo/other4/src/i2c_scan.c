#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include "hardware/i2c.h"
#include <stdint.h> 


#define I2C_PORT i2c0
#define SDA_PIN 16
#define SCL_PIN 17

int main() {
    stdio_init_all();
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    printf("Scanning I2C bus on pins 16(SDA)/17(SCL)...\n");
    sleep_ms(2000);

    for (uint8_t addr = 1; addr < 127; addr++) {
        uint8_t rxdata;
        int result = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);
        if (result >= 0) {
            printf("Found device at 0x%02X\n", addr);
        }
    }
    printf("Scan done.\n");
    while (true) sleep_ms(1000);
}
