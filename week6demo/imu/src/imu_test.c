#include <stdio.h>
#include "pico/stdlib.h"
#include "imu.h"

int main() {
    stdio_init_all();
    sleep_ms(2000);

    imu_init();

    int16_t ax, ay, az;
    while (true) {
        if (imu_read_raw(&ax, &ay, &az)) {
            printf("Accel raw: ax=%d ay=%d az=%d\n", ax, ay, az);
        } else {
            printf("IMU read failed\n");
        }
        sleep_ms(500);
    }
}
