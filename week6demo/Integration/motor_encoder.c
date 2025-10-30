#include "motor_encoder.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdint.h>        // <-- add this line


#define M1A 8
#define M1B 9
#define M2A 10
#define M2B 11
#define PWM_WRAP 1000u
#define PWM_DIVIDER 6.25f

static inline uint16_t duty(float x) {
    if (x > 1) x = 1;
    if (x < -1) x = -1;
    return (uint16_t)((x + 1) * PWM_WRAP / 2);
}

void motors_and_encoders_init(void) {
    gpio_set_function(M1A, GPIO_FUNC_PWM);
    gpio_set_function(M1B, GPIO_FUNC_PWM);
    gpio_set_function(M2A, GPIO_FUNC_PWM);
    gpio_set_function(M2B, GPIO_FUNC_PWM);

    uint slice1 = pwm_gpio_to_slice_num(M1A);
    uint slice2 = pwm_gpio_to_slice_num(M2A);

    pwm_set_wrap(slice1, PWM_WRAP);
    pwm_set_wrap(slice2, PWM_WRAP);
    pwm_set_clkdiv(slice1, PWM_DIVIDER);
    pwm_set_clkdiv(slice2, PWM_DIVIDER);
    pwm_set_enabled(slice1, true);
    pwm_set_enabled(slice2, true);
}

void motor_set(float left, float right) {
    pwm_set_gpio_level(M1A, duty(left));
    pwm_set_gpio_level(M1B, PWM_WRAP - duty(left));
    pwm_set_gpio_level(M2A, duty(right));
    pwm_set_gpio_level(M2B, PWM_WRAP - duty(right));
}

void motors_stop(void) { motor_set(0, 0); }
