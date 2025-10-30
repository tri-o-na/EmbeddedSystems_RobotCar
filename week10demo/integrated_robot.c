#include "robot_config.h"
#include "hardware/irq.h"
#include "hardware/timer.h" // Needed for absolute_time functions
#include <string.h>         // Needed for memset/memcpy

// ====================================
// 1. IMU Implementation (from accel_raw_mpu6050.c)
// ====================================

static void mpu_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    i2c_write_blocking(I2C_PORT, MPU_ADDR, buf, 2, false);
}

static void mpu_read(uint8_t reg, uint8_t *dst, size_t n) {
    i2c_write_blocking(I2C_PORT, MPU_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDR, dst, n, false);
}

static int16_t to_i16(uint8_t msb, uint8_t lsb) {
    int16_t v = (int16_t)((msb << 8) | lsb);
    return v;
}

void imu_init(void) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    // Wake MPU6050: PWR_MGMT_1 (0x6B) = 0
    mpu_write(0x6B, 0x00);
    sleep_ms(50);
}

void imu_read_raw(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t b[6];
    mpu_read(0x3B, b, 6); // ACCEL_X/Y/Z H:L
    *ax = to_i16(b[0], b[1]);
    *ay = to_i16(b[2], b[3]);
    *az = to_i16(b[4], b[5]);
}


// ====================================
// 2. Ultrasonic Implementation (from ultrasonic_hcsr04.c)
// ====================================

// Wait for a pin to reach a level with timeout in us
static bool wait_for_level(uint pin, bool level, uint32_t timeout_us) {
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    while (gpio_get(pin) != (int)level) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            return false;
        }
        tight_loop_contents();
    }
    return true;
}

void ultrasonic_init(void) {
    gpio_init(PIN_TRIG); gpio_set_dir(PIN_TRIG, GPIO_OUT); gpio_put(PIN_TRIG, 0);
    gpio_init(PIN_ECHO); gpio_set_dir(PIN_ECHO, GPIO_IN);
}

int32_t measure_echo_us(void) {
    // ensure trigger low
    gpio_put(PIN_TRIG, 0);
    sleep_us(2);
    // 10 us pulse
    gpio_put(PIN_TRIG, 1);
    sleep_us(10);
    gpio_put(PIN_TRIG, 0);

    // Wait for echo rising (timeout 30 ms)
    if (!wait_for_level(PIN_ECHO, true, 30000)) return -1;
    absolute_time_t start = get_absolute_time();

    // Wait for echo falling (timeout 30 ms)
    if (!wait_for_level(PIN_ECHO, false, 30000)) return -1;
    absolute_time_t end = get_absolute_time();

    return (int32_t)absolute_time_diff_us(start, end); // positive
}


// ====================================
// 3. Motors & Encoders (from motor_encoder.c)
// ====================================

static volatile uint32_t enc1_last_rise_us = 0;
static volatile uint32_t enc2_last_rise_us = 0;
static volatile uint32_t enc1_pulse_us = 0;
static volatile uint32_t enc2_pulse_us = 0;

static void encoder_isr(uint gpio, uint32_t events) {
    uint32_t now = to_us_since_boot(get_absolute_time());
    if (gpio == ENC1_A) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            enc1_last_rise_us = now;
        } else if ((events & GPIO_IRQ_EDGE_FALL) && enc1_last_rise_us) {
            enc1_pulse_us = now - enc1_last_rise_us;
        }
    } else if (gpio == ENC2_A) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            enc2_last_rise_us = now;
        } else if ((events & GPIO_IRQ_EDGE_FALL) && enc2_last_rise_us) {
            enc2_pulse_us = now - enc2_last_rise_us;
        }
    }
}

static void pwm_init_pin(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, PWM_DIVIDER);
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(gpio, 0);
}

static inline uint16_t duty_from_float(float s) {
    if (s < 0.0f) s = -s;
    if (s > 1.0f) s = 1.0f;
    return (uint16_t)(s * PWM_WRAP);
}

void motors_and_encoders_init(void) {
    // PWM outputs
    pwm_init_pin(M1A);
    pwm_init_pin(M1B);
    pwm_init_pin(M2A);
    pwm_init_pin(M2B);

    // Encoder inputs
    gpio_init(ENC1_A); gpio_set_dir(ENC1_A, GPIO_IN); gpio_pull_up(ENC1_A);
    gpio_init(ENC2_A); gpio_set_dir(ENC2_A, GPIO_IN); gpio_pull_up(ENC2_A);
    gpio_set_irq_enabled_with_callback(ENC1_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_isr);
    gpio_set_irq_enabled(ENC2_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

void motor_set(float m1, float m2) {
    uint16_t d1 = duty_from_float(m1);
    uint16_t d2 = duty_from_float(m2);

    if (m1 > 0) { pwm_set_gpio_level(M1A, d1); pwm_set_gpio_level(M1B, 0); }
    else if (m1 < 0) { pwm_set_gpio_level(M1A, 0); pwm_set_gpio_level(M1B, d1); }
    else { pwm_set_gpio_level(M1A, 0); pwm_set_gpio_level(M1B, 0); }

    if (m2 > 0) { pwm_set_gpio_level(M2A, d2); pwm_set_gpio_level(M2B, 0); }
    else if (m2 < 0) { pwm_set_gpio_level(M2A, 0); pwm_set_gpio_level(M2B, d2); }
    else { pwm_set_gpio_level(M2A, 0); pwm_set_gpio_level(M2B, 0); }
}

void motors_stop(void) {
    motor_set(0.0f, 0.0f);
}

uint32_t encoder_pulse_width_us(int motor_index) {
    if (motor_index == 1) return enc1_pulse_us;
    if (motor_index == 2) return enc2_pulse_us;
    return 0;
}


// ====================================
// 4. IR Line Sensors (from ir_line_sensor.c)
// ====================================

// Global state variables for LOW pulse width (active-low)
static volatile uint32_t a_last_fall_us = 0, b_last_fall_us = 0;
static volatile uint32_t a_low_pw_us = 0,   b_low_pw_us = 0;

static void do_irq(uint gpio, uint32_t events) {
    uint32_t now = to_us_since_boot(get_absolute_time());

    if (gpio == A_PIN_DO) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            a_last_fall_us = now;
        } else if ((events & GPIO_IRQ_EDGE_RISE) && a_last_fall_us) {
            a_low_pw_us = now - a_last_fall_us;
        }
    } else if (gpio == B_PIN_DO) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            b_last_fall_us = now;
        } else if ((events & GPIO_IRQ_EDGE_RISE) && b_last_fall_us) {
            b_low_pw_us = now - b_last_fall_us;
        }
    }
}

void ir_sensors_init(void) {
    // -------- ADC init --------
    adc_init();
    adc_gpio_init(26 + A_ADC_INPUT);
    adc_gpio_init(26 + B_ADC_INPUT);

    // -------- Digital pins init (DO) --------
    gpio_init(A_PIN_DO);
    gpio_set_dir(A_PIN_DO, GPIO_IN);
    gpio_pull_up(A_PIN_DO);

    gpio_init(B_PIN_DO);
    gpio_set_dir(B_PIN_DO, GPIO_IN);
    gpio_pull_up(B_PIN_DO);

    // IRQs for LOW pulse width on both DOs
    gpio_set_irq_enabled_with_callback(A_PIN_DO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &do_irq);
    gpio_set_irq_enabled(B_PIN_DO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    // Note: The original baseline/threshold calculation is not moved here;
    // that logic should be performed in the main control loop as needed.
}

// Additional functions needed for Demo 2 control logic:
uint16_t ir_read_adc(int sensor_index) {
    if (sensor_index == 0) {
        adc_select_input(A_ADC_INPUT);
        return adc_read();
    } else if (sensor_index == 1) {
        adc_select_input(B_ADC_INPUT);
        return adc_read();
    }
    return 0;
}

uint32_t ir_read_pulse_width_us(int sensor_index) {
    if (sensor_index == 0) return a_low_pw_us;
    if (sensor_index == 1) return b_low_pw_us;
    return 0;
}

int ir_read_digital_raw(int sensor_index) {
    if (sensor_index == 0) return gpio_get(A_PIN_DO);
    if (sensor_index == 1) return gpio_get(B_PIN_DO);
    return -1;
}