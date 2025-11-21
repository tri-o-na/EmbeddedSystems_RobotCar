#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stdbool.h>

// servo API
void servo_init(uint32_t gpio_pin);
void servo_move_to_center(void);
void servo_move_to_left(void);
void servo_move_to_right(void);
void servo_set_pulse_us(uint16_t pulse_us);

// New: slow scan API (delay_ms controls how slowly the servo steps)
uint16_t servo_scan_until_clear_slow(bool scan_left, float object_threshold, float (*get_distance_func)(void), uint32_t delay_ms);

// scan until distance reading > threshold, return last pulse_us position (1500 = center)
uint16_t servo_scan_until_clear(bool scan_left, float object_threshold, float (*get_distance_func)(void));

// high level wrappers (if used elsewhere)
void motors_stop(void);

#endif // SERVO_H