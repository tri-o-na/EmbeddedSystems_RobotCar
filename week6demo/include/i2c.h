#ifndef _HARDWARE_I2C_H
#define _HARDWARE_I2C_H

#include "hardware/structs/i2c.h"
#include "pico.h"
#include "pico/time.h"

#ifdef __cplusplus
extern "C" {
#endif

// Default I2C instances
extern i2c_inst_t i2c0_inst;
extern i2c_inst_t i2c1_inst;
#define i2c0 (&i2c0_inst)
#define i2c1 (&i2c1_inst)

// Function prototypes
void i2c_init(i2c_inst_t *i2c, uint baudrate);
int i2c_write_blocking(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop);
int i2c_read_blocking(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop);

#ifdef __cplusplus
}
#endif

#endif
