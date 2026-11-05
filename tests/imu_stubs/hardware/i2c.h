#ifndef TEST_IMU_I2C_H
#define TEST_IMU_I2C_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct { int unused; } i2c_inst_t;
extern i2c_inst_t test_i2c;
extern i2c_inst_t test_other_i2c;
#define i2c0 (&test_other_i2c)
#define i2c1 (&test_i2c)
unsigned i2c_init(i2c_inst_t *port, unsigned baud);
int i2c_write_timeout_us(i2c_inst_t *port, uint8_t addr, const uint8_t *bytes,
                         size_t count, bool nostop, unsigned timeout_us);
int i2c_read_timeout_us(i2c_inst_t *port, uint8_t addr, uint8_t *bytes,
                        size_t count, bool nostop, unsigned timeout_us);
#endif
