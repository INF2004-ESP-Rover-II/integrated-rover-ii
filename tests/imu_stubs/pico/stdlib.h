#ifndef TEST_IMU_STDLIB_H
#define TEST_IMU_STDLIB_H
#include <stdint.h>
#define GPIO_FUNC_I2C 3u
void gpio_set_function(unsigned pin, unsigned function);
void gpio_pull_up(unsigned pin);
void sleep_ms(uint32_t ms);
void sleep_us(uint64_t us);
uint64_t time_us_64(void);
#endif
