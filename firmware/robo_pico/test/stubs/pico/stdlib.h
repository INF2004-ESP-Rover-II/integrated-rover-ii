#ifndef MOTOR_TEST_STDLIB_H
#define MOTOR_TEST_STDLIB_H
#include <stdbool.h>
#include <stdint.h>
struct repeating_timer { int unused; };
uint32_t get_absolute_time(void);
uint32_t to_ms_since_boot(uint32_t value);
bool add_repeating_timer_ms(int32_t delay,
                            bool (*callback)(struct repeating_timer *),
                            void *user_data, struct repeating_timer *timer);
#endif
