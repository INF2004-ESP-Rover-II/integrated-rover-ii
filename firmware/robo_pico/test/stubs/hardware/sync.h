#ifndef MOTOR_TEST_SYNC_H
#define MOTOR_TEST_SYNC_H
#include <stdint.h>
uint32_t save_and_disable_interrupts(void);
void restore_interrupts(uint32_t state);
#endif
