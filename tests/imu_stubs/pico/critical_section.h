#ifndef TEST_IMU_CRITICAL_SECTION_H
#define TEST_IMU_CRITICAL_SECTION_H
#include <stdbool.h>
typedef struct { bool held; } critical_section_t;
void critical_section_init(critical_section_t *section);
void critical_section_enter_blocking(critical_section_t *section);
void critical_section_exit(critical_section_t *section);
#endif
