/**
 * @file test_support.h
 * @brief Tiny check helper shared by the Subsystem 4 host tests.
 */

#ifndef TEST_SUPPORT_H
#define TEST_SUPPORT_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Count a check. A failed check prints its line number and is
 *        remembered, so the test can still run to the end.
 * @param condition Result of the check.
 * @param line      Pass __LINE__ from the call site.
 */
void test_check(bool condition, uint32_t line);

/**
 * @brief Number of checks made so far.
 * @return Check count.
 */
uint32_t test_check_count(void);

/**
 * @brief Print the summary line and give the process exit status.
 * @param p_name Test name for the summary line.
 * @return 0 if every check passed, otherwise 1.
 */
int test_finish(const char * p_name);

#endif /* TEST_SUPPORT_H */

/* end of file test_support.h */
