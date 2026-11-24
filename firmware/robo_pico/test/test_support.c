/**
 * @file test_support.c
 * @brief Tiny check helper shared by the Subsystem 4 host tests.
 */

#include <inttypes.h>
#include <stdio.h>
#include "test_support.h"

/** Checks made so far. */
static uint32_t g_checks = 0u;

/** Checks that failed. */
static uint32_t g_failures = 0u;

void test_check(bool condition, uint32_t line)
{
    g_checks++;

    if (!condition)
    {
        g_failures++;
        (void)printf("FAIL line %" PRIu32 "\n", line);
    }
}

uint32_t test_check_count(void)
{
    return g_checks;
}

int test_finish(const char * p_name)
{
    int status = 0;

    if (0u != g_failures)
    {
        (void)printf("%s: %" PRIu32 " of %" PRIu32 " checks FAILED\n",
                     p_name, g_failures, g_checks);
        status = 1;
    }
    else
    {
        (void)printf("%s: %" PRIu32 " checks OK\n", p_name, g_checks);
    }

    return status;
}

/* end of file test_support.c */
