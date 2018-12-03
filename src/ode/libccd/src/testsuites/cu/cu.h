/***
 * CU - C unit testing framework
 * ---------------------------------
 * Copyright (c)2007,2008,2009 Daniel Fiser <danfis@danfis.cz>
 *
 *
 *  This file is part of CU.
 *
 *  CU is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  CU is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _CU_H_
#define _CU_H_

#ifdef CU_ENABLE_TIMER
# include <time.h>
#endif /* CU_ENABLE_TIMER */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/***** PUBLIC API *****/
/**
 * Define test
 */
#define TEST(name) \
    void name(void)

/**
 * Define testsuite
 */
#define TEST_SUITE(name) \
    cu_test_suite_t test_suite_##name[] =
/**
 * Must be on the end of list of tests.
 */
#define TEST_SUITE_CLOSURE \
    { NULL, NULL }

#define TEST_SUITES \
    cu_test_suites_t cu_test_suites[] =
#define TEST_SUITES_CLOSURE \
    { NULL, NULL }
#define TEST_SUITE_ADD(name) \
    { #name, test_suite_##name }

/**
 * Add test to testsuite
 */
#define TEST_ADD(name) \
    { #name, name }

#define CU_RUN(argc, argv) \
    cu_run(argc, argv)

/**
 * Set prefix for files printed out. Must contain trailing /.
 */
#define CU_SET_OUT_PREFIX(str) \
    cu_set_out_prefix(str)

/**
 * Assertations
 * Assertations with suffix 'M' (e.g. assertTrueM) is variation of macro
 * where is possible to specify error message.
 */
#define assertTrueM(a, message) \
    if (a){ \
        cu_success_assertation(); \
    }else{ \
        cu_fail_assertation(__FILE__, __LINE__, message); \
    }
#define assertTrue(a) \
    assertTrueM((a), #a " is not true")

#define assertFalseM(a, message) \
    assertTrueM(!(a), message)
#define assertFalse(a) \
    assertFalseM((a), #a " is not false")

#define assertEqualsM(a,b,message) \
    assertTrueM((a) == (b), message)
#define assertEquals(a,b) \
    assertEqualsM((a), (b), #a " not equals " #b)

#define assertNotEqualsM(a,b,message) \
    assertTrueM((a) != (b), message)
#define assertNotEquals(a,b) \
    assertNotEqualsM((a), (b), #a " equals " #b)
/***** PUBLIC API END *****/

#include <unistd.h>

#define CU_MAX_NAME_LENGTH 30

typedef void (*cu_test_func_t)(void);
typedef struct _cu_test_suite_t {
    const char *name;
    cu_test_func_t func;
} cu_test_suite_t;
typedef struct _cu_test_suites_t {
    const char *name;
    cu_test_suite_t *test_suite;
} cu_test_suites_t;

extern cu_test_suites_t cu_test_suites[];

extern const char *cu_current_test;
extern const char *cu_current_test_suite;

extern int cu_success_test_suites;
extern int cu_fail_test_suites;
extern int cu_success_tests;
extern int cu_fail_tests;
extern int cu_success_checks;
extern int cu_fail_checks;

#define CU_OUT_PREFIX_LENGTH 30
extern char cu_out_prefix[CU_OUT_PREFIX_LENGTH+1];

void cu_run(int argc, char *argv[]);
void cu_success_assertation(void);
void cu_fail_assertation(const char *file, int line, const char *msg);
void cu_set_out_prefix(const char *str);

/** Timer **/
#ifdef CU_ENABLE_TIMER
extern struct timespec __cu_timer;

/**
 * Returns value of timer. (as timespec struct)
 */
const struct timespec *cuTimer(void);

/**
 * Starts timer.
 */
void cuTimerStart(void);

/**
 * Stops timer and record elapsed time from last call of cuTimerStart().
 * Returns current value of timer.
 */
const struct timespec *cuTimerStop(void);
#endif /* CU_ENABLE_TIMER */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
