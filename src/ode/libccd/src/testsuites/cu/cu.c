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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>

#include "cu.h"

/** Declared here, because I didn't find header file where it is declared */
char *strsignal(int sig);

const char *cu_current_test;
const char *cu_current_test_suite;
int cu_success_test_suites = 0;
int cu_fail_test_suites = 0;
int cu_success_tests = 0;
int cu_fail_tests = 0;
int cu_success_checks = 0;
int cu_fail_checks = 0;

char cu_out_prefix[CU_OUT_PREFIX_LENGTH+1] = "";

/* globally used file descriptor for reading/writing messages */
int fd;

/* indicate if test was failed */
int test_failed;

/* codes of messages */
#define CHECK_FAILED '0'
#define CHECK_SUCCEED '1'
#define TEST_FAILED '2'
#define TEST_SUCCEED '3'
#define TEST_SUITE_FAILED '4'
#define TEST_SUITE_SUCCEED '5'
#define END '6'
#define TEST_NAME '7'

/* predefined messages */
#define MSG_CHECK_SUCCEED write(fd, "1\n", 2)
#define MSG_TEST_FAILED write(fd, "2\n", 2)
#define MSG_TEST_SUCCEED write(fd, "3\n", 2)
#define MSG_TEST_SUITE_FAILED write(fd, "4\n", 2)
#define MSG_TEST_SUITE_SUCCEED write(fd, "5\n", 2)
#define MSG_END write(fd, "6\n", 2)

/* length of buffers */
#define BUF_LEN 1000
#define MSGBUF_LEN 300

static void redirect_out_err(const char *testName);
static void close_out_err(void);
static void run_test_suite(const char *ts_name, cu_test_suite_t *ts);
static void receive_messages(void);

static void cu_run_fork(const char *ts_name, cu_test_suite_t *test_suite);
static void cu_print_results(void);

void cu_run(int argc, char *argv[])
{
    cu_test_suites_t *tss;
    int i;
    char found = 0;

    if (argc > 1){
        for (i=1; i < argc; i++){
            tss = cu_test_suites;
            while (tss->name != NULL && tss->test_suite != NULL){
                if (strcmp(argv[i], tss->name) == 0){
                    found = 1;
                    cu_run_fork(tss->name, tss->test_suite);
                    break;
                }
                tss++;
            }

            if (tss->name == NULL || tss->test_suite == NULL){
                fprintf(stderr, "ERROR: Could not find test suite '%s'\n", argv[i]);
            }
        }

        if (found == 1)
            cu_print_results();

    }else{
        tss = cu_test_suites;
        while (tss->name != NULL && tss->test_suite != NULL){
            cu_run_fork(tss->name, tss->test_suite);
            tss++;
        }
        cu_print_results();
    }

}

static void cu_run_fork(const char *ts_name, cu_test_suite_t *ts)
{
    int pipefd[2];
    int pid;
    int status;

    if (pipe(pipefd) == -1){
        perror("Pipe error");
        exit(-1);
    }

    fprintf(stdout, " -> %s [IN PROGESS]\n", ts_name);
    fflush(stdout);

    pid = fork();
    if (pid < 0){
        perror("Fork error");
        exit(-1);
    }

    if (pid == 0){
        /* close read end of pipe */
        close(pipefd[0]);

        fd = pipefd[1];

        /* run testsuite, messages go to fd */
        run_test_suite(ts_name, ts);

        MSG_END;
        close(fd);

        /* stop process where running testsuite */
        exit(0);
    }else{
        /* close write end of pipe */
        close(pipefd[1]);

        fd = pipefd[0];

        /* receive and interpret all messages */
        receive_messages();

        /* wait for children */
        wait(&status);
        if (!WIFEXITED(status)){ /* if child process ends up abnormaly */
            if (WIFSIGNALED(status)){
                fprintf(stdout, "Test suite was terminated by signal %d (%s).\n",
                        WTERMSIG(status), strsignal(WTERMSIG(status)));
            }else{
                fprintf(stdout, "Test suite terminated abnormaly!\n");
            }

            /* mark this test suite as failed, because was terminated
             * prematurely */
            cu_fail_test_suites++;
        }

        close(fd);

        fprintf(stdout, " -> %s [DONE]\n\n", ts_name);
        fflush(stdout);
    }

}

static void run_test_suite(const char *ts_name, cu_test_suite_t *ts)
{
    int test_suite_failed = 0;
    char buffer[MSGBUF_LEN];
    int len;

    /* set up current test suite name for later messaging... */
    cu_current_test_suite = ts_name;

    /* redirect stdout and stderr */
    redirect_out_err(cu_current_test_suite);

    while (ts->name != NULL && ts->func != NULL){
        test_failed = 0;

        /* set up name of test for later messaging */
        cu_current_test = ts->name;

        /* send message what test is currently running */
        len = snprintf(buffer, MSGBUF_LEN, "%c    --> Running %s...\n",
                       TEST_NAME, cu_current_test);
        write(fd, buffer, len);

        /* run test */
        (*(ts->func))();

        if (test_failed){
            MSG_TEST_FAILED;
            test_suite_failed = 1;
        }else{
            MSG_TEST_SUCCEED;
        }

        ts++; /* next test in test suite */
    }

    if (test_suite_failed){
        MSG_TEST_SUITE_FAILED;
    }else{
        MSG_TEST_SUITE_SUCCEED;
    }

    /* close redirected stdout and stderr */
    close_out_err();
}

static void receive_messages(void)
{
    char buf[BUF_LEN]; /* buffer */
    int buf_len; /* how many chars stored in buf */
    char bufout[MSGBUF_LEN]; /* buffer which can be printed out */
    int bufout_len;
    int state = 0; /* 0 - waiting for code, 1 - copy msg to stdout */
    int i;
    int end = 0; /* end of messages? */

    bufout_len = 0;
    while((buf_len = read(fd, buf, BUF_LEN)) > 0 && !end){
        for (i=0; i < buf_len; i++){

            /* Prepare message for printing out */
            if (state == 1 || state == 2){
                if (bufout_len < MSGBUF_LEN)
                    bufout[bufout_len++] = buf[i];
            }

            /* reset state on '\n' in msg */
            if (buf[i] == '\n'){
                /* copy messages out */
                if (state == 1)
                    write(1, bufout, bufout_len);
                if (state == 2)
                    write(2, bufout, bufout_len);

                state = 0;
                bufout_len = 0;
                continue;
            }

            if (state == 0){
                if (buf[i] == CHECK_FAILED){
                    cu_fail_checks++;
                    state = 2;
                }else if (buf[i] == TEST_NAME){
                    state = 1;
                }else if (buf[i] == CHECK_SUCCEED){
                    cu_success_checks++;
                }else if (buf[i] == TEST_FAILED){
                    cu_fail_tests++;
                }else if (buf[i] == TEST_SUCCEED){
                    cu_success_tests++;
                }else if (buf[i] == TEST_SUITE_FAILED){
                    cu_fail_test_suites++;
                }else if (buf[i] == TEST_SUITE_SUCCEED){
                    cu_success_test_suites++;
                }else if (buf[i] == END){
                    end = 1;
                    break;
                }
            }
        }
    }
}

void cu_success_assertation(void)
{
    MSG_CHECK_SUCCEED;
}

void cu_fail_assertation(const char *file, int line, const char *msg)
{
    char buf[MSGBUF_LEN];
    int len;

    len = snprintf(buf, MSGBUF_LEN, "%c%s:%d (%s::%s) :: %s\n",
            CHECK_FAILED,
            file, line, cu_current_test_suite, cu_current_test, msg);
    write(fd, buf, len);

    /* enable test_failed flag */
    test_failed = 1;
}

static void cu_print_results(void)
{
    fprintf(stdout, "\n");
    fprintf(stdout, "==================================================\n");
    fprintf(stdout, "|               |  failed  |  succeed  |  total  |\n");
    fprintf(stdout, "|------------------------------------------------|\n");
    fprintf(stdout, "| assertations: |  %6d  |  %7d  |  %5d  |\n",
                cu_fail_checks, cu_success_checks,
                cu_success_checks+cu_fail_checks);
    fprintf(stdout, "| tests:        |  %6d  |  %7d  |  %5d  |\n",
                cu_fail_tests, cu_success_tests,
                cu_success_tests+cu_fail_tests);
    fprintf(stdout, "| tests suites: |  %6d  |  %7d  |  %5d  |\n",
                cu_fail_test_suites, cu_success_test_suites,
                cu_success_test_suites+cu_fail_test_suites);
    fprintf(stdout, "==================================================\n");
}

void cu_set_out_prefix(const char *str)
{
    strncpy(cu_out_prefix, str, CU_OUT_PREFIX_LENGTH);
}

static void redirect_out_err(const char *test_name)
{
    char buf[100];

    snprintf(buf, 99, "%stmp.%s.out", cu_out_prefix, test_name);
    if (freopen(buf, "w", stdout) == NULL){
        perror("Redirecting of stdout failed");
        exit(-1);
    }

    snprintf(buf, 99, "%stmp.%s.err", cu_out_prefix, test_name);
    if (freopen(buf, "w", stderr) == NULL){
        perror("Redirecting of stderr failed");
        exit(-1);
    }
}

static void close_out_err(void)
{
    fclose(stdout);
    fclose(stderr);
}

#ifdef CU_ENABLE_TIMER
/* global variables for timer functions */
struct timespec __cu_timer;
static struct timespec __cu_timer_start, __cu_timer_stop;

const struct timespec *cuTimer(void)
{
    return &__cu_timer;
}

void cuTimerStart(void)
{
    clock_gettime(CLOCK_MONOTONIC, &__cu_timer_start);
}

const struct timespec *cuTimerStop(void)
{
    clock_gettime(CLOCK_MONOTONIC, &__cu_timer_stop);

    /* store into t difference between time_start and time_end */
    if (__cu_timer_stop.tv_nsec > __cu_timer_start.tv_nsec){
        __cu_timer.tv_nsec = __cu_timer_stop.tv_nsec - __cu_timer_start.tv_nsec;
        __cu_timer.tv_sec = __cu_timer_stop.tv_sec - __cu_timer_start.tv_sec;
    }else{
        __cu_timer.tv_nsec = __cu_timer_stop.tv_nsec + 1000000000L - __cu_timer_start.tv_nsec;
        __cu_timer.tv_sec = __cu_timer_stop.tv_sec - 1 - __cu_timer_start.tv_sec;
    }

    return &__cu_timer;
}
#endif /* CU_ENABLE_TIMER */
