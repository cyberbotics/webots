#include <webots/keyboard.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#ifdef _WIN32
#include <time.h>
#else
#include <sys/time.h>
#endif

#define TIMEOUT 2000

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

#ifdef _WIN32
  clock_t start = clock();
#else
  struct timeval t1, t2;
  gettimeofday(&t1, NULL);
#endif

  int event_type = wb_robot_wait_for_user_input_event(WB_EVENT_KEYBOARD, TIMEOUT);
  ts_assert_int_equal(event_type, WB_EVENT_NO_EVENT, "wb_robot_wait_for_user_input_event should return WB_EVENT_NO_EVENT.");

#ifdef _WIN32
  clock_t end = clock();
  double elapsedTime = ((double)(end - start)) / CLOCKS_PER_SEC;
#else
  gettimeofday(&t2, NULL);
  double elapsedTime = (t2.tv_sec - t1.tv_sec);
  elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000000.0;
#endif

  ts_assert_double_in_delta(
    elapsedTime, 0.0, 0.01,
    "wb_robot_wait_for_user_input_event should return immediatly when the keyboard is not enabled (and not after %lfsec).",
    elapsedTime);

  wb_keyboard_enable(TIME_STEP);

#ifdef _WIN32
  start = clock();
#else
  gettimeofday(&t1, NULL);
#endif
  event_type = wb_robot_wait_for_user_input_event(WB_EVENT_KEYBOARD, TIMEOUT);
  ts_assert_int_equal(event_type, WB_EVENT_NO_EVENT, "wb_robot_wait_for_user_input_event should return WB_EVENT_NO_EVENT.");

#ifdef _WIN32
  end = clock();
  elapsedTime = ((double)(end - start)) / CLOCKS_PER_SEC;
#else
  gettimeofday(&t2, NULL);
  elapsedTime = (t2.tv_sec - t1.tv_sec);
  elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000000.0;
#endif

  ts_assert_double_in_delta(
    elapsedTime, 0.001 * TIMEOUT + 0.1, 0.3,
    "wb_robot_wait_for_user_input_event should return after 2sec when the keyboard is enabled (and not after %lfsec).",
    elapsedTime);

  ts_send_success();
  return EXIT_SUCCESS;
}
