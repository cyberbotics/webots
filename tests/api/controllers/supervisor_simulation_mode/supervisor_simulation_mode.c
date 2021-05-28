#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <sys/time.h>
#include <unistd.h>

// perform a safe step
static void step(int time_step) {
  if (wb_robot_step(time_step) == -1)
    ts_assert_boolean_equal(0, "Controller stopped before having being able to conclude.");
}

// tools to measure a time interval in milliseconds
// note: a single time interval can be measured at the same time
static struct timeval start_time;

static void start_chrono() {
  gettimeofday(&start_time, NULL);
}

static unsigned long long stop_chrono() {
  struct timeval end_time;
  gettimeofday(&end_time, NULL);
  return (unsigned long long)1000 * (end_time.tv_sec - start_time.tv_sec) + (end_time.tv_usec - start_time.tv_usec) / 1000;
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int initial_mode = wb_supervisor_simulation_get_mode();
  printf("Initial mode=%d\n", initial_mode);

  int i;
  unsigned long long time_counter;
  double time_before_pause, time_after_pause;
  const int n_steps = 100;
  int time_step = wb_robot_get_basic_time_step();

  // measure mean time to perform n_steps simulation steps in fast mode
  wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_FAST);

  start_chrono();
  for (i = 0; i < n_steps; ++i)
    step(time_step);
  time_counter = stop_chrono();

  double mean_step_time_in_fast_mode = (double)time_counter / n_steps;
  printf("Mean step time in fast mode = %g\n", mean_step_time_in_fast_mode);

  ts_assert_double_in_delta(wb_robot_get_time(), 0.001 * time_step * n_steps, 0.001,
                            "Unexpected simulation time at the end of the fast mode test (found = %g).", wb_robot_get_time());

  // measure mean time to perform n_steps simulation steps in real-time mode
  wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_REAL_TIME);

  start_chrono();
  for (i = 0; i < n_steps; ++i)
    step(time_step);
  time_counter = stop_chrono();

  double mean_step_time_in_real_time_mode = (double)time_counter / n_steps;
  printf("Mean step time in real-time mode = %g\n", mean_step_time_in_real_time_mode);

  ts_assert_double_in_delta(wb_robot_get_time(), 0.001 * time_step * n_steps * 2, 0.001,
                            "Unexpected simulation time at the end of the real-time mode test (found = %g).",
                            wb_robot_get_time());

  // make sure that the real-time mode is taking more time than the fast mode
  ts_assert_boolean_equal(
    mean_step_time_in_fast_mode < mean_step_time_in_real_time_mode,
    "The mean time to perform steps in real-time mode (%g ms) is taking less time than in fast mode (%g ms).",
    mean_step_time_in_real_time_mode, mean_step_time_in_fast_mode);

  // pause the simulation, wait one second and resume the simulation.
  // check that the simulation time has not increased
  // and that the simulation has been indeed paused
  time_before_pause = wb_robot_get_time();
  start_chrono();
  wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
  sleep(1);  // wait one second
  wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_FAST);
  time_counter = stop_chrono();
  time_after_pause = wb_robot_get_time();

  ts_assert_double_equal(
    time_before_pause, time_after_pause,
    "A: The simulation time before pausing the simulator (%g s) and when resuming the simulator (%g s) is differing.",
    time_before_pause, time_after_pause);
  ts_assert_boolean_equal(time_counter >= 1000, "The time spent in the sleep is smaller than one second (%llu ms).",
                          time_counter);

  // test that pausing the simulator and calling robot step
  // with a 0 argument doesn't affect the simulation time
  // (e.g., the simulation time is not increasing)
  time_before_pause = wb_robot_get_time();
  wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
  step(0);
  wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_FAST);
  time_after_pause = wb_robot_get_time();

  ts_assert_double_equal(
    time_before_pause, time_after_pause,
    "B: The simulation time before pausing the simulator (%g s) and when resuming the simulator (%g s) is differing.",
    time_before_pause, time_after_pause);

  // restore the initial mode in order to go on the test suite
  // at the startup speed
  printf("Restore mode to %d\n", initial_mode);
  wb_supervisor_simulation_set_mode(initial_mode);

  // Note: if something is blocking, then the test suite time out mechanism should be raised
  ts_send_success();
  return EXIT_SUCCESS;
}
