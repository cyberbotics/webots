#include <webots/gps.h>
#include <webots/robot.h>
#include <webots/utils/motion.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <math.h>

#define TIME_STEP 40

static void wait_s(double time) {
  double starts = wb_robot_get_time();
  while (wb_robot_get_time() - starts < time)
    wb_robot_step(TIME_STEP);
}

static double distance(const double *v1, const double *v2) {
  return sqrt((v1[0] - v2[0]) * (v1[0] - v2[0]) + (v1[1] - v2[1]) * (v1[1] - v2[1]) + (v1[2] - v2[2]) * (v1[2] - v2[2]));
}

int main(int argc, char **argv) {
  bool regular = (strcmp(argv[1], "regular") == 0);
  bool loop = (strcmp(argv[1], "loop") == 0);
  bool reverse = (strcmp(argv[1], "reverse") == 0);

  char name[256];
  snprintf(name, sizeof(name), "motion_%s", argv[1]);
  ts_setup(name);

  ts_assert_int_equal(argc, 2, "Wrong arguments size");

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  wait_s(1.0);  // stablize the robot before playing the motion

  WbMotionRef forwards = wbu_motion_new("../../motions/HexapodForwards.motion");

  double expectedPosition[3];

  if (regular) {
    // Note (David): calibrated on my Ubuntu 64 with ODE MT
    expectedPosition[0] = -0.278282;
    expectedPosition[1] = 0.079743;
    expectedPosition[2] = -0.002273;
  } else if (loop) {
    // Note (David): calibrated on my Ubuntu 64 with ODE MT
    expectedPosition[0] = -0.539533;
    expectedPosition[1] = 0.079410;
    expectedPosition[2] = -0.011085;
    wbu_motion_set_loop(forwards, true);
  } else if (reverse) {
    // Note (David): calibrated on my Ubuntu 64 with ODE MT
    expectedPosition[0] = 0.278282;
    expectedPosition[1] = 0.079743;
    expectedPosition[2] = -0.002273;
    wbu_motion_set_reverse(forwards, true);
  } else {
    expectedPosition[0] = 0;
    expectedPosition[1] = 0;
    expectedPosition[2] = 0;
    ts_assert_boolean_equal(false, "Unexpected argument");
  }

  wbu_motion_play(forwards);

  if (loop) {
    double startTime = wb_robot_get_time();
    do
      wb_robot_step(TIME_STEP);
    while (wb_robot_get_time() < startTime + 0.002 * wbu_motion_get_duration(forwards));
    wbu_motion_stop(forwards);
  } else {
    do
      wb_robot_step(TIME_STEP);
    while (!wbu_motion_is_over(forwards));
  }
  wait_s(1.0);  // stabilize the robot before playing the motion

  const double *position = wb_gps_get_values(gps);
  double err = distance(position, expectedPosition);
  printf("pos = %f %f %f\n", position[0], position[1], position[2]);
  printf("expected = %f %f %f\n", expectedPosition[0], expectedPosition[1], expectedPosition[2]);
  printf("Error = %f\n", err);
  ts_assert_boolean_equal(err < 0.0001, "Unexpected target position (error = %g)", err);

  ts_send_success();
  return EXIT_SUCCESS;
}
