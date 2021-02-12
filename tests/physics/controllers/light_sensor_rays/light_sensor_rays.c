#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

static void check_sensor_value(WbDeviceTag tag, double expected, const char *message) {
  double value = wb_light_sensor_get_value(tag);
  ts_assert_double_in_delta(value, expected, 1.0, message, expected, value);
}

int main(int argc, char **argv) {
  ts_setup(argv[1]);

  double expectedStaticValue = 0.0;
  double expectedDynamicValue = 0.0;
  int time_step = wb_robot_get_basic_time_step();
  const char *robot_name = wb_robot_get_name();
  bool dynamic = true;
  if (strcmp(robot_name, "static") == 0)
    dynamic = false;

  WbDeviceTag ls_s = wb_robot_get_device("light sensor static");
  WbDeviceTag ls_d = wb_robot_get_device("light sensor dynamic");
  wb_light_sensor_enable(ls_s, time_step);
  wb_light_sensor_enable(ls_d, time_step);

  // initialize motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);

  // stabilize the system
  wb_robot_step(5 * time_step);

  if (dynamic) {
    // move sensors during ODE physics step
    wb_motor_set_velocity(left_motor, 20);
    wb_motor_set_velocity(right_motor, 20);
  }

  // static object
  if (dynamic)
    expectedStaticValue = 999.0;
  else
    expectedStaticValue = 999.0;
  check_sensor_value(ls_s, expectedStaticValue,
                     "The light sensor doesn't return "
                     "the correct initial measurement of static light: "
                     "expected %f, measured %f.");

  // dynamic object
  if (dynamic)
    expectedDynamicValue = 958.0;
  check_sensor_value(ls_d, expectedDynamicValue,
                     "The light sensor doesn't return "
                     "the correct initial measurement of dynamic light: "
                     "expected %f, measured %f.");

  // check ray position has been update before ray collision detection
  // when the robot moves
  wb_robot_step(time_step);

  // static object
  if (dynamic)
    expectedStaticValue = 0.0;
  check_sensor_value(ls_s, expectedStaticValue,
                     "The light sensor doesn't return "
                     "the correct measurement of static light after 1 step: "
                     "expected %f, measured %f.");

  // dynamic object
  if (dynamic)
    expectedDynamicValue = 841.0;
  else
    expectedDynamicValue = 208.0;
  check_sensor_value(ls_d, expectedDynamicValue,
                     "The light sensor doesn't return "
                     "the correct measurement of dynamic light after 1 step: "
                     "expected %f, measured %f.");

  wb_robot_step(time_step);

  // static object
  double staticExpected = 999.0;
  if (dynamic)
    staticExpected = 0.0;
  check_sensor_value(ls_s, staticExpected,
                     "The light sensor doesn't return "
                     "the correct measurement of static light after 2 steps: "
                     "expected %f, measured %f.");

  // dynamic object
  if (dynamic)
    expectedDynamicValue = 737.0;
  else
    expectedDynamicValue = 707.0;
  check_sensor_value(ls_d, expectedDynamicValue,
                     "The light sensor doesn't return "
                     "the correct measurement of dynamic light after 2 steps: "
                     "expected %f, measured %f.");

  ts_send_success();
  return EXIT_SUCCESS;
}
