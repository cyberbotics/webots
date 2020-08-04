#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define OBJ_VALUE 300.0
#define NONE_VALUE 0.0

static void check_sensor_value(WbDeviceTag tag, double expected, const char *message) {
  double value = wb_distance_sensor_get_value(tag);
  ts_assert_double_in_delta(value, expected, 0.005, message, expected, value);
}

int main(int argc, char **argv) {
  ts_setup(argv[1]);

  int time_step = wb_robot_get_basic_time_step();
  const char *robot_name = wb_robot_get_name();
  bool dynamic = true;
  if (strcmp(robot_name, "static") == 0)
    dynamic = false;

  WbDeviceTag ds_generic_s = wb_robot_get_device("ds_generic_static");
  WbDeviceTag ds_infra_red_s = wb_robot_get_device("ds_infra_red_static");
  WbDeviceTag ds_generic_d = wb_robot_get_device("ds_generic_dynamic");
  WbDeviceTag ds_infra_red_d = wb_robot_get_device("ds_infra_red_dynamic");
  wb_distance_sensor_enable(ds_generic_s, time_step);
  wb_distance_sensor_enable(ds_infra_red_s, time_step);
  wb_distance_sensor_enable(ds_generic_d, time_step);
  wb_distance_sensor_enable(ds_infra_red_d, time_step);

  // stabilize the system
  wb_robot_step(3 * time_step);

  if (dynamic) {
    // move sensors during ODE physics step
    WbDeviceTag left_motor = wb_robot_get_device("left motor");
    WbDeviceTag right_motor = wb_robot_get_device("right motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 20);
    wb_motor_set_velocity(right_motor, 20);
  }

  // static object

  // generic (using ODE rays)
  check_sensor_value(ds_generic_s, OBJ_VALUE,
                     "The generic distance sensor doesn't return "
                     "the correct initial distance of static object: "
                     "expected %f, measured %f.");

  // infra-red (using Webots ray collision detection)
  check_sensor_value(ds_infra_red_s, OBJ_VALUE,
                     "The infra-red distance sensor doesn't return "
                     "the correct initial distance of static object: "
                     "expected %f, measured %f.");

  // dynamic object

  // generic (using ODE rays)
  check_sensor_value(ds_generic_d, OBJ_VALUE,
                     "The generic distance sensor doesn't return "
                     "the correct initial distance of dynamic object: "
                     "expected %f, measured %f.");

  // infra-red (using Webots ray collision detection)
  check_sensor_value(ds_infra_red_d, OBJ_VALUE,
                     "The infra-red distance sensor doesn't return "
                     "the correct initial distance of dynamic object: "
                     "expected %f, measured %f.");

  // check ray position has been update before ray collision detection
  // when the robot moves
  wb_robot_step(time_step);

  // static object

  // generic (using ODE rays)
  check_sensor_value(ds_generic_s, OBJ_VALUE,
                     "The generic distance sensor doesn't return "
                     "the correct distance of static object after 1 step: "
                     "expected %f, measured %f.");

  // infra-red (using Webots ray collision detection)
  check_sensor_value(ds_infra_red_s, OBJ_VALUE,
                     "The infra-red distance sensor doesn't return "
                     "the correct distance of static object after 1 step: "
                     "expected %f, measured %f.");

  // dynamic object

  // generic (using ODE rays)
  check_sensor_value(ds_generic_d, OBJ_VALUE,
                     "The generic distance sensor doesn't return "
                     "the correct distance of dynamic object after 1 step: "
                     "expected %f, measured %f.");

  // infra-red (using Webots ray collision detection)
  check_sensor_value(ds_infra_red_d, OBJ_VALUE,
                     "The infra-red distance sensor doesn't return "
                     "the correct distance of dynamic object after 1 step: "
                     "expected %f, measured %f.");

  wb_robot_step(time_step);

  // static object

  // generic (using ODE rays)
  double staticExpected = OBJ_VALUE;
  if (dynamic)
    staticExpected = NONE_VALUE;
  check_sensor_value(ds_generic_s, staticExpected,
                     "The generic distance sensor doesn't return "
                     "the correct distance of static object after 2 steps: "
                     "expected %f, measured %f.");

  // infra-red (using Webots ray collision detection)
  check_sensor_value(ds_infra_red_s, staticExpected,
                     "The infra-red distance sensor doesn't return "
                     "the correct distance of static object after 2 steps: "
                     "expected %f, measured %f.");

  // dynamic object

  // generic (using ODE rays)
  check_sensor_value(ds_generic_d, NONE_VALUE,
                     "The generic distance sensor doesn't return "
                     "the correct distance of dynamic object after 2 steps: "
                     "expected %f, measured %f.");

  // infra-red (using Webots ray collision detection)
  check_sensor_value(ds_infra_red_d, NONE_VALUE,
                     "The infra-red distance sensor doesn't return "
                     "the correct distance of dynamic object after 2 steps: "
                     "expected %f, measured %f.");

  ts_send_success();
  return EXIT_SUCCESS;
}
