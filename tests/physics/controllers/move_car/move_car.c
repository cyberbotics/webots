#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  int time_step = (int)wb_robot_get_basic_time_step();

  WbDeviceTag right_sensor = wb_robot_get_device("right sensor");
  WbDeviceTag left_sensor = wb_robot_get_device("left sensor");
  wb_distance_sensor_enable(right_sensor, time_step);
  wb_distance_sensor_enable(left_sensor, time_step);

  WbDeviceTag motors[2];
  motors[0] = wb_robot_get_device("right_front_wheel");
  motors[1] = wb_robot_get_device("left_front_wheel");
  int i;
  for (i = 0; i < 2; ++i) {
    wb_motor_set_velocity(motors[i], 4.0);
    wb_motor_set_position(motors[i], 1000000);
  }

  WbDeviceTag left_steer_motor = wb_robot_get_device("left_steer");
  WbDeviceTag right_steer_motor = wb_robot_get_device("right_steer");
  wb_motor_set_position(left_steer_motor, 0.25);
  wb_motor_set_position(right_steer_motor, 0.25);

  wb_robot_step(5 * time_step);

  int steps = 15;
  while (steps > 0 && wb_robot_step(time_step) != -1) {
    ts_assert_double_is_bigger(wb_distance_sensor_get_value(right_sensor), 999.0,
                               "Right front wheel joint axis or anchor is wrong after artificial move.");
    ts_assert_double_is_bigger(wb_distance_sensor_get_value(left_sensor), 999.0,
                               "Left front wheel joint axis or anchor is wrong after artificial move.");
    --steps;
  }

  wb_robot_step(10 * time_step);

  steps = 100;
  while (steps > 0 && wb_robot_step(time_step) != -1) {
    ts_assert_double_is_bigger(wb_distance_sensor_get_value(right_sensor), 999.0,
                               "Right front wheel joint axis or anchor is wrong after artificial move.");
    ts_assert_double_is_bigger(wb_distance_sensor_get_value(left_sensor), 999.0,
                               "Left front wheel joint axis or anchor is wrong after artificial move.");
    --steps;
  }

  ts_assert_int_equal(steps, 0, "Simulation terminated before completing all the tests after artificial move.");

  ts_send_success();
  return EXIT_SUCCESS;
}
