#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  const WbDeviceTag motor = wb_robot_get_device("rotational motor");
  const WbDeviceTag position_sensor = wb_robot_get_device("position sensor");
  const int TIME_STEP = wb_robot_get_basic_time_step();

  wb_position_sensor_enable(position_sensor, TIME_STEP);
  wb_robot_battery_sensor_enable(TIME_STEP);
  wb_motor_set_position(motor, 2 * M_PI);
  wb_robot_step(TIME_STEP);
  double battery_level = wb_robot_battery_sensor_get_value();

  double theorical_battery_level = 504;
  ts_assert_double_in_delta(battery_level, theorical_battery_level, 1,
                            "The battery level measured by the battery sensor should be %g and not %g", theorical_battery_level,
                            battery_level);

  int i;
  for (i = 0; i < 10; ++i)
    wb_robot_step(TIME_STEP);

  // Because the robot should run out of battery in the middle of the previous loop,
  // the motor should be stopped and its position should match the one below.
  double position = wb_position_sensor_get_value(position_sensor);
  ts_assert_double_in_delta(position, 0.5739, 0.001,
                            "The position value measured by the position sensor should be 0.5739 and not %g", position);

  battery_level = wb_robot_battery_sensor_get_value();
  ts_assert_double_in_delta(battery_level, 0, 0.001, "The battery level measured by the battery sensor should be 0 and not %g",
                            battery_level);

  ts_send_success();
  return EXIT_SUCCESS;
}
