#include <webots/device.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

void test_get_motor(char *sensor_name, char *expected_motor_name) {
  WbDeviceTag position_sensor = wb_robot_get_device(sensor_name);
  const char *actual_sensor_name = wb_device_get_name(position_sensor);
  ts_assert_string_equal(actual_sensor_name, sensor_name, "wb_device_get_name(wb_robot_get_device(\"%s\")) returned \"%s\"",
                         sensor_name, actual_sensor_name);
  WbDeviceTag motor = wb_position_sensor_get_motor(position_sensor);
  const char *motor_name = wb_device_get_name(motor);
  ts_assert_string_equal(
    motor_name, expected_motor_name,
    "wb_device_get_name(wb_position_sensor_get_motor(wb_robot_get_device(\"%s\"))) returned \"%s\" instead of \"%s\"",
    sensor_name, motor_name, expected_motor_name);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  wb_robot_step(TIME_STEP);

  test_get_motor("position sensor1", "rotational motor1");
  test_get_motor("position sensor2", "rotational motor2");
  test_get_motor("position sensor3", "rotational motor3");

  // position sensor3 does not have an associated brake. Make sure we don't get
  // the brake associated with the first position sensor instead.
  WbDeviceTag brake = wb_position_sensor_get_brake(wb_robot_get_device("position sensor3"));
  ts_assert_int_equal(brake, 0, "wb_position_sensor_get_brake(wb_robot_get_device(\"position sensor3\")) != NULL");

  ts_send_success();
  return EXIT_SUCCESS;
}
