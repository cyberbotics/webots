#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 16
#define MULTIPLIER 2

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  // Test loading acceleration parameters. All motors have the same multiplier.
  // acceleration on file [5, 10, 20] -> [20, 20, 20] after loading
  for (int i = 0; i < 3; ++i) {
    WbDeviceTag motor = wb_robot_get_device_by_index(i);
    ts_assert_double_equal(wb_motor_get_acceleration(motor), 20, "Motor A: 'acceleration' should be 20 but isn't.");
  }
  // Test loading velocity parameters. All motors have the same multiplier.
  // maxVelocity on file [5, 10, 20] -> [20, 20, 20] after loading
  for (int i = 3; i < 6; ++i) {
    WbDeviceTag motor = wb_robot_get_device_by_index(i);
    ts_assert_double_equal(wb_motor_get_velocity(motor), 20, "Motor B: 'maxVelocity' should be 20 but isn't.");
  }
  // Test loading minPosition and maxPosition parameters. All motors have the same multiplier.
  // minPosition on file [-5, -10, -20] -> [-20, -20, -20] after loading
  // maxPosition on file [10, 20, 40] -> [40, 40, 40] after loading
  for (int i = 6; i < 9; ++i) {
    WbDeviceTag motor = wb_robot_get_device_by_index(i);
    ts_assert_double_equal(wb_motor_get_min_position(motor), -20, "Motor C: 'minPosition' should be -20 but isn't.");
    ts_assert_double_equal(wb_motor_get_max_position(motor), 40, "Motor C: 'maxPosition' should be 40 but isn't.");
  }
  // Test loading maxTorque parameters. All motors have the same multiplier.
  // maxTorque on file [5, 10, 20] -> [20, 20, 20] after loading
  for (int i = 9; i < 12; ++i) {
    WbDeviceTag motor = wb_robot_get_device_by_index(i);
    ts_assert_double_equal(wb_motor_get_max_torque(motor), 20, "Motor D: 'maxTorque' should be 20 but isn't.");
  }
  // Test multiplier.
  // multipliers on file [2, 0.5, 4] (which is [motor E1, motor E2, motor E3])
  // minPosition on file [-1, -1, -1] -> [-4, -1, -8] after loading
  // maxVelocity on file [10, 10, 10] -> [40, 10, 80] after loading
  // maxAccel. on file   [10, 10, 10] -> [40, 10, 80] after loading
  // maxTorque on file   [10, 10, 10] -> [20, 80, 10] after loading
  WbDeviceTag motor = wb_robot_get_device_by_index(12);
  ts_assert_double_equal(wb_motor_get_min_position(motor), -4, "Motor E: 'minPosition' should be -4 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_velocity(motor), 40, "Motor E: 'maxVelocity' should be 40 but isn't.");
  ts_assert_double_equal(wb_motor_get_acceleration(motor), 40, "Motor E: 'acceleration' should be 40 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_torque(motor), 20, "Motor E: 'maxTorque' should be 20 but isn't.");
  motor = wb_robot_get_device_by_index(13);
  ts_assert_double_equal(wb_motor_get_min_position(motor), -1, "Motor E: 'minPosition' should be -1 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_velocity(motor), 10, "Motor E: 'maxVelocity' should be 10 but isn't.");
  ts_assert_double_equal(wb_motor_get_acceleration(motor), 10, "Motor E: 'acceleration' should be 10 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_torque(motor), 80, "Motor E: 'maxTorque' should be 80 but isn't.");
  motor = wb_robot_get_device_by_index(14);
  ts_assert_double_equal(wb_motor_get_min_position(motor), -8, "Motor E: 'minPosition' should be -8 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_velocity(motor), 80, "Motor E: 'maxVelocity' should be 80 but isn't.");
  ts_assert_double_equal(wb_motor_get_acceleration(motor), 80, "Motor E: 'acceleration' should be 80 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_torque(motor), 10, "Motor E: 'maxTorque' should be 10 but isn't.");

  ts_send_success();
  return EXIT_SUCCESS;
}
