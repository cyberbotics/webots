#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 16
#define MULTIPLIER 2

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  // Test: acceleration paramter load. Note: all motors have the same multiplier.
  // acceleration on file [5, 10, 20] -> [5, 5, 5] after loading
  // same multiplier, different limits => the value of the first motor is enforced for all
  for (int i = 0; i < 3; ++i) {
    WbDeviceTag motor = wb_robot_get_device_by_index(i);
    ts_assert_double_equal(wb_motor_get_acceleration(motor), 5, "Motor A: 'acceleration' should be 5 but isn't.");
  }
  // acceleration on file [-1, 10, 20] -> [-1, -1, -1] after loading
  // same multiplier, the first is unlimited => all must be unlimited
  for (int i = 3; i < 6; ++i) {
    WbDeviceTag motor = wb_robot_get_device_by_index(i);
    ts_assert_double_equal(wb_motor_get_acceleration(motor), -1, "Motor B: 'acceleration' should be -1 but isn't.");
  }
  // Test loading velocity parameters. All motors have the same multiplier.
  // maxVelocity on file [5, 10, 20] -> [5, 5, 5] after loading
  // same multiplier, different limits => the value of the first motor is enforced for all
  for (int i = 6; i < 9; ++i) {
    WbDeviceTag motor = wb_robot_get_device_by_index(i);
    ts_assert_double_equal(wb_motor_get_velocity(motor), 5, "Motor C: 'maxVelocity' should be 5 but isn't.");
  }
  // Test loading minPosition and maxPosition parameters. All motors have the same multiplier.
  // minPosition on file [0, -10, -20] -> [0, 0, 0] after loading
  // maxPosition on file [0, 20, 40] -> [0, 0, 0] after loading
  // same multiplier, the first is unlimited => all must be unlimited
  for (int i = 9; i < 12; ++i) {
    WbDeviceTag motor = wb_robot_get_device_by_index(i);
    ts_assert_double_equal(wb_motor_get_min_position(motor), 0, "Motor D: 'minPosition' should be 0 but isn't.");
    ts_assert_double_equal(wb_motor_get_max_position(motor), 0, "Motor D: 'maxPosition' should be 0 but isn't.");
  }
  // minPosition on file [-5, -10, 0] -> [-5, -5, -5] after loading
  // maxPosition on file [10, 20, 0] -> [10, 10, 10] after loading
  // same multiplier, different limits => the value of the first motor is enforced for all
  for (int i = 12; i < 15; ++i) {
    WbDeviceTag motor = wb_robot_get_device_by_index(i);
    ts_assert_double_equal(wb_motor_get_min_position(motor), -5, "Motor E: 'minPosition' should be -5 but isn't.");
    ts_assert_double_equal(wb_motor_get_max_position(motor), 10, "Motor E: 'maxPosition' should be 10 but isn't.");
  }
  // Test loading maxTorque parameters. All motors have the same multiplier.
  // maxTorque on file [5, 10, 20] -> [5, 5, 5] after loading
  // same multiplier, different limits => the value of the first motor is enforced for all
  for (int i = 15; i < 18; ++i) {
    WbDeviceTag motor = wb_robot_get_device_by_index(i);
    ts_assert_double_equal(wb_motor_get_max_torque(motor), 5, "Motor F: 'maxTorque' should be 5 but isn't.");
  }
  // Test multiplier.
  // multipliers on file [2, 0.5, 4] (which is [motor G1, motor G2, motor G3])
  // minPosition on file [-2, -2, -2] -> [-2, -0.5, -4] after loading
  // maxPosition on file [4, 4, 4] -> [4, 1, 8] after loading
  // maxVelocity on file [10, 10, 10] -> [40, 10, 80] after loading
  // maxAccel. on file   [10, 10, 10] -> [40, 10, 80] after loading
  // maxTorque on file   [10, 10, 10] -> [20, 80, 10] after loading
  WbDeviceTag motor = wb_robot_get_device_by_index(18);
  ts_assert_double_equal(wb_motor_get_min_position(motor), -2, "Motor G: 'minPosition' should be -2 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_position(motor), 4, "Motor G: 'maxPosition' should be 4 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_velocity(motor), 10, "Motor G: 'maxVelocity' should be 10 but isn't.");
  ts_assert_double_equal(wb_motor_get_acceleration(motor), 10, "Motor G: 'acceleration' should be 10 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_torque(motor), 10, "Motor G: 'maxTorque' should be 10 but isn't.");
  motor = wb_robot_get_device_by_index(19);
  ts_assert_double_equal(wb_motor_get_min_position(motor), -0.5, "Motor G: 'minPosition' should be -0.5 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_position(motor), 1, "Motor G: 'maxPosition' should be 1 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_velocity(motor), 2.5, "Motor G: 'maxVelocity' should be 2.5 but isn't.");
  ts_assert_double_equal(wb_motor_get_acceleration(motor), 2.5, "Motor G: 'acceleration' should be 2.5 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_torque(motor), 40, "Motor G: 'maxTorque' should be 40 but isn't.");
  motor = wb_robot_get_device_by_index(20);
  ts_assert_double_equal(wb_motor_get_min_position(motor), -4, "Motor G: 'minPosition' should be -4 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_position(motor), 8, "Motor G: 'maxPosition' should be 8 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_velocity(motor), 20, "Motor G: 'maxVelocity' should be 20 but isn't.");
  ts_assert_double_equal(wb_motor_get_acceleration(motor), 20, "Motor G: 'acceleration' should be 20 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_torque(motor), 5, "Motor G: 'maxTorque' should be 5 but isn't.");
  // multipliers on file [2, -0.5, -4] (which is [motor H1, motor H2, motor H3])
  // minPosition on file [-2, -2, -2] -> [-2, -1, -8] after loading. Note: [4, -8] and [0.5, -1] have swapped position
  // maxPosition on file [4, 4, 4] -> [4, 0.5, 4] after loading. Note: [4, -8] and [0.5, -1] have swapped position
  // maxVelocity on file [10, 10, 10] -> [10, 2.5, 20] after loading
  // maxAccel. on file   [10, 10, 10] -> [10, 2.5, 20] after loading
  // maxTorque on file   [10, 10, 10] -> [10, 40, 5] after loading
  motor = wb_robot_get_device_by_index(21);
  ts_assert_double_equal(wb_motor_get_min_position(motor), -2, "Motor H: 'minPosition' should be -2 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_position(motor), 4, "Motor H: 'maxPosition' should be 4 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_velocity(motor), 10, "Motor H: 'maxVelocity' should be 10 but isn't.");
  ts_assert_double_equal(wb_motor_get_acceleration(motor), 10, "Motor H: 'acceleration' should be 10 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_torque(motor), 10, "Motor H: 'maxTorque' should be 10 but isn't.");
  motor = wb_robot_get_device_by_index(22);
  ts_assert_double_equal(wb_motor_get_min_position(motor), -1, "Motor H: 'minPosition' should be -1 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_position(motor), 0.5, "Motor H: 'maxPosition' should be 0.5 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_velocity(motor), 2.5, "Motor H: 'maxVelocity' should be 2.5 but isn't.");
  ts_assert_double_equal(wb_motor_get_acceleration(motor), 2.5, "Motor H: 'acceleration' should be 2.5 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_torque(motor), 40, "Motor H: 'maxTorque' should be 40 but isn't.");
  motor = wb_robot_get_device_by_index(23);
  ts_assert_double_equal(wb_motor_get_min_position(motor), -8, "Motor H: 'minPosition' should be -8 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_position(motor), 4, "Motor H: 'maxPosition' should be 4 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_velocity(motor), 20, "Motor H: 'maxVelocity' should be 20 but isn't.");
  ts_assert_double_equal(wb_motor_get_acceleration(motor), 20, "Motor H: 'acceleration' should be 20 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_torque(motor), 5, "Motor H: 'maxTorque' should be 5 but isn't.");

  ts_send_success();
  return EXIT_SUCCESS;
}
