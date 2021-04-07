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
    ts_assert_double_equal(wb_motor_get_acceleration(motor), 20, "Loaded 'acceleration' should be 20 but isn't.");
  }
  // Test loading velocity parameters. All motors have the same multiplier.
  // maxVelocity on file [5, 10, 20] -> [20, 20, 20] after loading
  for (int i = 3; i < 6; ++i) {
    WbDeviceTag motor = wb_robot_get_device_by_index(i);
    ts_assert_double_equal(wb_motor_get_velocity(motor), 20, "Loaded 'maxVelocity' should be 20 but isn't.");
  }
  // Test loading minPosition and maxPosition parameters. All motors have the same multiplier.
  // minPosition on file [-5, -10, -20] -> [-20, -20, -20] after loading
  // maxPosition on file [10, 20, 40] -> [40, 40, 40] after loading
  for (int i = 6; i < 9; ++i) {
    WbDeviceTag motor = wb_robot_get_device_by_index(i);
    ts_assert_double_equal(wb_motor_get_min_position(motor), -20, "Loaded 'minPosition' should be -20 but isn't.");
    ts_assert_double_equal(wb_motor_get_max_position(motor), 40, "Loaded 'maxPosition' should be 40 but isn't.");
  }
  // Test loading maxTorque parameters. All motors have the same multiplier.
  // maxTorque on file [5, 10, 20] -> [20, 20, 20] after loading
  for (int i = 9; i < 12; ++i) {
    WbDeviceTag motor = wb_robot_get_device_by_index(i);
    ts_assert_double_equal(wb_motor_get_max_torque(motor), 20, "Loaded 'maxTorque' should be 20 but isn't.");
  }
  // Test multiplier.
  // multipliers on file [2, 0.5, 4]
  // minPosition on file [-1, -1, -1] -> [-4, -1, -8] after loading
  // maxVelocity on file [10, 10, 10] -> [40, 10, 80] after loading
  // maxAccel. on file   [10, 10, 10] -> [40, 10, 80] after loading
  // maxTorque on file   [10, 10, 10] -> [20, 80, 10] after loading
  WbDeviceTag motor = wb_robot_get_device_by_index(12);
  ts_assert_double_equal(wb_motor_get_min_position(motor), -20, "Loaded 'minPosition' should be -20 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_velocity(motor), 40, "Loaded 'maxVelocity' should be 40 but isn't.");
  ts_assert_double_equal(wb_motor_get_acceleration(motor), 40, "Loaded 'acceleration' should be 40 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_torque(motor), 20, "Loaded 'maxTorque' should be 20 but isn't.");
  motor = wb_robot_get_device_by_index(13);
  ts_assert_double_equal(wb_motor_get_max_velocity(motor), 10, "Loaded 'maxVelocity' should be 10 but isn't.");
  ts_assert_double_equal(wb_motor_get_acceleration(motor), 10, "Loaded 'acceleration' should be 10 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_torque(motor), 80, "Loaded 'maxTorque' should be 80 but isn't.");
  motor = wb_robot_get_device_by_index(14);
  ts_assert_double_equal(wb_motor_get_max_velocity(motor), 80, "Loaded 'maxVelocity' should be 80 but isn't.");
  ts_assert_double_equal(wb_motor_get_acceleration(motor), 80, "Loaded 'acceleration' should be 80 but isn't.");
  ts_assert_double_equal(wb_motor_get_max_torque(motor), 10, "Loaded 'maxTorque' should be 10 but isn't.");

  // Test position control
  WbDeviceTag reference_motor = wb_robot_get_device("reference motor");
  WbDeviceTag reference_sensor = wb_robot_get_device("reference sensor");
  WbDeviceTag positive_multiplier_sensor = wb_robot_get_device("positive multiplier sensor");
  WbDeviceTag negative_multiplier_sensor = wb_robot_get_device("negative multiplier sensor");
  wb_position_sensor_enable(reference_sensor, TIME_STEP);
  wb_position_sensor_enable(positive_multiplier_sensor, TIME_STEP);
  wb_position_sensor_enable(negative_multiplier_sensor, TIME_STEP);

  wb_motor_set_position(reference_motor, INFINITY);
  wb_motor_set_velocity(reference_motor, 0.2);
  for (int i = 0; i < 500; i++) {
    wb_robot_step(TIME_STEP);
    double position_reference = wb_position_sensor_get_value(reference_sensor);
    double position_positive_multiplier = wb_position_sensor_get_value(positive_multiplier_sensor);
    double position_negative_multiplier = wb_position_sensor_get_value(negative_multiplier_sensor);
    printf("%f %f %f\n", position_reference, position_positive_multiplier, position_negative_multiplier);
    ts_assert_double_in_delta(position_positive_multiplier, position_reference * MULTIPLIER, 1e-9,
                              "The position of the motor isn't %d times that of the reference", MULTIPLIER);
    ts_assert_double_in_delta(position_negative_multiplier, position_reference * (-MULTIPLIER), 1e-9,
                              "The position of the motor isn't %d times that of the reference", -MULTIPLIER);
  }

  wb_motor_set_position(reference_motor, 0);
  wb_motor_set_velocity(reference_motor, 1);

  for (int i = 0; i < 200; i++) {
    wb_robot_step(TIME_STEP);
    double position_reference = wb_position_sensor_get_value(reference_sensor);
    double position_positive_multiplier = wb_position_sensor_get_value(positive_multiplier_sensor);
    double position_negative_multiplier = wb_position_sensor_get_value(negative_multiplier_sensor);
    printf("%f %f %f\n", position_reference, position_positive_multiplier, position_negative_multiplier);
    /*
    ts_assert_double_in_delta(position_positive_multiplier, position_reference * MULTIPLIER, 1e-9,
                              "The position of the motor isn't %d times that of the reference", MULTIPLIER);
    ts_assert_double_in_delta(position_negative_multiplier, position_reference * (-MULTIPLIER), 1e-9,
                              "The position of the motor isn't %d times that of the reference", -MULTIPLIER);
    */
  }

  wb_motor_set_torque(reference_motor, 0.001);

  for (int i = 0; i < 1000; i++) {
    wb_robot_step(TIME_STEP);
    double position_reference = wb_position_sensor_get_value(reference_sensor);
    double position_positive_multiplier = wb_position_sensor_get_value(positive_multiplier_sensor);
    double position_negative_multiplier = wb_position_sensor_get_value(negative_multiplier_sensor);
    printf("%f %f %f\n", position_reference, position_positive_multiplier, position_negative_multiplier);
    /*
    ts_assert_double_in_delta(position_positive_multiplier, position_reference * MULTIPLIER, 1e-9,
                              "The position of the motor isn't %d times that of the reference", MULTIPLIER);
    ts_assert_double_in_delta(position_negative_multiplier, position_reference * (-MULTIPLIER), 1e-9,
                              "The position of the motor isn't %d times that of the reference", -MULTIPLIER);
    */
  }

  // wb_robot_step(TIME_STEP);

  /*
    double position = wb_position_sensor_get_value(position_sensor);
    ts_assert_double_equal(
      position, NAN, "The position value measured by the position sensor should be NaN and not %g before the device is enabled",
      position);

    const double torque_feedback = wb_motor_get_torque_feedback(motor);
    ts_assert_double_equal(
      torque_feedback, NAN,
      "The torque feedback value measured by the motor should be NaN and not %g before the device is enabled", torque_feedback);

    const double feedback_sampling_period = wb_motor_get_torque_feedback_sampling_period(motor);
    ts_assert_double_equal(
      feedback_sampling_period, 0,
      "The torque feedback sampling period value measured by the motor should be 0 and not %g before the device is enabled",
      feedback_sampling_period);

    const double max_position = wb_motor_get_max_position(motor);
    ts_assert_double_equal(max_position, MAX_POS, "The max position value of the motor should be %g and not %g", MAX_POS,
                           max_position);

    const double min_position = wb_motor_get_min_position(motor);
    ts_assert_double_equal(min_position, MIN_POS, "The min position value of the motor should be %g and not %g", MIN_POS,
                           min_position);

    double velocity = wb_motor_get_velocity(motor);
    double max_velocity = wb_motor_get_max_velocity(motor);
    ts_assert_double_equal(max_velocity, MAX_VELOCITY, "The max velocity value of the motor should be %g and not %g",
                           MAX_VELOCITY, max_velocity);
    ts_assert_double_equal(velocity, max_velocity, "Velocity %g and max velocity %g values have to be equal", velocity,
                           max_velocity);

    double acceleration = wb_motor_get_acceleration(motor);
    ts_assert_double_equal(acceleration, MAX_ACCELERATION, "The acceleration value of the motor should be %g and not %g",
                           acceleration, MAX_ACCELERATION);

    double torque = wb_motor_get_available_torque(motor);
    double max_torque = wb_motor_get_max_torque(motor);
    ts_assert_double_equal(max_torque, MAX_TORQUE, "The max torque value of the motor should be %g and not %g", MAX_TORQUE,
                           max_torque);
    ts_assert_double_equal(torque, max_torque, "Torque %g and max torque %g values have to be equal", torque, max_torque);

    wb_position_sensor_enable(position_sensor, TIME_STEP);
    wb_motor_enable_torque_feedback(motor, TIME_STEP);
    wb_motor_set_control_pid(motor, CONTROL_P, CONTROL_I, CONTROL_D);
    wb_motor_set_available_torque(motor, TORQUE);
    wb_motor_set_position(motor, 100);

    wb_robot_step(TIME_STEP);

    const double target_position = wb_motor_get_target_position(motor);
    ts_assert_double_equal(target_position, max_position,
                           "The target position value measured by the motor should be %g and not %g", max_position,
                           target_position);
    int i;
    for (i = 0; i < NUMBER_OF_STEPS; ++i)
      wb_robot_step(TIME_STEP);

    position = wb_position_sensor_get_value(position_sensor);
    ts_assert_double_in_delta(position, REFERENCE_POSITION, 0.001,
                              "The position value measured by the position sensor should be %g and not %g", REFERENCE_POSITION,
                              position);

    torque = wb_motor_get_torque_feedback(motor);
    ts_assert_double_in_delta(torque, REFERENCE_TORQUE, 0.001,
                              "The torque feedback value measured by the motor should be %g and not %g", REFERENCE_TORQUE,
                              torque);

    velocity = wb_motor_get_velocity(motor);
    max_velocity = wb_motor_get_max_velocity(motor);
    ts_assert_double_equal(max_velocity, MAX_VELOCITY,
                           "The max velocity value of the motor should be %g and not %g after parameters changed", MAX_VELOCITY,
                           max_velocity);
    ts_assert_double_equal(velocity, MAX_VELOCITY, "Velocity should be %g and not %g after parameters changed", MAX_VELOCITY,
                           velocity);

    torque = wb_motor_get_available_torque(motor);
    max_torque = wb_motor_get_max_torque(motor);
    ts_assert_double_equal(max_torque, MAX_TORQUE,
                           "The max torque value of the motor should be %g and not %g after torque changed", MAX_TORQUE,
                           max_torque);
    ts_assert_double_equal(torque, TORQUE, "Torque should be %g and not %g after torque changed", TORQUE, torque);

    wb_motor_set_velocity(motor, VELOCITY);
    wb_motor_set_acceleration(motor, ACCELERATION);

    wb_robot_step(TIME_STEP);

    acceleration = wb_motor_get_acceleration(motor);
    ts_assert_double_equal(acceleration, ACCELERATION, "Acceleration should be %g and not %g after acceleration changed",
                           ACCELERATION, acceleration);

    velocity = wb_motor_get_velocity(motor);
    max_velocity = wb_motor_get_max_velocity(motor);
    ts_assert_double_equal(max_velocity, MAX_VELOCITY,
                           "The max velocity value of the motor should be %g and not %g after velocity changed", MAX_VELOCITY,
                           max_velocity);
    ts_assert_double_equal(velocity, VELOCITY, "Velocity should be %g and not %g after velocity changed", VELOCITY, velocity);
    */
  ts_send_success();
  return EXIT_SUCCESS;
}
