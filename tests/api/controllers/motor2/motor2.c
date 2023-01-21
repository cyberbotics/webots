#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define ACCELERATION 5
#define CONTROL_P 15
#define CONTROL_I 5
#define CONTROL_D 1
#define TORQUE 0.5
#define VELOCITY 10.0
#define NUMBER_OF_STEPS 100
// copied from the .wbt file
#define TIME_STEP 32
#define MAX_POS 1.58
#define MIN_POS -1.53
// saved from prior simulation
#define REFERENCE_POSITION 0.137485
#define REFERENCE_TORQUE 0.500022
#define REFERENCE_TORQUE_2 1.15779e-07

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  // Ensure that position is reported properly in velocity control mode
  const WbDeviceTag position_sensor = wb_robot_get_device("position sensor1");
  const WbDeviceTag motor = wb_position_sensor_get_motor(position_sensor);
  wb_position_sensor_enable(position_sensor, TIME_STEP);

  const WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(inertial_unit, TIME_STEP);

  wb_robot_step(10 * TIME_STEP);

  wb_motor_set_position(motor, INFINITY);
  wb_motor_set_velocity(motor, 0);
  const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
  int i;
  for (i = 0; i < NUMBER_OF_STEPS; ++i) {
    wb_motor_set_velocity(motor, VELOCITY * fabs(cos(i * 2 * M_PI / NUMBER_OF_STEPS)));
    wb_robot_step(TIME_STEP);
    rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
    printf("roll = %g, pitch = %g, yaw = %g\n", rpy[0], rpy[1], rpy[2]);
  }
  double position = wb_position_sensor_get_value(position_sensor);
  double normalized_position = fmod(position, 2 * M_PI);
  if (normalized_position > M_PI)
    normalized_position -= 2 * M_PI;
  ts_assert_double_in_delta(rpy[2], normalized_position, 0.01, "The normalized rotation measured by position sensor (%g) and the inertial unit (%g) are different",
                            normalized_position, rpy[2]);

  // The position itself should not be normalized to be between -pi and pi
  ts_assert_double_is_bigger(position, 2 * M_PI, "The position shoud be at least %g but is only %g", 2 * M_PI, position);

  printf("Returning to 0\n");
  wb_motor_set_velocity(motor, 0);
  wb_robot_step(TIME_STEP);
  for (i = 0; i < 25; ++i) {
    wb_robot_step(TIME_STEP);
    rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
    printf("i = %d roll = %g, pitch = %g, yaw = %g\n", i, rpy[0], rpy[1], rpy[2]);
  }
  printf("Should be stopped\n");
  wb_motor_set_velocity(motor, VELOCITY/10);
  wb_motor_set_position(motor, 0.0);
  wb_motor_set_control_pid(motor, 10.0, 0.0, 0.0);
  wb_motor_set_position(motor, 0.0);
  wb_motor_set_velocity(motor, VELOCITY/10);
  for (i = 0; i < 250; ++i) {
    wb_robot_step(TIME_STEP);
    rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
    printf("i = %d roll = %g, pitch = %g, yaw = %g\n", i, rpy[0], rpy[1], rpy[2]);
  }
  printf("Should be back at 0\n");

  const WbDeviceTag position_sensor2 = wb_robot_get_device("position sensor2");
  const WbDeviceTag motor2 = wb_position_sensor_get_motor(position_sensor2);
  wb_robot_step(TIME_STEP);

  position = wb_position_sensor_get_value(position_sensor2);
  ts_assert_double_equal(
    position, NAN, "The position value measured by the position sensor 2 should be NaN and not %g before the device is enabled",
    position);

  const double torque_feedback = wb_motor_get_torque_feedback(motor2);
  ts_assert_double_equal(
    torque_feedback, NAN,
    "The torque feedback value measured by the motor 2 should be NaN and not %g before the device is enabled", torque_feedback);

  const double feedback_sampling_period = wb_motor_get_torque_feedback_sampling_period(motor2);
  ts_assert_double_equal(
    feedback_sampling_period, 0,
    "The torque feedback sampling period value measured by the motor 2 should be 0 and not %g before the device is enabled",
    feedback_sampling_period);

  const double max_position = wb_motor_get_max_position(motor2);
  ts_assert_double_equal(max_position, MAX_POS, "The max position value measured by the motor 2 should be %g and not %g",
                         MAX_POS, max_position);

  const double min_position = wb_motor_get_min_position(motor2);
  ts_assert_double_equal(min_position, MIN_POS, "The min position value measured by the motor 2 should be %g and not %g",
                         MIN_POS, min_position);

  wb_position_sensor_enable(position_sensor2, TIME_STEP);
  wb_motor_enable_torque_feedback(motor2, TIME_STEP);
  wb_motor_set_control_pid(motor2, CONTROL_P, CONTROL_I, CONTROL_D);
  wb_motor_set_available_torque(motor2, TORQUE);
  wb_motor_set_acceleration(motor2, ACCELERATION);
  wb_motor_set_position(motor2, M_PI_2);

  wb_robot_step(TIME_STEP);

  const double target_position = wb_motor_get_target_position(motor2);
  ts_assert_double_equal(target_position, M_PI_2, "The target position value measured by the motor 2 should be %g and not %g",
                         M_PI_2, target_position);
  for (i = 0; i < NUMBER_OF_STEPS; ++i)
    wb_robot_step(TIME_STEP);

  position = wb_position_sensor_get_value(position_sensor2);
  ts_assert_double_in_delta(position, REFERENCE_POSITION, 0.001,
                            "The position value measured by the position sensor 2 should be %g and not %g", REFERENCE_POSITION,
                            position);

  const double torque = wb_motor_get_torque_feedback(motor2);
  ts_assert_double_in_delta(torque, REFERENCE_TORQUE, 0.0001,
                            "The first torque feedback value measured by the motor 2 should be %g and not %g", REFERENCE_TORQUE,
                            torque);

  wb_robot_step(TIME_STEP);

  wb_motor_set_torque(motor2, TORQUE);

  for (i = 0; i < NUMBER_OF_STEPS; ++i)
    wb_robot_step(TIME_STEP);

  const double torque2 = wb_motor_get_torque_feedback(motor2);
  ts_assert_double_in_delta(torque2, REFERENCE_TORQUE_2, 0.0001,
                            "The second torque feedback value measured by the motor 2 should be %g and not %g",
                            REFERENCE_TORQUE_2, torque2);

  ts_send_success();
  return EXIT_SUCCESS;
}
