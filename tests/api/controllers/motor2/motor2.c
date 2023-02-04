#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define ACCELERATION 5
#define CONTROL_P 15
#define CONTROL_I 5
#define CONTROL_D 1
#define TORQUE 0.5
#define NUMBER_OF_STEPS 20
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

  const WbDeviceTag position_sensor2 = wb_robot_get_device("position sensor2");
  const WbDeviceTag motor2 = wb_position_sensor_get_motor(position_sensor2);
  wb_robot_step(TIME_STEP);

  double position = wb_position_sensor_get_value(position_sensor2);
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
  int i;
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
