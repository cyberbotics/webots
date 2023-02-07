#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define VELOCITY 10.0
// copied from the .wbt file
#define TIME_STEP 32

void test_position_under_velocity_control(WbDeviceTag motor, WbDeviceTag position_sensor, int axis) {
  const WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(inertial_unit, TIME_STEP);
  wb_position_sensor_enable(position_sensor, TIME_STEP);
  wb_robot_step(TIME_STEP);

  const double *roll_pitch_yaw = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
  double position = wb_position_sensor_get_value(position_sensor);
  int i;

  double original_position = position;

  // Run the motor at varying velocities
  wb_motor_set_position(motor, INFINITY);
  wb_motor_set_velocity(motor, 0);
  for (i = 0; i < 100; ++i) {
    wb_motor_set_velocity(motor, VELOCITY * fabs(cos(i * 2 * M_PI / 100)));
    wb_robot_step(TIME_STEP);
    roll_pitch_yaw = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
  }

  // The new position should match the one provided by the inertial unit
  position = wb_position_sensor_get_value(position_sensor);
  double normalized_position = fmod(position, 2 * M_PI);
  if (normalized_position > M_PI)
    normalized_position -= 2 * M_PI;
  ts_assert_double_in_delta(roll_pitch_yaw[axis], normalized_position, 0.01,
                            "The normalized rotation measured by position sensor (%g) and the inertial unit (%g) are different",
                            normalized_position, roll_pitch_yaw[axis]);

  // The position itself should not be normalized to be between -pi and pi
  ts_assert_double_is_bigger(position, 2 * M_PI, "The position shoud be at least %g but is only %g", 2 * M_PI, position);

  // Returning to original position
  wb_motor_set_position(motor, original_position);
  wb_motor_set_velocity(motor, VELOCITY);
  double previous_position;
  for (i = 0; i < 400; ++i) {
    wb_robot_step(TIME_STEP);
    previous_position = position;
    position = wb_position_sensor_get_value(position_sensor);
    roll_pitch_yaw = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
    if (fabs(position - previous_position) < 1e-6) {
      break;
    }
  }
  ts_assert_double_in_delta(original_position, position, 0.01,
                            "The motor's position (%g) and it's original position (%g) are different", position,
                            original_position);
  ts_assert_double_is_bigger(1e-6, fabs(position - previous_position), "The motor has not stopped");
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  const WbDeviceTag position_sensor = wb_robot_get_device("position sensor1");
  const WbDeviceTag motor = wb_position_sensor_get_motor(position_sensor);
  test_position_under_velocity_control(motor, position_sensor, 2);

  const WbDeviceTag position_sensor2 = wb_robot_get_device("position sensor2");
  const WbDeviceTag motor2 = wb_position_sensor_get_motor(position_sensor2);
  test_position_under_velocity_control(motor2, position_sensor2, 1);

  ts_send_success();
  return EXIT_SUCCESS;
}
