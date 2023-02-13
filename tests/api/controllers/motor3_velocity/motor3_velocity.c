#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

// copied from the .wbt file
#define TIME_STEP 32

void test_position_under_velocity_control(const char *sensor_name, const char *motor_name, double max_velocity, int axis) {
  const WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(inertial_unit, TIME_STEP);

  const WbDeviceTag position_sensor = wb_robot_get_device(sensor_name);
  const WbDeviceTag motor = wb_robot_get_device(motor_name);

  wb_position_sensor_enable(position_sensor, TIME_STEP);
  wb_robot_step(TIME_STEP);

  const double *quaternion = wb_inertial_unit_get_quaternion(inertial_unit);
  double position = wb_position_sensor_get_value(position_sensor);
  int i;

  double original_position = position;

  // Run the motor at varying velocities
  wb_motor_set_position(motor, INFINITY);
  wb_motor_set_velocity(motor, 0);
  for (i = 0; i < 100; ++i) {
    wb_motor_set_velocity(motor, max_velocity * fabs(cos(i * 2 * M_PI / 100)));
    wb_robot_step(TIME_STEP);
    quaternion = wb_inertial_unit_get_quaternion(inertial_unit);
    position = wb_position_sensor_get_value(position_sensor);
  }

  // The new position should match the one provided by the inertial unit.
  // That means the associated quaternions should be the same or just have opposite signs.
  // Since they are unit quaternions, |q1*q2| should equal 1.
  double actual_quaternion[4] = {0.0, 0.0, 0.0, 0.0};
  actual_quaternion[3] = cos(position / 2);
  actual_quaternion[axis] = sin(position / 2);
  double dot_product = actual_quaternion[0] * quaternion[0] + actual_quaternion[1] * quaternion[1] +
                       actual_quaternion[2] * quaternion[2] + actual_quaternion[3] * quaternion[3];
  ts_assert_double_in_delta(fabs(dot_product), 1.0, 1e-6,
                            "The orientation associated with the position returned by %s is wrong: expected quaternion = "
                            "+/-(%g, %g, %g, %g) actual quaternion = (%g, %g, %g, %g)",
                            sensor_name, quaternion[0], quaternion[1], quaternion[2], quaternion[3], actual_quaternion[0],
                            actual_quaternion[1], actual_quaternion[2], actual_quaternion[3]);

  // The position itself should not be normalized to be between -pi and pi
  double min_expected_position = max_velocity / 2 * TIME_STEP / 1000.0 * 100.0;
  ts_assert_double_is_bigger(position, min_expected_position, "The position measured by %s shoud be at least %g but is only %g",
                             sensor_name, min_expected_position, position);

  // Returning to original position
  wb_motor_set_position(motor, original_position);
  wb_motor_set_velocity(motor, max_velocity);
  double previous_position = 0.0, previous_position2 = 0.0;
  for (i = 0; i < 400; ++i) {
    wb_robot_step(TIME_STEP);
    previous_position2 = previous_position;
    previous_position = position;
    position = wb_position_sensor_get_value(position_sensor);
    if (fabs(position - previous_position) < 1e-6 && fabs(previous_position - previous_position2) < 1e-6)
      break;
  }
  ts_assert_double_in_delta(original_position, position, 0.01,
                            "The position measured by %s (%g) and it's original position (%g) are different", sensor_name,
                            position, original_position);
  ts_assert_double_is_bigger(1e-6, fabs(position - previous_position), "The motor associated with %s has not stopped",
                             sensor_name);
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  wb_motor_set_position(wb_robot_get_device("rotational motor1"), 0.0);
  wb_motor_set_position(wb_robot_get_device("rotational motor2"), 0.0);
  wb_motor_set_position(wb_robot_get_device("rotational motor3"), 0.0);

  test_position_under_velocity_control("position sensor1", "rotational motor1", 10.0, 0);
  test_position_under_velocity_control("position sensor3", "rotational motor3", 10.0, 2);
  test_position_under_velocity_control("position sensor2", "rotational motor2", 0.10, 1);

  ts_send_success();
  return EXIT_SUCCESS;
}
