#include <webots/inertial_unit.h>
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
#define VELOCITY 10.0
#define NUMBER_OF_STEPS 20
// copied from the .wbt file
#define TIME_STEP 32
#define MAX_POS 1.58
#define MIN_POS -1.53
// saved from prior simulation
#define REFERENCE_POSITION 0.118272
#define REFERENCE_TORQUE 0.500022
#define REFERENCE_TORQUE_2 1.15779e-07

void test_position_under_velocity_control(WbDeviceTag motor, WbDeviceTag position_sensor, WbDeviceTag inertial_unit, int axis) {
  wb_inertial_unit_enable(inertial_unit, TIME_STEP);
  wb_position_sensor_enable(position_sensor, TIME_STEP);
  wb_robot_step(TIME_STEP);

  const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
  double position = wb_position_sensor_get_value(position_sensor);
  int i;

  double original_position = position;
  wb_motor_set_position(motor, INFINITY);
  wb_motor_set_velocity(motor, 0);
  for (i = 0; i < 100; ++i) {
    wb_motor_set_velocity(motor, VELOCITY * fabs(cos(i * 2 * M_PI / NUMBER_OF_STEPS)));
    wb_robot_step(TIME_STEP);
    rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
    printf("roll = %g, pitch = %g, yaw = %g\n", rpy[0], rpy[1], rpy[2]);
  }
  position = wb_position_sensor_get_value(position_sensor);
  double normalized_position = fmod(position, 2 * M_PI);
  if (normalized_position > M_PI)
    normalized_position -= 2 * M_PI;
  ts_assert_double_in_delta(rpy[axis], normalized_position, 0.01,
                            "The normalized rotation measured by position sensor (%g) and the inertial unit (%g) are different",
                            normalized_position, rpy[axis]);

  // The position itself should not be normalized to be between -pi and pi
  ts_assert_double_is_bigger(position, 2 * M_PI, "The position shoud be at least %g but is only %g", 2 * M_PI, position);

  printf("Returning to original position\n");
  wb_motor_set_position(motor, original_position);
  wb_motor_set_velocity(motor, VELOCITY);
  double prev_position;
  for (i = 0; i < 400; ++i) {
    wb_robot_step(TIME_STEP);
    prev_position = position;
    position = wb_position_sensor_get_value(position_sensor);
    rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
    printf("i = %d roll = %g, pitch = %g, yaw = %g\n", i, rpy[0], rpy[1], rpy[2]);
    if (fabs(position - prev_position) < 1e-6) {
      break;
    }
  }
  printf("Should be back at original position\n");
  ts_assert_double_in_delta(original_position, position, 0.01,
                            "The motor's position (%g) and it's original position (%g) are different", normalized_position,
                            rpy[axis]);
  ts_assert_double_is_bigger(1e-6, fabs(position - prev_position), "The motor has not stopped");
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  const WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");

  const WbDeviceTag position_sensor = wb_robot_get_device("position sensor1");
  const WbDeviceTag motor = wb_position_sensor_get_motor(position_sensor);
  test_position_under_velocity_control(motor, position_sensor, inertial_unit, 2);

  const WbDeviceTag position_sensor2 = wb_robot_get_device("position sensor2");
  const WbDeviceTag motor2 = wb_position_sensor_get_motor(position_sensor2);
  test_position_under_velocity_control(motor2, position_sensor2, inertial_unit, 1);

  ts_send_success();
  return EXIT_SUCCESS;
}
