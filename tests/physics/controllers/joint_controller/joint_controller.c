#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define POS_1 1.01
#define POS_2 0.6
#define POS_3 -2.45

void test_get_motor(WbDeviceTag position_sensor, WbDeviceTag expected_motor) {
  WbDeviceTag motor = wb_position_sensor_get_motor(position_sensor);
  ts_assert_int_equal(
    motor, expected_motor,
    "Wrong motor associated with position sensor. wb_position_sensor_get_motor(%d) returned %d instead of %d.", position_sensor,
    motor, expected_motor);
}

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag mot1 = wb_robot_get_device("rotational motor");
  WbDeviceTag mot2 = wb_robot_get_device("rotational motor 2");
  WbDeviceTag mot3 = wb_robot_get_device("rotational motor 3");

  WbDeviceTag ps1 = wb_robot_get_device("position sensor");
  WbDeviceTag ps2 = wb_robot_get_device("position sensor 2");
  WbDeviceTag ps3 = wb_robot_get_device("position sensor 3");

  // Test sibling device retrieval for a BallJoint.
  if (strcmp(wb_robot_get_name(), "robot(2)") == 0) {
    test_get_motor(ps1, mot1);
    test_get_motor(ps2, mot2);
    test_get_motor(ps3, mot3);

    // position sensor 3 does not have an associated brake.
    // Make sure we don't get the brake associated with the first position sensor instead.
    WbDeviceTag brake = wb_position_sensor_get_brake(ps3);
    ts_assert_int_equal(brake, 0,
                        "Brake returned for a position sensor that does not have a brake. wb_position_sensor_get_brake(%d)) "
                        "should be 0 but is %d",
                        ps3, brake);
  }

  wb_position_sensor_enable(ps1, TIME_STEP);
  wb_position_sensor_enable(ps2, TIME_STEP);
  wb_position_sensor_enable(ps3, TIME_STEP);

  wb_robot_step(TIME_STEP);

  wb_motor_set_position(mot1, POS_1);

  while (wb_robot_step(TIME_STEP) != -1 && fabs(wb_position_sensor_get_value(ps1) - POS_1) > 0.01) {
  };

  printf("Target 1 reached\n");

  wb_motor_set_position(mot2, POS_2);

  while (wb_robot_step(TIME_STEP) != -1 && fabs(wb_position_sensor_get_value(ps2) - POS_2) > 0.01) {
  };

  printf("Target 2 reached\n");

  wb_motor_set_position(mot3, POS_3);

  while (wb_robot_step(TIME_STEP) != -1 && fabs(wb_position_sensor_get_value(ps3) - POS_3) > 0.01) {
  };

  printf("Target 3 reached\n");

  wb_robot_cleanup();

  return 0;
}
