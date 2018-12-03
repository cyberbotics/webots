#include <webots/gyro.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#include <stdio.h>

#define TIME_STEP 64

const double expected_angle_rate[3] = {0.696, 0.348, 0.139};

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag wheels[2];
  wheels[0] = wb_robot_get_device("wheel1");
  wheels[1] = wb_robot_get_device("wheel2");
  wb_motor_set_position(wheels[0], INFINITY);
  wb_motor_set_position(wheels[1], INFINITY);

  wb_motor_set_velocity(wheels[0], -1.0);
  wb_motor_set_velocity(wheels[1], 1.0);

  WbDeviceTag gyro;
  gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, TIME_STEP);

  int n_step = 0;
  while (wb_robot_step(TIME_STEP) != -1) {
    double angle_rate = wb_gyro_get_values(gyro)[1];
    int index = n_step / 10;

    ts_assert_double_in_delta(angle_rate, expected_angle_rate[index], 0.01,
                              "Unexpected angle rate (expected = %lf, measured = %lf)", expected_angle_rate[index], angle_rate);
    printf("angle_rate: %lf excpected: %lf\n", angle_rate, expected_angle_rate[index]);
    ++n_step;
    if (n_step == 30)
      break;
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
