/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Demo/Test for the Gyro device
 *                A three-axis Gyro device is mounted on three orthogonal and motorized rotation axes.
 *                The demo activates the motor axes one after the other and prints the gyro output.
 *                The sensor output should correctly reflects the currently active motor/rotation axis.
 *                The red, green and blue cylinders are aligned with the Gyro's x, y and z axes respectively.
 *                Note that, the outputs climbs up to 10 rad/s because this is the maximum rotation speed
 *                (maxVelocity) of each motor.
 */

#include <webots/gyro.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/utils/ansi_codes.h>

#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 8

WbDeviceTag gyro;

static void run_for_a_while() {
  int t;
  for (t = 0; t < 10000; t += TIME_STEP) {
    if (wb_robot_step(TIME_STEP) == -1)
      exit(EXIT_SUCCESS);
    const double *vel = wb_gyro_get_values(gyro);
    ANSI_CLEAR_CONSOLE();
    printf("rotation axes: [ x y z ] = [ %+.2f %+.2f %+.2f ]\n", vel[0], vel[1], vel[2]);
  }
}

int main() {
  wb_robot_init(); /* necessary to initialize webots stuff */

  /* get and enable gyro */
  gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, TIME_STEP);

  /* get motor devices */
  WbDeviceTag motor_x = wb_robot_get_device("motor_x");
  WbDeviceTag motor_y = wb_robot_get_device("motor_y");
  WbDeviceTag motor_z = wb_robot_get_device("motor_z");

  /* 10 full rotations of motor x, y, and z in sequence */
  wb_motor_set_position(motor_x, 62.83);
  run_for_a_while();
  wb_motor_set_position(motor_y, 62.83);
  run_for_a_while();
  wb_motor_set_position(motor_z, 62.83);
  run_for_a_while();

  return 0;
}
