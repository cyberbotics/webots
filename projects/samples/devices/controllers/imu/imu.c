/*
 * Copyright 1996-2022 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
   Description:   Demo for InertialUnit node
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <webots/accelerometer.h>
#include <webots/compass.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define RAD_TO_DEG 57.2958

double integrate_gyro(double current_value, const double sample_value, double sample_time) {
  current_value += (sample_value * sample_time);
  return current_value;
}

double attitude_from_accelerometer(int axis, const double *accelerometer_values) {
  double output = 0.0;
  // For pitch axis should be 0 else roll will be calculated
  if (axis == 0)
    output = atan2(accelerometer_values[1], accelerometer_values[2]) * RAD_TO_DEG;
  else
    output = atan2(-accelerometer_values[0],
                   sqrt(pow(accelerometer_values[1], 2.0) + pow(accelerometer_values[2], 2.0))) * RAD_TO_DEG;
  return output;
}

int main(int argc, const char *argv[]) {
  // initialize webots API
  wb_robot_init();

  int time_step = wb_robot_get_basic_time_step();

  WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(inertial_unit, time_step);

  WbDeviceTag imu_accelerometer = wb_robot_get_device("MPU-9250 accelerometer");
  WbDeviceTag imu_gyro = wb_robot_get_device("MPU-9250 gyro");
  WbDeviceTag imu_compass = wb_robot_get_device("MPU-9250 compass");
  wb_accelerometer_enable(imu_accelerometer, time_step);
  wb_gyro_enable(imu_gyro, time_step);
  wb_compass_enable(imu_compass, time_step);
  double gyro_attitude[3] = {0.0, 0.0, 0.0};
  double accelerometer_attitude[2] = {0.0, 0.0};

  WbDeviceTag yaw_motor = wb_robot_get_device("yaw motor");
  WbDeviceTag pitch_motor = wb_robot_get_device("pitch motor");
  WbDeviceTag roll_motor = wb_robot_get_device("roll motor");

  int i;
  for (i = 0; true; i++) {
    // choose a random target
    srand(time(0));
    double yaw = rand() / (double)RAND_MAX * 2.0 * M_PI - M_PI;
    double pitch = -(rand() / (double)RAND_MAX * 2.3 - 0.8);
    double roll = rand() / (double)RAND_MAX * 2.0 * M_PI - M_PI;

    printf("new target #%d: roll/pitch/yaw=%f %f %f\n", i, roll, pitch, yaw);

    // start moving arm to target
    wb_motor_set_position(yaw_motor, yaw);
    wb_motor_set_position(pitch_motor, pitch);
    wb_motor_set_position(roll_motor, roll);

    int j;
    for (j = 0; true; j++) {
      // execute a simulation step
      if (wb_robot_step(time_step) == -1)
        break;

      const double *accelerometer_values = wb_accelerometer_get_values(imu_accelerometer);
      const double *gyro_values = wb_gyro_get_values(imu_gyro);
      gyro_attitude[0] = integrate_gyro(gyro_attitude[0], gyro_values[0], time_step / 1000);
      gyro_attitude[1] = integrate_gyro(gyro_attitude[1], gyro_values[1], time_step / 1000);
      gyro_attitude[2] = integrate_gyro(gyro_attitude[2], gyro_values[2], time_step / 1000);
      accelerometer_attitude[0] = attitude_from_accelerometer(0, accelerometer_values);
      accelerometer_attitude[1] = attitude_from_accelerometer(1, accelerometer_values);

      // read inertial unit values
      const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);

      printf("acc roll = %f\n", accelerometer_attitude[0]);
      // printf("gyr = %f\n", gyro_attitude[1]);
      printf("gnd roll = %f\n\n", rpy[0] * RAD_TO_DEG);
      
      printf("acc pitch = %f\n", accelerometer_attitude[1]);
      // printf("gyr = %f\n", gyro_attitude[1]);
      printf("gnd pitch = %f\n\n", rpy[1] * RAD_TO_DEG);

      // see if target position was reached
      if (fabs(rpy[0] - roll) < 0.01 && fabs(rpy[1] - pitch) < 0.01 && fabs(rpy[2] - yaw) < 0.01) {
        printf("reached target after %d simulation steps\n", j);
        break;
      }
    }
  }

  // cleanup webots resources
  wb_robot_cleanup();

  return 0;
}