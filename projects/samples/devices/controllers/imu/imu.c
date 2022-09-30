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

double *integrate_gyro(double *current_value, const double *gyro_values, double sample_time) {
  // compute rotation matrix from sample time and gyro rates
  const double identity_matrix[4][4] = {{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
  const double gyro_matrix[4][4] = {{0.0, -gyro_values[0], -gyro_values[1], -gyro_values[2]},
                                    {gyro_values[0], 0.0, gyro_values[2], -gyro_values[1]},
                                    {gyro_values[1], -gyro_values[2], 0.0, gyro_values[0]},
                                    {gyro_values[2], gyro_values[1], -gyro_values[0], 0.0}};

  double rotation_matrix[4][4];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      rotation_matrix[i][j] = sample_time * gyro_matrix[i][j];
      rotation_matrix[i][j] += identity_matrix[i][j];
    }
  }

  // apply gyro rates to the current quaternion
  static double new_quaternion[4] = {0.0, 0.0, 0.0, 0.0};
  double q_norm = 0.0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++)
      new_quaternion[i] += rotation_matrix[i][j] * current_value[j];
    q_norm += pow(new_quaternion[i], 2.0);
  }

  // normalize new quaternion
  q_norm = sqrt(q_norm);
  for (int i = 0; i < 4; i++)
    new_quaternion[i] = new_quaternion[i] / q_norm;

  return new_quaternion;
}

double attitude_from_accelerometer(int axis, const double *accelerometer_values) {
  double output = 0.0;
  // roll axis
  if (axis == 0)
    output = atan2(accelerometer_values[1], accelerometer_values[2]);
  // pitch axis
  else
    output = atan2(-accelerometer_values[0], sqrt(pow(accelerometer_values[1], 2.0) + pow(accelerometer_values[2], 2.0)));
  return output;
}

double heading_from_compass(double roll, double pitch, const double *compass_values) {
  const double mag_x =
    compass_values[0] * cos(pitch) + compass_values[1] * sin(roll) * sin(pitch) + compass_values[2] * cos(roll) * sin(pitch);
  const double mag_y = compass_values[1] * cos(roll) - compass_values[2] * sin(roll);
  const double yaw = atan2(mag_x, mag_y);

  return yaw;
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
  double gyro_attitude[4] = {1.0, 0.0, 0.0, 0.0};
  double accelerometer_attitude[2] = {0.0, 0.0};
  double compass_yaw = 0.0;

  WbDeviceTag yaw_motor = wb_robot_get_device("yaw motor");
  WbDeviceTag pitch_motor = wb_robot_get_device("pitch motor");
  WbDeviceTag roll_motor = wb_robot_get_device("roll motor");
  double yaw_table[4] = {1.6, 1.6, 0.0, 0.0};
  double pitch_table[4] = {0.0, 0.5, 0.5, 0.0};
  double roll_table[4] = {0.0, 0.0, 0.0, 0.0};

  int i;
  for (i = 0; true; i++) {
    // choose a random target
    srand(time(0));
    double yaw = /*yaw_table[i];      */ rand() / (double)RAND_MAX * 2.0 * M_PI - M_PI;
    double pitch = /*pitch_table[i];  */ rand() / (double)RAND_MAX * 1.8 - 0.9;
    double roll = /*roll_table[i];    */ rand() / (double)RAND_MAX * 2.0 * M_PI - M_PI;

    // start moving arm to target
    wb_motor_set_position(yaw_motor, yaw);
    wb_motor_set_position(pitch_motor, pitch);
    wb_motor_set_position(roll_motor, roll);

    int j;
    for (j = 0; true; j++) {
      // execute a simulation step
      if (wb_robot_step(time_step) == -1)
        break;

      // compute roll, pitch and yaw from accelerometer and compass
      const double *accelerometer_values = wb_accelerometer_get_values(imu_accelerometer);
      accelerometer_attitude[0] = attitude_from_accelerometer(0, accelerometer_values);
      accelerometer_attitude[1] = attitude_from_accelerometer(1, accelerometer_values);
      const double *compass_values = wb_compass_get_values(imu_compass);
      compass_yaw = heading_from_compass(accelerometer_attitude[0], accelerometer_attitude[1], compass_values);

      // compute new attitude quaternion from gyro angular rates
      const double *gyro_values = wb_gyro_get_values(imu_gyro);
      // printf("%.20f %.20f %.20f\n", gyro_values[0], gyro_values[1], gyro_values[2]);
      const double *new_gyro_attitude = integrate_gyro(gyro_attitude, gyro_values, (double)time_step / 1000);
      for (int k = 0; k < 4; ++k)
        gyro_attitude[k] = new_gyro_attitude[k];

      // convert gyro quaternion to Euler (wikipedia formulas)
      // roll (x-axis rotation)
      const double sinr_cosp = 2 * (gyro_attitude[0] * gyro_attitude[1] + gyro_attitude[2] * gyro_attitude[3]);
      const double cosr_cosp = 1 - 2 * (gyro_attitude[1] * gyro_attitude[1] + gyro_attitude[2] * gyro_attitude[2]);
      const double gyro_roll = atan2(sinr_cosp, cosr_cosp);

      // pitch (y-axis rotation)
      const double sinp = 2 * (gyro_attitude[0] * gyro_attitude[2] - gyro_attitude[3] * gyro_attitude[1]);
      double gyro_pitch = 0.0;
      if (abs(sinp) >= 1)
        gyro_pitch = copysign(M_PI / 2.0, sinp);  // use 90 degrees if out of range
      else
        gyro_pitch = asin(sinp);

      // yaw (z-axis rotation)
      const double siny_cosp = 2 * (gyro_attitude[0] * gyro_attitude[3] + gyro_attitude[1] * gyro_attitude[2]);
      const double cosy_cosp = 1 - 2 * (gyro_attitude[2] * gyro_attitude[2] + gyro_attitude[3] * gyro_attitude[3]);
      const double gyro_yaw = atan2(siny_cosp, cosy_cosp);

      // read inertial unit values (ground truth)
      const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);

      // print results for comparison after 100 steps
      if (j == 100) {
        printf("\nROLL\n");
        printf("accelerometer = %f\n", accelerometer_attitude[0] * RAD_TO_DEG);
        printf("gyroscope     = %f\n", gyro_roll * RAD_TO_DEG);
        printf("inertial unit = %f\n \n", rpy[0] * RAD_TO_DEG);
        printf("PITCH\n");
        printf("accelerometer = %f\n", accelerometer_attitude[1] * RAD_TO_DEG);
        printf("gyroscope     = %f\n", gyro_pitch * RAD_TO_DEG);
        printf("inertial unit = %f\n \n", rpy[1] * RAD_TO_DEG);
        printf("YAW\n");
        printf("compass       = %f\n", compass_yaw * RAD_TO_DEG);
        printf("gyroscope     = %f\n", gyro_yaw * RAD_TO_DEG);
        printf("inertial unit = %f\n", rpy[2] * RAD_TO_DEG);
        printf("=================================================================");
      }

      // see if target position was reached
      if (fabs(rpy[0] - roll) < 0.01 && fabs(rpy[1] - pitch) < 0.01 && fabs(rpy[2] - yaw) < 0.01) {
        break;
      }
    }
  }

  // cleanup webots resources
  wb_robot_cleanup();

  return 0;
}