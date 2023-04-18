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
   Description: Example demonstrating the difference between InertialUnit measurements
   and the fusion of multiple sensors: Accelerometer, Compass and Gyro.
*/

#include <stdio.h>
#include <stdlib.h>
#include <webots/accelerometer.h>
#include <webots/compass.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define RAD_TO_DEG 57.2958
#define NB_STEPS 100

// source: https://ahrs.readthedocs.io/en/latest/filters/angular.html
double *integrate_gyro(const double *current_value, const double *gyro_values, double sample_time) {
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

// source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_to_quaternion_conversion
double quaternion_to_roll(const double *quaternion) {
  const double sinr_cosp = 2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]);
  const double cosr_cosp = 1 - 2 * (pow(quaternion[1], 2.0) + pow(quaternion[2], 2.0));
  const double roll = atan2(sinr_cosp, cosr_cosp);
  return roll;
}

// source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_to_quaternion_conversion
double quaternion_to_pitch(const double *quaternion) {
  const double sinp = 2 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1]);
  double pitch = 0.0;
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2.0, sinp);  // use pi/2 radians if out of range
  else
    pitch = asin(sinp);
  return pitch;
}

// source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_to_quaternion_conversion
double quaternion_to_yaw(const double *quaternion) {
  const double siny_cosp = 2.0 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]);
  const double cosy_cosp = 1.0 - 2.0 * (pow(quaternion[2], 2.0) + pow(quaternion[3], 2.0));
  const double yaw = atan2(siny_cosp, cosy_cosp);
  return yaw;
}

// source: https://forum.arduino.cc/t/getting-pitch-and-roll-from-acceleromter-data/694148
double attitude_from_accelerometer(int axis, const double *accelerometer_values) {
  double output = 0.0;
  if (axis == 0)  // roll axis
    output = atan2(accelerometer_values[1], accelerometer_values[2]);
  else  // pitch axis
    output = atan2(-accelerometer_values[0], sqrt(pow(accelerometer_values[1], 2.0) + pow(accelerometer_values[2], 2.0)));
  return output;
}

// source: http://robo.sntiitk.in/2017/12/21/Beginners-Guide-to-IMU.html
double yaw_from_compass(double roll, double pitch, const double *compass_values) {
  const double mag_x =
    compass_values[0] * cos(pitch) + compass_values[1] * sin(roll) * sin(pitch) + compass_values[2] * cos(roll) * sin(pitch);
  const double mag_y = compass_values[1] * cos(roll) - compass_values[2] * sin(roll);
  const double yaw = atan2(mag_x, mag_y);

  return yaw;
}

double mean_error(const double *ground_truth, const double *estimation) {
  const double mean_error =
    (fabs(ground_truth[0] - estimation[0]) + fabs(ground_truth[1] - estimation[1]) + fabs(ground_truth[2] - estimation[2])) / 3;
  return mean_error;
}

int main(int argc, const char *argv[]) {
  // initialize webots API
  wb_robot_init();

  const int time_step = wb_robot_get_basic_time_step();

  WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(inertial_unit, time_step);

  WbDeviceTag imu_accelerometer = wb_robot_get_device("accelerometer");
  WbDeviceTag imu_gyro = wb_robot_get_device("gyro");
  WbDeviceTag imu_compass = wb_robot_get_device("compass");
  wb_accelerometer_enable(imu_accelerometer, time_step);
  wb_gyro_enable(imu_gyro, time_step);
  wb_compass_enable(imu_compass, time_step);
  double gyro_quaternion[4] = {1.0, 0.0, 0.0, 0.0};
  double absolute_attitude[3] = {0.0, 0.0, 0.0};
  double relative_attitude[3] = {0.0, 0.0, 0.0};

  WbDeviceTag roll_motor = wb_robot_get_device("roll motor");
  WbDeviceTag pitch_motor = wb_robot_get_device("pitch motor");
  WbDeviceTag yaw_motor = wb_robot_get_device("yaw motor");

  for (int i = 0; i < NB_STEPS; i++) {
    printf("%d targets before attitude comparison.\n", NB_STEPS - i);

    // choose a random target (based on seed)
    const double yaw_target = rand() / (double)RAND_MAX * 2.0 * M_PI - M_PI;
    const double pitch_target = rand() / (double)RAND_MAX * 1.8 - 0.9;
    const double roll_target = rand() / (double)RAND_MAX * 2.0 * M_PI - M_PI;

    // start moving arm to target
    wb_motor_set_position(roll_motor, roll_target);
    wb_motor_set_position(pitch_motor, pitch_target);
    wb_motor_set_position(yaw_motor, yaw_target);

    for (int j = 0; true; j++) {
      // execute a simulation step
      if (wb_robot_step(time_step) == -1)
        break;

      // compute roll, pitch and yaw from accelerometer and compass
      const double *accelerometer_values = wb_accelerometer_get_values(imu_accelerometer);
      const double *compass_values = wb_compass_get_values(imu_compass);
      absolute_attitude[0] = attitude_from_accelerometer(0, accelerometer_values);
      absolute_attitude[1] = attitude_from_accelerometer(1, accelerometer_values);
      absolute_attitude[2] = yaw_from_compass(absolute_attitude[0], absolute_attitude[1], compass_values);

      // compute new attitude quaternion from gyro angular rates
      const double *gyro_values = wb_gyro_get_values(imu_gyro);
      const double *new_gyro_quaternion = integrate_gyro(gyro_quaternion, gyro_values, (double)time_step / 1000);
      for (int k = 0; k < 4; ++k)
        gyro_quaternion[k] = new_gyro_quaternion[k];

      // convert gyro quaternion to Euler representation
      relative_attitude[0] = quaternion_to_roll(gyro_quaternion);
      relative_attitude[1] = quaternion_to_pitch(gyro_quaternion);
      relative_attitude[2] = quaternion_to_yaw(gyro_quaternion);

      // read inertial unit values (ground truth)
      const double *ground_truth_attitude = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);

      // print results for comparison at the start and after NB_STEPS
      if ((i == 0 || i == NB_STEPS - 1) && j == 300) {
        printf("\n \nInertial Unit (ground truth)\n");
        printf("Roll  = %f\n", ground_truth_attitude[0] * RAD_TO_DEG);
        printf("Pitch = %f\n", ground_truth_attitude[1] * RAD_TO_DEG);
        printf("Yaw   = %f\n \n", ground_truth_attitude[2] * RAD_TO_DEG);
        printf("\nAccelerometer and Compass (absolute)\n");
        printf("Roll  = %f\n", absolute_attitude[0] * RAD_TO_DEG);
        printf("Pitch = %f\n", absolute_attitude[1] * RAD_TO_DEG);
        printf("Yaw   = %f\n", absolute_attitude[2] * RAD_TO_DEG);
        const double abs_mean_error = mean_error(ground_truth_attitude, absolute_attitude) * RAD_TO_DEG;
        printf("Mean error in radians compared to ground truth = %f\n \n", abs_mean_error);
        printf("\nGyroscope (relative, subject to drift)\n");
        printf("Roll  = %f\n", relative_attitude[0] * RAD_TO_DEG);
        printf("Pitch = %f\n", relative_attitude[1] * RAD_TO_DEG);
        printf("Yaw   = %f\n", relative_attitude[2] * RAD_TO_DEG);
        const double rel_mean_error = mean_error(ground_truth_attitude, relative_attitude) * RAD_TO_DEG;
        printf("Mean error in radians compared to ground truth = %f\n \n", rel_mean_error);
      }

      // see if target position was reached
      if (fabs(ground_truth_attitude[0] - roll_target) < 0.01 && fabs(ground_truth_attitude[1] - pitch_target) < 0.01 &&
          fabs(ground_truth_attitude[2] - yaw_target) < 0.01)
        break;
    }
  }

  // cleanup webots resources
  wb_robot_cleanup();

  return 0;
}
