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
 * Description:   Demo code for sensor aquisition for Firebird 6 robot
 * Author:        Anant Malewar; Nex Robotics
 */

#include <webots/accelerometer.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/gyro.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>

#define TIME_STEP 64

int main(int argc, char **argv) {
  wb_robot_init();

  printf("Fire Bird 6 controller for sensor aquisition\n");

  int i;
  WbDeviceTag ps[8], sharp[8], acc1, cmpXY1, cmpZ1, gyro1, left_motor, right_motor, left_position_sensor, right_position_sensor;
  char ps_names[8][4] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

  char sharp_names[8][9] = {"sharp_00", "sharp_01", "sharp_02", "sharp_03", "sharp_04", "sharp_05", "sharp_06", "sharp_07"};

  for (i = 0; i < 8; ++i) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);

    sharp[i] = wb_robot_get_device(sharp_names[i]);
    wb_distance_sensor_enable(sharp[i], TIME_STEP);
  }

  // Enable Acceleometer
  acc1 = wb_robot_get_device("accelerometer_01");
  wb_accelerometer_enable(acc1, TIME_STEP);

  // Enable compass
  cmpXY1 = wb_robot_get_device("compassXY_01");
  wb_compass_enable(cmpXY1, TIME_STEP);
  cmpZ1 = wb_robot_get_device("compassZ_01");
  wb_compass_enable(cmpZ1, TIME_STEP);

  // Enable Gyro
  gyro1 = wb_robot_get_device("gyro_01");
  wb_gyro_enable(gyro1, TIME_STEP);

  // Get the motors and set target position to infinity (speed control)
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Enable the position sensors
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, TIME_STEP);
  wb_position_sensor_enable(right_position_sensor, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    // distance sensors
    double ps_values[8];
    for (i = 0; i < 8; ++i) {
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);
      printf("sonar%d = %f ", i, ps_values[i]);
    }
    printf("\n");

    double sharp_values[8];
    for (i = 0; i < 8; ++i) {
      sharp_values[i] = wb_distance_sensor_get_value(sharp[i]);
      printf("sharp%d = %f ", i, sharp_values[i]);
    }
    printf("\n");

    // read encoders
    const double left_encoder = wb_position_sensor_get_value(left_motor);
    const double right_encoder = wb_position_sensor_get_value(right_motor);
    printf("left encoder = %f ", left_encoder);
    printf("right encoder = %f ", right_encoder);
    printf("\n");

    // Read accelerometer
    const double *accXYZ;
    accXYZ = wb_accelerometer_get_values(acc1);
    printf("AX %f ", accXYZ[0]);
    printf("AY %f ", accXYZ[1]);
    printf("AZ %f ", accXYZ[2]);
    printf("\n");

    // Read compass
    const double *cmpXY, *cmpZ;
    cmpXY = wb_compass_get_values(cmpXY1);
    cmpZ = wb_compass_get_values(cmpZ1);
    printf("MX %f ", cmpXY[0]);
    printf("MY %f ", cmpXY[1]);
    printf("MZ %f ", cmpZ[2]);
    printf("\n");

    // calculate bearing
    double rad = atan2(cmpXY[0], cmpZ[2]);
    double bearing = (rad) / 3.1428 * 180.0;
    if (bearing < 0.0)
      bearing = bearing + 360.0;
    printf("bearing = %f \n", bearing);

    // Read gyro
    const double *gyroXYZ;
    gyroXYZ = wb_gyro_get_values(gyro1);
    printf("GX %f ", gyroXYZ[0]);
    printf("GY %f ", gyroXYZ[1]);
    printf("GZ %f ", gyroXYZ[2]);
    printf("\n");
  };

  wb_robot_cleanup();

  return 0;
}
