/*
 * Copyright 1996-2019 Cyberbotics Ltd.
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
 * Description:  TODO
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>

int main(int argc, char **argv) {
  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();

  // get devices
  WbDeviceTag sick = wb_robot_get_device("sick");
  WbDeviceTag motor = wb_robot_get_device("motor");

  wb_lidar_enable(sick, time_step);
  wb_lidar_enable_point_cloud(sick);
  int resolution = wb_lidar_get_horizontal_resolution(sick);
  int layers = wb_lidar_get_number_of_layers(sick);

  printf("Sick: resolution=%d, layers=%d\n", resolution, layers);

  wb_robot_step(time_step);
  // perform simulation steps
  while (wb_robot_step(time_step) != -1) {
    double t = wb_robot_get_time();

    int l, p;
    bool something = false;
    for (l = 0; l < layers; ++l) {
      const WbLidarPoint *layer = wb_lidar_get_layer_point_cloud(sick, l);
      for (p = 0; p < resolution; ++p) {
        WbLidarPoint point = layer[p];
        // printf("%.2f %.2f %.2f\n", point.x, point.y, point.z);
        if (point.z > -0.9) {
          something = true;
          break;
        }
      }
    }
    if (something)
      printf("Something has been detected!\n");

    wb_motor_set_position(motor, t);
  }

  wb_robot_cleanup();

  return 0;
}
