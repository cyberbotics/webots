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

#define THRESHOLD 3.0

int main(int argc, char **argv) {
  wb_robot_init();
  int time_step = 64;

  WbDeviceTag sick = wb_robot_get_device("sick");

  wb_lidar_enable(sick, time_step);
  wb_lidar_enable_point_cloud(sick);
  int resolution = wb_lidar_get_horizontal_resolution(sick);
  int layers = wb_lidar_get_number_of_layers(sick);

  int previous_counter = 0;

  while (wb_robot_step(time_step) != -1) {
    int p;
    int counter = 0;
    bool previous_obstacle = false;

    const WbLidarPoint *layer = wb_lidar_get_layer_point_cloud(sick, layers / 2);
    for (p = 0; p < resolution; ++p) {
      WbLidarPoint point = layer[p];
      bool obstacle = -point.z < THRESHOLD;
      if (obstacle && !previous_obstacle)
        counter++;
      previous_obstacle = obstacle;
    }

    if (counter != previous_counter) {
      printf("I see %d cans\n", counter);
      previous_counter = counter;
    }
  }

  wb_robot_cleanup();

  return 0;
}
