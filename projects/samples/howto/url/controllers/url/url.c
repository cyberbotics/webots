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
 * Description:  A dumb robot equipped with a camera, moving forward blindly.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

int main() {
  WbDeviceTag camera, left_motor, right_motor;
  int counter = 0;
  wb_robot_init();
  const int time_step = wb_robot_get_basic_time_step();

  /* Get the camera device, enable it, and store its width and height */
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, time_step);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  const WbNodeRef image_texture = wb_supervisor_node_get_from_def("IMAGE_TEXTURE");
  const WbFieldRef url = wb_supervisor_node_get_field(image_texture, "url");

  while (wb_robot_step(time_step) != -1) {
    wb_camera_get_image(camera);
    wb_motor_set_velocity(left_motor, 1);
    wb_motor_set_velocity(right_motor, 1);
    if (counter++ == 80) {
      wb_supervisor_field_set_mf_string(url, 0, "this_file_doesnt_exist.jpg");
      printf("The texture URL of the robot body was changed from the supervisor to a non-existing file.\n");
    } else if (counter == 160) {
      wb_supervisor_field_set_mf_string(url, 0,
                                        "https://raw.githubusercontent.com/cyberbotics/webots/R2021a/projects/appearances/"
                                        "protos/textures/brushed_steel/brushed_steel_base_color.jpg");
      printf("The texture URL of the robot body was changed from the supervisor to an existing URL.\n");
    }
  }

  wb_robot_cleanup();

  return 0;
}
