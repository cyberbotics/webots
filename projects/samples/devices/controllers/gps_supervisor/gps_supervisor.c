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
 * Description:  An implementation of a GPS with a Supervisor equipped with an
 *               Emitter, playing the role of the sattelite and a robot
 *               equipped with a Receiver, playing the role of the GPS receiver.
 *               This file is the supervisor controller.
 */

#include <webots/emitter.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TIME_STEP 64

int main() {
  wb_robot_init();

  /* get a handler to the emitter and to the robot */
  WbDeviceTag emitter = wb_robot_get_device("emitter");

  /* get the field which contains the position of the robot*/
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("GPS_ROBOT");
  WbFieldRef robot_translation = wb_supervisor_node_get_field(robot_node, "translation");

  while (wb_robot_step(TIME_STEP) != -1) {
    /* At each step, the position of the robot is get and sent through the emitter
     */
    const double *robot_coords = wb_supervisor_field_get_sf_vec3f(robot_translation);
    wb_emitter_send(emitter, robot_coords, 3 * sizeof(double));
  }

  wb_robot_cleanup();

  return 0;
}
