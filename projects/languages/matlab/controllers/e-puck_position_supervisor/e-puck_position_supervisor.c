/*
 * Copyright 1996-2018 Cyberbotics Ltd.
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
 * Description:  Simple Supervisor for matlab example
 */

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TIME_STEP 320

int main(int argc, const char *argv[]) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  WbDeviceTag emitter = wb_robot_get_device("emitter");
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  WbNodeRef epuck = wb_supervisor_node_get_from_def("E_PUCK");
  WbFieldRef trans = wb_supervisor_node_get_field(epuck, "translation");

  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    /* send 3 doubles to matlab */
    const double *pos = wb_supervisor_field_get_sf_vec3f(trans);
    wb_emitter_send(emitter, pos, 3 * sizeof(double));

    while (wb_receiver_get_queue_length(receiver) > 0) {
      // receive null-terminated 8 bit ascii string from matlab
      const char *string = wb_receiver_get_data(receiver);
      wb_supervisor_set_label(0, string, 0.01, 0.01, 0.1, 0x000000, 0.0, "Arial");
      wb_receiver_next_packet(receiver);
    }
  }

  /* Necessary to cleanup webots stuff */
  wb_robot_cleanup();

  return 0;
}
