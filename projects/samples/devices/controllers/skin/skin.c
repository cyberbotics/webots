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
 * Description: simple example of skin animation.
 */

#include <webots/robot.h>
#include <webots/skin.h>

#include <stdio.h>
#include <string.h>

int main(int argc, char **argv) {
  wb_robot_init();
  double time_step = wb_robot_get_basic_time_step();
  WbDeviceTag skin = wb_robot_get_device("skin");

  const int bones_count = wb_skin_get_bone_count(skin);
  printf("The skin model is made of %d bones:\n", bones_count);
  int leg_bone_index = -1;
  for (int i = 0; i < bones_count; ++i) {
    const char *name = wb_skin_get_bone_name(skin, i);
    printf("  Bone %d: %s\n\n", i, name);
    if (strcmp(name, "LeftLeg") == 0)
      leg_bone_index = i;
  }

  if (leg_bone_index == -1) {
    fprintf(stderr, "\"LeftLeg\" bone not found.");
    wb_robot_cleanup();
    return 1;
  }

  // get root 'Hips' bone position
  const double *absolute_hips_position = wb_skin_get_bone_position(skin, 0, true);
  double hips_position[3] = {absolute_hips_position[0], absolute_hips_position[1], absolute_hips_position[2]};

  // get initial right leg orientation
  const double *relative_leg_orientation = wb_skin_get_bone_orientation(skin, leg_bone_index, false);
  double leg_orientation[4] = {relative_leg_orientation[0], relative_leg_orientation[1], relative_leg_orientation[2],
                               relative_leg_orientation[3]};

  printf("Move 'LeftLeg' bone and change root 'Hips' position...\n");

  double leg_step = 0.01;
  while (wb_robot_step(time_step) != -1) {
    if (leg_orientation[3] > 1.5 || leg_orientation[3] < 0.1) {
      leg_step = -leg_step;
      if (leg_orientation[3] < 0.1) {
        // mode root 'Hips' bone
        if (hips_position[0] < 0.0)
          hips_position[0] = 0.5;
        else
          hips_position[0] = -0.5;
        wb_skin_set_bone_position(skin, 0, hips_position, true);
      }
    }
    leg_orientation[3] += leg_step;
    wb_skin_set_bone_orientation(skin, leg_bone_index, leg_orientation, false);
  };

  wb_robot_cleanup();
  return 0;
}
