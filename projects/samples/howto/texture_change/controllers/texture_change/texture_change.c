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
 * Description:  The controller of a supervisor which changes the texture used
 *               on an IndexdFaceSet.
 */

#include <stdio.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TIME_STEP 64

int main() {
  WbNodeRef image_texture;
  WbFieldRef url;
  int i = 0, j = 0;
  const char *text;

  wb_robot_init();
  image_texture = wb_supervisor_node_get_from_def("TEXTURE");
  if (image_texture == NULL)
    printf("TEXTURE was not found\n");
  url = wb_supervisor_node_get_field(image_texture, "url");
  while (wb_robot_step(TIME_STEP) != -1) {
    if (i++ == 30) { /* when the counter reaches 30, we change the texture. */
      if (j++ % 2)
        text = "webots://projects/default/worlds/textures/stone.jpg";
      else
        text = "webots://projects/default/worlds/textures/lightwood.jpg";
      wb_supervisor_field_set_mf_string(url, 0, text);
      printf("Changed texture to %s\n", text);
      i = 0;
    }
  }
  wb_robot_cleanup();
  return 0;
}
