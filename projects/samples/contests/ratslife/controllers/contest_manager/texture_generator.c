/*
 * Copyright 1996-2020 Cyberbotics Ltd.
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
 * Description:  Implementation of the functions of the texture_generator.h file
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "boolean.h"
#include "helper.h"
#include "parameters.h"
#include "texture_generator.h"

#include <webots/display.h>
#include <webots/supervisor.h>

static const char *tmp_dir;
static char tmp_file_name[1024];

#ifdef _WIN32
#define SEP "\\"
#else
#define SEP "/"
#endif

// retrieve the url field according to the solid name of a LegoWall proto
static WbFieldRef get_url_from_solid_name(const char *name) {
  WbNodeRef wall_node = wb_supervisor_node_get_from_def(name);
  if (!wall_node) {
    fprintf(stderr, "texture_generator.c::get_url_from_solid_name: cannot get the wall node (%s)\n", name);
    return NULL;
  }

  WbFieldRef url_field = wb_supervisor_node_get_field(wall_node, "textureUrl");
  if (!url_field) {
    fprintf(stderr, "texture_generator.c::get_url_from_solid_name: cannot get the url field\n");
    return NULL;
  }
  return url_field;
}

// test the existance of a file
static bool file_exists(const char *filename) {
  FILE *file = fopen(filename, "r");
  if (file) {
    fclose(file);
    return true;
  }
  return false;
}

// update the temporary directory
static void update_tmp_dir() {
  tmp_dir = getenv("TMP");
  if (tmp_dir)
    return;
  tmp_dir = getenv("TEMP");
  if (tmp_dir)
    return;
  tmp_dir = getenv("TMPDIR");
  if (tmp_dir)
    return;
#ifdef _WIN32
  tmp_dir = "C:\\Windows\\Temp";
#else
  tmp_dir = "/tmp";
#endif
}

// generate a texture for the LegoWall prototype randomly
// and store it in a file
static void generate_texture(WbDeviceTag display, const char *name) {
  int i, j, k;

  // get the display dimensions
  int width = wb_display_get_width(display);
  int height = wb_display_get_height(display);

  // fill the texture in white
  wb_display_set_color(display, WHITE);
  wb_display_fill_rectangle(display, 0, 0, width, height);

  // fill the black tiles in white
  wb_display_set_color(display, BLACK);
  wb_display_fill_rectangle(display, width / 2, 3 * height / 4, width / 2, height / 4);

  // generate the two parts of the texture
  int offset = 0;
  for (k = 0; k < 2; k++) {
    // select the color
    switch (RANDOM(0, 4)) {
      case 0:
        wb_display_set_color(display, RED);
        break;
      case 1:
        wb_display_set_color(display, GREEN);
        break;
      case 2:
        wb_display_set_color(display, BLUE);
        break;
      default:
        wb_display_set_color(display, YELLOW);
        break;
    }
    // select randomly the pixels
    for (i = offset; i < 4 + offset; i++) {
      for (j = 0; j < 3; j++) {
        if (RANDOM(0, 2))
          wb_display_fill_rectangle(display, 8 * i, 8 * j, 8, 8);
      }
    }
    offset += 4;
  }
  WbImageRef ir = wb_display_image_copy(display, 0, 0, width, height);
  wb_display_image_save(display, ir, name);
  wb_display_image_delete(display, ir);
}

// put the w argument into the tmp_file_name string
static void update_tmp_file_name(int w) {
  sprintf(tmp_file_name, "%s%sratslife_%d.jpg", tmp_dir, SEP, w);
}

// for each wall (LegoWall prototype) generate a random texture
void update_wall_textures(WbDeviceTag display) {
  int wall;

  // get the tmp dir
  update_tmp_dir();

  // for each wall
  for (wall = 0; wall < MAX_WALLS; wall++) {
    // get the wall name
    char wall_name[5] = "W000";
    wall_name[3] += wall % 10;
    wall_name[2] += (wall / 10) % 10;
    wall_name[1] += (wall / 100) % 10;

    // get the url field
    WbFieldRef url = get_url_from_solid_name(wall_name);

    // get a uniq temporary file name
    update_tmp_file_name(wall);

    // generate the texture into the display and save it
    generate_texture(display, tmp_file_name);

    // set the freshly created texture
    wb_supervisor_field_set_mf_string(url, 0, tmp_file_name);
  }
}

// Removed the generated textures
// This function should be only called at startup and at the end of the supervisor program
// and not in run time, otherwise the textures will be corrupted
void empty_the_temporary_path() {
  // get the tmp dir
  update_tmp_dir();

  int i;
  for (i = 0; i < MAX_WALLS; i++) {
    update_tmp_file_name(i);
    if (file_exists(tmp_file_name))
      remove(tmp_file_name);
  }
}
