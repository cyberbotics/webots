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
 * Description:  Implementation of the functions of the maze_builder.h file
 */

#include "maze_builder.h"

#include "helper.h"
#include "parameters.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <webots/supervisor.h>

// count the objects already placed
static int wall_id = 0, feeder_id = 0, epuckId = 0, interval_id = 0;

// place the object referenced by its definition at some place
static void place_object(const char *defName, double x, double y, double alpha) {
  WbNodeRef wall = wb_supervisor_node_get_from_def(defName);
  WbFieldRef translation_field = wb_supervisor_node_get_field(wall, "translation");
  WbFieldRef rotation_field = wb_supervisor_node_get_field(wall, "rotation");
  const double *translation = wb_supervisor_field_get_sf_vec3f(translation_field);

  double newTranslation[3];
  memcpy(newTranslation, translation, 3 * sizeof(double));
  double newRotation[4] = {0.0, 1.0, 0.0, 0.0};

  newTranslation[CX] = x;
  newTranslation[CZ] = y;
  newRotation[ALPHA] = alpha;

  wb_supervisor_field_set_sf_vec3f(translation_field, newTranslation);
  wb_supervisor_field_set_sf_rotation(rotation_field, newRotation);
}

// place a wall
static void place_wall(double x, double y, double alpha) {
  if (wall_id > MAX_WALLS) {
    assert(0);
    return;
  }

  char wall_name[5] = "W000";
  wall_name[3] += wall_id % 10;
  wall_name[2] += (wall_id / 10) % 10;
  wall_name[1] += (wall_id / 100) % 10;

  place_object(wall_name, x, y, alpha);

  wall_id++;
}

// place a feeder
static void place_feeder(double x, double y, double alpha) {
  if (feeder_id > MAX_FEEDERS) {
    assert(0);
    return;
  }

  char feeder_name[3] = "F0";
  feeder_name[1] += feeder_id;

  place_object(feeder_name, x, y, alpha);

  feeder_id++;
}

// place an epuck
static void place_epuck(double x, double y, double alpha) {
  if (epuckId > MAX_EPUCKS) {
    assert(0);
    return;
  }

  char epuck_name[3] = "R0";
  epuck_name[1] += epuckId;

  place_object(epuck_name, x, y, alpha);

  epuckId++;
}

// place an interval
static void place_interval(double x, double y) {
  if (interval_id > MAX_INTERVALS) {
    assert(0);
    return;
  }

  char intervaleName[5] = "I000";
  intervaleName[3] += interval_id % 10;
  intervaleName[2] += (interval_id / 10) % 10;
  intervaleName[1] += (interval_id / 100) % 10;

  place_object(intervaleName, x, y, 0.0);

  interval_id++;
}

// return the x position in meter of a given cell
static double cell_get_x_pos(Cell *c) {
  return RATIO * c->pos_x;
}

// return the y position in meter of a given cell
static double cell_get_y_pos(Cell *c) {
  return RATIO * c->pos_y;
}

// return the angle in radian that a special cell should have
static double cell_get_angle(Cell *c) {
  if (c->feeder != None) {
    switch (c->feeder) {
      case North:
        return M_PI;
      case South:
        return 0.0;
      case East:
        return M_PI_2;
      case West:
        return -M_PI_2;
      default:
        assert(0);
    }
  } else if (c->init_pos != None) {
    switch (c->init_pos) {
      case North:
        return 0.0;
      case South:
        return M_PI;
      case East:
        return -M_PI_2;
      case West:
        return M_PI_2;
      default:
        assert(0);
    }
  }
  return 0.0;
}

// return the x position in meter of a given link
static double link_get_x_pos(Link *l) {
  return 0.5 * (cell_get_x_pos(l->cell_A) + cell_get_x_pos(l->cell_B));
}

// return the y position in meter of a given link
static double link_get_y_pos(Link *l) {
  return 0.5 * (cell_get_y_pos(l->cell_A) + cell_get_y_pos(l->cell_B));
}

// return the angle in radian that a link should have
static double link_get_angle(Link *l) {
  Cell *a = l->cell_A;
  Cell *b = l->cell_B;
  if (a->pos_x != b->pos_x)
    return 0.0;
  else if (a->pos_y != b->pos_y)
    return M_PI_2;
  assert(0);
  return 0.0;
}

// place all the objects according to the given maze structure
void build_maze(Maze *maze) {
  int i, j, k, l;
  int dim = maze->width * maze->height;

  // place the external walls
  for (i = 0; i < maze->width; i++) {
    place_wall(RATIO * i, -RATIO * 0.5, M_PI_2);
    place_wall(RATIO * i, RATIO * (-0.5 + maze->height), M_PI_2);
  }
  for (i = 0; i < maze->height; i++) {
    place_wall(-RATIO * 0.5, RATIO * i, 0.0);
    place_wall(RATIO * (-0.5 + maze->height), RATIO * i, 0.0);
  }

  // place the internal walls
  LLIST *links = maze->links;
  while (links) {
    Link *link = links->data;
    if (link->wall)
      place_wall(link_get_x_pos(link), link_get_y_pos(link), link_get_angle(link));
    links = links->next;
  }

  // place gray Lego intervals between the external walls
  for (i = 0; i < maze->width + 1; i++) {
    place_interval(RATIO * (-0.5 + i), -0.5 * RATIO);
    place_interval(RATIO * (-0.5 + i), RATIO * (-0.5 + maze->height));
  }
  for (i = 1; i < maze->height; i++) {
    place_interval(-RATIO * 0.5, RATIO * (-0.5 + i));
    place_interval(RATIO * (-0.5 + maze->height), RATIO * (-0.5 + i));
  }

  // place gray Lego intervals between the internal walls
  double tolRatio = 1.1 * RATIO;
  for (j = 0; j < maze->height - 1; j++) {
    for (i = 0; i < maze->width - 1; i++) {
      double px = RATIO * (0.5 + i);
      double py = RATIO * (0.5 + j);
      bool need_interval = false;
      for (k = 0; k < dim; k++) {
        Cell *cell = maze->cells[k];
        for (l = 0; l < cell->link_number; l++) {
          Link *link = cell->links[l];
          if (link->wall) {
            double dx = px - link_get_x_pos(link);
            double dy = py - link_get_y_pos(link);
            if (sqrt(dx * dx + dy * dy) < tolRatio) {
              need_interval = true;
              break;
            }
          }
        }
        if (need_interval)
          break;
      }
      if (need_interval)
        place_interval(px, py);
    }
  }

  // place the feeders and the e-pucks
  for (i = 0; i < dim; i++) {
    Cell *c = maze->cells[i];
    if (c->feeder != None)
      place_feeder(cell_get_x_pos(c), cell_get_y_pos(c), cell_get_angle(c));
    else if (c->init_pos != None)
      place_epuck(cell_get_x_pos(c), cell_get_y_pos(c), cell_get_angle(c));
  }
}
