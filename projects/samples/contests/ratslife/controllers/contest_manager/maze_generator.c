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
 * Description:  Implementation of the functions of the maze_generator.h file
 */

#include "maze_generator.h"

#include "helper.h"
#include "linked_list.h"

#include <webots/display.h>

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// create a link between two cells
// don't check if a link already exists
static void create_link_between_two_cells(Maze *m, Cell *a, Cell *b) {
  // create the link
  Link *l = malloc(sizeof(Link));
  l->wall = true;
  l->cell_A = a;
  l->cell_B = b;
  l->removable = true;

  // add this link to the cells
  a->link_number++;
  a->links = realloc(a->links, a->link_number * sizeof(Link));
  a->links[a->link_number - 1] = l;
  b->link_number++;
  b->links = realloc(b->links, b->link_number * sizeof(Link));
  b->links[b->link_number - 1] = l;

  // add this link to the maze
  list_add(&m->links, l);
}

// get the next cell having a cell and one of its link
static Cell *get_linked_cell(const Cell *c, const Link *l) {
  if (l->cell_A == c)
    return l->cell_B;
  else if (l->cell_B == c)
    return l->cell_A;
  return NULL;
}

// get the orientation of cell b regard to cell a
static Orientation get_orientation(Cell *a, Cell *b) {
  if (a->pos_x > b->pos_x)
    return West;
  else if (a->pos_x < b->pos_x)
    return East;
  else if (a->pos_y > b->pos_y)
    return North;
  else if (a->pos_y < b->pos_y)
    return South;
  else
    assert(0);
  return None;
}

// create the cells and the links of a maze
// init the links with walls
void init_maze(Maze *maze, int width, int height) {
  int i, j;
  int dimension = width * height;

  // init the maze structure
  maze->width = width;
  maze->height = height;
  maze->cells = malloc(dimension * sizeof(Cell *));
  maze->links = NULL;

  // init the cells
  for (j = 0; j < height; j++) {
    for (i = 0; i < width; i++) {
      Cell *c = malloc(sizeof(Cell));
      c->pos_x = i;
      c->pos_y = j;
      c->visited = false;
      c->feeder = None;
      c->init_pos = None;
      c->link_number = 0;
      c->links = NULL;
      maze->cells[j * width + i] = c;
    }
  }

  // init the links between the cells
  // horizontically
  for (j = 0; j < height; j++) {
    for (i = 0; i < width - 1; i++)
      create_link_between_two_cells(maze, maze->cells[j * width + i], maze->cells[j * width + i + 1]);
  }
  // vertically
  for (i = 0; i < width; i++) {
    for (j = 0; j < height - 1; j++)
      create_link_between_two_cells(maze, maze->cells[j * width + i], maze->cells[(j + 1) * width + i]);
  }
}

// remove the walls randomly, set the feeders and the robot init pos
// inspired from the Randomized Prim's algorithm
bool generate_random_maze(Maze *maze) {
  // 1 Initialisation
  srand(time(NULL));
  int w = maze->width;
  int h = maze->height;
  int dim = w * h;
  int i, j;

  // 1.1 create a temporary link list
  LLIST *link_list = NULL;

  // 1.2 Start with a grid full of walls
  //   the maze should have already this state

  // 2 Pick a cell, mark it as part of the maze. Add the links of the cell to the link list
  Cell *cell = maze->cells[RANDOM(0, h) * w + RANDOM(0, w)];
  cell->visited = true;
  for (i = 0; i < cell->link_number; i++)
    list_add(&link_list, cell->links[i]);

  // 3 While there are links in the list
  while (list_size(link_list) != 0) {
    // 3.1 Pick a random link from the list
    int id = RANDOM(0, list_size(link_list));
    Link *link = list_at(&link_list, id)->data;
    list_remove(list_search(&link_list, link));

    // 3.2 If the cell on the opposite side isn't in the maze yet
    if (link->cell_A->visited == false || link->cell_B->visited == false) {
      Cell *c = (link->cell_A->visited) ? link->cell_B : link->cell_A;

      // 3.2.1 Make the link a passage and mark the cell on the opposite side as part of the maze
      link->wall = false;
      c->visited = true;

      // 3.2.2 Add the neighboring links of the cell to the wall list
      for (i = 0; i < c->link_number; i++) {
        if (!list_search(&link_list, c->links[i]))
          list_add(&link_list, c->links[i]);
      }
    }
  }

  // 4. Place the feeders and the init_pos
  // 4.1 Create a list of cells which can contain a feeder and the init_pos
  LLIST *cell_list = NULL;
  for (i = 0; i < dim; i++) {
    Cell *c = maze->cells[i];
    int wall_number = 0;
    for (j = 0; j < c->link_number; j++) {
      if (c->links[j]->wall)
        wall_number++;
    }
    if (wall_number == 3 || (wall_number == 2 && (c->pos_x == 0 || c->pos_x == w - 1 || c->pos_y == 0 || c->pos_y == h - 1)))
      list_add(&cell_list, c);
  }
  int cell_list_size = list_size(cell_list);
  if (cell_list_size < 6) {
    fprintf(stderr, "Cannot place the feeders and the init pos");
    return false;
  }

  // 4.2 choose randomly 6 different indexes into the cell list
  int indexes[6];
  for (i = 0; i < 6; i++) {
    bool ok = false;
    while (!ok) {
      ok = true;
      indexes[i] = RANDOM(0, cell_list_size);
      for (j = 0; j < i; j++) {
        if (indexes[j] == indexes[i]) {
          ok = false;
          break;
        }
      }
    }
  }

  // 4.3 set the feeders and the init pos
  for (i = 0; i < 6; i++) {
    Cell *c = list_at(&cell_list, indexes[i])->data;
    // search the orientation of the feeder - the same that the next cell having no wall
    for (j = 0; j < c->link_number; j++) {
      if (!c->links[j]->wall) {
        if (i < 4)
          c->feeder = get_orientation(c, get_linked_cell(c, c->links[j]));
        else
          c->init_pos = get_orientation(c, get_linked_cell(c, c->links[j]));
        break;
      }
    }
    // set the walls as unremovable (for step 5)
    for (j = 0; j < c->link_number; j++)
      c->links[j]->removable = false;
  }

  // 5. Remove some walls
  // remove 20% of the walls
  LLIST *tmp = maze->links;
  while (tmp) {
    Link *l = tmp->data;
    if (l->wall && l->removable) {
      int r = RANDOM(0, 100);  // Wow, random 100
      if (r < 20)
        l->wall = false;
    }
    tmp = tmp->next;
  }

  return true;
}

// free an initialized maze
void delete_maze(Maze *maze) {
  int i, j, k;
  int dimension = maze->width * maze->height;
  for (i = 0; i < dimension; i++) {
    Cell *cell = maze->cells[i];
    for (j = 0; j < cell->link_number; j++) {
      Link *link = cell->links[j];
      if (link) {
        Cell *linked_cell = get_linked_cell(cell, link);
        for (k = 0; k < linked_cell->link_number; k++) {
          if (linked_cell->links[k] == link)
            linked_cell->links[k] = NULL;
        }
        free(link);
        link = NULL;
      }
    }
    free(cell);
  }
  free(maze->cells);
}

// display the maze into a Display device
void display_maze(Maze *maze, WbDeviceTag display) {
  // check that the rendering is possible
  int display_width = wb_display_get_width(display);
  int display_height = wb_display_get_width(display);
  if (display_width != 2 * maze->width + 1 || display_height != 2 * maze->height + 1) {
    fprintf(stderr, "The dimension of the Display is badly defined\n");
    return;
  }

  // stuff
  int i, j, w;
  int gray = 0x808080, white = 0xFFFFFF, black = 0x000000, red = 0xFF0000, blue = 0x0000FF;

  // display background
  wb_display_set_color(display, white);
  wb_display_fill_rectangle(display, 0, 0, display_width, display_height);
  wb_display_set_color(display, gray);
  wb_display_draw_rectangle(display, 0, 0, display_width, display_height);
  // display the intervals
  for (j = 0; j < maze->height - 1; j++) {
    for (i = 0; i < maze->width - 1; i++)
      wb_display_draw_pixel(display, 2 * i + 2, 2 * j + 2);
  }

  // display the walls and the special cells (init and feeder)
  wb_display_set_color(display, black);
  for (j = 0; j < maze->height; j++) {
    for (i = 0; i < maze->width; i++) {
      Cell *c = maze->cells[maze->width * j + i];
      int indexX = (2 * i + 1);
      int indexY = (2 * j + 1);
      if (c->feeder != None) {
        wb_display_set_color(display, red);
        wb_display_draw_pixel(display, indexX, indexY);
        wb_display_set_color(display, black);
      }
      if (c->init_pos != None) {
        wb_display_set_color(display, blue);
        wb_display_draw_pixel(display, indexX, indexY);
        wb_display_set_color(display, black);
      }
      for (w = 0; w < c->link_number; w++) {
        if (c->links[w]->wall) {
          Cell *n = get_linked_cell(c, c->links[w]);
          Orientation o = get_orientation(c, n);
          switch (o) {
            case North:
              wb_display_draw_pixel(display, indexX, indexY - 1);
              break;
            case South:
              wb_display_draw_pixel(display, indexX, indexY + 1);
              break;
            case West:
              wb_display_draw_pixel(display, indexX - 1, indexY);
              break;
            case East:
              wb_display_draw_pixel(display, indexX + 1, indexY);
              break;
            default:
              assert(0);
          }
        }
      }
    }
  }
}
