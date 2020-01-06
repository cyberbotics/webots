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
 * Description:  Structure defining a maze
 */

#ifndef MAZE_DEFINITION_H
#define MAZE_DEFINITION_H

#include "boolean.h"
#include "linked_list.h"
// enum
typedef enum { North, South, East, West, None } Orientation;

// prototypes
struct _Link;
struct _Cell;
struct _Maze;

// link between two cells
typedef struct _Link {
  bool wall;       // is there a wall between cell_A and cell_B ?
  bool removable;  // Cannot be removed if located close to a feeder
  struct _Cell *cell_A;
  struct _Cell *cell_B;
} Link;

// a cell is made of up to 4 links
typedef struct _Cell {
  int pos_x;
  int pos_y;
  bool visited;
  Orientation feeder;
  Orientation init_pos;
  int link_number;
  struct _Link **links;  // array of links
} Cell;

// the maze
typedef struct _Maze {
  int width;
  int height;
  struct _Cell **cells;  // array of cells
  LLIST *links;          // list of links
} Maze;

#endif
