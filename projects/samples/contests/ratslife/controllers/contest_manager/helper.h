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
 * Description:  Definition of some generic helper functions
 */

#ifndef HELPER_H
#define HELPER_H

#define RANDOM(min, max) (rand() % (max - min) + min)  // random integer [min;max[
#define ROUND(x) ((int)(0.5 + x))
#define MAX(a, b) ((a) < (b) ? (b) : (a))

enum { CX, CY, CZ, ALPHA };
enum { R0, R1 };
enum { WHITE = 0xFFFFFF, RED = 0xFF0000, GREEN = 0x00FF00, BLUE = 0x0000FF, YELLOW = 0xFFFF00, BLACK = 0x000000 };

#endif
