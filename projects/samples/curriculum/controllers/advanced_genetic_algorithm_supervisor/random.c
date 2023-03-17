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

#include "random.h"

#include <math.h>
#include <stdlib.h>

int random_get_integer(int max) {
  return rand() % max;
}

double random_get_uniform() {
  return (double)rand() / RAND_MAX;
}

// polar form of the Box-Muller transformation
double random_get_gaussian() {
  double x1, w;
  do {
    x1 = 2.0 * random_get_uniform() - 1.0;
    const double x2 = 2.0 * random_get_uniform() - 1.0;
    w = x1 * x1 + x2 * x2;
  } while (w >= 1.0);

  return x1 * sqrt(-2.0 * log(w) / w);
}
