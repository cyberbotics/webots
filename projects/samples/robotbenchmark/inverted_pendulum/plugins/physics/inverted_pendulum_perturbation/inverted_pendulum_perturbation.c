/*
 * Copyright 1996-2022 Cyberbotics Ltd.
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
 * Apply some random force to the inverted pendulum
 * to simulate external perturbation (e.g. wind)
 */

#include <ode/ode.h>
#include <plugins/physics.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>

// number of steps during which the perturbation force is applied
#define FORCE_STEPS 4

static dBodyID pendulum_body = NULL;
static bool rand_seed_initialized = false;

static double rand2() {
  return (double)rand() / (double)RAND_MAX;
}

void webots_physics_init() {
  // get ODE body of the pendulum
  pendulum_body = dWebotsGetBodyFromDEF("PENDULUM");
  if (pendulum_body == 0)
    dWebotsConsolePrintf("WARNING: Solid node with DEF name 'PENDULUM' not found. Physics Plugin is disabled.");
}

void webots_physics_step() {
  if (pendulum_body == 0)
    // nothing to do
    return;

  static dVector3 forceVector = {0.0, 0.0, 0.0, 0.0};
  static int force_steps = 0;  // remaining steps where force is applied

  int data_size;
  const char *data = (const char *)dWebotsReceive(&data_size);
  if (data_size > 0) {
    double force;
    int seed;
    int filled_items = sscanf(data, "%lf %d", &force, &seed);
    if (filled_items > 0) {
      if (!rand_seed_initialized && filled_items > 1) {
        srand(seed);
        rand_seed_initialized = true;
      }
      // set force vector with random direction
      forceVector[1] = (rand2() > 0.5) ? force : -force;
      force_steps = FORCE_STEPS;
    }
  }

  if (force_steps > 0) {
    // apply force to the pendulum
    dBodyAddRelForce(pendulum_body, forceVector[0], forceVector[1], forceVector[2]);
    --force_steps;
  }
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  // nothing to do
  return 0;
}

void webots_physics_cleanup() {
  // nothing to do
}
