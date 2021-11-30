/*
 * Copyright 1996-2021 Cyberbotics Ltd.
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

#include <ode/ode.h>
#include <plugins/physics.h>

static pthread_mutex_t mutex;  // needed to run with multi-threaded version of ODE

void webots_physics_init() {
  pthread_mutex_init(&mutex, NULL);

  dBodyID ball;
  char name[8];

  for (int i = 0; i < 5; ++i) {
    sprintf(name, "BALL_%d", i+6);
    ball = dWebotsGetBodyFromDEF(name);
    dBodyAddTorque(ball, 0, 0, 30);
  }
}

void webots_physics_step() {
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  return 0;
}

void webots_physics_cleanup() {
  pthread_mutex_destroy(&mutex);
}
