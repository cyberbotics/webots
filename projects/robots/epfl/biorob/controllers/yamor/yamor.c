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
 * Description:  Modular robot simulation and demonstration of Webots connectors.
 *               This is the controller code a single YAMOR module and it will
 *               be instantiated N times. The modules behaviour is synchronized
 *               though a common time basis.
 */

#include <math.h>
#include <stdlib.h>
#include <webots/connector.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define CONTROL_STEP 32
#define N 8

static double t = 0.0; /* time elapsed since simulation start [s] */
static int id = -1;    /* this module's ID */

/* each module is equipped with a single motormotor and 2 connectors */
static WbDeviceTag motor, rear_connector, front_connector;

/* worm movement oscillations */
static const double F = 1.0; /* frequency */
static const double A = 0.9; /* amplitude */

static void loop_8_modules() {
  /*
   * Lookup positions for loop_8_modules locomotion
   * 4 motors are straight and 4 motors move to an angle of 90
   */
  const double LOOKUP[8] = {1, 1, 0, 0, 1, 1, 0, 0};

  wb_motor_set_position(motor, -1.5708 * LOOKUP[(id + (int)t) % 8]);
}

static void loop_6_modules() {
  /*
   * Lookup positions for loop_8_modules locomotion
   * 4 motors are straight and 4 motors move to an angle of 90
   */
  const double LOOKUP[6] = {0, 1, 1, 0, 1, 1};

  wb_motor_set_position(motor, -1.5708 * LOOKUP[(id + (int)t) % 6]);

  return;
}

static void worm_8_modules() {
  double shift = id * (2.0 * M_PI / 8); /* phase shift for this module */
  double phase = 2.0 * M_PI * F * t;

  wb_motor_set_position(motor, A * sin(phase + shift));
}

static void worm_6_modules() {
  double shift = id * (2.0 * M_PI / 6); /* phase shift for this module */
  double phase = 2.0 * M_PI * F * t;

  wb_motor_set_position(motor, A * sin(phase + shift));
}

static void connect_0_and_7() {
  static int done = 0;

  if (done)
    return;

  /* modules 0 and 7 should be aligned: try to lock */
  if (id == 0)
    wb_connector_lock(front_connector);
  if (id == 7)
    wb_connector_lock(rear_connector);

  done = 1;
}

static void connect_0_and_5() {
  static int done = 0;

  if (done)
    return;

  /* modules 0 and 5 should be aligned: try to lock */
  if (id == 0)
    wb_connector_lock(front_connector);
  if (id == 5)
    wb_connector_lock(rear_connector);

  done = 1;
}

static void drop_module_7() {
  static int done = 0;

  if (done)
    return;

  /* drop tail module */
  if (id == 0)
    wb_connector_unlock(front_connector);
  else if (id == 6)
    wb_connector_unlock(rear_connector);
  else if (id == 7) {
    wb_connector_unlock(front_connector);
    wb_connector_unlock(rear_connector);
  }

  done = 1;
}

static void drop_module_6() {
  static int done = 0;

  if (done)
    return;

  /* drop tail module */
  if (id == 5)
    wb_connector_unlock(rear_connector);
  else if (id == 6) {
    wb_connector_unlock(front_connector);
    wb_motor_set_position(motor, 1.5); /* move a bit to make it fall */
  }

  done = 1;
}

static void disconnect_all() {
  static int done = 0;

  if (done)
    return;

  /* detach everything */
  wb_connector_unlock(front_connector);
  wb_connector_unlock(rear_connector);
  done = 1;
}

static void do_nothing() {
}

/* demo steps and durations */
struct {
  void (*func)();
  float duration;
} states[] = {{worm_8_modules, 7}, {loop_8_modules, 1}, {do_nothing, 1},      {connect_0_and_7, 1}, {loop_8_modules, 9},
              {do_nothing, 1},     {drop_module_7, 1},  {loop_8_modules, 10}, {do_nothing, 1},      {drop_module_6, 1},
              {worm_6_modules, 8}, {do_nothing, 1},     {loop_6_modules, 1},  {do_nothing, 1},      {connect_0_and_5, 1},
              {loop_6_modules, 8}, {do_nothing, 1},     {disconnect_all, 1},  {loop_8_modules, 60}, {NULL, 1}};

static int state = 0;
static float deadline;

int main() {
  /* necessary to initialize Webots */
  wb_robot_init();

  /*
   * Find module id from robot name
   * The robot name is used to identify each module
   */
  const char *name = wb_robot_get_name();

  id = atoi(name + 7) - 1;

  /* find hardware devices */
  motor = wb_robot_get_device("motor");
  rear_connector = wb_robot_get_device("rear_connector");
  front_connector = wb_robot_get_device("front_connector");

  deadline = states[0].duration;

  while (wb_robot_step(CONTROL_STEP) != -1) {
    /* change state */
    if (t >= deadline) {
      state++;
      deadline += states[state].duration;
    }

    /* finished ? */
    if (!states[state].func)
      continue;

    /* call current state's function */
    (*states[state].func)();

    /* computed elapsed time */
    t += CONTROL_STEP / 1000.0;
  }

  wb_robot_cleanup();

  return 0;
}
