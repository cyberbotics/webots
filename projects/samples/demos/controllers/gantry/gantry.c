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
 * Description:  Demo of a gantry robot playing the Hanoi Towers
 *               The robot moves three colored blocks from platform 1 to
 *               platform 2, using the recursive Hanoi Towers algorithm.
 *               The blocks original order is preserved.
 */

#include <webots/motor.h>
#include <webots/robot.h>

#define NUM_MOTORS 9
#define POS_MOTORS 6
#define CONTROL_STEP 2300

/* predefined platform positions */
#define Z1 0.0
#define Z2 5.0
#define Z3 10.0
#define X1 0.8
#define X2 0.0
#define X3 -0.8
#define ALPHA1 -3.1416
#define ALPHA2 0.0
#define ALPHA3 +3.1416
#define HI 0.0
#define LO 0.72
#define OPEN 0.00
#define CLOSE 0.09

static int motors[NUM_MOTORS];
static int num_states;
static int state;
typedef void (*CommandType)();

/*
 * The number of moves required by the Hanoi Towers
 * algorithm is 2^N-1, where N is the number of blocks.
 * Each move requires 8 function calls.
 */
static CommandType commands[(64 - 1) * 8];

/* actuation vectors */
static const double target[3][POS_MOTORS] = {
  {Z1, Z1, Z1, Z1, X1, ALPHA1}, /* platform 1 */
  {Z2, Z2, Z2, Z2, X2, ALPHA2}, /* platform 2 */
  {Z3, Z3, Z3, Z3, X3, ALPHA3}, /* platform 3 */
};

static void open_grip() {
  wb_motor_set_position(motors[7], OPEN);
  wb_motor_set_position(motors[8], OPEN);
}

static void close_grip() {
  wb_motor_set_position(motors[7], CLOSE);
  wb_motor_set_position(motors[8], CLOSE);
}

static void raise() {
  wb_motor_set_position(motors[6], HI);
}

static void lower() {
  wb_motor_set_position(motors[6], LO);
}

static void goto1() {
  int i;

  for (i = 0; i < POS_MOTORS; i++)
    wb_motor_set_position(motors[i], target[0][i]);
}

static void goto2() {
  int i;
  for (i = 0; i < POS_MOTORS; i++)
    wb_motor_set_position(motors[i], target[1][i]);
}

static void goto3() {
  int i;
  for (i = 0; i < POS_MOTORS; i++)
    wb_motor_set_position(motors[i], target[2][i]);
}

static void move(int pos) {
  switch (pos) {
    case 1:
      commands[num_states++] = goto1;
      break;
    case 2:
      commands[num_states++] = goto2;
      break;
    case 3:
      commands[num_states++] = goto3;
      break;
  }
}

/*
 * Move a single block from src to dst
 * as a sequence of 8 motions.
 */
static void moveDisc(int src, int dst) {
  move(src);
  commands[num_states++] = lower;
  commands[num_states++] = close_grip;
  commands[num_states++] = raise;
  move(dst);
  commands[num_states++] = lower;
  commands[num_states++] = open_grip;
  commands[num_states++] = raise;
}

/* Move an entire pile of blocks recursively */
static void moveTower(int n, int src, int aux, int dst) {
  if (n == 0)
    return;

  moveTower(n - 1, src, dst, aux);
  moveDisc(src, dst);
  moveTower(n - 1, aux, src, dst);
}

int main() {
  const char *MOTOR_NAMES[NUM_MOTORS] = {"wheel1_motor", "wheel2_motor", "wheel3_motor", "wheel4_motor", "bridge_motor",
                                         "turret_motor", "lift_motor",   "grip_motor1",  "grip_motor2"};
  int i;
  num_states = 0;
  state = 0;

  wb_robot_init();

  for (i = 0; i < NUM_MOTORS; i++)
    motors[i] = wb_robot_get_device(MOTOR_NAMES[i]);

  moveTower(3, 1, 2, 3);

  while (wb_robot_step(CONTROL_STEP) != -1) {
    if (state < num_states)
      (*commands[state])();
    state++;
  }

  wb_robot_cleanup();

  return 0;
}
