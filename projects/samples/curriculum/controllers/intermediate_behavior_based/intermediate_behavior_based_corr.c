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

// Included libraries
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

// Global defines
#define THRESHOLD_DIST 50
#define TIME_STEP 32  // [ms] // time step of the simulation
#define TIME_STEP_CAM 32
#define SIMULATION 0  // for robot_get_mode() function
#define REALITY 2     // for robot_get_mode() function
#define LEFT 0        // Left side
#define RIGHT 1       // right side
#define NO_SIDE 2;    // neither left or right
#define SPEED_UNIT 0.00628

// 8 IR proximity sensors
#define NB_DIST_SENS 8
#define PS_RIGHT_10 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_10 7
WbDeviceTag ps[NB_DIST_SENS]; /* proximity sensors */
int ps_value[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
int ps_offset_sim[NB_DIST_SENS] = {35, 35, 35, 35, 35, 35, 35, 35};
int ps_offset_real[NB_DIST_SENS] = {175, 117, 108, 137, 108, 160, 109, 138};  // to be modified according to your robot
int obstacle[NB_DIST_SENS];  // will contain a boolean information about obstacles

// motors
WbDeviceTag left_motor, right_motor;

// camera
WbDeviceTag cam;
unsigned short width, height;

/*****************************
 *
 *  Functions
 *
 ******************************/

// This function returns the position
// of the peak contained in the array given
// in argument
int find_middle(const int tab[], int sizeTab) {
  int i, j;
  int copy[sizeTab];
  int mid = 0;
  int nb_best = sizeTab / 10;
  int index_bests[nb_best];

  // copy the tab, calculate the mean and
  // test if all the values are identical
  int identical = 1;
  for (i = 0; i < sizeTab; i++) {
    copy[i] = tab[i];
    mid += tab[i];
    if (tab[i] != tab[0])
      identical = 0;
  }
  if (identical)
    return sizeTab / 2;
  mid /= sizeTab;

  // take the best values of the tab
  for (i = 0; i < nb_best; i++) {
    int index = -1;
    int max = 0;
    for (j = 0; j < sizeTab; j++) {
      if (max < copy[j] && copy[j] > mid) {
        max = copy[j];
        index = j;
      }
    }
    assert(index >= 0);
    index_bests[i] = index;
    copy[index] = 0;
  }

  // calculate the position mean of th best values
  int firstMean = 0;
  int count = 0;
  for (i = 0; i < nb_best; i++) {
    if (index_bests[i] != -1) {
      firstMean += index_bests[i];
      count++;
    }
  }
  if (count == 0)
    return sizeTab / 2;
  firstMean /= count;

  // eliminate extrem values
  int secondMean = 0;
  count = 0;
  for (i = 0; i < nb_best; i++) {
    if (index_bests[i] < firstMean + sizeTab / 10 && index_bests[i] > firstMean - sizeTab / 10) {
      count++;
      secondMean += index_bests[i];
    }
  }
  if (count == 0)
    return sizeTab / 2;

  return secondMean / count;
}

// return the mean of the values of an array
int mean(const int array[], int size) {
  if (size == 0)
    return 0;
  int sum = 0, i;
  for (i = 0; i < size; i++)
    sum += array[i];
  return sum / size;
}

/*****************************
 *
 *  Modules
 *
 ******************************/

// Obstacle avoidance module
#define OAM_K_90 0.08
#define OAM_K_45 0.12
#define OAM_K_10 0.12
#define OAM_K_MAX_DELTAS 700
#define WFM_MAX_SPEED 100
int oam_speed[2];
int side = NO_SIDE;
void oam(void) {
  double oam_delta = 0;
  oam_speed[LEFT] = oam_speed[RIGHT] = 0;

  if (obstacle[PS_LEFT_10] || obstacle[PS_LEFT_45] || obstacle[PS_LEFT_90]) {
    oam_delta -= (int)(OAM_K_90 * ps_value[PS_LEFT_90]);
    oam_delta -= (int)(OAM_K_45 * ps_value[PS_LEFT_45]);
    oam_delta -= (int)(OAM_K_10 * ps_value[PS_LEFT_10]);
    if (oam_delta < -WFM_MAX_SPEED)
      side = LEFT;

    /* ... */

  } else if (obstacle[PS_RIGHT_10] || obstacle[PS_RIGHT_45] || obstacle[PS_RIGHT_90]) {
    oam_delta += (int)(OAM_K_90 * ps_value[PS_RIGHT_90]);
    oam_delta += (int)(OAM_K_45 * ps_value[PS_RIGHT_45]);
    oam_delta += (int)(OAM_K_10 * ps_value[PS_RIGHT_10]);
    if (oam_delta > WFM_MAX_SPEED)
      side = RIGHT;

    /* ... */
  }

  if (oam_delta > OAM_K_MAX_DELTAS)
    oam_delta = OAM_K_MAX_DELTAS;
  if (oam_delta < -OAM_K_MAX_DELTAS)
    oam_delta = -OAM_K_MAX_DELTAS;

  oam_speed[LEFT] -= oam_delta;
  oam_speed[RIGHT] += oam_delta;
}

// obstacle following module
int wfm_speed[] = {0, 0};
void wfm(void) {
  switch (side) {
    case LEFT:
      wfm_speed[LEFT] = -WFM_MAX_SPEED;
      wfm_speed[RIGHT] = WFM_MAX_SPEED;
      break;
    case RIGHT:
      wfm_speed[LEFT] = WFM_MAX_SPEED;
      wfm_speed[RIGHT] = -WFM_MAX_SPEED;
      break;
    default:
      wfm_speed[LEFT] = 0;
      wfm_speed[RIGHT] = 0;
  }
}

// Line following module
#define MAX_DELTA 300.0f
int lfm_speed[2] = {0, 0};
int lfm_active = 1;
void lfm(int array[], int size) {
  if (lfm_active) {
    int delta = find_middle(array, size) - width / 2;
    lfm_speed[LEFT] = MAX_DELTA * delta / size;
    lfm_speed[RIGHT] = -lfm_speed[LEFT];
  } else
    lfm_speed[RIGHT] = lfm_speed[LEFT] = 0;
}

// line entering module
#define SENSIBILITY 10
int previous_mean[] = {0, 0, 0};
int current_mean[] = {0, 0, 0};
int is_in[] = {0, 0, 0};
void lem(const int array[], int size) {
  int left[size / 10];
  int right[size / 10];
  int middle[size / 10];
  int i;

  for (i = 0; i < size / 10; i++) {
    left[i] = array[i];
    right[i] = array[size - 1 - i];
    middle[i] = array[size / 2 - size / 20 + i];
  }

  current_mean[0] = mean(left, size / 10);
  current_mean[1] = mean(middle, size / 10);
  current_mean[2] = mean(right, size / 10);

  for (i = 0; i < 3; i++) {
    if (current_mean[i] > previous_mean[i] + SENSIBILITY) {
      is_in[i] = 1;
      side = NO_SIDE;
    }
  }

  if (is_in[0] || is_in[1] || is_in[2])
    lfm_active = 1;
}
// line leaving module
void llm(int array[], int size) {
  int i;
  for (i = 0; i < 3; i++) {
    if (current_mean[i] < previous_mean[i] - SENSIBILITY)
      is_in[i] = 0;
    previous_mean[i] = current_mean[i];
  }

  if (!is_in[0] && !is_in[1] && !is_in[2] && i < size)
    is_in[i] = 1;
}

/*****************************
 *
 *  Standard Functions
 *
 ******************************/

static void reset(void) {
  int it;

  // get distance sensors
  char textPS[] = "ps0";
  for (it = 0; it < NB_DIST_SENS; it++) {
    ps[it] = wb_robot_get_device(textPS);
    textPS[2]++;
  }

  // enable distance sensor and light sensor devices
  int i;
  for (i = 0; i < NB_DIST_SENS; i++)
    wb_distance_sensor_enable(ps[i], TIME_STEP);

  // get a handler to the motors and set target position to infinity (speed control)
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // enable the camera
  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, TIME_STEP_CAM);
  width = wb_camera_get_width(cam);
  height = wb_camera_get_height(cam);
}

static int run(void) {
  int ps_offset[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
  int i, gray[width];
  unsigned char *image;
  const int speed[2] = {100, 100};  // speed without modules
  int mode = wb_robot_get_mode();

  // 0. Preprocessing
  // Obtain the correct offset
  if (mode == SIMULATION) {
    for (i = 0; i < NB_DIST_SENS; i++)
      ps_offset[i] = ps_offset_sim[i];
  } else {
    for (i = 0; i < NB_DIST_SENS; i++)
      ps_offset[i] = ps_offset_real[i];
  }

  // 1. Get the sensors values
  // obstacle[] will contain a boolean information about a collision
  for (i = 0; i < NB_DIST_SENS; i++) {
    ps_value[i] = (int)wb_distance_sensor_get_value(ps[i]);
    obstacle[i] = ps_value[i] - ps_offset[i] > THRESHOLD_DIST;
  }

  image = wb_camera_get_image(cam);
  for (i = 0; i < width; i++)
    gray[i] = 255 - wb_camera_image_get_gray(image, width, i, 0);

  // 2. Behavior-based robotic:
  // call the modules in the right order
  oam();
  lem(gray, width);
  lfm(gray, width);
  llm(gray, width);
  wfm();

  // 3. Send the values to actuators
  wb_motor_set_velocity(left_motor, SPEED_UNIT * (speed[LEFT] + lfm_speed[LEFT] + oam_speed[LEFT] + wfm_speed[LEFT]));
  wb_motor_set_velocity(right_motor, SPEED_UNIT * (speed[RIGHT] + lfm_speed[RIGHT] + oam_speed[RIGHT] + wfm_speed[RIGHT]));

  return TIME_STEP;
}

int main() {
  wb_robot_init();

  reset();

  while (wb_robot_step(TIME_STEP) != -1)
    run();

  wb_robot_cleanup();

  return 0;
}
