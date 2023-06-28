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
 * Description:  A controller moving various robots using the Braitenberg method.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define MAX_SENSOR_NUMBER 16
#define RANGE (1024 / 2)
#define BOUND(x, a, b) (((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x))

static WbDeviceTag sensors[MAX_SENSOR_NUMBER], camera, left_motor, right_motor;
static double matrix[MAX_SENSOR_NUMBER][2];
static int num_sensors;
static double range;
static int time_step = 0;
static double max_speed = 0.0;
static double speed_unit = 1.0;

/* We use this variable to enable a camera device if the robot has one. */
static int camera_enabled;

static void initialize() {
  /* necessary to initialize Webots */
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();

  const char *robot_name = wb_robot_get_name();

  const char e_puck_name[] = "ps0";
  const char khepera_name[] = "ds0";
  const char koala_name[] = "ds0";
  const char pioneer2_name[] = "ds0";

  char sensors_name[5];
  const double(*temp_matrix)[2];

  camera_enabled = 0;
  range = RANGE;

  const double e_puck_matrix[8][2] = {{150, -35}, {100, -15}, {80, -10},  {-10, -10},
                                      {-10, -10}, {-10, 80},  {-30, 100}, {-20, 150}};
  const double khepera3_matrix[9][2] = {{-5000, -5000},  {-20000, 40000}, {-30000, 50000}, {-70000, 70000}, {70000, -60000},
                                        {50000, -40000}, {40000, -20000}, {-5000, -5000},  {-10000, -10000}};
  const double khepera_matrix[8][2] = {{-2, 4}, {-3, 5}, {-7, 7}, {7, -6}, {5, -4}, {4, -2}, {-0.5, -0.5}, {-0.5, -0.5}};
  const double pioneer2_matrix[16][2] = {{-1, 15}, {-3, 13}, {-3, 8},  {-2, 7}, {-3, -4}, {-4, -2}, {-3, -2}, {-1, -1},
                                         {-1, -1}, {-2, -3}, {-2, -4}, {4, -3}, {7, -5},  {7, -3},  {10, -2}, {11, -1}};
  const double koala_matrix[16][2] = {{17, -1}, {8, -2},  {4, -3},  {9, -2}, {5, -3}, {-4, -2}, {-4, -2}, {-2, -2},
                                      {-2, -2}, {-2, -4}, {-2, -4}, {-4, 5}, {-3, 8}, {-3, 5},  {-2, 10}, {-1, 15}};

  /*
   * Here we adapt the generic variables to the specificity of the current
   * robot. We need to adapt the number of sensors it has, their name and
   * finally the matrix used by the Braitenberg algorithm.
   */
  if (strncmp(robot_name, "e-puck", 6) == 0) {
    const double epuck_max_speed = 6.28;
    const double epuck_speed_unit = 0.00628;

    num_sensors = 8;
    sprintf(sensors_name, "%s", e_puck_name);
    temp_matrix = e_puck_matrix;
    max_speed = epuck_max_speed;
    speed_unit = epuck_speed_unit;

    if (strcmp(robot_name, "e-puck camera") == 0)
      camera_enabled = 1;
  } else if (strncmp(robot_name, "Khepera III", 8) == 0) {
    const double khepera3_max_speed = 19.1;
    const double khepera3_speed_unit = 0.00053429;

    num_sensors = 9;
    sprintf(sensors_name, "%s", khepera_name);
    temp_matrix = khepera3_matrix;
    range = 2000;
    max_speed = khepera3_max_speed;
    speed_unit = khepera3_speed_unit;
  } else if (strncmp(robot_name, "khepera", 7) == 0) {
    const double khepera_max_speed = 1.0;
    const double khepera_speed_unit = 1.0;

    num_sensors = 8;
    sprintf(sensors_name, "%s", khepera_name);
    temp_matrix = khepera_matrix;
    max_speed = khepera_max_speed;
    speed_unit = khepera_speed_unit;
  } else if (strcmp(robot_name, "koala") == 0) {
    const double koala_max_speed = 10.0;
    const double koala_speed_unit = 0.1;

    num_sensors = 16;
    sprintf(sensors_name, "%s", koala_name);
    temp_matrix = koala_matrix;
    max_speed = koala_max_speed;
    speed_unit = koala_speed_unit;
  } else if (strcmp(robot_name, "pioneer2") == 0) {
    const double pioneer2_max_speed = 10.0;
    const double pioneer2_speed_unit = 0.1;

    num_sensors = 16;
    sprintf(sensors_name, "%s", pioneer2_name);
    temp_matrix = pioneer2_matrix;
    max_speed = pioneer2_max_speed;
    speed_unit = pioneer2_speed_unit;
  } else {
    fprintf(stderr, "This controller doesn't support this robot\n");
    exit(EXIT_FAILURE);
  }

  int i;
  for (i = 0; i < num_sensors; i++) {
    sensors[i] = wb_robot_get_device(sensors_name);
    wb_distance_sensor_enable(sensors[i], time_step);

    if ((i + 1) >= 10) {
      sensors_name[2] = '1';
      sensors_name[3]++;

      if ((i + 1) == 10) {
        sensors_name[3] = '0';
        sensors_name[4] = '\0';
      }
    } else
      sensors_name[2]++;

    int j;
    for (j = 0; j < 2; j++)
      matrix[i][j] = temp_matrix[i][j];
  }

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  if (camera_enabled == 1) {
    camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, time_step);
  }

  printf("The %s robot is initialized, it uses %d distance sensors\n", robot_name, num_sensors);
}

int main() {
  initialize();

  while (wb_robot_step(time_step) != -1) {
    int i, j;
    double speed[2];
    double sensors_value[MAX_SENSOR_NUMBER];

    /* If there is a camera, we need to update refresh it. */
    if (camera_enabled == 1)
      wb_camera_get_image(camera);

    for (i = 0; i < num_sensors; i++)
      sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);

    /*
     * The Braitenberg algorithm is really simple, it simply computes the
     * speed of each wheel by summing the value of each sensor multiplied by
     * its corresponding weight. That is why each sensor must have a weight
     * for each wheel.
     */
    for (i = 0; i < 2; i++) {
      speed[i] = 0.0;

      for (j = 0; j < num_sensors; j++) {
        /*
         * We need to recenter the value of the sensor to be able to get
         * negative values too. This will allow the wheels to go
         * backward too.
         */
        speed[i] += speed_unit * matrix[j][i] * (1.0 - (sensors_value[j] / range));
      }

      speed[i] = BOUND(speed[i], -max_speed, max_speed);
    }

    /* Set the motor speeds */
    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
  }

  return 0;
}
