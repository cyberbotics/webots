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
 * Description:  Reading a CSV file, getting motors positions, and playing the CSV file.
 *               CSV files are motion files used to control the real Hoap-2 robot.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <webots/emitter.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

typedef enum {
  body_joint_1,
  lleg_joint_1,
  lleg_joint_3,
  lleg_joint_2,
  lleg_joint_4,
  lleg_joint_5,
  lleg_joint_6,
  rleg_joint_1,
  rleg_joint_3,
  rleg_joint_2,
  rleg_joint_4,
  rleg_joint_5,
  rleg_joint_6,
  larm_joint_1,
  larm_joint_2,
  larm_joint_3,
  larm_joint_4,
  larm_joint_5,
  rarm_joint_1,
  rarm_joint_2,
  rarm_joint_3,
  rarm_joint_4,
  rarm_joint_5,
  head_joint_2,
  head_joint_1
} joints;

static WbDeviceTag emitter, left_touch, right_touch, gps;
static double motor_position[] = {
  0.0,                          /* body */
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, /* lleg */
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, /* rleg */
  0.0, 0.0, 0.0, 0.0, 0.0,      /* larm */
  0.0, 0.0, 0.0, 0.0, 0.0,      /* rarm */
  0.0, 0.0                      /* head */
};
static double next_position[] = {
  0.0,                          /* body */
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, /* lleg */
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, /* rleg */
  0.0, 0.0, 0.0, 0.0, 0.0,      /* larm */
  0.0, 0.0, 0.0, 0.0, 0.0,      /* rarm */
  0.0, 0.0                      /* head */
};
static WbDeviceTag joint[25];         /* all the motors */
static WbDeviceTag joint_sensors[25]; /* associated_sensors */

static const char *joint_number_to_name(int num) {
  switch (num) {
    case body_joint_1:
      return "body_joint_1";
    case lleg_joint_1:
      return "lleg_joint_1";
    case lleg_joint_3:
      return "lleg_joint_3";
    case lleg_joint_2:
      return "lleg_joint_2";
    case lleg_joint_4:
      return "lleg_joint_4";
    case lleg_joint_5:
      return "lleg_joint_5";
    case lleg_joint_6:
      return "lleg_joint_6";
    case rleg_joint_1:
      return "rleg_joint_1";
    case rleg_joint_3:
      return "rleg_joint_3";
    case rleg_joint_2:
      return "rleg_joint_2";
    case rleg_joint_4:
      return "rleg_joint_4";
    case rleg_joint_5:
      return "rleg_joint_5";
    case rleg_joint_6:
      return "rleg_joint_6";
    case larm_joint_1:
      return "larm_joint_1";
    case larm_joint_2:
      return "larm_joint_2";
    case larm_joint_3:
      return "larm_joint_3";
    case larm_joint_4:
      return "larm_joint_4";
    case larm_joint_5:
      return "larm_joint_5";
    case rarm_joint_1:
      return "rarm_joint_1";
    case rarm_joint_2:
      return "rarm_joint_2";
    case rarm_joint_3:
      return "rarm_joint_3";
    case rarm_joint_4:
      return "rarm_joint_4";
    case rarm_joint_5:
      return "rarm_joint_5";
    case head_joint_2:
      return "head_joint_2";
    case head_joint_1:
      return "head_joint_1";
    default:
      return "none";
  }
}

int main(int argc, char *argv[]) {
  int i, sampling;
  int com_interval;
  int pos_from_cvs[22];
  char l[500];
  int file_ended = 0;
  const int pulse[] = {209,  209, 209,  209, -209, -209, -209, 209,  -209, 209, 209, 209, -209,
                       -209, 209, -209, 209, 0,    209,  209,  -209, -209, 0,   0,   0};
  int tempMotor[] = {0,                /* body */
                     0, 0, 0, 0, 0, 0, /* lleg */
                     0, 0, 0, 0, 0, 0, /* rleg */
                     0, 0, 0, 0, 0,    /* larm */
                     0, 0, 0, 0, 0,    /* rarm */
                     0, 0};            /* head */

  /* initialize Webots */
  wb_robot_init();

  int control_step;
  const char *filename;
  if (strcmp(argv[1], "sumo") == 0) {
    filename = "sumo.csv";
    control_step = 64;
  } else {
    filename = "walk.csv"; /* name should be "Hoap-2 walk" */
    control_step = 50;
  }

  for (i = 0; i < 25; i++) {
    joint[i] = wb_robot_get_device(joint_number_to_name(i));
    joint_sensors[i] = wb_motor_get_position_sensor(joint[i]);
    wb_position_sensor_enable(joint_sensors[i], control_step);
  }

  /*
   * The following devices are available, although they are not used in this
   * controller program.
   */
  left_touch = wb_robot_get_device("left touch");
  right_touch = wb_robot_get_device("right touch");
  wb_touch_sensor_enable(left_touch, control_step);
  wb_touch_sensor_enable(right_touch, control_step);
  gps = wb_robot_get_device("gps");
  emitter = wb_robot_get_device("emitter");
  FILE *file = fopen(filename, "r");
  if (file == NULL) {
    printf("unable to locate the %s file\n", filename);
    return 1;
  }

  tempMotor[larm_joint_5] = 0; /* currently not needed, */
  tempMotor[rarm_joint_5] = 0; /* because we don't use the fingers */

  next_position[larm_joint_5] = 0.0;
  next_position[rarm_joint_5] = 0.0;

  char *ptr = fgets(l, 500, file);
  if (ptr == NULL) {
    fprintf(stderr, "Error while reading the %s file\n", filename);
    fclose(file);
    return 1;
  }

  sscanf(l,
         "%d, %*d, %d, %d, %d, %d ,%d, %d, %d, %d, %d, %d, %d, %d, %d, "
         "%d, %d, %d, %d, %d, %d, %d, %d, %*c ,%*c ,%*c,%*c ",
         &sampling, &tempMotor[rleg_joint_1], &tempMotor[rleg_joint_2], &tempMotor[rleg_joint_3], &tempMotor[rleg_joint_4],
         &tempMotor[rleg_joint_5], &tempMotor[rleg_joint_6], &tempMotor[rarm_joint_1], &tempMotor[rarm_joint_2],
         &tempMotor[rarm_joint_3], &tempMotor[rarm_joint_4], &tempMotor[lleg_joint_1], &tempMotor[lleg_joint_2],
         &tempMotor[lleg_joint_3], &tempMotor[lleg_joint_4], &tempMotor[lleg_joint_5], &tempMotor[lleg_joint_6],
         &tempMotor[larm_joint_1], &tempMotor[larm_joint_2], &tempMotor[larm_joint_3], &tempMotor[larm_joint_4],
         &tempMotor[body_joint_1]);

  for (i = 0; i < 23; i++)
    motor_position[i] = tempMotor[i] * (M_PI / 180.0) / pulse[i];

  motor_position[larm_joint_5] = 0.0;
  motor_position[rarm_joint_5] = 0.0;
  com_interval = control_step / sampling;

  /* We wait a little bit before starting. */
  wb_robot_step(50 * control_step);

  for (i = 0; i < 25; i++)
    next_position[i] = wb_position_sensor_get_value(joint_sensors[i]);

  for (;;) {           /* The robot never dies! */
    if (!file_ended) { /* else don't need to run, just a step */
      for (i = 0; i < com_interval; i++) {
        if (!file_ended) {
          /* We have reached the end of the file. */
          if (fgets(l, 500, file) == NULL) {
            fclose(file);
            file_ended = 1;
          }
        }
      }
      if (file_ended == 0) {
        /* Read the data from the current line... */
        sscanf(l,
               "%*d, %*d, %d, %d, %d, %d ,%d, %d, %d, %d, %d,"
               " %d, %d, %d, %d, %d, %d, %d, %d, %d, %d,"
               " %d, %d, %*c ,%*c ,%*c,%*c ",
               &pos_from_cvs[rleg_joint_1], &pos_from_cvs[rleg_joint_2], &pos_from_cvs[rleg_joint_3],
               &pos_from_cvs[rleg_joint_4], &pos_from_cvs[rleg_joint_5], &pos_from_cvs[rleg_joint_6],
               &pos_from_cvs[rarm_joint_1], &pos_from_cvs[rarm_joint_2], &pos_from_cvs[rarm_joint_3],
               &pos_from_cvs[rarm_joint_4], &pos_from_cvs[lleg_joint_1], &pos_from_cvs[lleg_joint_2],
               &pos_from_cvs[lleg_joint_3], &pos_from_cvs[lleg_joint_4], &pos_from_cvs[lleg_joint_5],
               &pos_from_cvs[lleg_joint_6], &pos_from_cvs[larm_joint_1], &pos_from_cvs[larm_joint_2],
               &pos_from_cvs[larm_joint_3], &pos_from_cvs[larm_joint_4], &pos_from_cvs[body_joint_1]);
        for (i = 0; i < 22; i++) /* convert to radian */
          next_position[i] = pos_from_cvs[i] * (M_PI / 180.0) / pulse[i];
      }

      /* Now assign the new angle values. */
      wb_motor_set_position(joint[body_joint_1], next_position[body_joint_1]);
      wb_motor_set_position(joint[lleg_joint_1], next_position[lleg_joint_1]);
      wb_motor_set_position(joint[lleg_joint_2], next_position[lleg_joint_2]);
      wb_motor_set_position(joint[lleg_joint_3], next_position[lleg_joint_3]);
      wb_motor_set_position(joint[lleg_joint_4], next_position[lleg_joint_4]);
      wb_motor_set_position(joint[lleg_joint_5], next_position[lleg_joint_5]);
      wb_motor_set_position(joint[lleg_joint_6], next_position[lleg_joint_6]);
      wb_motor_set_position(joint[rleg_joint_1], next_position[rleg_joint_1]);
      wb_motor_set_position(joint[rleg_joint_2], next_position[rleg_joint_2]);
      wb_motor_set_position(joint[rleg_joint_3], next_position[rleg_joint_3]);
      wb_motor_set_position(joint[rleg_joint_4], next_position[rleg_joint_4]);
      wb_motor_set_position(joint[rleg_joint_5], next_position[rleg_joint_5]);
      wb_motor_set_position(joint[rleg_joint_6], next_position[rleg_joint_6]);
      wb_motor_set_position(joint[larm_joint_1], next_position[larm_joint_1]);
      wb_motor_set_position(joint[larm_joint_2], next_position[larm_joint_2]);
      wb_motor_set_position(joint[larm_joint_3], next_position[larm_joint_3]);
      wb_motor_set_position(joint[larm_joint_4], next_position[larm_joint_4]);
      wb_motor_set_position(joint[rarm_joint_1], next_position[rarm_joint_1]);
      wb_motor_set_position(joint[rarm_joint_2], next_position[rarm_joint_2]);
      wb_motor_set_position(joint[rarm_joint_3], next_position[rarm_joint_3]);
      wb_motor_set_position(joint[rarm_joint_4], next_position[rarm_joint_4]);
    } else {
      for (i = 0; i < 25; i++)
        next_position[i] = wb_position_sensor_get_value(joint_sensors[i]);
    }
    wb_robot_step(control_step); /* run one step */

    double left_force = wb_touch_sensor_get_value(left_touch) / 10.0;
    double right_force = wb_touch_sensor_get_value(right_touch) / 10.0;
    double sum_force = left_force + right_force;

    printf("Touch sensors: left force: %4.1f N right force: %4.1f N -> sum: %4.1f N\n", left_force, right_force, sum_force);
  }

  return 0;
}
