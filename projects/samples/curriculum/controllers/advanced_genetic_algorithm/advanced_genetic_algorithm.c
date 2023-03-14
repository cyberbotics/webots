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

// Description:   Robot execution code for genetic algorithm

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#define NUM_SENSORS 8
#define NUM_WHEELS 2
#define GENOTYPE_SIZE (NUM_SENSORS * NUM_WHEELS)
#define SPEED_UNIT 0.00628

// sensor to wheels multiplication matrix
// each each sensor has a weight for each wheel
double matrix[NUM_SENSORS][NUM_WHEELS];

WbDeviceTag sensors[NUM_SENSORS];     // proximity sensors
WbDeviceTag receiver;                 // for receiving genes from Supervisor
WbDeviceTag left_motor, right_motor;  // motors

// check if a new set of genes was sent by the Supervisor
// in this case start using these new genes immediately
void check_for_new_genes() {
  if (wb_receiver_get_queue_length(receiver) > 0) {
    // check that the number of genes received match what is expected
    assert(wb_receiver_get_data_size(receiver) == GENOTYPE_SIZE * sizeof(double));

    // copy new genes directly in the sensor/actuator matrix
    // we don't use any specific mapping nor left/right symmetry
    // it's the GA's responsability to find a functional mapping
    memcpy(matrix, wb_receiver_get_data(receiver), GENOTYPE_SIZE * sizeof(double));

    // prepare for receiving next genes packet
    wb_receiver_next_packet(receiver);
  }
}

static double clip_value(double value, double min_max) {
  if (value > min_max)
    return min_max;
  else if (value < -min_max)
    return -min_max;

  return value;
}

void sense_compute_and_actuate() {
  // read sensor values
  double sensor_values[NUM_SENSORS];
  int i, j;
  for (i = 0; i < NUM_SENSORS; i++)
    sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);

  // compute actuation using Braitenberg's algorithm:
  // The speed of each wheel is computed by summing the value
  // of each sensor multiplied by the corresponding weight of the matrix.
  // By chance, in this case, this works without any scaling of the sensor values nor of the
  // wheels speed but this type of scaling may be necessary with a different problem
  double wheel_speed[NUM_WHEELS] = {0.0, 0.0};
  for (i = 0; i < NUM_WHEELS; i++) {
    for (j = 0; j < NUM_SENSORS; j++)
      wheel_speed[i] += SPEED_UNIT * matrix[j][i] * sensor_values[j];
  }

  // clip to e-puck max speed values to avoid warning
  wheel_speed[0] = clip_value(wheel_speed[0], 6.28);
  wheel_speed[1] = clip_value(wheel_speed[1], 6.28);

  // actuate e-puck wheels
  wb_motor_set_velocity(left_motor, wheel_speed[0]);
  wb_motor_set_velocity(right_motor, wheel_speed[1]);
}

int main(int argc, const char *argv[]) {
  wb_robot_init();  // initialize Webots

  // find simulation step in milliseconds (WorldInfo.basicTimeStep)
  int time_step = wb_robot_get_basic_time_step();

  // find and enable proximity sensors
  char name[32];
  int i;
  for (i = 0; i < NUM_SENSORS; i++) {
    sprintf(name, "ps%d", i);
    sensors[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(sensors[i], time_step);
  }

  // find and enable receiver
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);

  // get a handler to the motors and set target position to infinity (speed control)
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // initialize matrix to zero, hence the robot
  // wheels will initially be stopped
  memset(matrix, 0.0, sizeof(matrix));

  // run until simulation is restarted
  while (wb_robot_step(time_step) != -1) {
    check_for_new_genes();
    sense_compute_and_actuate();
  }

  wb_robot_cleanup();  // cleanup Webots
  return 0;            // ignored
}
