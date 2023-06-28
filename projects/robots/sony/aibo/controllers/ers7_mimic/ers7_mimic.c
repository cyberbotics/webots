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
 * Description:  A controller for the real Aibo ERS-210 robot.
 * Author:       Lukas Hohl
 */

/*
 * this controller is quite entertaining: releasing the motors on one of the robot's
 * legs so that it hangs loose, it makes the remaining legs 'mimic' that one as it
 * gets rotated or bent by hand (author: Lukas Hohl)
 */

#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define SIMULATION_STEP 16
#define NUM_JOINTS_PER_LEG 3

enum {        /* leg indices enumeration */
       LFLEG, /* left fore leg */
       LHLEG, /* left hind leg */
       RFLEG, /* right fore leg */
       RHLEG, /* right hind leg */
       NUM_LEGS
};

#define MASTER_LEG RFLEG /* master leg */

static WbDeviceTag leg_motors[NUM_LEGS][NUM_JOINTS_PER_LEG];
static WbDeviceTag leg_position_sensor[NUM_LEGS][NUM_JOINTS_PER_LEG];

int main() {
  leg_motors[LFLEG][0] = wb_robot_get_device("PRM:/r2/c1-Joint2:21");
  leg_motors[LFLEG][1] = wb_robot_get_device("PRM:/r2/c1/c2-Joint2:22");
  leg_motors[LFLEG][2] = wb_robot_get_device("PRM:/r2/c1/c2/c3-Joint2:23");
  leg_motors[LHLEG][0] = wb_robot_get_device("PRM:/r3/c1-Joint2:31");
  leg_motors[LHLEG][1] = wb_robot_get_device("PRM:/r3/c1/c2-Joint2:32");
  leg_motors[LHLEG][2] = wb_robot_get_device("PRM:/r3/c1/c2/c3-Joint2:33");
  leg_motors[RFLEG][0] = wb_robot_get_device("PRM:/r4/c1-Joint2:41");
  leg_motors[RFLEG][1] = wb_robot_get_device("PRM:/r4/c1/c2-Joint2:42");
  leg_motors[RFLEG][2] = wb_robot_get_device("PRM:/r4/c1/c2/c3-Joint2:43");
  leg_motors[RHLEG][0] = wb_robot_get_device("PRM:/r5/c1-Joint2:51");
  leg_motors[RHLEG][1] = wb_robot_get_device("PRM:/r5/c1/c2-Joint2:52");
  leg_motors[RHLEG][2] = wb_robot_get_device("PRM:/r5/c1/c2/c3-Joint2:53");

  leg_position_sensor[LFLEG][0] = wb_robot_get_device("PRM:/r2/c1-JointSensor2:21");
  leg_position_sensor[LFLEG][1] = wb_robot_get_device("PRM:/r2/c1/c2-JointSensor2:22");
  leg_position_sensor[LFLEG][2] = wb_robot_get_device("PRM:/r2/c1/c2/c3-JointSensor2:23");
  leg_position_sensor[LHLEG][0] = wb_robot_get_device("PRM:/r3/c1-JointSensor2:31");
  leg_position_sensor[LHLEG][1] = wb_robot_get_device("PRM:/r3/c1/c2-JointSensor2:32");
  leg_position_sensor[LHLEG][2] = wb_robot_get_device("PRM:/r3/c1/c2/c3-JointSensor2:33");
  leg_position_sensor[RFLEG][0] = wb_robot_get_device("PRM:/r4/c1-JointSensor2:41");
  leg_position_sensor[RFLEG][1] = wb_robot_get_device("PRM:/r4/c1/c2-JointSensor2:42");
  leg_position_sensor[RFLEG][2] = wb_robot_get_device("PRM:/r4/c1/c2/c3-JointSensor2:43");
  leg_position_sensor[RHLEG][0] = wb_robot_get_device("PRM:/r5/c1-JointSensor2:51");
  leg_position_sensor[RHLEG][1] = wb_robot_get_device("PRM:/r5/c1/c2-JointSensor2:52");
  leg_position_sensor[RHLEG][2] = wb_robot_get_device("PRM:/r5/c1/c2/c3-JointSensor2:53");

  /* release master leg motors, enable position reading */
  int j;
  for (j = 0; j < NUM_JOINTS_PER_LEG; j++) {
    wb_motor_set_available_torque(leg_motors[MASTER_LEG][j], 0.0);
    wb_position_sensor_enable(leg_position_sensor[MASTER_LEG][j], SIMULATION_STEP);
  }

  while (wb_robot_step(SIMULATION_STEP) != -1) {
    for (j = 0; j < NUM_JOINTS_PER_LEG; j++) {
      /* master position */
      double master = wb_position_sensor_get_value(leg_position_sensor[MASTER_LEG][j]);
      int i;
      for (i = 0; i < NUM_LEGS; i++) {
        if (i != MASTER_LEG) {
          /* set remaining legs to master */
          wb_motor_set_position(leg_motors[i][j], master);
        }
      }
    }
  }

  wb_robot_cleanup();

  return 0;
}
