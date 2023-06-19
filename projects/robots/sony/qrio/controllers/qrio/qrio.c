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
 * Description:  A controller which makes the Humanoid robot dance
 * Author:       Stephane Mojon
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 64
#define PI = 2 * asin(1);

typedef struct {
  int joint;
  double amplitude;
  int omega;
  double fi;
  double constant;
} geneStruct;

typedef geneStruct *geneID;

enum joints {
  back_1,
  back_2,
  left_hip_1,
  left_hip_2,
  left_hip_3,
  left_knee,
  left_ankle_1,
  left_ankle_2,
  left_shoulder_1,
  left_shoulder_2,
  left_elbow,
  neck,
  right_hip_1,
  right_hip_2,
  right_hip_3,
  right_knee,
  right_ankle_1,
  right_ankle_2,
  right_shoulder_1,
  right_shoulder_2,
  right_elbow
};

static char *jointNum2Text(int num) {
  switch (num) {
    case back_1:
      return "back_1";
    case back_2:
      return "back_2";
    case left_hip_1:
      return "left_hip_1";
    case left_hip_2:
      return "left_hip_2";
    case left_hip_3:
      return "left_hip_3";
    case left_knee:
      return "left_knee";
    case left_ankle_1:
      return "left_ankle_1";
    case left_ankle_2:
      return "left_ankle_2";
    case left_shoulder_1:
      return "left_shoulder_1";
    case left_shoulder_2:
      return "left_shoulder_2";
    case left_elbow:
      return "left_elbow";
    case neck:
      return "neck";
    case right_hip_1:
      return "right_hip_1";
    case right_hip_2:
      return "right_hip_2";
    case right_hip_3:
      return "right_hip_3";
    case right_knee:
      return "right_knee";
    case right_ankle_1:
      return "right_ankle_1";
    case right_ankle_2:
      return "right_ankle_2";
    case right_shoulder_1:
      return "right_shoulder_1";
    case right_shoulder_2:
      return "right_shoulder_2";
    case right_elbow:
      return "right_elbow";
    default:
      return "none";
  }
}

static geneID newRandGene(void) {
  geneID newGene = malloc(sizeof(geneStruct));

  newGene->joint = 0;
  newGene->amplitude = 1.4;
  newGene->omega = 1;
  newGene->fi = 0.0;
  newGene->constant = 0.0;

  return newGene;
}

int main() {
  int i;
  int targetJoint;
  double t = 0.0;
  const int numOfGenes = 21;
  const int numOfDOF = 21; /* must be equal to the number of elements in the joints enum */
  geneID gene[21];
  double futurPosition[21];
  WbDeviceTag joint[21];

  wb_robot_init();

  /* Here we retreive all the existing devices of the robot. */
  for (i = 0; i < 21; i++) {
    joint[i] = wb_robot_get_device(jointNum2Text(i));
  }

  for (i = 0; i < numOfDOF; i++) {
    wb_motor_set_position(joint[i], 0);
    futurPosition[i] = 0.0;
  }

  for (i = 0; i < numOfGenes; i++) {
    gene[i] = newRandGene();

    if (i == back_1 || i == back_2 || i == left_hip_3 || i == right_hip_3 || i == left_ankle_2 || i == right_ankle_2)
      gene[i]->joint = -1;
    else
      gene[i]->joint = i;

    if (i == left_knee || i == right_knee)
      gene[i]->amplitude *= 2;
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    for (i = 0; i < numOfGenes; i++) {
      targetJoint = gene[i]->joint;

      if (targetJoint == -1)
        continue;
      futurPosition[targetJoint] += (gene[i]->amplitude * sin(gene[i]->omega * t + gene[i]->fi) + gene[i]->constant);

      if (futurPosition[targetJoint] < 0)
        futurPosition[targetJoint] = 0;
    }

    for (i = 0; i < numOfDOF; i++) {
      wb_motor_set_position(joint[i], futurPosition[i]);
      futurPosition[i] = 0.0;
    }

    t = t + 0.1;
  }

  wb_robot_cleanup();

  return 0;
}
