// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Sample Webots controller for the humanoid marathon benchmark.
// This controller uses built-in motion manager modules to get the OP2 to walk.

#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Gyro.hpp>
#include <webots/LED.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

using namespace webots;
using namespace managers;

// Names of position sensors needed to get the corresponding device and read the measurements.
static const char *positionSensorNames[] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR", "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL",
  "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR",    "AnkleL",    "FootR",  "FootL",  "Neck",  "Head"};

int main(int argc, char *argv[]) {
  webots::Robot robot;
  const int timeStep = robot.getBasicTimeStep();
  RobotisOp2MotionManager motion(&robot);
  // retrieve devices
  LED *headLed = robot.getLED("HeadLed");
  LED *eyeLed = robot.getLED("EyeLed");
  Gyro *gyro = robot.getGyro("Gyro");
  const int n = sizeof(positionSensorNames) / sizeof(char *);
  // Enable all the position sensors
  for (int i = 0; i < n; i++) {
    char name[32];
    snprintf(name, 32, "%sS", positionSensorNames[i]);
    PositionSensor *sensor = robot.getPositionSensor(name);
    sensor->enable(timeStep);
  }
  // Initialize the LED devices
  headLed->set(0xff0000);
  eyeLed->set(0xa0a0ff);
  // enable the gyro
  gyro->enable(timeStep);

  // perform one simulation step to get sensor data
  robot.step(timeStep);

  // Page 1: stand up
  // Page 9: assume walking position
  motion.playPage(1, true);
  motion.playPage(9, true);
  RobotisOp2GaitManager gait(&robot, "");
  gait.start();
  gait.setXAmplitude(0.0);
  gait.setYAmplitude(0.0);
  gait.setBalanceEnable(true);
  double amplitude = 0.5;
  int loop = 0;
  // Main loop: perform a simulation step until the simulation is over.
  // At the beginning, start walking on the spot.
  // After 45 timesteps, begin taking steps forward.
  while (robot.step(timeStep) != -1) {
    if (loop == 45)
      gait.setXAmplitude(amplitude);
    gait.step(timeStep);
    loop += 1;
  }

  return 1;
}
