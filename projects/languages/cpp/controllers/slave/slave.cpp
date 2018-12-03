// Copyright 1996-2018 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Description:  This controller gives to its robot the following behavior:
 *               According to the messages it receives, the robot change its
 *               behavior.
 */

#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>

using namespace std;
using namespace webots;

static const double maxSpeed = 10.0;

class Slave : public Robot {
public:
  Slave();
  void run();

private:
  enum Mode { STOP, MOVE_FORWARD, AVOID_OBSTACLES, TURN };

  static double boundSpeed(double speed);

  int timeStep;
  Mode mode;
  Receiver *receiver;
  Camera *camera;
  DistanceSensor *distanceSensors[2];
  Motor *motors[2];
};

Slave::Slave() {
  timeStep = 32;
  mode = AVOID_OBSTACLES;
  camera = getCamera("camera");
  camera->enable(4 * timeStep);
  receiver = getReceiver("receiver");
  receiver->enable(timeStep);
  motors[0] = getMotor("left wheel motor");
  motors[1] = getMotor("right wheel motor");
  motors[0]->setPosition(std::numeric_limits<double>::infinity());
  motors[1]->setPosition(std::numeric_limits<double>::infinity());
  motors[0]->setVelocity(0.0);
  motors[1]->setVelocity(0.0);
  string distanceSensorNames("ds0");
  for (int i = 0; i < 2; i++) {
    distanceSensors[i] = getDistanceSensor(distanceSensorNames);
    distanceSensors[i]->enable(timeStep);
    distanceSensorNames[2]++;  // for getting "ds1","ds2",...
  }
}

double Slave::boundSpeed(double speed) {
  return std::min(maxSpeed, std::max(-maxSpeed, speed));
}

void Slave::run() {
  // main loop
  while (step(timeStep) != -1) {
    // Read sensors, particularly the order of the supervisor
    if (receiver->getQueueLength() > 0) {
      string message((const char *)receiver->getData());
      receiver->nextPacket();

      cout << "I should " << message << "!" << endl;

      if (message.compare("avoid obstacles") == 0)
        mode = AVOID_OBSTACLES;
      else if (message.compare("move forward") == 0)
        mode = MOVE_FORWARD;
      else if (message.compare("stop") == 0)
        mode = STOP;
      else if (message.compare("turn") == 0)
        mode = TURN;
    }
    double delta = distanceSensors[0]->getValue() - distanceSensors[1]->getValue();
    double speeds[2] = {0.0, 0.0};

    // send actuators commands according to the mode
    switch (mode) {
      case AVOID_OBSTACLES:
        speeds[0] = boundSpeed(maxSpeed / 2.0 + 0.1 * delta);
        speeds[1] = boundSpeed(maxSpeed / 2.0 - 0.1 * delta);
        break;
      case MOVE_FORWARD:
        speeds[0] = maxSpeed;
        speeds[1] = maxSpeed;
        break;
      case TURN:
        speeds[0] = maxSpeed / 2.0;
        speeds[1] = -maxSpeed / 2.0;
        break;
      default:
        break;
    }
    motors[0]->setVelocity(speeds[0]);
    motors[1]->setVelocity(speeds[1]);
  }
}

int main() {
  Slave *controller = new Slave();
  controller->run();
  delete controller;
  return 0;
}
