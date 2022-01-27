// Copyright 1996-2021 Cyberbotics Ltd.
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

#include "Wrapper.hpp"

#include "Camera.hpp"
#include "Communication.hpp"
#include "Device.hpp"
#include "DeviceManager.hpp"
#include "Led.hpp"
#include "Motor.hpp"
#include "RobotisOp2InputPacket.hpp"
#include "RobotisOp2OutputPacket.hpp"
#include "Sensor.hpp"
#include "Time.hpp"

#include <webots/robot.h>

#include <iostream>
#include <vector>

#include <cstdio>
#include <cstdlib>

#define PORT 5023

using namespace std;

Communication *Wrapper::cCommunication = NULL;
Time *Wrapper::cTime = NULL;
bool Wrapper::cSuccess = true;

void Wrapper::init() {
  DeviceManager::instance();

  cCommunication = new Communication;
}

void Wrapper::cleanup() {
  delete cCommunication;
  delete cTime;

  DeviceManager::cleanup();
}

bool Wrapper::start(const char *ip) {
  if (!ip)
    return false;

  delete cTime;

  Time trialTime;

  cSuccess = false;
  while (!cSuccess && trialTime.currentSimulationTime() < 10000) {  // try to connect for 10 seconds
    cSuccess = cCommunication->initialize(ip, PORT);
    // Wait before trying to reconnect. It gives some time to the remote process
    // to launch. It also avoids to print too many "Retry" messages.
    Time::wait(1000);
    cout << "Retry to connect to " << ip << ":" << PORT << endl;
  }

  if (cSuccess)
    cTime = new Time();
  else {
    cTime = NULL;
    cerr << "Cannot connect to " << ip << ":" << PORT << endl;
  }

  return cSuccess;
}

void Wrapper::stop() {
  if (!hasFailed())
    stopActuators();

  cCommunication->close();

  if (cTime) {
    delete cTime;
    cTime = NULL;
  }
}

int Wrapper::robotStep(int step) {
  // get simulation time at the beginning of this step
  int beginStepTime = cTime->currentSimulationTime();

  // apply to sensors
  DeviceManager::instance()->apply(beginStepTime);

  // setup the output packet
  RobotisOp2OutputPacket outputPacket;
  outputPacket.apply(beginStepTime);

  // 3 trials before giving up
  for (int i = 0; i < 3; i++) {
    // send the output packet
    cSuccess = cCommunication->sendPacket(&outputPacket);
    if (!cSuccess) {
      cerr << "Failed to send packet to ROBOTIS OP2. Retry (" << (i + 1) << ")..." << endl;
      continue;
    }

    // setup and receive the input packet
    RobotisOp2InputPacket inputPacket;
    cSuccess = cCommunication->receivePacket(&inputPacket);
    if (!cSuccess) {
      cerr << "Failed to receive packet from ROBOTIS OP2. Retry (" << (i + 1) << ")..." << endl;
      continue;
    }
    inputPacket.decode(beginStepTime, outputPacket);

    if (cSuccess)
      break;
  }
  if (!cSuccess)
    return 0;

  // Time management -> in order to be always as close as possible to 1.0x
  int newTime = cTime->currentSimulationTime();

  int static oldTime = 0;
  double static timeStep = step;

  if (newTime < oldTime)
    oldTime = newTime;

  // calculate difference between this time step and the previous one
  int difference = (newTime - oldTime);
  if (difference > 10 * step)  // if time difference is too big (simulation was stopped for example)
    difference = 10 * step;    // set the difference to 10 * TimeStep

  // Recalculate time actual time step
  // The time step is not calculate only on one step,
  // but it take also in count the previous time step
  // with a bigger importance to the most recent
  timeStep = timeStep * 9 + difference;
  timeStep = timeStep / 10;

  if ((int)timeStep < step) {  // the packet is sent at time
    Time::wait((step - timeStep) + 0.5);
    oldTime = cTime->currentSimulationTime();
    return 0;
  } else {  // the delay asked is not fulfilled
    oldTime = newTime;
    return timeStep - step;
  }
}

void Wrapper::stopActuators() {
  // reset all the requests

  for (int i = 0; i < 5; i++) {
    Led *led = DeviceManager::instance()->led(i);
    led->resetLedRequested();
  }

  vector<Device *>::const_iterator it;
  const vector<Device *> &devices = DeviceManager::instance()->devices();
  for (it = devices.begin(); it < devices.end(); ++it) {
    Sensor *s = dynamic_cast<Sensor *>(*it);
    if (s)
      s->resetSensorRequested();
  }

  // send the packet
  robotStep(0);
}

void Wrapper::setSamplingPeriod(WbDeviceTag tag, int samplingPeriod) {
  Device *device = DeviceManager::instance()->findDeviceFromTag(tag);
  Sensor *sensor = dynamic_cast<Sensor *>(device);
  if (sensor) {
    sensor->setLastRefreshTime(0);
    sensor->setSamplingPeriod(samplingPeriod);
  } else
    cerr << "Wrapper::setSamplingPeriod: unknown device" << endl;
}

void Wrapper::ledSet(WbDeviceTag tag, int state) {
  Device *device = DeviceManager::instance()->findDeviceFromTag(tag);
  Led *led = dynamic_cast<Led *>(device);
  if (led) {
    led->setLedRequested();
    led->setState(state);
  }
}

void Wrapper::motorSetPosition(WbDeviceTag tag, double position) {
  Device *device = DeviceManager::instance()->findDeviceFromTag(tag);
  MotorR *motor = dynamic_cast<MotorR *>(device);
  if (motor) {
    motor->setMotorRequested();
    motor->setPositionRequested();
    motor->setPosition(position);
  }
}

void Wrapper::motorSetVelocity(WbDeviceTag tag, double velocity) {
  Device *device = DeviceManager::instance()->findDeviceFromTag(tag);
  MotorR *motor = dynamic_cast<MotorR *>(device);
  if (motor) {
    motor->setMotorRequested();
    motor->setVelocityRequested();
    motor->setVelocity(velocity);
  }
}

void Wrapper::motorSetAcceleration(WbDeviceTag tag, double acceleration) {
  Device *device = DeviceManager::instance()->findDeviceFromTag(tag);
  MotorR *motor = dynamic_cast<MotorR *>(device);
  if (motor) {
    motor->setMotorRequested();
    motor->setAccelerationRequested();
    motor->setAcceleration(acceleration);
  }
}

void Wrapper::motorSetAvailableTorque(WbDeviceTag tag, double torque) {
  Device *device = DeviceManager::instance()->findDeviceFromTag(tag);
  MotorR *motor = dynamic_cast<MotorR *>(device);
  if (motor) {
    motor->setMotorRequested();
    motor->setAvailableTorqueRequested();
    motor->setAvailableTorque(torque);
  }
}

void Wrapper::motorSetTorque(WbDeviceTag tag, double torque) {
  Device *device = DeviceManager::instance()->findDeviceFromTag(tag);
  MotorR *motor = dynamic_cast<MotorR *>(device);
  if (motor) {
    motor->setMotorRequested();
    motor->setTorqueRequested();
    motor->setTorque(torque);
  }
}

void Wrapper::motorSetControlPID(WbDeviceTag tag, double p, double i, double d) {
  Device *device = DeviceManager::instance()->findDeviceFromTag(tag);
  MotorR *motor = dynamic_cast<MotorR *>(device);
  if (motor) {
    motor->setMotorRequested();
    motor->setControlPIDRequested();
    motor->setControlPID(p, i, d);
  }
}
