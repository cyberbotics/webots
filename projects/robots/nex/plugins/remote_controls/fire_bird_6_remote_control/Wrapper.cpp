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

#include "Communication.hpp"
#include "Device.hpp"
#include "DeviceManager.hpp"
#include "FireBird6InputPacket.hpp"
#include "FireBird6OutputPacket.hpp"
#include "Motor.hpp"
#include "Sensor.hpp"
#include "Serial.hpp"
#include "Time.hpp"

#include <iostream>
#include <vector>

#include <cstdio>
#include <cstdlib>

using namespace std;

Communication *Wrapper::cCommunication = NULL;
Time *Wrapper::cTime = NULL;
bool Wrapper::cSuccess = true;
int *Wrapper::cUploadReturnValue = NULL;

void Wrapper::init() {
  cout << "In Init" << endl;
  DeviceManager::instance();
  cout << "DeviceManager::instance() : Done" << endl;

  cCommunication = new Communication;
  cout << "new Communication : Done" << endl;
}

void Wrapper::cleanup() {
  delete cCommunication;
  delete cUploadReturnValue;

  DeviceManager::cleanup();
}

bool Wrapper::start(const char *args) {
  cout << "In Start" << endl;
  if (!args)
    return false;
  cCommunication->initialize(args);
  cTime = new Time();
  cSuccess = cCommunication->isInitialized();
  return cSuccess;
}

void Wrapper::stop() {
  if (!hasFailed())
    stopActuators();

  cCommunication->cleanup();

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
  FireBird6OutputPacket outputPacket;
  outputPacket.apply(beginStepTime);

  // 3 trials before giving up
  for (int i = 0; i < 3; i++) {
    // send the output packet
    cSuccess = cCommunication->sendPacket(&outputPacket);
    if (!cSuccess) {
      cerr << "Failed to send packet to the Fire Bird. Retry (" << (i + 1) << ")..." << endl;
      continue;
    }

    // setup and receive the input packet
    int answerSize = outputPacket.answerSize();
    FireBird6InputPacket inputPacket(answerSize);
    cSuccess = cCommunication->receivePacket(&inputPacket);
    if (!cSuccess) {
      cerr << "Failed to receive packet from the Fire Bird. Retry (" << (i + 1) << ")..." << endl;
      continue;
    }
    inputPacket.decode(beginStepTime, outputPacket);
    break;
  }
  if (!cSuccess)
    return 0;

  // get simulation time at the end of this step
  int endStepTime = cTime->currentSimulationTime();

  // according to the step duration, either wait
  // or returns the delay
  int deltaStepTime = endStepTime - beginStepTime;
  if (deltaStepTime <= step) {  // the packet is sent at time
    Time::wait(step - deltaStepTime);
    return 0;
  } else  // the delay asked is not fulfilled
    return deltaStepTime - step;
}

void Wrapper::stopActuators() {
  // reset all the requests
  for (int i = 0; i < 2; i++) {
    Motor *motor = DeviceManager::instance()->motor(i);
    motor->resetVelocityRequested();
  }

  vector<Device *>::const_iterator it;
  const vector<Device *> &devices = DeviceManager::instance()->devices();
  for (it = devices.begin(); it < devices.end(); ++it) {
    Sensor *s = dynamic_cast<Sensor *>(*it);
    if (s)
      s->resetSensorRequested();
  }

  // reset actuators
  for (int i = 0; i < 2; i++)
    motorSetVelocity(DeviceManager::instance()->motor(i)->tag(), 0.0);

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

void Wrapper::motorSetVelocity(WbDeviceTag tag, double velocity) {
  Device *device = DeviceManager::instance()->findDeviceFromTag(tag);
  Motor *motor = dynamic_cast<Motor *>(device);
  if (motor) {
    motor->setVelocityRequested();
    motor->setVelocity(velocity);
  }
}

void *Wrapper::findAvailablePorts(void *) {
  Serial::updatePorts();
  const vector<std::string> *comPorts = Serial::availablePorts();
  return (void *)comPorts;
}

void *Wrapper::callCustomFunction(void *args) {
  if (args == NULL) {
    return findAvailablePorts(args);
  }

  return NULL;
}
