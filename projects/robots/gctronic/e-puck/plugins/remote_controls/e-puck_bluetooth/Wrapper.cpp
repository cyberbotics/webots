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

#include "Wrapper.hpp"

#include "Camera.hpp"
#include "Communication.hpp"
#include "Device.hpp"
#include "DeviceManager.hpp"
#include "EPuckInputPacket.hpp"
#include "EPuckOutputPacket.hpp"
#include "Led.hpp"
#include "Motor.hpp"
#include "Sensor.hpp"
#include "Serial.hpp"
#include "Time.hpp"
#include "Uploader.hpp"
#include "UploaderData.hpp"

#include <iostream>
#include <vector>

#include <cstdio>
#include <cstdlib>

using namespace std;

Communication *Wrapper::cCommunication = NULL;
Time *Wrapper::cTime = NULL;
bool Wrapper::cSuccess = true;
bool Wrapper::cCameraInitialized = false;

void Wrapper::init() {
  DeviceManager::instance();

  cCommunication = new Communication;
}

void Wrapper::cleanup() {
  delete cCommunication;

  DeviceManager::cleanup();
}

bool Wrapper::start(const char *args) {
  if (!args)
    return false;
  DeviceManager::instance()->camera()->checkResolution();
  cCommunication->initialize(std::string(args));
  cTime = new Time();
  cSuccess = cCommunication->isInitialized();
  cCameraInitialized = false;
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
  int beginStepTime = cTime->elapsedTime();

  // apply to sensors
  DeviceManager::instance()->apply(beginStepTime);

  // setup the output packet
  EPuckOutputPacket outputPacket;
  outputPacket.apply(beginStepTime);

  // camera initialisation if required
  if (outputPacket.isCameraRequested())
    initializeCamera();

  // 3 trials before giving up
  for (int i = 0; i < 3; i++) {
    // send the output packet
    cSuccess = cCommunication->sendPacket(&outputPacket);
    if (!cSuccess) {
      cerr << "Failed to send packet to the e-puck. Retry (" << (i + 1) << ")..." << endl;
      continue;
    }

    // setup and receive the input packet
    int answerSize = outputPacket.answerSize();
    EPuckInputPacket inputPacket(answerSize);
    cSuccess = cCommunication->receivePacket(&inputPacket);
    if (!cSuccess) {
      cerr << "Failed to receive packet from the e-puck. Retry (" << (i + 1) << ")..." << endl;
      continue;
    }
    inputPacket.decode(beginStepTime, outputPacket);
    break;
  }
  if (!cSuccess)
    return 0;

  // get simulation time at the end of this step
  int endStepTime = cTime->elapsedTime();

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
  for (int i = 0; i < 10; i++) {
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

  // reset actuators
  for (int i = 0; i < 2; i++)
    motorSetVelocity(DeviceManager::instance()->motor(i)->tag(), 0.0);
  for (int i = 0; i < 10; i++)
    ledSet(DeviceManager::instance()->led(i)->tag(), 0);

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

void Wrapper::ledSet(WbDeviceTag tag, int state) {
  Device *device = DeviceManager::instance()->findDeviceFromTag(tag);
  Led *led = dynamic_cast<Led *>(device);
  if (led) {
    led->setLedRequested();
    led->setState(state);
  }
}

void *Wrapper::findAvailablePorts(void *) {
  Serial::updatePorts();
  const vector<std::string> *comPorts = Serial::availablePorts();
  // we need to convert this to a C string to pass it to the robot window
  // which is written in C and is unable to handle a vector<std::string>
  // the C string simply list the available ports, separated by a '\n'.
  int size = comPorts->size();
  size_t count = size;  // room for the '\n' characters and final '\0' character
  for (int i = 0; i < size; i++)
    count += comPorts->at(i).length();  // room for the file names
  if (count == 0)
    // no ports found
    return NULL;

  char *port = static_cast<char *>(malloc(count));
  int n = 0;
  for (int i = 0; i < size; i++) {
    int l = comPorts->at(i).length();
    for (int j = 0; j < l; j++)
      port[n++] = comPorts->at(i)[j];
    port[n++] = '\n';
  }
  port[n - 1] = '\0';
  return port;
}

void Wrapper::initializeCamera() {
  if (hasFailed() || cCameraInitialized)
    return;

  Camera *camera = DeviceManager::instance()->camera();

  char *answer = cCommunication->talk(camera->generateInitialisationCommand().c_str());

  if (answer) {
    cCameraInitialized = true;
    free(answer);
  } else
    cSuccess = false;
}

void *Wrapper::callCustomFunction(void *args) {
  if (args == NULL)
    return findAvailablePorts(args);

  static int uploadReturnValue = 0;
  UploaderData uploaderData = static_cast<UploaderData *>(args)[0];
  if (uploaderData.command == UPLOADER_DATA_CONNECT) {
    uploadReturnValue = Uploader::connect(uploaderData.data);
  } else if (uploaderData.command == UPLOADER_DATA_DISCONNECT) {
    Uploader::disconnect();
    uploadReturnValue = 0;
  } else if (uploaderData.command == UPLOADER_DATA_CANCEL) {
    Uploader::cancelUpload();
    uploadReturnValue = 0;
  } else  // UPLOADER_DATA_SEND_FILE
    uploadReturnValue = Uploader::send(uploaderData.robot_id, uploaderData.data, uploaderData.progress_callback);

  return &uploadReturnValue;
}
