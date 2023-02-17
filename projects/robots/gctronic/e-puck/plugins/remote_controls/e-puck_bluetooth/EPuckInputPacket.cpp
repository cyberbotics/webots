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

#include "EPuckInputPacket.hpp"

#include "Camera.hpp"
#include "Device.hpp"
#include "DeviceManager.hpp"
#include "EPuckOutputPacket.hpp"
#include "Sensor.hpp"
#include "SingleValueSensor.hpp"
#include "TripleValuesSensor.hpp"

#include <webots/remote_control.h>

#include <iostream>

#include <cstdlib>

using namespace std;

EPuckInputPacket::EPuckInputPacket(int maxSize) : Packet(maxSize, maxSize) {
}

EPuckInputPacket::~EPuckInputPacket() {
}

void EPuckInputPacket::decode(int simulationTime, const EPuckOutputPacket &outputPacket) const {
  // ---
  // Sensors
  // ---

  // the order of the sensors should match with EPuckOutputPacket::apply()

  int currentPos = 0;

  if (outputPacket.isAccelerometerRequested()) {
    static const double calibration_k[3] = {-9.81 / 800.0, 9.81 / 800.0, 9.81 / 800.0};
    static const double calibration_offset = -2000.0;

    double values[3];
    for (int i = 0; i < 3; i++) {
      values[i] = readUShortAt(currentPos);
      currentPos += sizeof(unsigned short);
      // raw to SI
      values[i] = calibration_k[i] * (values[i] + calibration_offset);
    }

    TripleValuesSensor *accelerometer = DeviceManager::instance()->accelerometer();
    wbr_accelerometer_set_values(accelerometer->tag(), values);
    accelerometer->resetSensorRequested();
  }

  // read value of optional ground sensor
  if (outputPacket.areGroundSensorRequested()) {
    for (int i = 0; i < 3; i++) {
      SingleValueSensor *gs = DeviceManager::instance()->groundSensor(i);
      if (gs) {
        double value = readUShortAt(currentPos);
        currentPos += sizeof(unsigned short);

        wbr_distance_sensor_set_value(gs->tag(), value);
        gs->resetSensorRequested();
        gs->setLastRefreshTime(simulationTime);
      }
    }
  }

  if (outputPacket.areDistanceSensorRequested()) {
    for (int i = 0; i < 8; i++) {
      double value = readUShortAt(currentPos);
      currentPos += sizeof(unsigned short);

      SingleValueSensor *ds = DeviceManager::instance()->distanceSensor(i);
      wbr_distance_sensor_set_value(ds->tag(), value);
      ds->resetSensorRequested();
      ds->setLastRefreshTime(simulationTime);
    }
  }

  if (outputPacket.areLightSensorRequested()) {
    for (int i = 0; i < 8; i++) {
      double value = readUShortAt(currentPos);
      currentPos += sizeof(unsigned short);

      SingleValueSensor *ls = DeviceManager::instance()->lightSensor(i);
      wbr_light_sensor_set_value(ls->tag(), value);
      ls->resetSensorRequested();
      ls->setLastRefreshTime(simulationTime);
    }
  }

  if (outputPacket.isEncoderRequested()) {
    for (int i = 0; i < 2; i++) {
      double value = readShortAt(currentPos) / 159.23;  // 159.23 = encoder_resolution / ( 2 * pi)
      currentPos += sizeof(short);
      SingleValueSensor *ps = DeviceManager::instance()->positionSensor(i);
      wbr_position_sensor_set_value(ps->tag(), value);
      ps->resetSensorRequested();
      ps->setLastRefreshTime(simulationTime);
    }
  }

  if (outputPacket.isCameraRequested()) {
    int mode = (int)readUCharAt(currentPos++);
    int wh1 = (int)readUCharAt(currentPos++);  // can be width or height depending on the mode
    int wh2 = (int)readUCharAt(currentPos++);  // can be width or height depending on the mode
    const unsigned char *rawImage = (const unsigned char *)&(data()[currentPos]);

    Camera *camera = DeviceManager::instance()->camera();
    camera->resetSensorRequested();

    unsigned char *rgbImage = static_cast<unsigned char *>(malloc(4 * wh1 * wh2));

    if (camera->rawToBgraImage(rgbImage, rawImage, mode, wh1, wh2))
      wbr_camera_set_image(camera->tag(), static_cast<const unsigned char *>(rgbImage));

    free(rgbImage);
  }
}
