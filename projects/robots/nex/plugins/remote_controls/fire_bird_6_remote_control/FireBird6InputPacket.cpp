// Copyright 1996-2022 Cyberbotics Ltd.
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

#include "FireBird6InputPacket.hpp"

#include "Device.hpp"
#include "DeviceManager.hpp"
#include "FireBird6OutputPacket.hpp"
#include "Sensor.hpp"
#include "SingleValueSensor.hpp"
#include "TripleValuesSensor.hpp"

#include <webots/remote_control.h>

#include <iostream>

#include <cstdlib>

using namespace std;

FireBird6InputPacket::FireBird6InputPacket(int maxSize) : Packet(maxSize, maxSize) {
}

FireBird6InputPacket::~FireBird6InputPacket() {
}

void FireBird6InputPacket::decode(int simulationTime, const FireBird6OutputPacket &outputPacket) const {
  // ---
  // Sensors
  // ---

  // the order of the sensors should match with FireBird6OutputPacket::apply()

  int currentPos = 5;

  if (outputPacket.areDistanceSensorRequested()) {
    for (int i = 0; i < 8; i++) {
      double value = readUCharAt(currentPos + i) * 0.05;  // sonar distance in meters

      SingleValueSensor *ds = DeviceManager::instance()->distanceSensor(i);
      wbr_distance_sensor_set_value(ds->tag(), value);
      ds->resetSensorRequested();
      ds->setLastRefreshTime(simulationTime);
    }
  }
  currentPos += 8 * sizeof(unsigned char);  // sonars

  if (outputPacket.areSharpDistanceSensorRequested()) {
    for (int i = 0; i < 8; i++) {
      double value = readUCharAt(currentPos + i);

      if (value > 220)
        value = 220;
      if (value < 30)
        value = 30;
      value = (((12.5 * 255) / (value * 3.3)) - 0.42) / 100;  // sharp sensor distance in meters

      SingleValueSensor *ds = DeviceManager::instance()->sharpDistanceSensor(i);
      wbr_distance_sensor_set_value(ds->tag(), value);
      ds->resetSensorRequested();
      ds->setLastRefreshTime(simulationTime);
    }
  }
  currentPos += 8 * sizeof(unsigned char);  // sharp distance sensors

  // read value of optional ground sensor
  currentPos += 8 * sizeof(unsigned char);  // ground / line sensors

  if (outputPacket.isAccelerometerRequested()) {
    double values[3];
    for (int i = 0; i < 3; i++)
      values[i] = readShortAt(currentPos + (i * 2)) * 9.80665 / 4096.0;  // convert to m/sec squared

    TripleValuesSensor *accelerometer = DeviceManager::instance()->accelerometer();
    wbr_accelerometer_set_values(accelerometer->tag(), values);
    accelerometer->resetSensorRequested();
  }
  currentPos += 6 * sizeof(unsigned char);  // accelerometer

  if (outputPacket.isGyroRequested()) {
    double values[3];
    for (int i = 0; i < 3; i++) {
      // convert to rad per sec for range 500 DPS in L3G4200D
      values[i] = readShortAt(currentPos + (i * 2)) * 3.055555 / (10000);
    }

    TripleValuesSensor *gyroscope = DeviceManager::instance()->gyroscope();
    wbr_gyro_set_values(gyroscope->tag(), values);
    gyroscope->resetSensorRequested();
  }
  currentPos += 6 * sizeof(unsigned char);  // gyro

  if (outputPacket.isMagnetometerRequested()) {
    double values[3];
    values[0] = readShortAt_MSBFirst(currentPos) / 1100.0;
    // cppcheck-suppress knownArgument
    // cppcheck-suppress constArgument
    values[1] = readShortAt_MSBFirst(currentPos + 2) / 1100.0;
    // different scaling for Z Axis
    // cppcheck-suppress knownArgument
    // cppcheck-suppress constArgument
    values[2] = readShortAt_MSBFirst(currentPos + 4) / 980.0;

    // set the 3 values in X, Y and Z of both XY and Z sensor.
    TripleValuesSensor *magXY = DeviceManager::instance()->magnetometerXY();
    wbr_compass_set_values(magXY->tag(), values);
    magXY->resetSensorRequested();

    TripleValuesSensor *magZ = DeviceManager::instance()->magnetometerZ();
    wbr_compass_set_values(magZ->tag(), values);
    magZ->resetSensorRequested();
  }
  currentPos += 6 * sizeof(unsigned char);  // magnetometer

  currentPos += 2 * sizeof(unsigned char);  // pot
  currentPos += 2 * sizeof(unsigned char);  // servo
  currentPos += 3 * sizeof(unsigned char);  // batt

  if (outputPacket.isEncoderRequested()) {
    currentPos += 5;  // size of received header for encoders
    for (int i = 0; i < 2; i++) {
      double value = readIntAt(currentPos);
      currentPos += 9;
      SingleValueSensor *ps = DeviceManager::instance()->positionSensor(i);
      wbr_position_sensor_set_value(ps->tag(), value);
      ps->resetSensorRequested();
      ps->setLastRefreshTime(simulationTime);
    }
  }
}
