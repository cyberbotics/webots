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

#include "RobotisOp2InputPacket.hpp"

#include "Camera.hpp"
#include "Device.hpp"
#include "DeviceManager.hpp"
#include "Motor.hpp"
#include "RobotisOp2OutputPacket.hpp"
#include "Sensor.hpp"
#include "SingleValueSensor.hpp"
#include "TripleValuesSensor.hpp"

#include <webots/remote_control.h>

#include <cassert>
#include <cstdlib>
#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

using namespace std;

RobotisOp2InputPacket::RobotisOp2InputPacket() : Packet(300000) {
  CameraR *camera = DeviceManager::instance()->camera();
  mCameraWidth = camera->width();
  mCameraHeight = camera->height();
}

RobotisOp2InputPacket::~RobotisOp2InputPacket() {
}

void RobotisOp2InputPacket::decode(int simulationTime, const RobotisOp2OutputPacket &outputPacket) {
  // the order of the sensors should match with RobotisOp2OutputPacket::apply()

  int currentPos = 5;
  // Accelerometer
  if (outputPacket.isAccelerometerRequested()) {
    double values[3];
    for (int i = 0; i < 3; i++) {
      values[i] = (double)readIntAt(currentPos);
      currentPos += 4;
    }
    TripleValuesSensor *accelerometer = DeviceManager::instance()->accelerometer();
    wbr_accelerometer_set_values(accelerometer->tag(), values);
    accelerometer->resetSensorRequested();
  }

  // Gyro
  if (outputPacket.isGyroRequested()) {
    double values[3];
    for (int i = 0; i < 3; i++) {
      values[i] = (double)readIntAt(currentPos);
      currentPos += 4;
    }
    TripleValuesSensor *gyro = DeviceManager::instance()->gyro();
    wbr_gyro_set_values(gyro->tag(), values);
    gyro->resetSensorRequested();
  }

  // Camera
  if (outputPacket.isCameraRequested()) {
    int image_length = readIntAt(currentPos);
    currentPos += 4;

    CameraR *camera = DeviceManager::instance()->camera();
    unsigned char *image = readJpegImage(getBufferFromPos(currentPos), image_length);
    // Convert RGB buffer to BGRA buffer
    static unsigned char imageBGRA[320 * 240 * 4];
    for (int i = 0; i < mCameraHeight; i++) {
      for (int j = 0; j < mCameraWidth; j++) {
        imageBGRA[i * 4 * mCameraWidth + j * 4 + 0] = image[i * 3 * mCameraWidth + j * 3 + 2];
        imageBGRA[i * 4 * mCameraWidth + j * 4 + 1] = image[i * 3 * mCameraWidth + j * 3 + 1];
        imageBGRA[i * 4 * mCameraWidth + j * 4 + 2] = image[i * 3 * mCameraWidth + j * 3 + 0];
        imageBGRA[i * 4 * mCameraWidth + j * 4 + 3] = 255;
      }
    }
    free(image);
    wbr_camera_set_image(camera->tag(), (const unsigned char *)imageBGRA);
    camera->resetSensorRequested();
    currentPos += image_length;
  }

  // PositionSensor
  for (int i = 0; i < 20; i++) {
    if (outputPacket.isPositionSensorRequested(i)) {
      double value = (double)readIntAt(currentPos) / 10000;
      currentPos += 4;
      SingleValueSensor *positionSensor = DeviceManager::instance()->positionSensor(i);
      wbr_position_sensor_set_value(positionSensor->tag(), value);
      positionSensor->resetSensorRequested();
    }
  }

  // Motor torque feedback
  for (int i = 0; i < 20; i++) {
    if (outputPacket.isMotorForceFeedback(i)) {
      double value = (double)readIntAt(currentPos) / 10000;
      currentPos += 4;
      MotorR *motor = DeviceManager::instance()->motor(i);
      wbr_motor_set_torque_feedback(motor->tag(), value);
      motor->resetSensorRequested();
    }
  }
}

//================================
unsigned char *RobotisOp2InputPacket::readJpegImage(const unsigned char *data, unsigned int length) {
  int width, height, number_of_components;
  return stbi_load_from_memory(data, length, &width, &height, &number_of_components, STBI_rgb);
}
