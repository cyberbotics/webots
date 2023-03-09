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

/*
 * Description:  Defines a packet sending from the remote control library to the e-puck
 */

#ifndef EPUCK_OUTPUT_PACKET_HPP
#define EPUCK_OUTPUT_PACKET_HPP

#include "Packet.hpp"

class Device;

class EPuckOutputPacket : public Packet {
public:
  EPuckOutputPacket();
  virtual ~EPuckOutputPacket();

  virtual void clear();

  int answerSize() const { return mAnswerSize; }

  void apply(int simulationTime);

  bool areDistanceSensorRequested() const { return mDistanceSensorRequested; }
  bool areGroundSensorRequested() const { return mGroundSensorRequested; }
  bool areLightSensorRequested() const { return mLightSensorRequested; }
  bool isAccelerometerRequested() const { return mAccelerometerRequested; }
  bool isCameraRequested() const { return mCameraRequested; }
  bool isEncoderRequested() const { return mEncoderRequested; }

private:
  int mAnswerSize;

  bool mDistanceSensorRequested;
  bool mGroundSensorRequested;
  bool mLightSensorRequested;
  bool mAccelerometerRequested;
  bool mCameraRequested;
  bool mEncoderRequested;
};

#endif
