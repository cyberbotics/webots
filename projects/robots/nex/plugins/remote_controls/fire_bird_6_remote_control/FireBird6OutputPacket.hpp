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

/*
 * Description:  Defines a packet sent from remote control library to the Fire Bird 6
 */

#ifndef FIREBIRD6_OUTPUT_PACKET_HPP
#define FIREBIRD6_OUTPUT_PACKET_HPP

#include "Packet.hpp"

class Device;

class FireBird6OutputPacket : public Packet {
public:
  FireBird6OutputPacket();
  virtual ~FireBird6OutputPacket();

  virtual void clear();

  int answerSize() const { return mAnswerSize; }

  void apply(int simulationTime);

  bool areDistanceSensorRequested() const { return mDistanceSensorRequested; }
  bool areSharpDistanceSensorRequested() const { return mSharpDistanceSensorRequested; }
  bool areLightSensorRequested() const { return mLightSensorRequested; }
  bool isAccelerometerRequested() const { return mAccelerometerRequested; }
  bool isGyroRequested() const { return mGyroRequested; }
  bool isMagnetometerRequested() const { return mMagnetometerRequested; }
  bool isEncoderRequested() const { return mEncoderRequested; }

private:
  int mAnswerSize;

  bool mDistanceSensorRequested;
  bool mSharpDistanceSensorRequested;
  bool mLightSensorRequested;
  bool mAccelerometerRequested;
  bool mGyroRequested;
  bool mMagnetometerRequested;
  bool mEncoderRequested;
};

#endif
