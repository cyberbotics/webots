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

/*
 * Description:  Defines a packet sending from the remote control library to the ROBOTIS OP2
 */

#ifndef ROBOTISOP2_OUTPUT_PACKET_HPP
#define ROBOTISOP2_OUTPUT_PACKET_HPP

#include "Packet.hpp"

class Device;

class RobotisOp2OutputPacket : public Packet {
public:
  RobotisOp2OutputPacket();
  virtual ~RobotisOp2OutputPacket();
  virtual void clear();
  void apply(int simulationTime);
  bool isAccelerometerRequested() const { return mAccelerometerRequested; }
  bool isGyroRequested() const { return mGyroRequested; }
  bool isCameraRequested() const { return mCameraRequested; }
  bool isPositionSensorRequested(int at) const { return mPositionSensorRequested[at]; }
  bool isMotorForceFeedback(int at) const { return mMotorTorqueFeedback[at]; }

private:
  bool mAccelerometerRequested;
  bool mGyroRequested;
  bool mCameraRequested;
  bool mPositionSensorRequested[20];
  bool mMotorTorqueFeedback[20];
};

#endif
