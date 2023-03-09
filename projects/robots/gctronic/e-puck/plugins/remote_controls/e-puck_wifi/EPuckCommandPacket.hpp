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

#ifndef EPUCK_COMMAND_PACKET_HPP
#define EPUCK_COMMAND_PACKET_HPP

class Device;

#define EPUCK_COMMAND_PACKET_SIZE 21

class EPuckCommandPacket {
public:
  EPuckCommandPacket();
  virtual ~EPuckCommandPacket();

  virtual void clear();
  const char *data() const { return mData; }
  int apply(int simulationTime);
  bool areDistanceSensorRequested() const { return mDistanceSensorRequested; }
  bool areGroundSensorRequested() const { return mGroundSensorRequested; }
  bool areLightSensorRequested() const { return mLightSensorRequested; }
  bool isAccelerometerRequested() const { return mAccelerometerRequested; }
  bool isEncoderRequested() const { return mEncoderRequested; }
  bool isCameraRequested() const { return ((mData[1] & 1) == 1); }

private:
  bool mDistanceSensorRequested;
  bool mGroundSensorRequested;
  bool mLightSensorRequested;
  bool mAccelerometerRequested;
  bool mEncoderRequested;
  char mData[EPUCK_COMMAND_PACKET_SIZE];
};

#endif
