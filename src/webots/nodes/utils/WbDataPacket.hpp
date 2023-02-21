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

#ifndef WB_DATA_PACKET_HPP
#define WB_DATA_PACKET_HPP

#include "WbVector3.hpp"

class WbEmitter;
class QString;

// communication packet or datagram
class WbDataPacket {
public:
  enum { UNKNOWN = 0, RADIO, SERIAL, INFRA_RED };
  static int decodeMediumType(const QString &string);

  WbDataPacket(WbEmitter *emitter, int channel, const void *data, int size);
  WbDataPacket(WbDataPacket &other);
  ~WbDataPacket();

  WbEmitter *emitter() const { return mEmitter; }
  int channel() const { return mChannel; }
  const void *data() const { return mData; }
  int dataSize() const { return mDataSize; }
  const WbVector3 &emitterDir() const { return mEmitterDir; }
  void setEmitterDir(const WbVector3 &dir) { mEmitterDir = dir; }
  double signalStrength() const { return mSignalStrength; }
  void setSignalStrength(double signalStrength) { mSignalStrength = signalStrength; }

private:
  WbDataPacket &operator=(const WbDataPacket &);  // non copyable
  WbEmitter *mEmitter;
  int mChannel;  // emitter's channel
  char *mData;
  int mDataSize;
  WbVector3 mEmitterPos;
  WbVector3 mEmitterDir;
  double mSignalStrength;
};

#endif
