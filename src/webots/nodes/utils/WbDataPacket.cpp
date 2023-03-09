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

#include "WbDataPacket.hpp"
#include "WbVector3.hpp"

#include <limits>

int WbDataPacket::decodeMediumType(const QString &string) {
  if (string == "radio")
    return RADIO;
  else if (string == "serial")
    return SERIAL;
  else if (string == "infra-red")
    return INFRA_RED;
  else
    return UNKNOWN;
}

WbDataPacket::WbDataPacket(WbEmitter *emitter, int channel, const void *data, int size) :
  mEmitter(emitter),
  mChannel(channel),
  mDataSize(size),
  mEmitterDir(NAN, NAN, NAN),
  mSignalStrength(std::numeric_limits<double>::infinity()) {
  mData = new char[size];
  memcpy(mData, data, size);
};

WbDataPacket::WbDataPacket(WbDataPacket &other) :
  mEmitter(other.mEmitter),
  mChannel(other.mChannel),
  mDataSize(other.mDataSize),
  mEmitterPos(NAN, NAN, NAN),
  mEmitterDir(NAN, NAN, NAN),
  mSignalStrength(std::numeric_limits<double>::infinity()) {
  mData = new char[mDataSize];
  memcpy(mData, other.mData, mDataSize);
}

WbDataPacket::~WbDataPacket() {
  delete[] mData;
}
