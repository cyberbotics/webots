// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "WbEmitter.hpp"
#include "WbDataPacket.hpp"
#include "WbFieldChecker.hpp"
#include "WbReceiver.hpp"

#include "../../lib/Controller/api/messages.h"

#include <QtCore/QDataStream>

#include <cassert>

void WbEmitter::init() {
  mType = findSFString("type");
  mRange = findSFDouble("range");
  mMaxRange = findSFDouble("maxRange");
  mAperture = findSFDouble("aperture");
  mChannel = findSFInt("channel");
  mBaudRate = findSFInt("baudRate");
  mByteSize = findSFInt("byteSize");
  mBufferSize = findSFInt("bufferSize");
  mNeedToSetRange = false;
  mNeedToSetChannel = false;
  mNeedToSetBufferSize = false;
  mMediumType = WbDataPacket::UNKNOWN;
  mByteRate = -1.0;
}

WbEmitter::WbEmitter(WbTokenizer *tokenizer) : WbSolidDevice("Emitter", tokenizer) {
  init();
}

WbEmitter::WbEmitter(const WbEmitter &other) : WbSolidDevice(other) {
  init();
}

WbEmitter::WbEmitter(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbEmitter::~WbEmitter() {
  qDeleteAll(mQueue);
}

void WbEmitter::preFinalize() {
  WbSolidDevice::preFinalize();

  updateTransmissionSetup();
}

void WbEmitter::postFinalize() {
  WbSolidDevice::postFinalize();

  connect(mType, &WbSFString::changed, this, &WbEmitter::updateTransmissionSetup);
  connect(mRange, &WbSFDouble::changed, this, &WbEmitter::updateRange);
  connect(mMaxRange, &WbSFDouble::changed, this, &WbEmitter::updateTransmissionSetup);
  connect(mAperture, &WbSFDouble::changed, this, &WbEmitter::updateTransmissionSetup);
  connect(mChannel, &WbSFInt::changed, this, &WbEmitter::updateChannel);
  connect(mBaudRate, &WbSFInt::changed, this, &WbEmitter::updateTransmissionSetup);
  connect(mByteSize, &WbSFInt::changed, this, &WbEmitter::updateTransmissionSetup);
  connect(mBufferSize, &WbSFInt::changed, this, &WbEmitter::updateBufferSize);
}

void WbEmitter::updateTransmissionSetup() {
  if (mBaudRate->value() < 0)
    mByteRate = -1.0;
  else
    mByteRate = (double)mBaudRate->value() / (double)mByteSize->value() / 1000.0;

  mMediumType = WbDataPacket::decodeMediumType(mType->value());
  if (mMediumType == WbDataPacket::UNKNOWN) {
    warn(tr("Unknown 'type': \"%1\".").arg(mType->value()));
    mMediumType = WbDataPacket::RADIO;
  }

  WbFieldChecker::checkDoubleIsInRangeWithIncludedBoundsOrDisabled(this, mAperture, 0, 2 * M_PI, -1, -1);
  WbFieldChecker::checkDoubleIsPositiveOrDisabled(this, mMaxRange, -1, -1);
  WbFieldChecker::checkIntIsGreaterOrEqual(this, mByteSize, 8, 8);
}

void WbEmitter::updateBufferSize() {
  WbFieldChecker::checkIntIsPositiveOrDisabled(this, mBufferSize, -1, -1);
  mNeedToSetBufferSize = true;
}
void WbEmitter::updateRange() {
  WbFieldChecker::checkDoubleIsPositiveOrDisabled(this, mRange, -1, -1);
  if (mMaxRange->value() != -1.0 && (mRange->value() > mMaxRange->value() || mRange->value() == -1.0)) {
    warn(tr("'range' must be less than or equal to 'maxRange'."));
    mRange->setValue(mMaxRange->value());
  }
  mNeedToSetRange = true;
}

void WbEmitter::updateChannel() {
  mNeedToSetChannel = true;
}

void WbEmitter::writeConfigure(QDataStream &stream) {
  stream << tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)mBufferSize->value();
  stream << (int)mChannel->value();
  stream << (double)mByteRate;
  stream << (double)mRange->value();
  stream << (double)mMaxRange->value();

  mNeedToSetRange = false;
  mNeedToSetChannel = false;
  mNeedToSetBufferSize = false;
}

void WbEmitter::writeAnswer(QDataStream &stream) {
  if (mNeedToSetRange) {
    stream << tag();
    stream << (unsigned char)C_EMITTER_SET_RANGE;
    stream << (double)mRange->value();
    mNeedToSetRange = false;
  }
  if (mNeedToSetChannel) {
    stream << tag();
    stream << (unsigned char)C_EMITTER_SET_CHANNEL;
    stream << (int)mChannel->value();
    mNeedToSetChannel = false;
  }
  if (mNeedToSetBufferSize) {
    stream << tag();
    stream << (unsigned char)C_EMITTER_SET_BUFFER_SIZE;
    stream << (int)mBufferSize->value();
    mNeedToSetBufferSize = false;
  }
}

void WbEmitter::handleMessage(QDataStream &stream) {
  unsigned char command;
  unsigned int size;
  int channel;
  double range;
  char *data;

  stream >> (unsigned char &)command;
  switch (command) {
    case C_EMITTER_SEND:
      stream >> (int &)channel;
      mChannel->setValue(channel);
      stream >> (double &)range;
      mRange->setValue(range);
      stream >> (int &)size;
      data = new char[size];
      stream.readRawData(data, size);
      mQueue.enqueue(new WbDataPacket(this, mChannel->value(), data, size));
      delete[] data;
      return;

    case C_EMITTER_SET_CHANNEL:
      stream >> (int &)channel;
      mChannel->setValue(channel);
      return;

    case C_EMITTER_SET_RANGE:
      stream >> (double &)range;
      mRange->setValue(range);
      return;

    default:
      assert(0);
  }
}

void WbEmitter::prePhysicsStep(double ms) {
  WbSolidDevice::prePhysicsStep(ms);

  // forward queued packets to receivers
  while (!mQueue.isEmpty()) {
    WbDataPacket *packet = mQueue.takeFirst();
    WbReceiver::transmitPacket(packet);
  }
}

void WbEmitter::reset() {
  WbSolidDevice::reset();
  qDeleteAll(mQueue);
  mQueue.clear();
}
