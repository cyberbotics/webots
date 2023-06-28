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

#include "WbEmitter.hpp"

#include "WbDataPacket.hpp"
#include "WbDataStream.hpp"
#include "WbFieldChecker.hpp"
#include "WbReceiver.hpp"

#include "../../controller/c/messages.h"

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
  mAllowedChannels = findMFInt("allowedChannels");
  mNeedToSetRange = false;
  mNeedToSetChannel = false;
  mNeedToSetBufferSize = false;
  mNeedToSetAllowedChannels = false;
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
  updateAllowedChannels();
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
  connect(mAllowedChannels, &WbMFInt::changed, this, &WbEmitter::updateAllowedChannels);
}

void WbEmitter::updateTransmissionSetup() {
  if (mBaudRate->value() < 0)
    mByteRate = -1.0;
  else
    mByteRate = (double)mBaudRate->value() / (double)mByteSize->value() / 1000.0;

  mMediumType = WbDataPacket::decodeMediumType(mType->value());
  if (mMediumType == WbDataPacket::UNKNOWN) {
    parsingWarn(tr("Unknown 'type': \"%1\".").arg(mType->value()));
    mMediumType = WbDataPacket::RADIO;
  }

  WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBoundsAndNotDisabled(this, mAperture, 0, 2 * M_PI, -1, -1);
  WbFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mMaxRange, -1, -1);
  WbFieldChecker::resetIntIfLess(this, mByteSize, 8, 8);
}

void WbEmitter::updateBufferSize() {
  WbFieldChecker::resetIntIfNonPositiveAndNotDisabled(this, mBufferSize, -1, -1);
  mNeedToSetBufferSize = true;
}
void WbEmitter::updateRange() {
  WbFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mRange, -1, -1);
  if (mMaxRange->value() != -1.0 && (mRange->value() > mMaxRange->value() || mRange->value() == -1.0)) {
    parsingWarn(tr("'range' must be less than or equal to 'maxRange'."));
    mRange->setValue(mMaxRange->value());
  }
  mNeedToSetRange = true;
}

bool WbEmitter::isChannelAllowed() {
  const int allowedChannelsSize = mAllowedChannels->size();
  if (allowedChannelsSize > 0) {
    const int currentChannel = (int)mChannel->value();
    for (int i = 0; i < allowedChannelsSize; i++) {
      if (currentChannel == mAllowedChannels->item(i))
        return true;
    }
    return false;
  }
  return true;
}

void WbEmitter::updateAllowedChannels() {
  if (!isChannelAllowed()) {
    parsingWarn(
      tr("'allowedChannels' does not contain current 'channel'. Setting 'channel' to %1.").arg(mAllowedChannels->item(0)));
    mChannel->setValue(mAllowedChannels->item(0));
  }

  mNeedToSetAllowedChannels = true;
}

void WbEmitter::updateChannel() {
  if (!isChannelAllowed()) {
    parsingWarn(tr("'channel' is not included in 'allowedChannels'. Setting 'channel' to %1").arg(mAllowedChannels->item(0)));
    mChannel->setValue(mAllowedChannels->item(0));
  }

  mNeedToSetChannel = true;
}

void WbEmitter::writeConfigure(WbDataStream &stream) {
  stream << tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)mBufferSize->value();
  stream << (int)mChannel->value();
  stream << (double)mByteRate;
  stream << (double)mRange->value();
  stream << (double)mMaxRange->value();
  stream << (int)mAllowedChannels->size();
  for (int i = 0; i < mAllowedChannels->size(); i++)
    stream << (int)mAllowedChannels->item(i);

  mNeedToSetRange = false;
  mNeedToSetChannel = false;
  mNeedToSetBufferSize = false;
  mNeedToSetAllowedChannels = false;
}

void WbEmitter::writeAnswer(WbDataStream &stream) {
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
  if (mNeedToSetAllowedChannels) {
    stream << tag();
    stream << (unsigned char)C_EMITTER_SET_ALLOWED_CHANNELS;
    stream << (int)mAllowedChannels->size();
    for (int i = 0; i < mAllowedChannels->size(); i++)
      stream << (int)mAllowedChannels->item(i);
  }
}

void WbEmitter::handleMessage(QDataStream &stream) {
  unsigned char command;
  unsigned int size;
  int newChannel;
  double newRange;
  char *data;

  stream >> command;
  switch (command) {
    case C_EMITTER_SEND:
      stream >> newChannel;
      mChannel->setValue(newChannel);
      stream >> newRange;
      mRange->setValue(newRange);
      stream >> size;
      data = new char[size];
      stream.readRawData(data, size);
      mQueue.enqueue(new WbDataPacket(this, mChannel->value(), data, size));
      delete[] data;
      return;

    case C_EMITTER_SET_CHANNEL:
      stream >> newChannel;
      mChannel->setValue(newChannel);
      return;

    case C_EMITTER_SET_RANGE:
      stream >> newRange;
      mRange->setValue(newRange);
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

void WbEmitter::reset(const QString &id) {
  WbSolidDevice::reset(id);
  qDeleteAll(mQueue);
  mQueue.clear();
}
