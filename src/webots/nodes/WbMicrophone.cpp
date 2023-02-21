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

#include "WbMicrophone.hpp"

#include "WbDataStream.hpp"
#include "WbSFDouble.hpp"
#include "WbSensor.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <cassert>

void WbMicrophone::init() {
  mSoundSample = NULL;
  mSampleSize = 0;
  mSensor = NULL;

  mAperture = findSFDouble("aperture");
  mSensitivity = findSFDouble("sensitivity");
}

WbMicrophone::WbMicrophone(WbTokenizer *tokenizer) : WbSolidDevice("Microphone", tokenizer) {
  init();
}

WbMicrophone::WbMicrophone(const WbMicrophone &other) : WbSolidDevice(other) {
  init();
}

WbMicrophone::WbMicrophone(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbMicrophone::~WbMicrophone() {
  delete[] mSoundSample;
  delete mSensor;
}

void WbMicrophone::preFinalize() {
  WbSolidDevice::preFinalize();
  mSensor = new WbSensor();
  updateAperture();
}

void WbMicrophone::postFinalize() {
  WbSolidDevice::postFinalize();
  connect(mAperture, &WbSFDouble::changed, this, &WbMicrophone::updateAperture);
}

void WbMicrophone::updateAperture() {
  if (mAperture->value() < 0.0 && mAperture->value() != -1.0)
    parsingWarn(tr("'aperture' must be either -1 (infinity) or between 0 and 2*pi."));
}

void WbMicrophone::writeConfigure(WbDataStream &stream) {
  mSensor->connectToRobotSignal(robot());

  stream << tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (double)mAperture->value();
  stream << (double)mSensitivity->value();
}

void WbMicrophone::writeAnswer(WbDataStream &stream) {
  if (isPowerOn() && mSensor->needToRefresh()) {
    // get sample from plugin
    computeValue();
    if (mSoundSample) {
      stream << tag();
      stream << (unsigned char)C_MICROPHONE_RECEIVE;
      stream << (int)mSampleSize;
      stream.writeRawData(mSoundSample, mSampleSize);
      delete[] mSoundSample;
      mSoundSample = NULL;
    }
    mSensor->updateTimer();
  }
}

void WbMicrophone::handleMessage(QDataStream &stream) {
  unsigned char command;
  stream >> command;

  switch (command) {
    case C_SET_SAMPLING_PERIOD: {
      short rate;
      stream >> rate;
      mSensor->setRefreshRate(rate);
      return;
    }
    default:
      assert(0);
  }
}

void WbMicrophone::receiveSoundSample(char *sample, int size) {
  mSoundSample = sample;
  mSampleSize = size;
}

void WbMicrophone::computeValue() {
  emit soundSampleRequested(mSensor->lastUpdate());
}
