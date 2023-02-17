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

#ifndef WB_MICROPHONE_HPP
#define WB_MICROPHONE_HPP

#include "WbSFDouble.hpp"
#include "WbSolidDevice.hpp"

class WbSensor;

class WbMicrophone : public WbSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbMicrophone(WbTokenizer *tokenizer = NULL);
  WbMicrophone(const WbMicrophone &other);
  explicit WbMicrophone(const WbNode &other);
  virtual ~WbMicrophone();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_MICROPHONE; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &stream) override;
  void writeAnswer(WbDataStream &stream) override;
  void writeConfigure(WbDataStream &stream) override;

  // field accessors
  double aperture() const { return mAperture->value(); }
  double sensitivity() const { return mSensitivity->value(); }

  // receive sample from plugin
  void receiveSoundSample(char *sample, int size);

signals:
  void soundSampleRequested(double lastUpdate);

protected:
private:
  // user accessible fields
  WbSFDouble *mAperture;
  WbSFDouble *mSensitivity;

  WbSensor *mSensor;
  char *mSoundSample;  // current sound sample
  int mSampleSize;     // size of current sound sample

  WbMicrophone &operator=(const WbMicrophone &);  // non copyable
  WbNode *clone() const override { return new WbMicrophone(*this); }
  void init();
  void computeValue();

private slots:
  void updateAperture();
};

#endif
