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

#ifndef WB_EMITTER_HPP
#define WB_EMITTER_HPP

#include "WbSFDouble.hpp"
#include "WbSFInt.hpp"
#include "WbSolidDevice.hpp"

#include <QtCore/QQueue>

class WbDataPacket;

class WbEmitter : public WbSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbEmitter(WbTokenizer *tokenizer = NULL);
  WbEmitter(const WbEmitter &other);
  explicit WbEmitter(const WbNode &other);
  virtual ~WbEmitter();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_EMITTER; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeConfigure(QDataStream &) override;
  void writeAnswer(QDataStream &) override;
  void prePhysicsStep(double ms) override;
  void reset(const QString &id) override;

  // field accessors
  int channel() const { return mChannel->value(); }
  double range() const { return mRange->value(); }
  double aperture() const { return mAperture->value(); }
  int mediumType() const { return mMediumType; }

private:
  int mMediumType;                // UNKNOWN, RADIO, SERIAL or INFRA_RED
  double mByteRate;               // expressed in bytes per millisecond;
  QQueue<WbDataPacket *> mQueue;  // emission queue
  bool mNeedToSetRange;
  bool mNeedToSetChannel;
  bool mNeedToSetBufferSize;

  // user accessible fields
  WbSFString *mType;
  WbSFDouble *mRange;
  WbSFDouble *mMaxRange;
  WbSFDouble *mAperture;
  WbSFInt *mChannel;
  WbSFInt *mBaudRate;
  WbSFInt *mByteSize;
  WbSFInt *mBufferSize;

  // private functions
  WbEmitter &operator=(const WbEmitter &);  // non copyable
  WbNode *clone() const override { return new WbEmitter(*this); }
  void init();

private slots:
  void updateTransmissionSetup();
  void updateBufferSize();
  void updateRange();
  void updateChannel();
};

#endif  // WB_EMITTER_HPP
