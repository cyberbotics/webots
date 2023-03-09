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

#ifndef WB_RECEIVER_HPP
#define WB_RECEIVER_HPP

#include "WbMFInt.hpp"
#include "WbSFDouble.hpp"
#include "WbSFInt.hpp"
#include "WbSolidDevice.hpp"

#include <QtCore/QQueue>

class WbDataPacket;
class WbEmitter;
class WbSensor;
class Transmission;

class WbReceiver : public WbSolidDevice {
  Q_OBJECT

public:
  static WbReceiver *createPhysicsReceiver();

  // constructors and destructor
  explicit WbReceiver(WbTokenizer *tokenizer = NULL);
  WbReceiver(const WbReceiver &other);
  explicit WbReceiver(const WbNode &other);
  virtual ~WbReceiver();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_RECEIVER; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(WbDataStream &) override;
  void writeConfigure(WbDataStream &) override;
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  bool refreshSensorIfNeeded() override;
  void reset(const QString &id) override;

  static void transmitData(int channel, const void *data, int size);
  static void transmitPacket(WbDataPacket *packet);

  void rayCollisionCallback(dGeomID geom, WbSolid *obstacle);

  // field accessors
  double aperture() const { return mAperture->value(); }
  int channel() const { return mChannel->value(); }
  int mediumType() const { return mMediumType; }

signals:
  void dataReceived(const void *data, int size) const;

private:
  int mMediumType;                          // UNKNOWN, RADIO, SERIAL or INFRA_RED
  QList<Transmission *> mTransmissionList;  // packets waiting for potential collision detection callback
  QList<WbDataPacket *> mWaitingQueue;      // queue of packets received since last sensor update
  QList<WbDataPacket *> mReadyQueue;        // queue of packets received before last sensor update
  WbSensor *mSensor;
  int mLastWaitingPacketIndex;

  // user accessible fields
  WbSFString *mType;
  WbSFDouble *mAperture;
  WbSFInt *mChannel;
  WbSFInt *mBaudRate;
  WbSFInt *mByteSize;
  WbSFInt *mBufferSize;
  WbSFDouble *mSignalStrengthNoise;
  WbSFDouble *mDirectionNoise;
  WbMFInt *mAllowedChannels;

  bool mNeedToConfigure;

  // private functions
  WbReceiver &operator=(const WbReceiver &);  // non copyable
  WbNode *clone() const override { return new WbReceiver(*this); }
  void init();
  void receiveData(int channel, const void *data, int size);
  void receivePacketIfPossible(WbDataPacket *packet);
  bool checkApertureAndRange(const WbEmitter *emitter, const WbReceiver *receiver, bool checkRangeOnly = false) const;
  void updateRaysSetupIfNeeded() override;
  bool isChannelAllowed();

private slots:
  void updateTransmissionSetup();
  void updateAllowedChannels();
};

#endif  // WB_RECEIVER_HPP
