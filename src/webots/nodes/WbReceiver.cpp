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

#include "WbReceiver.hpp"

#include "WbDataPacket.hpp"
#include "WbDataStream.hpp"
#include "WbEmitter.hpp"
#include "WbFieldChecker.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeGeomData.hpp"
#include "WbRandom.hpp"
#include "WbSensor.hpp"
#include "WbWorld.hpp"

#include <webots/receiver.h>  // for WB_CHANNEL_BROADCAST
#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <cassert>

static QList<WbReceiver *> gReceiverList;
static bool gIsInPhysicsPlugin = false;

class Transmission {
public:
  Transmission(WbDataPacket *packet, WbReceiver *r, dSpaceID spaceId) : mPacket(packet), mCollided(false) {
    assert(spaceId && packet && r);
    WbEmitter *e = packet->emitter();
    assert(e);
    mEmittingRobot = e->robot();
    assert(mEmittingRobot);

    // compute ray direction and length
    const WbVector3 &te = e->matrix().translation();
    WbVector3 dir = r->matrix().translation() - te;

    // setup ray geom for ODE collision detection
    mGeom = dCreateRay(spaceId, dir.length());
    dGeomSetDynamicFlag(mGeom);
    dGeomRaySet(mGeom, te[0], te[1], te[2], dir[0], dir[1], dir[2]);

    // set receiver as callback data in case there is a collision
    dGeomSetData(mGeom, new WbOdeGeomData(r));
  };

  ~Transmission() {
    if (mGeom) {
      WbOdeGeomData *odeGeomData = static_cast<WbOdeGeomData *>(dGeomGetData(mGeom));
      delete odeGeomData;
      dGeomDestroy(mGeom);
    }
  }

  WbDataPacket *packet() { return mPacket; }
  bool hasCollided() const { return mCollided; }
  void setCollided() { mCollided = true; }
  const WbRobot *emittingRobot() const { return mEmittingRobot; }
  dGeomID geom() const { return mGeom; }

  void recomputeRayDirection(const WbVector3 &receiverTranslation) {
    // compute ray direction and length
    WbEmitter *e = mPacket->emitter();
    e->updateTransformForPhysicsStep();
    const WbVector3 &te = e->matrix().translation();
    WbVector3 dir = receiverTranslation - te;
    dGeomRaySetLength(mGeom, dir.length());
    dGeomRaySet(mGeom, te[0], te[1], te[2], dir[0], dir[1], dir[2]);
  }

private:
  WbDataPacket *mPacket;    // packet being transmitted
  WbRobot *mEmittingRobot;  // robot that emitted the packet
  dGeomID mGeom;            // geom that checks collision of this packet
  bool mCollided;           // the geom has collided yet
};

WbReceiver *WbReceiver::createPhysicsReceiver() {
  gIsInPhysicsPlugin = true;
  WbReceiver *r = new WbReceiver();
  gIsInPhysicsPlugin = false;
  return r;
}

void WbReceiver::init() {
  if (gIsInPhysicsPlugin) {
    mSensor = NULL;
    return;
  }
  mNeedToConfigure = false;
  mSensor = NULL;
  mLastWaitingPacketIndex = 0;
  mMediumType = WbDataPacket::UNKNOWN;
  mType = findSFString("type");
  mAperture = findSFDouble("aperture");
  mChannel = findSFInt("channel");
  mBaudRate = findSFInt("baudRate");
  mByteSize = findSFInt("byteSize");
  mBufferSize = findSFInt("bufferSize");
  mSignalStrengthNoise = findSFDouble("signalStrengthNoise");
  mDirectionNoise = findSFDouble("directionNoise");
  mAllowedChannels = findMFInt("allowedChannels");
}

WbReceiver::WbReceiver(WbTokenizer *tokenizer) : WbSolidDevice("Receiver", tokenizer) {
  init();
}

WbReceiver::WbReceiver(const WbReceiver &other) : WbSolidDevice(other) {
  init();
}

WbReceiver::WbReceiver(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbReceiver::~WbReceiver() {
  delete mSensor;

  // remove collision detection pending packets
  qDeleteAll(mTransmissionList);

  // remove unsent packets
  qDeleteAll(mReadyQueue);
  qDeleteAll(mWaitingQueue);
  mLastWaitingPacketIndex = 0;

  // remove myself
  gReceiverList.removeAll(this);
}

void WbReceiver::preFinalize() {
  WbSolidDevice::preFinalize();

  mSensor = new WbSensor();
  updateTransmissionSetup();
  updateAllowedChannels();

  gReceiverList.append(this);  // add myself
}

void WbReceiver::postFinalize() {
  WbSolidDevice::postFinalize();

  connect(mChannel, &WbSFInt::changed, this, &WbReceiver::updateTransmissionSetup);
  connect(mType, &WbSFString::changed, this, &WbReceiver::updateTransmissionSetup);
  connect(mAperture, &WbSFDouble::changed, this, &WbReceiver::updateTransmissionSetup);
  connect(mBufferSize, &WbSFInt::changed, this, &WbReceiver::updateTransmissionSetup);
  connect(mBaudRate, &WbSFInt::changed, this, &WbReceiver::updateTransmissionSetup);
  connect(mByteSize, &WbSFInt::changed, this, &WbReceiver::updateTransmissionSetup);
  connect(mSignalStrengthNoise, &WbSFDouble::changed, this, &WbReceiver::updateTransmissionSetup);
  connect(mDirectionNoise, &WbSFDouble::changed, this, &WbReceiver::updateTransmissionSetup);
  connect(mAllowedChannels, &WbMFInt::changed, this, &WbReceiver::updateAllowedChannels);
}

void WbReceiver::updateTransmissionSetup() {
  mMediumType = WbDataPacket::decodeMediumType(mType->value());
  if (mMediumType == WbDataPacket::UNKNOWN) {
    parsingWarn(tr("Unknown 'type': \"%1\".").arg(mType->value()));
    mMediumType = WbDataPacket::RADIO;
  }

  WbFieldChecker::resetDoubleIfNotInRangeWithIncludedBoundsAndNotDisabled(this, mAperture, 0, 2 * M_PI, -1, -1);
  WbFieldChecker::resetDoubleIfNegative(this, mSignalStrengthNoise, 0);
  WbFieldChecker::resetDoubleIfNegative(this, mDirectionNoise, 0);
  WbFieldChecker::resetIntIfNonPositiveAndNotDisabled(this, mBufferSize, -1, -1);
  WbFieldChecker::resetIntIfNonPositiveAndNotDisabled(this, mBaudRate, -1, -1);
  WbFieldChecker::resetIntIfLess(this, mByteSize, 8, 8);

  if (!isChannelAllowed()) {
    parsingWarn(tr("'channel' is not included in 'allowedChannels'. Setting 'channel' to %1").arg(mAllowedChannels->item(0)));
    mChannel->setValue(mAllowedChannels->item(0));
  }

  mNeedToConfigure = true;
}

bool WbReceiver::isChannelAllowed() {
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

void WbReceiver::updateAllowedChannels() {
  if (!isChannelAllowed()) {
    parsingWarn(
      tr("'allowedChannels' does not contain current 'channel'. Setting 'channel' to %1.").arg(mAllowedChannels->item(0)));
    mChannel->setValue(mAllowedChannels->item(0));
  }

  mNeedToConfigure = true;
}

void WbReceiver::writeConfigure(WbDataStream &stream) {
  // TODO disable in remote or not ?
  mSensor->connectToRobotSignal(robot(), false);

  stream << tag();
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)mBufferSize->value();
  stream << (int)mChannel->value();
  stream << (int)mAllowedChannels->size();
  for (int i = 0; i < mAllowedChannels->size(); i++)
    stream << (int)mAllowedChannels->item(i);
  mNeedToConfigure = false;
}

void WbReceiver::writeAnswer(WbDataStream &stream) {
  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    for (int i = 0; i < mReadyQueue.size(); i++) {
      WbDataPacket *packet = mReadyQueue[i];
      stream << tag();
      stream << (unsigned char)C_RECEIVER_RECEIVE;
      stream << (int)mChannel->value();
      const WbVector3 emitterDir = packet->emitterDir();
      stream << (double)emitterDir[0];
      stream << (double)emitterDir[1];
      stream << (double)emitterDir[2];
      stream << (double)packet->signalStrength();
      stream << (int)packet->dataSize();
      stream.writeRawData(static_cast<const char *>(packet->data()), packet->dataSize());
      delete packet;
    }
    mReadyQueue.clear();
    mSensor->resetPendingValue();
  }

  if (mNeedToConfigure)
    writeConfigure(stream);
}

void WbReceiver::handleMessage(QDataStream &stream) {
  unsigned char command;
  stream >> command;

  switch (command) {
    case C_SET_SAMPLING_PERIOD: {
      short rate;
      stream >> rate;
      mSensor->setRefreshRate(rate);
      return;
    }
    case C_RECEIVER_SET_CHANNEL: {
      int receiverChannel;
      stream >> receiverChannel;
      mChannel->setValue(receiverChannel);
      return;
    }
    default:
      assert(0);
  }
}

void WbReceiver::prePhysicsStep(double ms) {
  WbSolidDevice::prePhysicsStep(ms);
  if (mSensor->isEnabled() && !mTransmissionList.isEmpty())
    // receiver or emitter could move during physics step
    subscribeToRaysUpdate(mTransmissionList[0]->geom());
}

void WbReceiver::updateRaysSetupIfNeeded() {
  updateTransformForPhysicsStep();
  const WbVector3 position = matrix().translation();
  // update receiver position in pending packets
  foreach (Transmission *t, mTransmissionList)
    t->recomputeRayDirection(position);
}

// This function searches and destroys the information packets that have collided with an obstacle.
// It is called when the collision detection is over, that means
// after the last call to the function: WbReceiver::rayCollisionCallback()
// All the packets that did not collide with an obstacle are appended to the Receiver's buffer.
void WbReceiver::postPhysicsStep() {
  WbSolidDevice::postPhysicsStep();

  if (mMediumType == WbDataPacket::INFRA_RED) {
    // check aperture, range and if communication is blocked by an obstacles
    // forward and delete all pending packets
    mLastWaitingPacketIndex = mWaitingQueue.size();
    WbDataPacket *packet = NULL;
    foreach (Transmission *t, mTransmissionList) {
      packet = t->packet();
      if (packet->emitter() == NULL ||  // sent from physics plugin
          (checkApertureAndRange(packet->emitter(), this) && !t->hasCollided()))
        // push to waiting queue: ready for sending to controller
        mWaitingQueue.append(packet);
      else
        delete packet;

      delete t;
    }
    mTransmissionList.clear();

  } else {
    // check aperture and range
    for (int i = (mWaitingQueue.size() - 1); i >= mLastWaitingPacketIndex; --i) {
      WbDataPacket *packet = mWaitingQueue[i];
      if (packet->emitter() != NULL && !checkApertureAndRange(packet->emitter(), this, true)) {
        // remove and delete packet
        mWaitingQueue.removeAll(packet);
        delete packet;
      }
    }
  }

  if (mLastWaitingPacketIndex < mWaitingQueue.size()) {
    const WbMatrix4 &m = matrix();

    // compute emitter direction and signal strength
    for (int i = mLastWaitingPacketIndex; i < mWaitingQueue.size(); ++i) {
      WbDataPacket *packet = mWaitingQueue.at(i);
      if (packet->emitter() == NULL)
        // sent from supervisor
        continue;

      const WbVector3 &t = packet->emitter()->matrix().translation();
      const WbVector3 &emitterPos = m.pseudoInversed(t);

      double dist = emitterPos.length();  // Compute distance from emitter
      // compute the emitter direction vector
      if (mDirectionNoise->value() == 0)  // No noise on direction
        packet->setEmitterDir(emitterPos / dist);
      else {
        WbVector3 emitterNoisedPos(emitterPos[0], emitterPos[1], emitterPos[2]);  // Extraction of the real emitter position
        emitterNoisedPos[0] += dist * mDirectionNoise->value() * WbRandom::nextGaussian();  // Add noise on each components
        emitterNoisedPos[1] += dist * mDirectionNoise->value() * WbRandom::nextGaussian();
        emitterNoisedPos[2] += dist * mDirectionNoise->value() * WbRandom::nextGaussian();
        // Recompute distance from new position in order to have a correct normalization of the vector
        double newDist = emitterNoisedPos.length();
        packet->setEmitterDir(emitterNoisedPos / newDist);  // Send noisy position normalized
      }

      // simulate signal strength from known distance
      if (mSignalStrengthNoise->value() == 0)  // No noise on the signal strength
        packet->setSignalStrength(1.0 / (dist * dist));
      else {
        double signalStrength = 1.0 / (dist * dist);
        signalStrength *= (1 + mSignalStrengthNoise->value() * WbRandom::nextGaussian());  // add noise
        if (signalStrength < 0)
          signalStrength = 0;
        packet->setSignalStrength(signalStrength);  // Send noisy signal strength
      }
    }
  }
}

// if the packet is blocked by an obstacle we can discard it
void WbReceiver::rayCollisionCallback(dGeomID geom, WbSolid *obstacle) {
  foreach (Transmission *t, mTransmissionList) {
    if (t->geom() == geom) {
      // we want to ignore collision of ray with the emitting robot
      // (collision with receiver's robot is already filtered out by odeNearCallback())
      if (!t->hasCollided() && obstacle->robot() != t->emittingRobot())
        t->setCollided();

      return;  // we have found the right packet: no need to look further in the list
    }
  }

  assert(0);  // should never be reached
}

bool WbReceiver::refreshSensorIfNeeded() {
  if (!isPowerOn() || !mSensor->needToRefresh())
    return false;

  mReadyQueue.append(mWaitingQueue);
  mWaitingQueue.clear();
  mLastWaitingPacketIndex = 0;
  mSensor->updateTimer();
  return true;
}

void WbReceiver::reset(const QString &id) {
  WbSolidDevice::reset(id);
  qDeleteAll(mTransmissionList);
  mTransmissionList.clear();
  qDeleteAll(mReadyQueue);
  mReadyQueue.clear();
  qDeleteAll(mWaitingQueue);
  mWaitingQueue.clear();
  mLastWaitingPacketIndex = 0;
}

// return true is a transmission respects the emitter's and receiver's aperture
bool WbReceiver::checkApertureAndRange(const WbEmitter *emitter, const WbReceiver *receiver, bool checkRangeOnly) const {
  WbVector3 eTranslation = emitter->matrix().translation();
  WbVector3 rTranslation = receiver->matrix().translation();

  // out of range ?
  if (emitter->range() != -1.0) {
    double range2 = emitter->range() * emitter->range();
    double distance = (rTranslation - eTranslation).length2();
    if (distance > range2)
      return false;
  }

  if (checkRangeOnly)
    return true;

  // emission: check that receiver is within emitter's cone
  if (emitter->aperture() > 0.0) {
    const WbVector3 e2r = rTranslation - eTranslation;
    const WbVector4 eAxisX4 = emitter->matrix().column(0);
    const WbVector3 eAxisX3(eAxisX4[0], eAxisX4[1], eAxisX4[2]);
    if (eAxisX3.angle(e2r) > emitter->aperture() / 2.0)
      return false;
  }

  // reception: check that emitter is within receiver's cone
  if (receiver->aperture() > 0.0) {
    const WbVector3 r2e = eTranslation - rTranslation;
    const WbVector4 rAxisX4 = receiver->matrix().column(0);
    const WbVector3 rAxisX3(rAxisX4[0], rAxisX4[1], rAxisX4[2]);
    if (rAxisX3.angle(r2e) > receiver->aperture() / 2.0)
      return false;
  }

  return true;
}

void WbReceiver::transmitPacket(WbDataPacket *packet) {
  const WbEmitter *e = packet->emitter();
  // look for emitting robot
  const WbRobot *eRobot = e->robot();
  assert(eRobot);

  // loop through emitters
  for (int i = 0; i < gReceiverList.size(); i++) {
    WbReceiver *r = gReceiverList.at(i);

    // was the receiver enabled ?
    if (r->mSensor->isEnabled()) {
      // media types must match
      if (e->mediumType() == r->mediumType()) {
        // channel numbers must match
        if ((packet->channel() == r->channel()) || (packet->channel() == WB_CHANNEL_BROADCAST) ||
            (r->channel() == WB_CHANNEL_BROADCAST)) {
          // look for receiving robot
          const WbRobot *rRobot = r->robot();
          assert(rRobot);

          // robot cannot send message to self
          if (eRobot != rRobot) {
            // aperture will be checked in postPhysicsStep
            // we check collision only for infra-red
            if (r->mediumType() != WbDataPacket::INFRA_RED)
              // push to waiting queue: ready for sending to controller
              // range will be checked in postPhysicsStep
              r->mWaitingQueue.append(new WbDataPacket(*packet));
            else
              // register for collision detection (append)
              r->mTransmissionList.append(new Transmission(new WbDataPacket(*packet), r, WbOdeContext::instance()->space()));
          }
        }
      }
    }
  }

  // the physics receiver channel is 0
  const WbReceiver *physicsReceiver = WbWorld::instance()->worldInfo()->physicsReceiver();
  if (e->channel() == 0 && physicsReceiver)
    emit physicsReceiver->dataReceived(packet->data(), packet->dataSize());

  // we are done with this packet: let's destroy it !
  delete packet;
}

void WbReceiver::receiveData(int channel, const void *data, int size) {
  mWaitingQueue.append(new WbDataPacket(NULL, channel, static_cast<const char *>(data), size));
}

void WbReceiver::transmitData(int channel, const void *data, int size) {
  for (int i = 0; i < gReceiverList.size(); i++) {
    WbReceiver *r = gReceiverList.at(i);
    if (r->mSensor->isEnabled() && channel == r->channel())
      r->receiveData(channel, data, size);
  }
}
