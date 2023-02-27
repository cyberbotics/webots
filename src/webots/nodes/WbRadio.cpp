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

#include "WbRadio.hpp"

#include "WbDataStream.hpp"
#include "WbRadioPlugin.hpp"
#include "WbSFInt.hpp"
#include "WbSensor.hpp"

#include <QtCore/QDataStream>
#include <cassert>
#include "../../../include/plugins/radio.h"
#include "../../controller/c/messages.h"

static QList<WbRadio *> radioList;  // list of radio nodes
static bool pluginLoadFailed = false;

void WbRadio::init() {
  mSensor = NULL;
  mID = -1;
  mReceivedEvents.clear();
  mNeedUpdateSetup = false;

  mProtocol = findSFString("protocol");
  mTxPowerMin = findSFDouble("txPowerMin");
  mTxPowerMax = findSFDouble("txPowerMax");
  mAddress = findSFString("address");
  mRxSensitivity = findSFDouble("rxSensitivity");
  mTxPower = findSFDouble("txPower");
  mFrequency = findSFDouble("frequency");
  mChannel = findSFInt("channel");
  mBitrate = findSFInt("bitrate");

  radioList.append(this);  // add self
}

WbRadio::WbRadio(WbTokenizer *tokenizer) : WbSolidDevice("Radio", tokenizer) {
  init();
}

WbRadio::WbRadio(const WbRadio &other) : WbSolidDevice(other) {
  init();
}

WbRadio::WbRadio(const WbNode &other) : WbSolidDevice(other) {
  init();
}

WbRadio::~WbRadio() {
  radioList.removeAll(this);  // remove self

  WbRadioPlugin *plugin = WbRadioPlugin::instance();

  if (plugin) {
    // delete matching object in the plugin
    plugin->deleteRadio(mID);

    // when the last radio node is destroyed: unload plugin
    if (!radioList.isEmpty()) {
      WbRadioPlugin::cleanupAndUnload();
      pluginLoadFailed = false;
    }
  }

  delete mSensor;
}

void WbRadio::preFinalize() {
  WbSolidDevice::preFinalize();

  mSensor = new WbSensor();
}

void WbRadio::postFinalize() {
  WbSolidDevice::postFinalize();

  // try to load the plugin
  if (!WbRadioPlugin::instance() && !pluginLoadFailed) {
    WbRadioPlugin::loadAndInit("omnet");
    if (!WbRadioPlugin::instance())
      pluginLoadFailed = true;
  }

  connect(mProtocol, &WbSFString::changed, this, &WbRadio::updateSetup);
  connect(mAddress, &WbSFString::changed, this, &WbRadio::updateSetup);
  connect(mFrequency, &WbSFDouble::changed, this, &WbRadio::updateSetup);
  connect(mChannel, &WbSFInt::changed, this, &WbRadio::updateSetup);
  connect(mBitrate, &WbSFInt::changed, this, &WbRadio::updateSetup);
  connect(mRxSensitivity, &WbSFDouble::changed, this, &WbRadio::updateSetup);
  connect(mTxPower, &WbSFDouble::changed, this, &WbRadio::updateSetup);
}

void WbRadio::updateSetup() {
  // some test on the value can eventually be added here

  // force to resend the 'C_CONFIGURE'
  mNeedUpdateSetup = true;

  // update parameter of this radio in the plugin
  WbRadioPlugin *plugin = WbRadioPlugin::instance();
  if (!plugin)
    return;
  assert(mID != -1);

  plugin->setProtocol(mID, mProtocol->value().toStdString().c_str());
  plugin->setAddress(mID, mAddress->value().toStdString().c_str());
  plugin->setFrequency(mID, mFrequency->value());
  plugin->setChannel(mID, mChannel->value());
  plugin->setBitrate(mID, mBitrate->value());
  plugin->setRxSensitivity(mID, mRxSensitivity->value());
  plugin->setTxPower(mID, mTxPower->value());
}

void WbRadio::handleMessage(QDataStream &stream) {
  WbRadioPlugin *plugin = WbRadioPlugin::instance();
  unsigned char command;
  stream >> command;

  switch (command) {
    case C_SET_SAMPLING_PERIOD:
      short refreshRate;
      stream >> refreshRate;
      mSensor->setRefreshRate(refreshRate);
      return;

    case C_RADIO_SET_ADDRESS: {
      int adressSize = 0;
      stream >> adressSize;
      char address[adressSize];
      stream.readRawData(address, adressSize);
      mAddress->setValue(address);
      if (plugin)
        plugin->setAddress(mID, address);
      return;
    }

    case C_RADIO_SET_FREQUENCY:
      double frequency;
      stream >> frequency;
      mFrequency->setValue(frequency);
      if (plugin)
        plugin->setFrequency(mID, frequency);
      return;

    case C_RADIO_SET_CHANNEL:
      int channel;
      stream >> channel;
      mChannel->setValue(channel);
      if (plugin)
        plugin->setChannel(mID, channel);
      return;

    case C_RADIO_SET_BITRATE:
      int bitrate;
      stream >> bitrate;
      mBitrate->setValue(bitrate);
      if (plugin)
        plugin->setBitrate(mID, bitrate);
      return;

    case C_RADIO_SET_RX_SENSITIVITY:
      double rxSensitivity;
      stream >> rxSensitivity;
      mRxSensitivity->setValue(rxSensitivity);
      if (plugin)
        plugin->setRxSensitivity(mID, rxSensitivity);
      return;

    case C_RADIO_SET_TX_POWER:
      double txPower;
      stream >> txPower;
      mTxPower->setValue(txPower);
      if (plugin)
        plugin->setTxPower(mID, txPower);
      return;

    case C_RADIO_SEND: {
      int destSize = 0;
      stream >> destSize;
      char dest[destSize];
      stream.readRawData(dest, destSize);
      int dataSize = 0;
      stream >> dataSize;
      char data[dataSize];  // 'void *' previously
      stream.readRawData(data, dataSize);
      double delay;
      stream >> delay;
      if (plugin)
        plugin->send(mID, dest, data, dataSize, delay);
      return;
    }

    default:
      assert(0);
  }
}

void WbRadio::writeConfigure(WbDataStream &stream) {
  stream << tag();
  stream << (unsigned char)C_CONFIGURE;
  QByteArray address = mAddress->value().toUtf8();
  stream.writeRawData(address.constData(), address.size() + 1);
  stream << (double)mFrequency->value();
  stream << (int)mChannel->value();
  stream << (int)mBitrate->value();
  stream << (double)mRxSensitivity->value();
  stream << (double)mTxPower->value();
}

// called from WbWorld.cpp
void WbRadio::createAndSetupPluginObjects() {
  WbRadioPlugin *plugin = WbRadioPlugin::instance();
  if (!plugin)
    return;

  // initialize plugin data structures
  plugin->init();

  // create plugin objects
  int radioNB = radioList.count();
  for (int i = 0; i < radioNB; ++i) {
    WbRadio *radio = radioList.at(i);
    assert(radio->mID == -1);
    radio->mID = plugin->newRadio();
  }

  // setup plugin objects
  for (int i = 0; i < radioNB; ++i) {
    WbRadio *radio = radioList.at(i);
    assert(radio->mID != -1);
    plugin->setProtocol(radio->mID, radio->mProtocol->value().toStdString().c_str());
    plugin->setAddress(radio->mID, radio->mAddress->value().toStdString().c_str());
    plugin->setFrequency(radio->mID, radio->mFrequency->value());
    plugin->setChannel(radio->mID, radio->mChannel->value());
    plugin->setBitrate(radio->mID, radio->mBitrate->value());
    plugin->setRxSensitivity(radio->mID, radio->mRxSensitivity->value());
    plugin->setTxPower(radio->mID, radio->mTxPower->value());
    plugin->setCallback(radio->mID, radio, &WbRadio::staticReceiveCallback);
  }
}

void WbRadio::runPlugin(double ms) {
  WbRadioPlugin *plugin = WbRadioPlugin::instance();
  if (!plugin)
    return;

  plugin->run(ms / 1000.0);
}

void WbRadio::postPhysicsStep() {
  WbSolidDevice::postPhysicsStep();

  WbRadioPlugin *plugin = WbRadioPlugin::instance();
  if (!plugin)
    return;

  const WbVector3 &trans = matrix().translation();
  plugin->move(mID, trans.x(), trans.y(), trans.z());
}

void WbRadio::staticReceiveCallback(const int radio, const struct WebotsRadioEvent *event) {
  int radioNB = radioList.count();
  for (int i = 0; i < radioNB; ++i) {
    WbRadio *r = radioList.at(i);
    if (r->mID == radio) {
      r->receiveCallback(event);
      break;
    }
  }
}

// copy WebotsRadioEvent and duplicate substructures
static struct WebotsRadioEvent *radio_event_duplicate(const struct WebotsRadioEvent *orig) {
  struct WebotsRadioEvent *copy = new struct WebotsRadioEvent;
  copy->type = orig->type;
  copy->data = new char[orig->data_size];
  memcpy(static_cast<void *>(const_cast<char *>(copy->data)), orig->data, orig->data_size);
  copy->data_size = orig->data_size;
  int len = strlen(orig->from) + 1;
  copy->from = new char[len];
  strncpy(const_cast<char *>(copy->from), orig->from, len);
  copy->rssi = orig->rssi;
  return copy;
}

// destroy WebotsRadioEvent and substructures
static void radio_event_destroy(struct WebotsRadioEvent *p) {
  delete[] p->data;
  delete[] p->from;
  delete p;
}

void WbRadio::receiveCallback(const struct WebotsRadioEvent *event) {
  // yvan: Radio N>2 bug was fixed here: a *deep* copy of WebotsRadioEvent is required
  struct WebotsRadioEvent *copy = radio_event_duplicate(event);
  mReceivedEvents.append(copy);
}

void WbRadio::writeAnswer(WbDataStream &stream) {
  if (mNeedUpdateSetup) {
    writeConfigure(stream);
    mNeedUpdateSetup = false;
  }

  if (mSensor->needToRefresh()) {
    int receivedEventsNB = mReceivedEvents.count();
    for (int i = 0; i < receivedEventsNB; ++i) {
      struct WebotsRadioEvent *event = mReceivedEvents.at(i);
      stream << tag();
      stream << (unsigned char)C_RADIO_RECEIVE;
      stream << (double)event->rssi;
      QByteArray from = QString(event->from).toUtf8();
      stream.writeRawData(from.constData(), from.size() + 1);
      stream << (int)event->data_size;
      stream.writeRawData(event->data, event->data_size);
      radio_event_destroy(event);
    }
    mReceivedEvents.clear();
  }
}
