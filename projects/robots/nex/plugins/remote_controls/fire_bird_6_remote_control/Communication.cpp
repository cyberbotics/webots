// Copyright 1996-2022 Cyberbotics Ltd.
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

/*
 * Description:  Implementation of Communication.hpp functions
 */

#include "Communication.hpp"
#include "Packet.hpp"
#include "Serial.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>

#include <cstdlib>
#include <cstring>

using namespace std;

static int trial = 3;

Communication::Communication() : mInitialized(false), mSerial(NULL) {
}

Communication::~Communication() {
  cleanup();
}

bool Communication::initialize(const string &port) {
  try {
    mSerial = new Serial(port);
    char *answer;
    char testCMD[5] = {0x4e, 0x45, 0x58, 0x05, 0x01};
    const unsigned char safetyOFF[5] = {0x4e, 0x45, 0x58, 0x89, 0x00};

    // Test message for checking communication
    answer = talk(testCMD);

    if (!answer)
      throw runtime_error("Cannot talk with the robot");

    if (!mSerial)
      throw runtime_error("Cannot talk with the robot");

    // turn off safety
    mSerial->write((const char *)safetyOFF, 5);

    cout << "Running real Fire Bird (" << &answer[0] << ")" << endl;
    free(answer);

    mInitialized = true;
    return true;
  } catch (const runtime_error &e) {
    cerr << "Connection failed with this error" << endl;
    cerr << e.what() << endl;
    cerr << endl;
    cleanup();
    return false;
  }
}

void Communication::cleanup() {
  mInitialized = false;
  if (mSerial) {
    delete mSerial;
    mSerial = NULL;
  }
}

bool Communication::sendPacket(const Packet *packet) {
  try {
    if (mSerial) {
      int packetSize = packet->size();
      if (packetSize > 0)
        mSerial->write(packet->data(), packetSize);
      return true;
    }
  } catch (const runtime_error &e) {
    cerr << "Cannot send packet to Fire Bird: " << e.what() << endl;
  }
  return false;
}

bool Communication::receivePacket(Packet *packet) {
  try {
    if (mSerial) {
      int packetSize = packet->size();
      if (packetSize > 0) {
        int readed = mSerial->read(packet->data(), packetSize, true);
        if (readed != packetSize) {
          stringstream s;
          s << "Expected data not received (expected=" << packetSize << ", received=" << readed << ")" << endl;
          mSerial->drain();
          throw runtime_error(s.str());
        }
      }
      return true;
    }
  } catch (const runtime_error &e) {
    cerr << "Cannot receive packet from Fire Bird: " << e.what() << endl;
  }
  return false;
}

char *Communication::talk(const char *source) {
  static const char *errorPrefix = "Cannot talk to Fire Bird: ";
  if (!mSerial) {
    cerr << errorPrefix << "serial failed" << endl;
    return NULL;
  }

  for (int i = 0; i < trial; i++) {  // several trials
    try {
      char *answer = (char *)malloc(13);
      mSerial->write(source, 5);
      mSerial->read(answer, 13, 1);
      if (answer[0] == 'F')
        return answer;

      free(answer);

      throw runtime_error("Unexpected result");
    } catch (const runtime_error &e) {
      cerr << errorPrefix << e.what() << endl;
    }
    cerr << "Talk to Fire Bird failed... Retry (" << (i + 1) << ")" << endl;
  }

  return NULL;
}

char *Communication::readLine() {
  static const char *errorPrefix = "Cannot read line: ";
  if (!mSerial) {
    cerr << errorPrefix << "serial failed" << endl;
    return NULL;
  }

  try {
    return mSerial->readLine();
  } catch (const runtime_error &e) {
    cerr << errorPrefix << e.what() << endl;
  }
  return NULL;
}
