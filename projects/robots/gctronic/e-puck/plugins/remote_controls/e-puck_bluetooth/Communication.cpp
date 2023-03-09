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
    // Note: the first talk always fail because the first command is badly interpreted
    // A first write has to be performed to test
    mSerial->write("V\n", 2);
    mSerial->drain();
    char *answer = talk("V\n");
    if (!answer)
      throw runtime_error("Cannot talk with the robot");
    const char *robot;
    if (answer[10] == '1')
      robot = "e-puck";
    else
      robot = "e-puck2";
    cout << "Running real " << robot << " (" << &answer[2] << ")" << endl;
    // The firmware string should be "Version 1.5.3 November 2016 (Webots)" for e-puck 1 and
    // it should start with "Version 2." for e-puck-2
    static const char *expectedFirmware1 = "Version 1.5.3 November 2016 (Webots)";
    static const char *expectedFirmware2 = "Version 2.";
    if (strcasecmp(&answer[2], expectedFirmware1) && strncasecmp(&answer[2], expectedFirmware2, strlen(expectedFirmware2))) {
      cerr << "The firmware installed on the e-puck is " << &answer[2] << endl;
      cerr << "It is not compatible with the Webots remote-control protocol." << endl;
      cerr << "Please install a compatible firmware." << endl;
      free(answer);
      return false;
    }
    free(answer);

    answer = mSerial->readLine();
    free(answer);

    answer = talk("S\n");
    if (!answer)
      throw runtime_error("Cannot talk with the robot");
    free(answer);

    mInitialized = true;
    return true;
  } catch (const runtime_error &e) {
    cerr << "Bluetooth connection failed with this error" << endl;
    cerr << e.what() << endl;
    cerr << endl;
    cerr << "Please check:" << endl;
    cerr << "- your Bluetooth interface is setup properly on your computer" << endl;
    cerr << "- the ID of the robot corresponds to the bluetooth connection" << endl;
    cerr << "- the robot runs the latest version of the sercom program" << endl;
    cerr << "- the robot is not too far away from the Bluetooth antenna of the computer" << endl;
    cerr << "- the robot is switched on (the green led should be on)" << endl;
    cerr << "- the battery of the robot is fully charged (the red led should off)" << endl;
    cerr << "- the robot reset button was pressed" << endl;
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
    cerr << "Cannot send packet to e-puck: " << e.what() << endl;
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
    cerr << "Cannot receive packet from e-puck: " << e.what() << endl;
  }
  return false;
}

char *Communication::talk(const char *source) {
  static const char *errorPrefix = "Cannot talk to e-puck: ";
  if (!mSerial) {
    cerr << errorPrefix << "serial failed" << endl;
    return NULL;
  }
  char expectedFirstChar = source[0] + 32;  // lower case answer
  for (int i = 0; i < trial; i++) {         // several trials
    try {
      char *answer = mSerial->talk(source);
      if (answer && strlen(answer) > 0 && answer[0] == expectedFirstChar)
        return answer;
      if (answer)
        free(answer);
      throw runtime_error("Unexpected result");
    } catch (const runtime_error &e) {
      cerr << errorPrefix << e.what() << endl;
    }
    cerr << "Talk to e-puck failed... Retry (" << (i + 1) << ")" << endl;
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
