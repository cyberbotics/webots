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
 * Description:  Defines an interface to communicate on the serial port
 */

#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <cassert>
#endif

class Serial {
public:
  explicit Serial(const std::string &port);
  ~Serial();
  void drain();  // read data until no more data
  void write(const char *packet, int size);
  int read(char *packet, int size, bool wait);
  char *talk(const char *source);
  char *readLine();

  static void updatePorts();
  static const std::vector<std::string> *availablePorts() { return &portNames; }

private:
  static std::vector<std::string> portNames;

  void throwFatalException(const std::string &errorMessage, bool displayLastError = false) const;

  std::string mName;
#ifdef _WIN32
  static DWORD readFileThread(void *param);
  HANDLE mFd;
#else
  int mFd;
#endif
};

#endif
