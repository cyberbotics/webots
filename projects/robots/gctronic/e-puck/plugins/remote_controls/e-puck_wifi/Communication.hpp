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

#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include <webots/types.h>
#include <string>

class EPuckCommandPacket;
class EPuckInputPacket;

class Communication {
public:
  Communication();
  virtual ~Communication();

  bool initialize(const std::string &ip);
  void cleanup();
  bool isInitialized() const { return mFd > 0; }
  bool send(const char *, int size);
  int receive(char *, int size, bool block);

private:
  int mFd;
};

#endif
