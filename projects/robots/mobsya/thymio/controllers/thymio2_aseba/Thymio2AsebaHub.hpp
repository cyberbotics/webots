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

// Strongly inspired from https://github.com/aseba-community/aseba/blob/master/targets/challenge/challenge.cpp

#ifndef THYMIO2_ASEBA_STUDIO_HUB_HPP
#define THYMIO2_ASEBA_STUDIO_HUB_HPP

#ifndef ASEBA_ASSERT
#define ASEBA_ASSERT
#endif

#include "thymio2_definitions.h"

#include <vm/vm.h>

#include <dashel/dashel.h>

#include <valarray>

class Thymio2AsebaHub : public Dashel::Hub {
public:
  explicit Thymio2AsebaHub(int port);
  virtual ~Thymio2AsebaHub();

  void connectionCreated(Dashel::Stream *stream);
  void incomingData(Dashel::Stream *stream);
  void connectionClosed(Dashel::Stream *stream, bool abnormal);
  void sendEvents(bool events[]);

  Dashel::Stream *stream;
  AsebaVMState vm;
  std::valarray<unsigned short> bytecode;
  std::valarray<signed short> stack;
  _vmVariables variables;

  unsigned short int lastMessageSource;
  std::valarray<unsigned char> lastMessageData;
};

#endif
