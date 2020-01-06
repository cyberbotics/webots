// Copyright 1996-2020 Cyberbotics Ltd.
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

#ifndef IPR_COLLABORATION_HPP
#define IPR_COLLABORATION_HPP

#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include "IPR.hpp"

class IPRCollaboration : public IPR {
public:
  enum CollaborationState { GRAB_CUBE = 0, GIVE_CUBE = 1, LEAVE_CUBE = 2, THROW_CUBE = 3 };

  IPRCollaboration();
  ~IPRCollaboration();

  void waitForSignal(CollaborationState stateSignal);
  void emitSignal(CollaborationState stateSignal) const;

private:
  int *mMessage;
  Emitter *mEmitter;
  Receiver *mReceiver;
};

#endif
