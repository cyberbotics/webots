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

#include "IPRCollaboration.hpp"

IPRCollaboration::IPRCollaboration() : IPR() {
  mEmitter = getEmitter("emitter");
  mReceiver = getReceiver("receiver");
  if (mReceiver)
    mReceiver->enable(basicTimeStep());

  mMessage = new int[1];
}

IPRCollaboration::~IPRCollaboration() {
  delete[] mMessage;
}

void IPRCollaboration::waitForSignal(CollaborationState stateSignal) {
  while (true) {
    if (mReceiver->getQueueLength() > 0) {
      const int *state = (const int *)mReceiver->getData();
      if (state[0] == stateSignal) {
        mReceiver->nextPacket();
        break;
      }
      mReceiver->nextPacket();
    }

    step(basicTimeStep());
  }
}

void IPRCollaboration::emitSignal(CollaborationState stateSignal) const {
  mMessage[0] = stateSignal;
  mEmitter->send(mMessage, sizeof(int));
}
