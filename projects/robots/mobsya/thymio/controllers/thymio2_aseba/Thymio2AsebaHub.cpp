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

// Strongly inspired from https://github.com/aseba-community/aseba/blob/master/targets/challenge/challenge.cpp

#include "Thymio2AsebaHub.hpp"

#include <common/consts.h>
#include <common/productids.h>
#include <transport/buffer/vm-buffer.h>

#include <cstring>  // memcpy
#include <iostream>
#include <sstream>

using namespace std;

static Thymio2AsebaHub *thymio2 = NULL;

Thymio2AsebaHub::Thymio2AsebaHub(int port) : stream(0) {
  thymio2 = this;

  vm.nodeId = 1;

  bytecode.resize(512);
  vm.bytecode = &bytecode[0];
  vm.bytecodeSize = bytecode.size();

  stack.resize(64);
  vm.stack = &stack[0];
  vm.stackSize = stack.size();

  vm.variables = reinterpret_cast<sint16 *>(&variables);
  vm.variablesSize = sizeof(variables) / sizeof(sint16);

  try {
    std::ostringstream o;
    o << "tcpin:port=" << port;
    Dashel::Hub::connect(o.str());
    cout << "Server started on port " << port << "..." << std::endl;
  } catch (Dashel::DashelException &e) {
    cerr << "Cannot create listening port " << port << ": " << e.what() << std::endl;
    abort();
  }

  AsebaVMInit(&vm);

  variables.productid = ASEBA_PID_THYMIO2;  // TODO: is this useful ?
}

Thymio2AsebaHub::~Thymio2AsebaHub() {
  thymio2 = NULL;
}

void Thymio2AsebaHub::connectionCreated(Dashel::Stream *stream) {
  std::string targetName = stream->getTargetName();
  if (targetName.substr(0, targetName.find_first_of(':')) == "tcp") {
    cout << "New client connected." << endl;
    if (this->stream) {
      closeStream(this->stream);
      cerr << " Disconnected old client." << endl;
    }
    this->stream = stream;
  }
}

void Thymio2AsebaHub::incomingData(Dashel::Stream *stream) {
  uint16 temp;
  uint16 len;

  stream->read(&temp, 2);
  len = bswap16(temp);
  stream->read(&temp, 2);
  lastMessageSource = bswap16(temp);
  lastMessageData.resize(len + 2);
  stream->read(&lastMessageData[0], lastMessageData.size());

  if (bswap16(*(uint16 *)&lastMessageData[0]) >= 0xA000)
    AsebaProcessIncomingEvents(&vm);
  else
    cerr << "Non debug event dropped." << endl;
}

void Thymio2AsebaHub::connectionClosed(Dashel::Stream *stream, bool abnormal) {
  if (stream == this->stream) {
    this->stream = 0;
    vm.breakpointsCount = 0;
  }
  if (abnormal)
    cerr << "Client has disconnected unexpectedly." << endl;
  else
    cout << "Client has disconnected properly." << endl;
}

void Thymio2AsebaHub::sendEvents(bool events[]) {
  bool vmHasRun = false;

  if (AsebaMaskIsClear(vm.flags, ASEBA_VM_STEP_BY_STEP_MASK) || AsebaMaskIsClear(vm.flags, ASEBA_VM_EVENT_ACTIVE_MASK)) {
    for (int i = 0; i < EVENT_COUNT; ++i) {
      if (events[i]) {
        AsebaVMSetupEvent(&vm, ASEBA_EVENT_LOCAL_EVENTS_START - i);
        AsebaVMRun(&vm, 1000);
        vmHasRun = true;
      }
    }
  }

  if (!vmHasRun)
    AsebaVMRun(&vm, 1000);
}

extern "C" {

void AsebaPutVmToSleep(AsebaVMState *vm) {
}

void AsebaSendBuffer(AsebaVMState *vm, const uint8 *data, uint16 length) {
  Dashel::Stream *stream = thymio2->stream;
  if (!stream) {
    cerr << "Invalid stream" << std::endl;
    return;
  }

  try {
    uint16 temp;
    temp = bswap16(length - 2);
    stream->write(&temp, 2);
    temp = bswap16(vm->nodeId);
    stream->write(&temp, 2);
    stream->write(data, length);
    stream->flush();
  } catch (Dashel::DashelException &e) {
    cerr << "Cannot write to socket: " << stream->getFailReason() << std::endl;
  }
}

uint16 AsebaGetBuffer(AsebaVMState *vm, uint8 *data, uint16 maxLength, uint16 *source) {
  if (thymio2->lastMessageData.size()) {
    *source = thymio2->lastMessageSource;
    memcpy(data, &thymio2->lastMessageData[0], thymio2->lastMessageData.size());
  }
  return thymio2->lastMessageData.size();
}

extern AsebaVMDescription vmDescription;
const AsebaVMDescription *AsebaGetVMDescription(AsebaVMState *vm) {
  return &vmDescription;
}

extern AsebaLocalEventDescription localEvents[];
const AsebaLocalEventDescription *AsebaGetLocalEventsDescriptions(AsebaVMState *vm) {
  return localEvents;
}

extern AsebaNativeFunctionDescription *nativeFunctionsDescriptions[];
const AsebaNativeFunctionDescription *const *AsebaGetNativeFunctionsDescriptions(AsebaVMState *vm) {
  return nativeFunctionsDescriptions;
}

extern AsebaNativeFunctionPointer nativeFunctions[];
void AsebaNativeFunction(AsebaVMState *vm, uint16 id) {
  nativeFunctions[id](vm);
}

void AsebaWriteBytecode(AsebaVMState *vm) {
}

void AsebaResetIntoBootloader(AsebaVMState *vm) {
}

#ifndef DISABLE_WEAK_CALLBACKS
void AsebaVMRunCB(AsebaVMState *vm) {
}

void AsebaVMErrorCB(AsebaVMState *vm, const char *message) {
}

void AsebaVMResetCB(AsebaVMState *vm) {
}
#endif

void AsebaAssert(AsebaVMState *vm, AsebaAssertReason reason) {
  cerr << "\nFatal error: ";
  switch (vm->nodeId) {
    case 1:
      cerr << "left motor module";
      break;
    case 2:
      cerr << "right motor module";
      break;
    case 3:
      cerr << "proximity sensors module";
      break;
    case 4:
      cerr << "distance sensors module";
      break;
    default:
      cerr << "unknown module";
      break;
  }
  cerr << " has produced exception: ";
  switch (reason) {
    case ASEBA_ASSERT_UNKNOWN:
      cerr << "undefined";
      break;
    case ASEBA_ASSERT_UNKNOWN_UNARY_OPERATOR:
      cerr << "unknown unary operator";
      break;
    case ASEBA_ASSERT_UNKNOWN_BINARY_OPERATOR:
      cerr << "unknown binary operator";
      break;
    case ASEBA_ASSERT_UNKNOWN_BYTECODE:
      cerr << "unknown bytecode";
      break;
    case ASEBA_ASSERT_STACK_OVERFLOW:
      cerr << "stack overflow";
      break;
    case ASEBA_ASSERT_STACK_UNDERFLOW:
      cerr << "stack underflow";
      break;
    case ASEBA_ASSERT_OUT_OF_VARIABLES_BOUNDS:
      cerr << "out of variables bounds";
      break;
    case ASEBA_ASSERT_OUT_OF_BYTECODE_BOUNDS:
      cerr << "out of bytecode bounds";
      break;
    case ASEBA_ASSERT_STEP_OUT_OF_RUN:
      cerr << "step out of run";
      break;
    case ASEBA_ASSERT_BREAKPOINT_OUT_OF_BYTECODE_BOUNDS:
      cerr << "breakpoint out of bytecode bounds";
      break;
    case ASEBA_ASSERT_EMIT_BUFFER_TOO_LONG:
      cerr << "tried to emit a buffer too long";
      break;
    default:
      cerr << "unknown exception";
      break;
  }
  cerr << ".\npc = " << vm->pc << ", sp = " << vm->sp;
  cerr << "\nResetting VM" << std::endl;
  AsebaVMInit(vm);
}
}
