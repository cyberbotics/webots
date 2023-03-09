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

#ifndef WB_ODE_DEBUGGER_HPP
#define WB_ODE_DEBUGGER_HPP

#include <ode/ode.h>

class WbOdeDebugger {
public:
  WbOdeDebugger();
  ~WbOdeDebugger();

  static WbOdeDebugger *instance() { return cOdeDebugger; }

  // cppcheck-suppress functionStatic
  void toggleDebugging(bool);

  // cppcheck-suppress functionStatic
  void step();

private:
  static WbOdeDebugger *cOdeDebugger;

  // dWorldID odeWorld;
  // dSpaceID odeSpace;

  // TODO_WREN
  // Ogre::SceneNode* odeDebugNode;

  // Ogre::ManualObject* makeClusterCell(const Ogre::String name, WbSFVector3 *_position, WbSFVector3 *_size, WbSFVector3
  // *_color);
  // void cleanupExtraManualObjects(int k);
};

#endif
