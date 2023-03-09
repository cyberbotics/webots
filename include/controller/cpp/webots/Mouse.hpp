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

#ifndef MOUSE_HPP
#define MOUSE_HPP

#include "../../c/webots/mouse_state.h"

namespace webots {
  typedef WbMouseState MouseState;

  class Mouse {
  public:
    Mouse() {}  // Use Robot::getMouse() instead
    virtual ~Mouse() {}
    virtual void enable(int samplingPeriod);
    virtual void disable();
    int getSamplingPeriod() const;
    void enable3dPosition();
    void disable3dPosition();
    bool is3dPositionEnabled() const;
    MouseState getState() const;
  };
}  // namespace webots

#endif  // MOUSE_HPP
