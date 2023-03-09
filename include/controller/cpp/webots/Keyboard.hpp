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

#ifndef KEYBOARD_HPP
#define KEYBOARD_HPP

namespace webots {
  class Keyboard {
  public:
    enum {
      END = 312,
      HOME,
      LEFT,
      UP,
      RIGHT,
      DOWN,
      PAGEUP = 366,
      PAGEDOWN,
      NUMPAD_HOME = 375,
      NUMPAD_LEFT,
      NUMPAD_UP,
      NUMPAD_RIGHT,
      NUMPAD_DOWN,
      NUMPAD_END = 382,
      KEY = 0x0000ffff,
      SHIFT = 0x00010000,
      CONTROL = 0x00020000,
      ALT = 0x00040000
    };

    Keyboard() {}  // Use Robot::getKeyboard() instead
    virtual ~Keyboard() {}
    virtual void enable(int samplingPeriod);
    virtual void disable();
    int getSamplingPeriod() const;
    int getKey() const;
  };
}  // namespace webots

#endif  // KEYBOARD_HPP
