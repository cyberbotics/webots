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
 * Description:  Class used to read keyboard input with X11
 * Author:       david.mansolino@epfl.ch
 */

#ifndef KEYBOARD_INTERFACE_HPP
#define KEYBOARD_INTERFACE_HPP

#define NKEYS 200

#define PRESSED 1
#define PRESSED_AND_RELEASE 2
#define VALIDATED 3
#define VALIDATED_STILL_PRESSED 4

#define WB_KEYBOARD_KEY 0x000ffff
#define WB_KEYBOARD_SHIFT 0x0010000
#define WB_KEYBOARD_CONTROL 0x0020000
#define WB_KEYBOARD_ALT 0x0040000

#define WB_KEYBOARD_LEFT 314
#define WB_KEYBOARD_UP 315
#define WB_KEYBOARD_RIGHT 316
#define WB_KEYBOARD_DOWN 317
#define WB_KEYBOARD_PAGEUP 366
#define WB_KEYBOARD_PAGEDOWN 367
#define WB_KEYBOARD_HOME 313
#define WB_KEYBOARD_END 312

class KeyboardInterface {
public:
  KeyboardInterface();
  virtual ~KeyboardInterface() {}

  static void createWindow();
  static void closeWindow();
  void startListenKeyboard();
  void initialiseKeyPressed();
  void resetKeyPressed();

  int getKeyPressed();

private:
  void setKeyPressed(int key);
  void setKeyReleased(int key);

  int mKeyPressed[NKEYS];
};

#endif
