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

#ifndef JOYSTICK_INTERFACE__HPP
#define JOYSTICK_INTERFACE__HPP

#include <map>

namespace webots {
  class Driver;
  class Joystick;
}  // namespace webots

class JoystickInterface {
public:
  explicit JoystickInterface(webots::Driver *driver);
  JoystickInterface(webots::Driver *driver, const char *configFile);
  virtual ~JoystickInterface() {}

  bool step();

private:
  static void fatal(const std::string &txt);
  static bool fileExists(const std::string &name);
  static double convertFeedback(int raw, int minimum, int maximum);

  void init(webots::Driver *driver, const char *configFile);

  int mGear;
  webots::Driver::WiperMode mWiperMode;

  webots::Driver *mDriver;
  webots::Joystick *mJoystick;

  std::map<const std::string, int> mAxesMap;
  std::map<const std::string, int> mAxesBoundsMap;
  std::map<const std::string, int> mButtonsMap;
  std::map<const std::string, double> mGainMap;
};

#endif  // JOYSTICK_INTERFACE__HPP
