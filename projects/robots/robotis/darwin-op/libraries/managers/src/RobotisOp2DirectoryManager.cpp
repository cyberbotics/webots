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

#include "RobotisOp2DirectoryManager.hpp"

#include <cstdlib>

using namespace std;
using namespace managers;

extern "C" {
char *wbu_system_getenv(const char *);
}

const string &RobotisOp2DirectoryManager::getDataDirectory() {
#ifdef CROSSCOMPILATION
  static string path = "/robotis/Data/";
#else
  char *WEBOTS_HOME = wbu_system_getenv("WEBOTS_HOME");
  static string path = string(WEBOTS_HOME)
#ifdef __APPLE__
                       + "/Contents"
#endif
                       + "/projects/robots/robotis/darwin-op/libraries/robotis-op2/robotis/Data/";
#endif
  return path;
}
