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

// Description:   Helper class allowing to retrieve directories

#ifndef ROBOTISOP2_DIRECTORY_MANAGER_HPP
#define ROBOTISOP2_DIRECTORY_MANAGER_HPP

#include <string>

namespace managers {
  class RobotisOp2DirectoryManager {
  public:
    static const std::string &getDataDirectory();
  };
}  // namespace managers

#endif
