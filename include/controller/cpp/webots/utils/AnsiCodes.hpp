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

#ifndef ANSICODES_HPP
#define ANSICODES_HPP

#include <string>

namespace webots {
  class AnsiCodes {
  public:
    static const std::string RESET;
    static const std::string BOLD;
    static const std::string UNDERLINE;

    static const std::string BLACK_FOREGROUND;
    static const std::string RED_FOREGROUND;
    static const std::string GREEN_FOREGROUND;
    static const std::string YELLOW_FOREGROUND;
    static const std::string BLUE_FOREGROUND;
    static const std::string MAGENTA_FOREGROUND;
    static const std::string CYAN_FOREGROUND;
    static const std::string WHITE_FOREGROUND;

    static const std::string BLACK_BACKGROUND;
    static const std::string RED_BACKGROUND;
    static const std::string GREEN_BACKGROUND;
    static const std::string YELLOW_BACKGROUND;
    static const std::string BLUE_BACKGROUND;
    static const std::string MAGENTA_BACKGROUND;
    static const std::string CYAN_BACKGROUND;
    static const std::string WHITE_BACKGROUND;

    static const std::string CLEAR_SCREEN;
  };
}
#endif /* ANSICODES_HPP */
