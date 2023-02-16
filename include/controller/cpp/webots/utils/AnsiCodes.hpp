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

#ifndef ANSICODES_HPP
#define ANSICODES_HPP

#include <string>

namespace webots {
  namespace AnsiCodes {
    static const std::string RESET = "\x1b[0m";
    static const std::string BOLD = "\x1b[1m";
    static const std::string UNDERLINE = "\x1b[4m";

    static const std::string BLACK_FOREGROUND = "\x1b[30m";
    static const std::string RED_FOREGROUND = "\x1b[31m";
    static const std::string GREEN_FOREGROUND = "\x1b[32m";
    static const std::string YELLOW_FOREGROUND = "\x1b[33m";
    static const std::string BLUE_FOREGROUND = "\x1b[34m";
    static const std::string MAGENTA_FOREGROUND = "\x1b[35m";
    static const std::string CYAN_FOREGROUND = "\x1b[36m";
    static const std::string WHITE_FOREGROUND = "\x1b[37m";

    static const std::string BLACK_BACKGROUND = "\x1b[40m";
    static const std::string RED_BACKGROUND = "\x1b[41m";
    static const std::string GREEN_BACKGROUND = "\x1b[42m";
    static const std::string YELLOW_BACKGROUND = "\x1b[43m";
    static const std::string BLUE_BACKGROUND = "\x1b[44m";
    static const std::string MAGENTA_BACKGROUND = "\x1b[45m";
    static const std::string CYAN_BACKGROUND = "\x1b[46m";
    static const std::string WHITE_BACKGROUND = "\x1b[47m";

    static const std::string CLEAR_SCREEN = "\x1b[2J";
  }  // namespace AnsiCodes
}  // namespace webots
#endif /* ANSICODES_HPP */
