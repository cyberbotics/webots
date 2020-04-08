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

// This file is a dirty hack allowing swig to generate correct constant strings for both Python and Java APIs.

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
  const std::string AnsiCodes::RESET = "\x1b[0m";
  const std::string AnsiCodes::BOLD = "\x1b[1m";
  const std::string AnsiCodes::UNDERLINE = "\x1b[4m";

  const std::string AnsiCodes::BLACK_FOREGROUND = "\x1b[30m";
  const std::string AnsiCodes::RED_FOREGROUND = "\x1b[31m";
  const std::string AnsiCodes::GREEN_FOREGROUND = "\x1b[32m";
  const std::string AnsiCodes::YELLOW_FOREGROUND = "\x1b[33m";
  const std::string AnsiCodes::BLUE_FOREGROUND = "\x1b[34m";
  const std::string AnsiCodes::MAGENTA_FOREGROUND = "\x1b[35m";
  const std::string AnsiCodes::CYAN_FOREGROUND = "\x1b[36m";
  const std::string AnsiCodes::WHITE_FOREGROUND = "\x1b[37m";

  const std::string AnsiCodes::BLACK_BACKGROUND = "\x1b[40m";
  const std::string AnsiCodes::RED_BACKGROUND = "\x1b[41m";
  const std::string AnsiCodes::GREEN_BACKGROUND = "\x1b[42m";
  const std::string AnsiCodes::YELLOW_BACKGROUND = "\x1b[43m";
  const std::string AnsiCodes::BLUE_BACKGROUND = "\x1b[44m";
  const std::string AnsiCodes::MAGENTA_BACKGROUND = "\x1b[45m";
  const std::string AnsiCodes::CYAN_BACKGROUND = "\x1b[46m";
  const std::string AnsiCodes::WHITE_BACKGROUND = "\x1b[47m";

  const std::string AnsiCodes::CLEAR_SCREEN = "\x1b[2J";
}  // namespace webots
#endif /* ANSICODES_HPP */
