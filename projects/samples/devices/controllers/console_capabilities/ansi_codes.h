/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ANSI_CODES_H
#define ANSI_CODES_H

#include <stdio.h>

#define ANSI_STYLE_RESET "\x1b[0m"

#define ANSI_BOLD_STYLE "\x1b[1m"
#define ANSI_UNDERLINE_STYLE "\x1b[4m"

#define ANSI_COLOR_BG_BLACK "\x1b[40m"
#define ANSI_COLOR_BG_RED "\x1b[41m"
#define ANSI_COLOR_BG_GREEN "\x1b[42m"
#define ANSI_COLOR_BG_YELLOW "\x1b[43m"
#define ANSI_COLOR_BG_BLUE "\x1b[44m"
#define ANSI_COLOR_BG_MAGENTA "\x1b[45m"
#define ANSI_COLOR_BG_CYAN "\x1b[46m"
#define ANSI_COLOR_BG_WHITE "\x1b[47m"

#define ANSI_COLOR_FG_BLACK "\x1b[30m"
#define ANSI_COLOR_FG_RED "\x1b[31m"
#define ANSI_COLOR_FG_GREEN "\x1b[32m"
#define ANSI_COLOR_FG_YELLOW "\x1b[33m"
#define ANSI_COLOR_FG_BLUE "\x1b[34m"
#define ANSI_COLOR_FG_MAGENTA "\x1b[35m"
#define ANSI_COLOR_FG_CYAN "\x1b[36m"
#define ANSI_COLOR_FG_WHITE "\x1b[37m"

#define ANSI_CLEAR_SCREEN "\x1b[2J"

// Convenient macros
#define PRINT_IN_BLACK(x) puts(ANSI_COLOR_FG_BLACK x ANSI_STYLE_RESET)
#define PRINT_IN_RED(x) puts(ANSI_COLOR_FG_RED x ANSI_STYLE_RESET)
#define PRINT_IN_GREEN(x) puts(ANSI_COLOR_FG_GREEN x ANSI_STYLE_RESET)
#define PRINT_IN_YELLOW(x) puts(ANSI_COLOR_FG_YELLOW x ANSI_STYLE_RESET)
#define PRINT_IN_BLUE(x) puts(ANSI_COLOR_FG_BLUE x ANSI_STYLE_RESET)
#define PRINT_IN_MAGENTA(x) puts(ANSI_COLOR_FG_MAGENTA x ANSI_STYLE_RESET)
#define PRINT_IN_CYAN(x) puts(ANSI_COLOR_FG_CYAN x ANSI_STYLE_RESET)
#define PRINT_IN_WHITE(x) puts(ANSI_COLOR_FG_WHITE x ANSI_STYLE_RESET)

#define CONSOLE_CLEAR puts(ANSI_CLEAR_SCREEN)

#endif
