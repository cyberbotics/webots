/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef WBU_ANSI_CODES_H
#define WBU_ANSI_CODES_H

#ifndef __cplusplus
#include <stdio.h>
#endif

#define ANSI_RESET "\x1b[0m"

#define ANSI_BOLD "\x1b[1m"
#define ANSI_UNDERLINE "\x1b[4m"

#define ANSI_BLACK_BACKGROUND "\x1b[40m"
#define ANSI_RED_BACKGROUND "\x1b[41m"
#define ANSI_GREEN_BACKGROUND "\x1b[42m"
#define ANSI_YELLOW_BACKGROUND "\x1b[43m"
#define ANSI_BLUE_BACKGROUND "\x1b[44m"
#define ANSI_MAGENTA_BACKGROUND "\x1b[45m"
#define ANSI_CYAN_BACKGROUND "\x1b[46m"
#define ANSI_WHITE_BACKGROUND "\x1b[47m"

#define ANSI_BLACK_FOREGROUND "\x1b[30m"
#define ANSI_RED_FOREGROUND "\x1b[31m"
#define ANSI_GREEN_FOREGROUND "\x1b[32m"
#define ANSI_YELLOW_FOREGROUND "\x1b[33m"
#define ANSI_BLUE_FOREGROUND "\x1b[34m"
#define ANSI_MAGENTA_FOREGROUND "\x1b[35m"
#define ANSI_CYAN_FOREGROUND "\x1b[36m"
#define ANSI_WHITE_FOREGROUND "\x1b[37m"

#define ANSI_CLEAR_SCREEN "\x1b[2J"

#ifndef __cplusplus
// Convenient macros - Used only in C
#define ANSI_PRINTF_IN_BLACK(x, ...) printf(ANSI_BLACK_FOREGROUND x ANSI_RESET, ##__VA_ARGS__)
#define ANSI_PRINTF_IN_RED(x, ...) printf(ANSI_RED_FOREGROUND x ANSI_RESET, ##__VA_ARGS__)
#define ANSI_PRINTF_IN_GREEN(x, ...) printf(ANSI_GREEN_FOREGROUND x ANSI_RESET, ##__VA_ARGS__)
#define ANSI_PRINTF_IN_YELLOW(x, ...) printf(ANSI_YELLOW_FOREGROUND x ANSI_RESET, ##__VA_ARGS__)
#define ANSI_PRINTF_IN_BLUE(x, ...) printf(ANSI_BLUE_FOREGROUND x ANSI_RESET, ##__VA_ARGS__)
#define ANSI_PRINTF_IN_MAGENTA(x, ...) printf(ANSI_MAGENTA_FOREGROUND x ANSI_RESET, ##__VA_ARGS__)
#define ANSI_PRINTF_IN_CYAN(x, ...) printf(ANSI_CYAN_FOREGROUND x ANSI_RESET, ##__VA_ARGS__)
#define ANSI_PRINTF_IN_WHITE(x, ...) printf(ANSI_WHITE_FOREGROUND x ANSI_RESET, ##__VA_ARGS__)

#define ANSI_CLEAR_CONSOLE() printf(ANSI_CLEAR_SCREEN "\n")
#endif /* __cplusplus */

#endif /* WBU_ANSI_CODES_H */
