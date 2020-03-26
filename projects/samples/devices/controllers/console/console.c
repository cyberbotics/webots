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

#include <webots/robot.h>

#include <webots/utils/ansi_codes.h>

#define TIME_STEP 64

int main(int argc, char **argv) {
  wb_robot_init();

  ANSI_PRINTF_IN_GREEN("This text will not be seen because we are going to clear the screen");
  ANSI_CONSOLE_CLEAR;

  ANSI_PRINTF_IN_RED("Red text!");
  ANSI_PRINTF_IN_GREEN("Green text!");
  ANSI_PRINTF_IN_YELLOW("Yellow text!");
  ANSI_PRINTF_IN_CYAN("Cyan text!");
  ANSI_PRINTF_IN_BLUE("Blue text!");
  ANSI_PRINTF_IN_MAGENTA("Magenta text!");
  ANSI_PRINTF_IN_WHITE("White text!");
  printf("\n");

  printf("%sRed background only%s\n", ANSI_RED_BACKGROUND, ANSI_RESET);
  printf("%sGreen background only%s\n", ANSI_GREEN_BACKGROUND, ANSI_RESET);
  printf("%sYellow background only%s\n", ANSI_YELLOW_BACKGROUND, ANSI_RESET);
  printf("%sCyan background only%s\n", ANSI_CYAN_BACKGROUND, ANSI_RESET);
  printf("%sBlue background only%s\n", ANSI_BLUE_BACKGROUND, ANSI_RESET);
  printf("%sMangenta background only%s\n", ANSI_MAGENTA_BACKGROUND, ANSI_RESET);
  printf("%sWhite background only%s\n\n", ANSI_WHITE_BACKGROUND, ANSI_RESET);

  printf("%sBlue background only specified text!%s\n\n", ANSI_BLUE_BACKGROUND, ANSI_RESET);

  printf("%s%sRed background on Blue text%s\n", ANSI_RED_BACKGROUND, ANSI_BLUE_FOREGROUND, ANSI_RESET);
  printf("%s%sGreen background on Yellow%s\n", ANSI_GREEN_BACKGROUND, ANSI_YELLOW_FOREGROUND, ANSI_RESET);
  printf("%s%sYellow background on Magenta text%s\n", ANSI_YELLOW_BACKGROUND, ANSI_MAGENTA_FOREGROUND, ANSI_RESET);
  printf("%s%sCyan background on Red text%s\n", ANSI_CYAN_BACKGROUND, ANSI_RED_FOREGROUND, ANSI_RESET);
  printf("%s%sBlue background on White text%s\n", ANSI_BLUE_BACKGROUND, ANSI_WHITE_FOREGROUND, ANSI_RESET);
  printf("%s%sMangenta background on Green text%s\n", ANSI_MAGENTA_BACKGROUND, ANSI_GREEN_FOREGROUND, ANSI_RESET);
  printf("%s%sWhite background on Black text%s\n", ANSI_WHITE_BACKGROUND, ANSI_BLACK_FOREGROUND, ANSI_RESET);

  printf("%s%sBold style green text%s\n", ANSI_GREEN_FOREGROUND, ANSI_BOLD, ANSI_RESET);
  printf("%s%sUnderlined style green text%s\n", ANSI_GREEN_FOREGROUND, ANSI_UNDERLINE, ANSI_RESET);

  wb_robot_cleanup();
  return 0;
}
