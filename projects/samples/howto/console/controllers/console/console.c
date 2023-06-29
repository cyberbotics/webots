/*
 * Copyright 1996-2023 Cyberbotics Ltd.
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

#include <webots/robot.h>
#include <webots/utils/ansi_codes.h>

#define TIME_STEP 64

int main(int argc, char **argv) {
  wb_robot_init();

  ANSI_PRINTF_IN_GREEN("This text will disappear in about 4 seconds because we are going to clear the console...\n");
  wb_robot_step(4096);
  ANSI_CLEAR_CONSOLE();

  ANSI_PRINTF_IN_RED("Red text!\n");
  ANSI_PRINTF_IN_GREEN("Green text!\n");
  ANSI_PRINTF_IN_YELLOW("Yellow text!\n");
  ANSI_PRINTF_IN_CYAN("Cyan text!\n");
  ANSI_PRINTF_IN_BLUE("Blue text!\n");
  ANSI_PRINTF_IN_MAGENTA("Magenta text!\n");
  ANSI_PRINTF_IN_WHITE("White text!\n");
  ANSI_PRINTF_IN_BLACK("Black text!\n");

  printf("%sBlue background only specified text%s\n\n", ANSI_BLUE_BACKGROUND, ANSI_RESET);

  ANSI_PRINTF_IN_BLACK("%sRed background only\n", ANSI_RED_BACKGROUND);
  ANSI_PRINTF_IN_BLACK("%sGreen background only\n", ANSI_GREEN_BACKGROUND);
  ANSI_PRINTF_IN_BLACK("%sYellow background only\n", ANSI_YELLOW_BACKGROUND);
  ANSI_PRINTF_IN_BLACK("%sCyan background only\n", ANSI_CYAN_BACKGROUND);
  ANSI_PRINTF_IN_BLACK("%sBlue background only\n", ANSI_BLUE_BACKGROUND);
  ANSI_PRINTF_IN_BLACK("%sMangenta background only\n", ANSI_MAGENTA_BACKGROUND);
  ANSI_PRINTF_IN_BLACK("%sWhite background only\n\n", ANSI_WHITE_BACKGROUND);

  ANSI_PRINTF_IN_BLUE("%sBlue text on Red background\n", ANSI_RED_BACKGROUND);
  ANSI_PRINTF_IN_YELLOW("%sYellow text on Green background\n", ANSI_GREEN_BACKGROUND);
  ANSI_PRINTF_IN_MAGENTA("%sMagenta text on Yellow background\n", ANSI_YELLOW_BACKGROUND);
  ANSI_PRINTF_IN_RED("%sRed text on Cyan background\n", ANSI_CYAN_BACKGROUND);
  ANSI_PRINTF_IN_WHITE("%sWhite text on Blue background\n", ANSI_BLUE_BACKGROUND);
  ANSI_PRINTF_IN_GREEN("%sGreen text on Magenta background\n", ANSI_MAGENTA_BACKGROUND);
  ANSI_PRINTF_IN_BLACK("%sBlack text on White background\n", ANSI_WHITE_BACKGROUND);
  ANSI_PRINTF_IN_WHITE("%sWhite text on Black background\n\n", ANSI_BLACK_BACKGROUND);

  ANSI_PRINTF_IN_GREEN("%sGreen Bold style text\n", ANSI_BOLD);
  ANSI_PRINTF_IN_GREEN("%sGreen Underlined style text\n", ANSI_UNDERLINE);

  wb_robot_cleanup();
  return 0;
}
