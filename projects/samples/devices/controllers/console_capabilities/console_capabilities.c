#include <stdio.h>
#include <unistd.h>
#include <webots/robot.h>

#include "ansi_codes.h"

#define TIME_STEP 64

int main(int argc, char **argv)
{
  wb_robot_init();

  PRINT_IN_RED("Red text!");
  PRINT_IN_GREEN("Green text!");
  PRINT_IN_YELLOW("Yellow text!");
  PRINT_IN_CYAN("Cyan text!");
  PRINT_IN_BLUE("Blue text!");
  PRINT_IN_MAGENTA("Magenta text!");
  PRINT_IN_WHITE("White text!");
  printf("\n");

  printf("%sRed background only%s\n", ANSI_COLOR_BG_RED, ANSI_COLOR_RESET);
  printf("%sGreen background only%s\n", ANSI_COLOR_BG_GREEN, ANSI_COLOR_RESET);
  printf("%sYellow background only%s\n", ANSI_COLOR_BG_YELLOW, ANSI_COLOR_RESET);
  printf("%sCyan background only%s\n", ANSI_COLOR_BG_CYAN, ANSI_COLOR_RESET);
  printf("%sBlue background only%s\n", ANSI_COLOR_BG_BLUE, ANSI_COLOR_RESET);
  printf("%sMangenta background only%s\n", ANSI_COLOR_BG_MAGENTA, ANSI_COLOR_RESET);
  printf("%sWhite background only%s\n\n", ANSI_COLOR_BG_WHITE, ANSI_COLOR_RESET);

  printf("%sBlue background only specified text!%s\n\n", ANSI_COLOR_BG_BLUE, ANSI_COLOR_RESET);

  printf("%s%sRed background on Blue text%s\n", ANSI_COLOR_BG_RED, ANSI_COLOR_FG_BLUE, ANSI_COLOR_RESET);
  printf("%s%sGreen background on Yellow%s\n", ANSI_COLOR_BG_GREEN, ANSI_COLOR_FG_YELLOW, ANSI_COLOR_RESET);
  printf("%s%sYellow background on Magenta text%s\n", ANSI_COLOR_BG_YELLOW, ANSI_COLOR_FG_MAGENTA, ANSI_COLOR_RESET);
  printf("%s%sCyan background on Red text%s\n", ANSI_COLOR_BG_CYAN, ANSI_COLOR_FG_RED, ANSI_COLOR_RESET);
  printf("%s%sBlue background on White text%s\n", ANSI_COLOR_BG_BLUE, ANSI_COLOR_FG_WHITE, ANSI_COLOR_RESET);
  printf("%s%sMangenta background on Green text%s\n", ANSI_COLOR_BG_MAGENTA, ANSI_COLOR_FG_GREEN, ANSI_COLOR_RESET);
  printf("%s%sWhite background on Black text%s\n", ANSI_COLOR_BG_WHITE, ANSI_COLOR_FG_BLACK, ANSI_COLOR_RESET);

  wb_robot_cleanup();
  return 0;
}

