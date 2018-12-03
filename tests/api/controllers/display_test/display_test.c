/*
 * Description:  This world tests controller reset on robots with display
                 devices.
 */

#include <webots/differential_wheels.h>
#include <webots/display.h>
#include <webots/robot.h>

#include <stdlib.h>
#include <time.h>

#define SPEED 40
#define TIME_STEP 64

#define EMOTICON_WIDTH 14
#define EMOTICON_HEIGHT 14
#define EMOTICONS_NUMBER_X 5
#define EMOTICONS_NUMBER_Y 11

int main() {
  wb_robot_init();

  int counter = 0;
  srand(time(NULL));
  WbDeviceTag emoticon_display = wb_robot_get_device("emoticon_display");
  WbImageRef emoticonsImage =
    wb_display_image_load(emoticon_display, "../../../../projects/samples/devices/controllers/display/emoticons.png");

  wb_differential_wheels_set_speed(SPEED, -SPEED);

  while (wb_robot_step(TIME_STEP) != -1) {
    counter++;

    if (counter % 30 == 1) {
      int x = -EMOTICON_WIDTH * (rand() % EMOTICONS_NUMBER_X);
      int y = -EMOTICON_HEIGHT * (rand() % EMOTICONS_NUMBER_Y);
      wb_display_image_paste(emoticon_display, emoticonsImage, x, y, true);
    }
  }

  wb_display_image_delete(emoticon_display, emoticonsImage);
  wb_robot_cleanup();

  return 0;
}
