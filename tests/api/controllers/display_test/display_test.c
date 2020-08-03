/*
 * Description:  This world tests controller reset on robots with display
                 devices.
 */

#include <webots/display.h>
#include <webots/motor.h>
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

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, SPEED);
  wb_motor_set_velocity(right_motor, -SPEED);

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
