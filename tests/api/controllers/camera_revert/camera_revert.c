// Description: Test if camera images are the same after reverting and running the same simulation.
//              This test saves 3 camera images (corresponding to a green, red and blue blob) and compares the images obtained
//              on the first run to the images obtained after a revert. If the images differ, the test fails.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/utils/system.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define SPEED 5
#define TIME_STEP 64
enum BLOB_TYPE { RED, GREEN, BLUE, NONE };

int main(int argc, char *argv[]) {
  WbDeviceTag camera, left_motor, right_motor;
  int width, height;
  int pause_counter = 0;
  int left_speed, right_speed;
  int i, j;
  int red, blue, green;
  const unsigned char *image;
  const char *color_names[3] = {"red", "green", "blue"};
  char *filenames[3][2];
  enum BLOB_TYPE current_blob;
  filenames[0][0] = "red_blob_0.png";
  filenames[1][0] = "green_blob_0.png";
  filenames[2][0] = "blue_blob_0.png";
  filenames[0][1] = "red_blob_1.png";
  filenames[1][1] = "green_blob_1.png";
  filenames[2][1] = "blue_blob_1.png";

  // cleanup old image files if any
  if (access(filenames[1][1], F_OK) == 0) {  // the green blob is the first found
    for (i = 0; i < 3; i++) {                // delete all the image files
      for (j = 0; j < 2; j++) {
        unlink(filenames[i][0]);
        unlink(filenames[i][1]);
      }
    }
  }

  int run_count = 1 + access(filenames[0][0], F_OK);

  ts_setup(argv[0]);
  // Get the camera device, enable it, and store its width and height
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);

  left_motor = wb_robot_get_device("left motor");
  right_motor = wb_robot_get_device("right motor");

  while (wb_robot_step(TIME_STEP) != -1) {
    image = wb_camera_get_image(camera);
    if (pause_counter > 0)
      pause_counter--;
    // Case 1
    // A blob has been found recently
    // The robot wait in front of him until pause_counter
    // is decremented enough
    if (pause_counter > 10) {
      left_speed = 0;
      right_speed = 0;
    }
    // Case 2
    // A blob has been found quite recently
    // The robot begins to turn but don't analyse the image for a while,
    // otherwise the same blob would be found again
    else if (pause_counter > 0) {
      left_speed = -SPEED;
      right_speed = SPEED;
    }
    // Case 3
    // The robot turns and analyse the camera image in order
    // to find a new blob
    else {  // pause_counter == 0
      // Reset the sums
      red = 0;
      green = 0;
      blue = 0;
      // Here we analyse the image from the camera. The goal is to detect a
      // blob (a spot of color) of a defined color in the middle of our
      // screen.
      // In order to achieve that we simply parse the image pixels of the
      // center of the image, and sum the color components individually
      for (i = width / 3; i < 2 * width / 3; i++) {
        for (j = height / 2; j < 3 * height / 4; j++) {
          red += wb_camera_image_get_red(image, width, i, j);
          blue += wb_camera_image_get_blue(image, width, i, j);
          green += wb_camera_image_get_green(image, width, i, j);
        }
      }
      // If a component is much more represented than the other ones, a blob is detected
      if ((red > 3 * green) && (red > 3 * blue))
        current_blob = RED;
      else if ((green > 3 * red) && (green > 3 * blue))
        current_blob = GREEN;
      else if ((blue > 3 * red) && (blue > 3 * green))
        current_blob = BLUE;
      else
        current_blob = NONE;
      // Case 3a
      // No blob is detected
      // the robot continues to turn
      if (current_blob == NONE) {
        left_speed = -SPEED;
        right_speed = SPEED;
      }
      // Case 3b: a blob is detected
      // the robot stops, stores the image, and changes its state
      else {
        left_speed = 0;
        right_speed = 0;
        printf("Looks like I found a %s blob.\n", color_names[current_blob]);
        wb_camera_save_image(camera, filenames[current_blob][run_count], 100);
        if (current_blob == BLUE) {  // the blue blob is the last one found by the robot
          if (run_count == 0)
            wb_supervisor_world_reload();
          else {  // compare the sizes of the image files
            int size[3];
            int mismatch = 0;
            for (i = 0; i < 3; i++) {
              FILE *f = fopen(filenames[i][0], "r");
              fseek(f, 0L, SEEK_END);
              size[i] = ftell(f);
              fclose(f);
              f = fopen(filenames[i][1], "r");
              fseek(f, 0L, SEEK_END);
              mismatch = ftell(f) - size[i];
              fclose(f);
              if (mismatch != 0)
                break;
            }
            ts_assert_int_equal(mismatch, 0, "File sizes differ for '%s' (%d bytes) and '%s' (%d bytes).", filenames[i][0],
                                size[i], filenames[i][1], size[i] + mismatch);
            int difference = 0;
            if (mismatch == 0) {  // if sizes match, check image files contents
              for (i = 0; i < 3; i++) {
                char *buffer[2];
                FILE *f[2];
                for (j = 0; j < 2; j++) {
                  buffer[j] = malloc(size[i]);
                  f[j] = fopen(filenames[i][j], "rb");
                  ts_assert_int_equal(fread(buffer[j], sizeof(char), size[i], f[j]), size[i], "Cannot read '%s' image file.",
                                      filenames[i][j]);
                }
                for (j = 0; j < size[i]; j++) {
                  if (buffer[0][j] != buffer[1][j]) {
                    difference = i;
                    break;
                  }
                }
                for (j = 0; j < 2; j++) {
                  free(buffer[j]);
                  fclose(f[j]);
                }
                if (difference != 0)
                  break;
              }
            }
            ts_assert_int_equal(difference, 0, "Files differ for %s and %s", filenames[difference][0],
                                filenames[difference][1]);
            // delete all the image files
            for (i = 0; i < 3; i++) {
              for (j = 0; j < 2; j++) {
                unlink(filenames[i][0]);
                unlink(filenames[i][1]);
              }
            }
            ts_send_success();
            return EXIT_SUCCESS;
          }
        }
        pause_counter = 20;
      }
    }
    // set the motor speeds.
    double position = left_speed > 0 ? INFINITY : -INFINITY;
    wb_motor_set_position(left_motor, position);
    position = right_speed > 0 ? INFINITY : -INFINITY;
    wb_motor_set_position(right_motor, position);
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }
  wb_robot_cleanup();
  return 0;
}
