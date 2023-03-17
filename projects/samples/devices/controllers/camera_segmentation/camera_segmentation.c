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

/*
 * Description:  An example of use of a camera device with recognition segmentation capability.
 */

#include <webots/camera.h>
#include <webots/display.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define SPEED 1.5
#define TIME_STEP 64

int main() {
  WbDeviceTag camera, display, left_motor, right_motor;
  WbImageRef segmented_image;

  wb_robot_init();

  /* Get the camera device, enable the camera, the recognition and the segmentation functionalities */
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_recognition_enable(camera, TIME_STEP);
  wb_camera_recognition_enable_segmentation(camera);
  const int width = wb_camera_get_width(camera);
  const int height = wb_camera_get_height(camera);

  /* Get the display device */
  display = wb_robot_get_device("segmented image display");

  /* Get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  /* Set the motors speed */
  wb_motor_set_velocity(left_motor, -SPEED);
  wb_motor_set_velocity(right_motor, SPEED);

  /* Main loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    if (wb_camera_recognition_is_segmentation_enabled(camera) && wb_camera_recognition_get_sampling_period(camera) > 0) {
      /* Get the segmented image and display it in the Display */
      const unsigned char *data = wb_camera_recognition_get_segmentation_image(camera);
      if (data) {
        segmented_image = wb_display_image_new(display, width, height, data, WB_IMAGE_BGRA);
        wb_display_image_paste(display, segmented_image, 0, 0, false);
        wb_display_image_delete(display, segmented_image);
      }
    }
  }

  wb_robot_cleanup();

  return 0;
}
