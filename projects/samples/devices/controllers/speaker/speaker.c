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
 * Description:  An example of use of a speaker device.
 */

#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/speaker.h>

#include <stdio.h>

#define TIME_STEP 64

int main() {
  WbDeviceTag speaker;

  wb_robot_init();

  /* get the speaker */
  speaker = wb_robot_get_device("speaker");

  /* do a circle around the center of the arena */
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 8);
  wb_motor_set_velocity(right_motor, 10);

  /* use the speaker to play a sound file */
  wb_speaker_play_sound(speaker, speaker, "sounds/robot_sound.wav", 1.0, 1.0, 0.0, true);
  printf("I am going to use the speaker to play a sound file while I am moving in circle around the center of the arena.\n");
  printf("Please make sure the sound is enabled and do not move the viewpoint.\n");

  /* control loop */
  double offset = wb_robot_get_time();
  while (wb_robot_step(TIME_STEP) != -1) {
    // every 5 seconds play a second sound file using the same speaker
    if (wb_robot_get_time() - offset > 5.0) {
      wb_speaker_play_sound(speaker, speaker, "sounds/robot_bip.wav", 0.7, 1.0, 0.0, false);
      offset = wb_robot_get_time();
    }
  }

  wb_robot_cleanup();

  return 0;
}
