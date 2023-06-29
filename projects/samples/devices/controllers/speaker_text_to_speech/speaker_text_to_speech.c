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
 * Description:  An example of use of a speaker device to perform text-to-speech.
 */

#include <webots/robot.h>
#include <webots/speaker.h>

#define TIME_STEP 64

int main() {
  WbDeviceTag speaker;

  wb_robot_init();

  /* get the speaker */
  speaker = wb_robot_get_device("speaker");

#ifdef _WIN32
  wb_speaker_set_engine(speaker, "microsoft");
#endif

  wb_speaker_set_language(speaker, "en-US");

  /* use the speaker to perform text-to-speech */
  wb_speaker_speak(
    speaker,
    "Hello! Using the text-to-speech of the Speaker device, I can speak 6 different languages: "
    "English with US or UK accent, German, Spanish, French and Italian. "
    "Using tags I can modulate my speech, like for example change <prosody pitch=\"+16.8st\">the pitch of my voice</prosody>, "
    "<prosody pitch=\"-15st\">and speak with a very low pitch</prosody>. "
    "<prosody rate=\"0.5\">And I can change the speed</prosody><prosody rate=\"1.5\">at which I speak</prosody>. "
    "I can also <prosody volume=\"20\">adjust the volume of my voice</prosody>. "
    "Last but not least, I can imitate animals: <audio src=\"sounds/cow.wav\">Meuh</audio>",
    1.0);

  /* empty control loop */
  while (wb_robot_step(TIME_STEP) != -1) {
  }

  wb_robot_cleanup();

  return 0;
}
