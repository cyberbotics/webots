// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************************************/
/* Description:  Wrapper of the Speaker Webots API for the ROBOTIS OP2 real robot                        */
/*******************************************************************************************************/

#ifndef SPEAKER_HPP
#define SPEAKER_HPP

#include <webots/Device.hpp>

#include <sys/wait.h>
#include <unistd.h>

namespace webots {
  class Speaker : public Device {
  public:
    Speaker(const std::string &name);  // Use Robot::getSpeaker() instead
    virtual ~Speaker();
    static void playSound(Speaker *left, Speaker *right, const std::string &sound, double volume, double pitch, double balance,
                          bool loop);
    void stop(const std::string &sound);
    void setLanguage(const std::string &language);
    std::string getLanguage() { return mLanguage; }
    void speak(const std::string &text, double volume);

    // kept only for backward compatibility should not be used (since Webots 8.5)
    virtual void playFile(const char *filename) __attribute__((deprecated));
    virtual void playFileWait(const char *filename) __attribute__((deprecated));
    virtual void speak(const char *text, const char *voice = "en", int speed = 175) __attribute__((deprecated));
    virtual void speakFile(const char *filename, const char *voice = "en", int speed = 175) __attribute__((deprecated));

  private:
    pid_t mSpeakPID;
    std::string mLanguage;
  };
}  // namespace webots

#endif  // SPEAKER_HPP
