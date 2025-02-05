// Copyright 1996-2024 Cyberbotics Ltd.
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

#ifndef SPEAKER_HPP
#define SPEAKER_HPP

#include <webots/Device.hpp>

namespace webots {
  class Speaker : public Device {
  public:
    explicit Speaker(const std::string &name) : Device(name) {}  // Use Robot::getSpeaker() instead
    explicit Speaker(WbDeviceTag tag) : Device(tag) {}
    virtual ~Speaker() {}
    static void playSound(Speaker *left, Speaker *right, const std::string &sound, double volume, double pitch, double balance,
                          bool loop);
    bool isSoundPlaying(const std::string &sound) const;
    void stop(const std::string &sound);
    bool setEngine(const std::string &engine);
    bool setLanguage(const std::string &language);
    std::string getEngine();
    std::string getLanguage();
    void speak(const std::string &text, double volume);
    bool isSpeaking() const;
  };
}  // namespace webots

#endif  // SPEAKER_HPP
