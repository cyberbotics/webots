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

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/speaker.h>
#include <webots/Speaker.hpp>

using namespace webots;

void Speaker::playSound(Speaker *left, Speaker *right, const std::string &sound, double volume, double pitch, double balance,
                        bool loop) {
  wb_speaker_play_sound(left->getTag(), right->getTag(), sound.c_str(), volume, pitch, balance, loop);
}

bool Speaker::isSoundPlaying(const std::string &sound) const {
  return wb_speaker_is_sound_playing(getTag(), sound.c_str());
}

void Speaker::stop(const std::string &sound) {
  wb_speaker_stop(getTag(), sound.c_str());
}

bool Speaker::setEngine(const std::string &engine) {
  return wb_speaker_set_engine(getTag(), engine.c_str());
}

bool Speaker::setLanguage(const std::string &language) {
  return wb_speaker_set_language(getTag(), language.c_str());
}

std::string Speaker::getEngine() {
  return std::string(wb_speaker_get_engine(getTag()));
}

std::string Speaker::getLanguage() {
  return std::string(wb_speaker_get_language(getTag()));
}

void Speaker::speak(const std::string &text, double volume) {
  wb_speaker_speak(getTag(), text.c_str(), volume);
}

bool Speaker::isSpeaking() const {
  return wb_speaker_is_speaking(getTag());
}
