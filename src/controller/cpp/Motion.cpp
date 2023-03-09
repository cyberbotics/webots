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

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/utils/motion.h>
#include <webots/utils/Motion.hpp>

using namespace std;
using namespace webots;

Motion::~Motion() {
  wbu_motion_delete(motionRef);
}

Motion::Motion(const string &fileName) {
  motionRef = wbu_motion_new(fileName.c_str());
}

bool Motion::isValid() const {
  return motionRef != NULL;
}

void Motion::play() {
  wbu_motion_play(motionRef);
}

void Motion::stop() {
  wbu_motion_stop(motionRef);
}

int Motion::getDuration() const {
  return wbu_motion_get_duration(motionRef);
}

int Motion::getTime() const {
  return wbu_motion_get_time(motionRef);
}

void Motion::setTime(int time) {
  wbu_motion_set_time(motionRef, time);
}

void Motion::setReverse(bool reverse) {
  wbu_motion_set_reverse(motionRef, reverse);
}

void Motion::setLoop(bool loop) {
  wbu_motion_set_loop(motionRef, loop);
}

bool Motion::isOver() const {
  return wbu_motion_is_over(motionRef);
}
