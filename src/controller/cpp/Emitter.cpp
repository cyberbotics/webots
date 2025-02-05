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
#include <webots/emitter.h>
#include <webots/Emitter.hpp>

using namespace webots;

int Emitter::send(const void *data, int size) {
  return wb_emitter_send(getTag(), data, size);
}

int Emitter::getBufferSize() const {
  return wb_emitter_get_buffer_size(getTag());
}

void Emitter::setChannel(int channel) {
  wb_emitter_set_channel(getTag(), channel);
}

int Emitter::getChannel() const {
  return wb_emitter_get_channel(getTag());
}

double Emitter::getRange() const {
  return wb_emitter_get_range(getTag());
}

void Emitter::setRange(double range) {
  wb_emitter_set_range(getTag(), range);
}
