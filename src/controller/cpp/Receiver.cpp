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
#include <webots/receiver.h>
#include <webots/Receiver.hpp>

using namespace webots;

void Receiver::enable(int sampling_period) {
  wb_receiver_enable(getTag(), sampling_period);
}

void Receiver::disable() {
  wb_receiver_disable(getTag());
}

int Receiver::getSamplingPeriod() const {
  return wb_receiver_get_sampling_period(getTag());
}

void Receiver::setChannel(int channel) {
  wb_receiver_set_channel(getTag(), channel);
}

int Receiver::getChannel() const {
  return wb_receiver_get_channel(getTag());
}

int Receiver::getQueueLength() const {
  return wb_receiver_get_queue_length(getTag());
}

void Receiver::nextPacket() {
  wb_receiver_next_packet(getTag());
}

int Receiver::getDataSize() const {
  return wb_receiver_get_data_size(getTag());
}

const void *Receiver::getData() const {
  return wb_receiver_get_data(getTag());
}

double Receiver::getSignalStrength() const {
  return wb_receiver_get_signal_strength(getTag());
}

const double *Receiver::getEmitterDirection() const {
  return wb_receiver_get_emitter_direction(getTag());
}
