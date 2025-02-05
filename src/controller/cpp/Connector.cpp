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
#include <webots/connector.h>
#include <webots/Connector.hpp>

using namespace webots;

void Connector::enablePresence(int sampling_period) {
  wb_connector_enable_presence(getTag(), sampling_period);
}

void Connector::disablePresence() {
  wb_connector_disable_presence(getTag());
}

int Connector::getPresenceSamplingPeriod() const {
  return wb_connector_get_presence_sampling_period(getTag());
}

int Connector::getPresence() const {
  return wb_connector_get_presence(getTag());
}

bool Connector::isLocked() const {
  return wb_connector_is_locked(getTag());
}

void Connector::lock() {
  wb_connector_lock(getTag());
}

void Connector::unlock() {
  wb_connector_unlock(getTag());
}
