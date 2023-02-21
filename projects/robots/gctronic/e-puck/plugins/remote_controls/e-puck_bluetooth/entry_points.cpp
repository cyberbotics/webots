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

#include "entry_points.hpp"
#include "Wrapper.hpp"

bool wbr_init(WbrInterface *ri) {
  Wrapper::init();

  ri->mandatory.wbr_start = Wrapper::start;
  ri->mandatory.wbr_stop = Wrapper::stop;
  ri->mandatory.wbr_has_failed = Wrapper::hasFailed;
  ri->mandatory.wbr_robot_step = Wrapper::robotStep;
  ri->mandatory.wbr_stop_actuators = Wrapper::stopActuators;

  ri->wbr_custom_function = Wrapper::callCustomFunction;

  ri->wbr_set_sampling_period = Wrapper::setSamplingPeriod;
  ri->wbr_led_set = Wrapper::ledSet;
  ri->wbr_motor_set_velocity = Wrapper::motorSetVelocity;
  ri->wbr_motor_set_torque_sampling_period = Wrapper::motorSetTorqueSamplingPeriod;

  return true;
}

void wbr_cleanup() {
  Wrapper::cleanup();
}
