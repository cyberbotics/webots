// Copyright 1996-2020 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Description:  Defines the entry point of the robot window library
 */

#ifndef ENTRY_POINTS_HPP
#define ENTRY_POINTS_HPP

extern "C" {
bool wbw_init();
void wbw_cleanup();
void wbw_pre_update_gui();
void wbw_update_gui();
void wbw_read_sensors();
void wbw_write_actuators();
void wbw_show();
}

#endif
