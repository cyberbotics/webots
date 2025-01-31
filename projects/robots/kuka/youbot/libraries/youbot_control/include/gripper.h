/*
 * Copyright 1996-2024 Cyberbotics Ltd.
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
 * Description:   Allows to handle the gipper
 */

#ifndef GRIPPER_H
#define GRIPPER_H

#ifdef __cplusplus
extern "C" {
#endif

void gripper_init();

void gripper_grip();  // dangerous to grip an object with this function -> creates a lot of internal constraints
void gripper_release();
void gripper_set_gap(double gap);

#ifdef __cplusplus
}
#endif

#endif
