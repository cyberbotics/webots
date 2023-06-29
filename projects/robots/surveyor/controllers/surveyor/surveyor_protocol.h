/*
 * Copyright 1996-2023 Cyberbotics Ltd.
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
 * Description:  Main header file of the interface with the Surveyor.
 */

#ifndef SURVEYOR_H
#define SURVEYOR_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * surveyor_init must be called once, before using the robot to ensure the
 * good behaviour of the simulation.
 *
 * surveyor_send is the main interaction function for the Surveyor robot.
 *
 * It takes, as argument, first a command for the Surveyor from its own
 * protocol and secondly the buffer to return the answer.
 *
 * For more precise informations about these functions, please have a look
 * at surveyor.c
 */
void surveyor_init(void);
void surveyor_send(unsigned char *, unsigned char *);
void surveyor_update(int);

#ifdef __cplusplus
}
#endif
#endif
