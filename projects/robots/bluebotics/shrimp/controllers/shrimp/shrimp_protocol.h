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
 * Description:  Main header file of the interface with the Shrimp.
 */

#ifndef SHRIMP_H
#define SHRIMP_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * shrimp_init must be called once, before using the robot to ensure the
 * good behavior of the simulation.
 *
 * shrimp_send is the main interaction function for the Shrimp robot of
 * Bluebotics.
 * It takes, as argument, a command for the Shrimp from its own protocol
 * (further information about this protocol in 'Controlling the Shrimp')
 * and returns the answer as an array of bytes.
 *
 * For more precise informations about these functions, please have a look
 * at shrimp.c
 */
void shrimp_init(void);
unsigned char *shrimp_send(unsigned char *);

#ifdef __cplusplus
}
#endif
#endif
