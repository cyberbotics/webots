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
 * Description:  Header file of code used for the Scan.
 */

#ifndef SURVEYOR_SCAN_H
#define SURVEYOR_SCAN_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * surveyor_init_wander must be called once before any Wandering starts.
 * It takes as an argument the image of the camera of the Surveyor.
 *
 * surveyor_wandering must be called eveytime a movement decision needs to
 * be taken in Wandering mode, namely at every time step.
 * It takes as an input argument the current image of the camera and
 * returns the code of the command which must be executed.
 *
 * surveyor_get_scan_values returns the current values of the Scan using
 * the form defined by the protocol.
 * It takes as an argument the buffer which will be filled by the values.
 * This buffer must be at least 90 char long. If the scan has not been
 * working it will not fill the buffer.
 */
void surveyor_init_wander(const unsigned char *);
unsigned char surveyor_wandering(const unsigned char *);
void surveyor_get_scan_values(unsigned char *);

#ifdef __cplusplus
}
#endif
#endif
