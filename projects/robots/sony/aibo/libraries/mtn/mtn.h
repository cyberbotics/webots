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

#ifndef MTN_H
#define MTN_H

//
// The MTN functions provide a facility for reading and playing back motions occurring simultaneously on several motor devices.
// The file format used for these motions is compatible with the Sony MTN file format used with the Sony Aibo robots.
// A motion file may contain all the information necessary for a walking gait.
//

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _MTN MTN;

// open an MTN motion file specified by the filename parameter.
MTN *mtn_new(const char *filename);

// returns NULL, "file not found", "missing joint: ", ...
const char *mtn_get_error();

// start MTN playback, simultaneously with currently playing MTNs;
// once started, an MTN cannot be stopped
void mtn_play(MTN *mtn);

// set all the MTN motor position for current step
void mtn_step(int ms);

// total length of the movement in milliseconds
int mtn_get_length(MTN *mtn);

// current time (ms) of the execution of the MTN file
int mtn_get_time(MTN *mtn);

// return 1 if the MTN movement has completed, 0 otherwise
int mtn_is_over(MTN *mtn);

// used for debugging
void mtn_fprint(FILE *fd, MTN *mtn);

// delete the resources used by a MTN object
void mtn_delete(MTN *mtn);

#ifdef __cplusplus
}
#endif

#endif
