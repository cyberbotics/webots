/*
 * Copyright 1996-2025 Cyberbotics Ltd.
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

#ifndef PICO_TTS_WRAPPER_H
#define PICO_TTS_WRAPPER_H

// Rename C keyword 'this' to avoid conflicts
#define this pico_this

#ifdef __cplusplus
extern "C" {
#endif

#include <picoapi.h>
#include <picoapid.h>
#include <picoos.h>

#ifdef __cplusplus
}
#endif

#undef this

#endif  // PICO_TTS_WRAPPER_H
