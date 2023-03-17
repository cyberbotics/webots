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

/**********************************************************************************/
/* Description:  Webots C programming interface for the Speaker node              */
/**********************************************************************************/

#ifndef WB_SPEAKER_H
#define WB_SPEAKER_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void wb_speaker_play_sound(WbDeviceTag left, WbDeviceTag right, const char *sound, double volume, double pitch, double balance,
                           bool loop);
void wb_speaker_stop(WbDeviceTag tag, const char *sound);
bool wb_speaker_is_sound_playing(WbDeviceTag tag, const char *sound);

// "pico" or "microsoft" (Windows only)
bool wb_speaker_set_engine(WbDeviceTag tag, const char *engine);
// "en-US", "en-UK", "de-DE", "es-ES", "fr-FR", "it-IT", etc.
bool wb_speaker_set_language(WbDeviceTag tag, const char *language);
const char *wb_speaker_get_engine(WbDeviceTag tag);
const char *wb_speaker_get_language(WbDeviceTag tag);
void wb_speaker_speak(WbDeviceTag tag, const char *text, double volume);
bool wb_speaker_is_speaking(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_SPEAKER_H */
