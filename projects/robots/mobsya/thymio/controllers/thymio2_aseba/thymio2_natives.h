/*
 * Copyright 1996-2022 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Strongly inspired from https://github.com/aseba-community/aseba-target-thymio2/blob/master/thymio_natives.h

#ifndef THYMIO2_NATIVES_H
#define THYMIO2_NATIVES_H

#include <vm/natives.h>

extern AsebaNativeFunctionDescription AsebaNativeDescription_poweroff;
void power_off(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription__system_reboot;
void AsebaNative__system_reboot(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription__system_settings_read;
void AsebaNative__system_settings_read(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription__system_settings_write;
void AsebaNative__system_settings_write(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription__system_settings_flash;
void AsebaNative__system_settings_flash(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_set_led;
void set_led(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_record;
void sound_record(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_play;
void sound_playback(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_replay;
void sound_replay(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_sound_system;
void sound_system(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_set_led_circle;
void set_led_circle(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_set_led_rgb_top;
void set_rgb_top(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_set_led_rgb_bl;
void set_rgb_bl(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_set_led_rgb_br;
void set_rgb_br(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_play_freq;
void play_freq(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_set_led_buttons;
void set_buttons_leds(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_set_hprox_leds;
void set_hprox_leds(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_set_vprox_leds;
void set_vprox_leds(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_set_rc_leds;
void set_rc_leds(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_set_sound_leds;
void set_sound_leds(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_set_ntc_leds;
void set_ntc_leds(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_set_wave;
void set_wave(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_prox_network;
void prox_network(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_sd_open;
void thymio_native_sd_open(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_sd_write;
void thymio_native_sd_write(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_sd_read;
void thymio_native_sd_read(AsebaVMState *vm);

extern AsebaNativeFunctionDescription AsebaNativeDescription_sd_seek;
void thymio_native_sd_seek(AsebaVMState *vm);

#define THYMIO_NATIVES_DESCRIPTIONS                                                                                          \
  &AsebaNativeDescription_set_led, &AsebaNativeDescription_record, &AsebaNativeDescription_play,                             \
    &AsebaNativeDescription_replay, &AsebaNativeDescription_sound_system, &AsebaNativeDescription_set_led_circle,            \
    &AsebaNativeDescription_set_led_rgb_top, &AsebaNativeDescription_set_led_rgb_bl, &AsebaNativeDescription_set_led_rgb_br, \
    &AsebaNativeDescription_play_freq, &AsebaNativeDescription_set_led_buttons, &AsebaNativeDescription_set_hprox_leds,      \
    &AsebaNativeDescription_set_vprox_leds, &AsebaNativeDescription_set_rc_leds, &AsebaNativeDescription_set_sound_leds,     \
    &AsebaNativeDescription_set_ntc_leds, &AsebaNativeDescription_set_wave, &AsebaNativeDescription_prox_network,            \
    &AsebaNativeDescription_sd_open, &AsebaNativeDescription_sd_write, &AsebaNativeDescription_sd_read,                      \
    &AsebaNativeDescription_sd_seek

#define THYMIO_NATIVES_FUNCTIONS                                                                                          \
  set_led, sound_record, sound_playback, sound_replay, sound_system, set_led_circle, set_rgb_top, set_rgb_bl, set_rgb_br, \
    play_freq, set_buttons_leds, set_hprox_leds, set_vprox_leds, set_rc_leds, set_sound_leds, set_ntc_leds, set_wave,     \
    prox_network, thymio_native_sd_open, thymio_native_sd_write, thymio_native_sd_read, thymio_native_sd_seek

#endif
