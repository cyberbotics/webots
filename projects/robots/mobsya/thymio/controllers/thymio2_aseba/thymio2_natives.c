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

// Strongly inspired from https://github.com/aseba-community/aseba-target-thymio2/blob/master/thymio_natives.c
// and https://github.com/aseba-community/aseba-target-thymio2/blob/master/skel-usb.c

#include "thymio2_natives.h"

AsebaNativeFunctionDescription AsebaNativeDescription_poweroff = {"_poweroff", "Poweroff", {{0, 0}}};

void power_off(AsebaVMState *vm) {
}

AsebaNativeFunctionDescription AsebaNativeDescription__system_reboot = {"_system.reboot",
                                                                        "Reboot the microcontroller",
                                                                        {{0, 0}}};

void AsebaNative__system_reboot(AsebaVMState *vm) {
}

AsebaNativeFunctionDescription AsebaNativeDescription__system_settings_read = {"_system.settings.read",
                                                                               "Read a setting",
                                                                               {{1, "address"}, {1, "value"}, {0, 0}}};

void AsebaNative__system_settings_read(AsebaVMState *vm) {
  /*uint16 address = vm->variables[*/ AsebaNativePopArg(vm) /*]*/;
  /*uint16 destidx =*/AsebaNativePopArg(vm);
}

AsebaNativeFunctionDescription AsebaNativeDescription__system_settings_write = {"_system.settings.write",
                                                                                "Write a setting",
                                                                                {{1, "address"}, {1, "value"}, {0, 0}}};

void AsebaNative__system_settings_write(AsebaVMState *vm) {
  /*uint16 address = vm->variables[*/ AsebaNativePopArg(vm) /*]*/;
  /*uint16 sourceidx =*/AsebaNativePopArg(vm);
}

AsebaNativeFunctionDescription AsebaNativeDescription__system_settings_flash = {"_system.settings.flash",
                                                                                "Write the settings into flash",
                                                                                {{0, 0}}};

void AsebaNative__system_settings_flash(AsebaVMState *vm) {
}

AsebaNativeFunctionDescription AsebaNativeDescription_set_led = {"_leds.set",
                                                                 "Set the led",
                                                                 {{1, "led"}, {1, "brightness"}, {0, 0}}};

AsebaNativeFunctionDescription AsebaNativeDescription_record = {"sound.record",
                                                                "Start recording of rN.wav",
                                                                {
                                                                  {1, "N"},
                                                                  {0, 0},
                                                                }};

AsebaNativeFunctionDescription AsebaNativeDescription_play = {"sound.play",
                                                              "Start playback of pN.wav",
                                                              {
                                                                {1, "N"},
                                                                {0, 0},
                                                              }};

AsebaNativeFunctionDescription AsebaNativeDescription_replay = {"sound.replay",
                                                                "Start playback of rN.wav",
                                                                {
                                                                  {1, "N"},
                                                                  {0, 0},
                                                                }};

AsebaNativeFunctionDescription AsebaNativeDescription_sound_system = {"sound.system",
                                                                      "Start playback of system sound N",
                                                                      {
                                                                        {1, "N"},
                                                                        {0, 0},
                                                                      }};

void sound_playback(AsebaVMState *vm) {
  /*int number = vm->variables[*/ AsebaNativePopArg(vm) /*]*/;
}

void sound_record(AsebaVMState *vm) {
  /*int number = vm->variables[*/ AsebaNativePopArg(vm) /*]*/;
}

void sound_replay(AsebaVMState *vm) {
  /*int number = vm->variables[*/ AsebaNativePopArg(vm) /*]*/;
}

void sound_system(AsebaVMState *vm) {
  /*int number = vm->variables[*/ AsebaNativePopArg(vm) /*]*/;
}

AsebaNativeFunctionDescription AsebaNativeDescription_set_led_circle = {"leds.circle",
                                                                        "Set circular ring leds",
                                                                        {
                                                                          {1, "l0"},
                                                                          {1, "l1"},
                                                                          {1, "l2"},
                                                                          {1, "l3"},
                                                                          {1, "l4"},
                                                                          {1, "l5"},
                                                                          {1, "l6"},
                                                                          {1, "l7"},
                                                                          {0, 0},
                                                                        }};

AsebaNativeFunctionDescription AsebaNativeDescription_set_led_rgb_top = {"leds.top",
                                                                         "Set RGB top led",
                                                                         {
                                                                           {1, "red"},
                                                                           {1, "green"},
                                                                           {1, "blue"},
                                                                           {0, 0},
                                                                         }};

AsebaNativeFunctionDescription AsebaNativeDescription_set_led_rgb_br = {"leds.bottom.right",
                                                                        "Set RGB botom right led",
                                                                        {
                                                                          {1, "red"},
                                                                          {1, "green"},
                                                                          {1, "blue"},
                                                                          {0, 0},
                                                                        }};

AsebaNativeFunctionDescription AsebaNativeDescription_set_led_rgb_bl = {"leds.bottom.left",
                                                                        "Set RGB botom left led",
                                                                        {
                                                                          {1, "red"},
                                                                          {1, "green"},
                                                                          {1, "blue"},
                                                                          {0, 0},
                                                                        }};

AsebaNativeFunctionDescription AsebaNativeDescription_set_led_buttons = {"leds.buttons",
                                                                         "Set buttons leds",
                                                                         {
                                                                           {1, "l0"},
                                                                           {1, "l1"},
                                                                           {1, "l2"},
                                                                           {1, "l3"},
                                                                           {0, 0},
                                                                         }};

AsebaNativeFunctionDescription AsebaNativeDescription_set_hprox_leds = {"leds.prox.h",
                                                                        "Set horizontal proximity leds",
                                                                        {
                                                                          {1, "l0"},
                                                                          {1, "l1"},
                                                                          {1, "l2"},
                                                                          {1, "l3"},
                                                                          {1, "l4"},
                                                                          {1, "l5"},
                                                                          {1, "l6"},
                                                                          {1, "l7"},
                                                                          {0, 0},
                                                                        }};

AsebaNativeFunctionDescription AsebaNativeDescription_set_vprox_leds = {"leds.prox.v",
                                                                        "Set vertical proximity leds",
                                                                        {
                                                                          {1, "l0"},
                                                                          {1, "l1"},
                                                                          {0, 0},
                                                                        }};

AsebaNativeFunctionDescription AsebaNativeDescription_set_rc_leds = {"leds.rc",
                                                                     "Set rc led",
                                                                     {
                                                                       {1, "led"},
                                                                       {0, 0},
                                                                     }};

AsebaNativeFunctionDescription AsebaNativeDescription_set_sound_leds = {"leds.sound",
                                                                        "Set sound led",
                                                                        {
                                                                          {1, "led"},
                                                                          {0, 0},
                                                                        }};

AsebaNativeFunctionDescription AsebaNativeDescription_set_ntc_leds = {"leds.temperature",
                                                                      "Set ntc led",
                                                                      {
                                                                        {1, "red"},
                                                                        {1, "blue"},
                                                                        {0, 0},
                                                                      }};

AsebaNativeFunctionDescription AsebaNativeDescription_play_freq = {"sound.freq",
                                                                   "Play frequency",
                                                                   {
                                                                     {1, "Hz"},
                                                                     {1, "ds"},
                                                                     {0, 0},
                                                                   }};

void play_freq(AsebaVMState *vm) {
  /*int freq = vm->variables[*/ AsebaNativePopArg(vm) /*]*/;
  /*int time = vm->variables[*/ AsebaNativePopArg(vm) /*]*/;
}

#define WAVEFORM_SIZE 142
AsebaNativeFunctionDescription AsebaNativeDescription_set_wave = {"sound.wave",
                                                                  "Set the primary wave of the tone generator",
                                                                  {
                                                                    {WAVEFORM_SIZE, "wave"},
                                                                    {0, 0},
                                                                  }};

void set_wave(AsebaVMState *vm) {
  /*int *wave = (int *) vm->variables + */ AsebaNativePopArg(vm);
}

AsebaNativeFunctionDescription AsebaNativeDescription_prox_network = {"prox.comm.enable",
                                                                      "Enable or disable the proximity communication",
                                                                      {
                                                                        {1, "state"},
                                                                        {0, 0},
                                                                      }};

void prox_network(AsebaVMState *vm) {
  /*int enable = vm->variables[*/ AsebaNativePopArg(vm) /*]*/;
}

AsebaNativeFunctionDescription AsebaNativeDescription_sd_open = {"sd.open",
                                                                 "Open a file on the SD card",
                                                                 {
                                                                   {1, "number"},
                                                                   {1, "status"},
                                                                   {0, 0},
                                                                 }};

void thymio_native_sd_open(AsebaVMState *vm) {
  /*int no = vm->variables[*/ AsebaNativePopArg(vm) /*]*/;
  /*unsigned int status = */ AsebaNativePopArg(vm);
}

AsebaNativeFunctionDescription AsebaNativeDescription_sd_write = {"sd.write",
                                                                  "Write data to the opened file",
                                                                  {
                                                                    {-1, "data"},
                                                                    {1, "written"},
                                                                    {0, 0},
                                                                  }};

void thymio_native_sd_write(AsebaVMState *vm) {
  /*unsigned char * data = (unsigned char *) (vm->variables + */ AsebaNativePopArg(vm) /*)*/;
  /*uint16 status = */ AsebaNativePopArg(vm);
  /*uint16 length = */ AsebaNativePopArg(vm) /* * 2*/;
}

AsebaNativeFunctionDescription AsebaNativeDescription_sd_read = {"sd.read",
                                                                 "Read data from the opened file",
                                                                 {
                                                                   {-1, "data"},
                                                                   {1, "read"},
                                                                   {0, 0},
                                                                 }};

void thymio_native_sd_read(AsebaVMState *vm) {
  /*unsigned char * data = (unsigned char *) (vm->variables + */ AsebaNativePopArg(vm) /*)*/;
  /*uint16 status =*/AsebaNativePopArg(vm);
  /*uint16 length =*/AsebaNativePopArg(vm) /* * 2*/;
}

AsebaNativeFunctionDescription AsebaNativeDescription_sd_seek = {"sd.seek",
                                                                 "Seek the opened file",
                                                                 {
                                                                   {1, "position"},
                                                                   {1, "status"},
                                                                   {0, 0},
                                                                 }};

void thymio_native_sd_seek(AsebaVMState *vm) {
  /*unsigned long seek = vm->variables[*/ AsebaNativePopArg(vm) /*]*/;
  /*unsigned int status = */ AsebaNativePopArg(vm);
}
