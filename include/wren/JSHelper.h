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

#ifndef WR_JSHELPER_H
#define WR_JSHELPER_H

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

float *wrjs_color_array(float r, float g, float b);
float *wrjs_array4(float element0, float element1, float element2, float element3);
char *wrjs_array4_char(float element0, float element1, float element2, float element3);
int *wrjs_pointerOnInt(int nbr);
int *wrjs_pointerOnIntBis(int nbr);
char *wrjs_pointerOnFloat(float nbr);
char *wrjs_dummy_texture();
char *wrjs_dummy_small_texture();

void wrjs_init_context(int width, int height);
void wrjs_exit();
const char *wrjs_load_hdr_file(int *w, char *url);
void wrjs_free_hdr_file(char *pointer);
#ifdef __cplusplus
}
#endif

#endif  // WR_VIEWPORT_H
