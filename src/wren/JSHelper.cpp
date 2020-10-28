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

#ifdef __EMSCRIPTEN__
#include <wren/JSHelper.h>

#include <GL/gl.h>
#include <GLES3/gl3.h>
#include <emscripten/html5.h>

float *wrjs_color_array(float r, float g, float b) {
  static float array[3];
  array[0] = r;
  array[1] = g;
  array[2] = b;

  return array;
}

float *wrjs_array4(float element0, float element1, float element2, float element3) {
  static float array[4];
  array[0] = element0;
  array[1] = element1;
  array[2] = element2;
  array[3] = element3;

  return array;
}

char *wrjs_pointerOnFloat(float nbr) {
  static float number = nbr;
  return reinterpret_cast<char *>(&number);
}

char *wrjs_dummy_texture() {
  static char data[256 * 256 * 4];
  for (int d = 0; d < (256 * 256); ++d) {
    data[4 * d] = (d) % 0xff;
    data[4 * d + 1] = (d + d) % 0xff;
    data[4 * d + 2] = (d * d) % 0xff;
    data[4 * d + 3] = 0xff;
  }
  return data;
}

void wrjs_init_context(int width, int height) {
  EmscriptenWebGLContextAttributes attr;
  emscripten_webgl_init_context_attributes(&attr);
  attr.alpha = attr.depth = attr.stencil = attr.antialias = attr.preserveDrawingBuffer = attr.failIfMajorPerformanceCaveat = 0;
  attr.enableExtensionsByDefault = 1;
  attr.premultipliedAlpha = 0;
  attr.majorVersion = 2;
  attr.minorVersion = 2;
  EMSCRIPTEN_WEBGL_CONTEXT_HANDLE ctx = emscripten_webgl_create_context("#canvas", &attr);

  emscripten_set_canvas_element_size("#canvas", width, height);

  emscripten_webgl_make_context_current(ctx);
}

#endif
