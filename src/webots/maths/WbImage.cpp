// Copyright 1996-2022 Cyberbotics Ltd.
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

#include "WbImage.hpp"

#include <omp.h>
#include <cstdlib>
#include <cstring>

#define CLAMP(v, low, high) ((v) < (low) ? (low) : ((v) > (high) ? (high) : (v)))

WbImage *WbImage::downscale(int width, int height, int xBlurRadius, int yBlurRadius) {
  // 3x3 convolution matrix to blur.
  static const int gaussBlur[3][3] = {
    {1, 2, 1},
    {2, 4, 2},
    {1, 2, 1},
  };
  const int channels = 4;
  const int dstSize = width * height * channels;
  const int srcSize = mWidth * mHeight * channels;
  unsigned char *pixels = static_cast<unsigned char *>(malloc(dstSize));

  // downscale and apply the convolution matrix.
  for (int j = 0; j < height; ++j) {
#pragma omp parallel for

    for (int i = 0; i < width; ++i) {
      for (int c = 0; c < channels; ++c) {
        const int destIndex = (i + (j * width)) * channels + c;
        unsigned char acc = 0;
        for (int u = -1; u < 2; ++u) {
          const int x = CLAMP(mWidth * i / width + u * xBlurRadius, 0, mWidth - 1);
          for (int v = -1; v < 2; ++v) {
            const int y = CLAMP(mHeight * j / height + v * yBlurRadius, 0, mHeight - 1);
            const int srcIndex = CLAMP((int)(x + y * mWidth) * channels + c, 0, srcSize);
            acc += gaussBlur[u + 1][v + 1] * mData[srcIndex] / 16;
          }
        }
        pixels[destIndex] = acc;
      }
    }
  }
  return new WbImage(pixels, width, height);
}
