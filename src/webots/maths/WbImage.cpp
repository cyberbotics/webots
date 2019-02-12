// Copyright 1996-2018 Cyberbotics Ltd.
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

// #include <cmath>
// #include <cstdio>
#include <cstdlib>
#include <cstring>

#define CLAMP(v, low, high) ((v) < (low) ? (low) : ((v) > (high) ? (high) : (v)))

static const float gaussBlur[3][3] = {
  {1.0f / 16.0f, 2.0f / 16.0f, 1.0f / 16.0f},
  {2.0f / 16.0f, 4.0f / 16.0f, 2.0f / 16.0f},
  {1.0f / 16.0f, 2.0f / 16.0f, 1.0f / 16.0f},
};

WbImage *WbImage::downscale(int width, int height) {
  const int size = mWidth * mHeight * mChannels;
  unsigned char *pixels = (unsigned char *)malloc(size);
  memset(pixels, 0, size);

  const float widthRatio = (float)mWidth / width;
  const float heightRatio = (float)mHeight / height;
  const int blurRadius = 2;

  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      for (int c = 0; c < mChannels; ++c) {
        int destIndex = (i + (j * width)) * mChannels + c;
        for (int u = -1; u < 2; ++u) {
          for (int v = -1; v < 2; ++v) {
            int srcIndex = (int)(CLAMP(widthRatio * i + u * blurRadius, 0, mWidth) + (CLAMP(heightRatio * j + v * blurRadius, 0, mHeight) * mWidth)) * mChannels + c;
            pixels[destIndex] += (unsigned char)(gaussBlur[u + 1][v + 1] * mData[srcIndex]);
          }
        }
      }
    }
  }

  return new WbImage(pixels, width, height, mChannels);
}
