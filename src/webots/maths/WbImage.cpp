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

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

static float *createGaussianConvolutionMatix(int width, int height, float sigma) {
  const int size = width * height;
  float *kernel = (float *)malloc(sizeof(float) * size);
  double sum = 0.0;
  int i, j;
  for (j = 0; j < height; ++j) {
    for (i = 0; i < width; ++i) {
      kernel[i + j * width] = exp(-(pow(i - width / 2, 2) + pow(j - height / 2, 2)) / (2.0 * sigma * sigma)) / (2.0 * M_PI * sigma * sigma);
      sum += kernel[i + j * width];
    }
  }
  for (j = 0; j < height; ++j) {
    for (i = 0; i < width; ++i) {
      kernel[i + j * width] /= sum;
    }
  }
  return kernel;
}

WbImage *WbImage::downscale(int width, int height) {
  float *convolutionMatrix = createGaussianConvolutionMatix(5, 5, 2.0);
  int i, j;
  for (j = 0; j < 5; ++j) {
    for (i = 0; i < 5; ++i) {
      printf("[%d, %d]: %f\n", i, j, convolutionMatrix[i + j * 5]);
    }
  }
  const int size = mWidth * mHeight * mChannels;
  unsigned char *pixels = (unsigned char *)malloc(size);
  memset(pixels, 0, size);
  WbImage *image = new WbImage(pixels, width, height, mChannels);
  return image;
}
