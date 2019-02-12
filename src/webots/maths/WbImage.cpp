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

static float *createGaussianConvolutionMatrix(int width, int height, float sigma) {
  const int size = width * height;
  float *kernel = (float *)malloc(sizeof(float) * size);
  double sum = 0.0;
  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      kernel[i + j * width] =
        exp(-(pow(i - width / 2, 2) + pow(j - height / 2, 2)) / (2.0 * sigma * sigma)) / (2.0 * M_PI * sigma * sigma);
      sum += kernel[i + j * width];
    }
  }
  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      kernel[i + j * width] /= sum;
    }
  }
  return kernel;
}

WbImage *WbImage::applyConvolution(float *matrix, int matrixWidth, int matrixHeight) {
  const int size = mWidth * mHeight * mChannels;
  unsigned char *pixels = (unsigned char *)malloc(size);
  memset(pixels, 0, size);

  const int halfMatrixWidth = ceil(0.5f * matrixWidth);
  const int halfMatrixHeight = ceil(0.5f * matrixHeight);

  for (int j = halfMatrixHeight; j < mHeight - halfMatrixHeight; ++j) {
    for (int i = halfMatrixWidth; i < mWidth - halfMatrixWidth; ++i) {
      for (int c = 0; c < mChannels; ++c) {
        for (int v = 0; v < matrixHeight; ++v) {
          for (int u = 0; u < matrixWidth; ++u) {
            int pixelIndex = (i - (u - matrixWidth / 2) + (j - (v - matrixHeight / 2)) * mWidth) * mChannels + c;
            pixels[pixelIndex] += (unsigned char)((float)mData[pixelIndex] * matrix[u + v * matrixWidth]);
          }
        }
      }
    }
  }

  return new WbImage(pixels, mWidth, mHeight, mChannels);
}

WbImage *WbImage::noFilterDownscale(int width, int height) {
  const int size = width * height * mChannels;
  unsigned char *pixels = (unsigned char *)malloc(size);

  const float widthRatio = (float) mWidth / width;
  const float heightRatio = (float) mHeight / height;

  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      for (int c = 0; c < mChannels; ++c) {
        int srcIndex = (int)(widthRatio * i + (heightRatio * j * mWidth) + 0.5) * mChannels + c;
        int destIndex = (i + (j * width)) * mChannels + c;
        pixels[destIndex] = mData[srcIndex];
      }
    }
  }

  return new WbImage(pixels, width, height, mChannels);
}

WbImage *WbImage::downscale(int width, int height) {
  // reference: https://stackoverflow.com/questions/49879466/downscaling-images-using-bilinear-and-bicubic-algorithms
  float *convolutionMatrix = createGaussianConvolutionMatrix(5, 5, 10.0);
  WbImage *blurredImage = applyConvolution(convolutionMatrix, 5, 5);
  free(convolutionMatrix);
  return blurredImage->noFilterDownscale(width, height);
}
