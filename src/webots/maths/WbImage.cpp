#include "WbImage.hpp"

#include <cstdlib>
#include <cstdio>
#include <cstring>

WbImage *WbImage::downscale(int width, int height) {
  const int size = mWidth * mHeight * mChannels;
  unsigned char *pixels = (unsigned char *) malloc(size);
  memset(pixels, 0, size);
  WbImage *image = new WbImage(pixels, width, height, mChannels);
  return image;
}
