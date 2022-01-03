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

#ifndef WB_IMAGE_HPP
#define WB_IMAGE_HPP

class WbImage {
public:
  WbImage(unsigned char *data, int width, int height) : mData(data), mWidth(width), mHeight(height) {}
  virtual ~WbImage() {}

  const unsigned char *data() const { return mData; }
  int width() const { return mWidth; }
  int height() const { return mHeight; }

  WbImage *downscale(int width, int height, int xBlurRadius, int yBlurRadius);

private:
  unsigned char *mData;
  int mWidth;
  int mHeight;
};

#endif
