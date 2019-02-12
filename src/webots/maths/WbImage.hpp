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

#ifndef WB_IMAGE_HPP
#define WB_IMAGE_HPP

class WbImage {
public:
  WbImage(const unsigned char *data, int width, int height, int channels) :
    mData(data),
    mWidth(width),
    mHeight(height),
    mChannels(channels)
  {}

  const unsigned char *data() const { return mData; }
  int width() const { return mWidth; }
  int height() const { return mHeight; }
  int channels() const { return mChannels; }

  WbImage *downscale(int width, int height);

private:
  //WbImage *applyConvolution();

  const unsigned char *mData;
  int mWidth;
  int mHeight;
  int mChannels;
};

#endif
