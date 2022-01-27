// Copyright 1996-2021 Cyberbotics Ltd.
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

#ifndef IMAGE_REF_HPP
#define IMAGE_REF_HPP

#define WB_USING_CPP_API
#include "../../c/webots/types.h"

namespace webots {
  class ImageRef {
  public:
    // Use Display::imageNew(), Display::imageCopy() or Display::imageLoad() instead
    explicit ImageRef(const WbImageRef &imageRef) : imageRef(imageRef) {}
    // Use Display::imageDelete() instead
    virtual ~ImageRef() {}
    WbImageRef getImageRef() { return imageRef; }

  private:
    WbImageRef imageRef;
  };
}  // namespace webots
#endif  // IMAGE_REF_HPP
