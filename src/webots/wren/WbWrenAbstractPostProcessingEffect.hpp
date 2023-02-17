// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WB_ABSTRACT_POST_PROCESSING_EFFECT_HPP
#define WB_ABSTRACT_POST_PROCESSING_EFFECT_HPP

struct WrViewport;
struct WrPostProcessingEffect;

#include <wren/texture.h>

class WbWrenAbstractPostProcessingEffect {
public:
  WbWrenAbstractPostProcessingEffect();
  virtual ~WbWrenAbstractPostProcessingEffect();

  virtual void setup(WrViewport *viewport) = 0;
  virtual void detachFromViewport();
  bool hasBeenSetup() const { return mHasBeenSetup; }
  void setTextureFormat(WrTextureInternalFormat format) { mTextureFormat = format; }

  WrViewport *wrenViewport() const { return mWrenViewport; }

protected:
  virtual void applyParametersToWren() = 0;

  WrViewport *mWrenViewport;
  WrPostProcessingEffect *mWrenPostProcessingEffect;
  bool mHasBeenSetup;
  WrTextureInternalFormat mTextureFormat;
};

#endif  // WB_ABSTRACT_POST_PROCESSING_EFFECT_HPP
