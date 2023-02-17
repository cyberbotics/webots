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

#ifndef TEXTURE_RTT_HPP
#define TEXTURE_RTT_HPP

#include "Texture.hpp"

namespace wren {

  // Texture which can be used to back a framebuffer.
  class TextureRtt : public Texture {
  public:
    // Encapsulate memory management
    static TextureRtt *createTextureRtt() { return new TextureRtt(); }
    static TextureRtt *copyTextureRtt(const TextureRtt *original);

    WrTextureType type() const override { return WR_TEXTURE_TYPE_RTT; }
    unsigned int glName() const override { return mGlName; }
    // Needs to be called by the using framebuffer during prepareGl
    void prepareGl() override;

    void initializeData(bool enabled) { mInitializeData = enabled; }
    void setGlName(unsigned int glName) override { mGlName = glName; }

  protected:
    TextureRtt();
    virtual ~TextureRtt() {}

    unsigned int mGlName;
    bool mInitializeData;
  };

}  // namespace wren

#endif  // TEXTURE_RTT_HPP
