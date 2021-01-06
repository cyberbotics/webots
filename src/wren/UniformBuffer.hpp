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

#ifndef UNIFORMBUFFER_HPP
#define UNIFORMBUFFER_HPP

#include "GlUser.hpp"

namespace wren {

  class UniformBuffer : public GlUser {
  public:
    UniformBuffer(unsigned int binding, int size);
    ~UniformBuffer();

    unsigned int glName() const { return mGlName; }
    unsigned int binding() const { return mBinding; }

    void writeValue(const void *data) const;

    void bind() const;
    void release() const;

  protected:
    void prepareGl() override;
    void cleanupGl() override;

    unsigned int mGlName;
    unsigned int mBinding;
    int mSize;
  };

}  // namespace wren

#endif  // UNIFORM_BUFFER_HPP
