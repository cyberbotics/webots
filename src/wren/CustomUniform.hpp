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

#ifndef CUSTOM_UNIFORM_HPP
#define CUSTOM_UNIFORM_HPP

#include "Constants.hpp"

#include <string>

namespace wren {

  class CustomUniformBase {
  public:
    CustomUniformBase(const std::string &name);
    virtual ~CustomUniformBase() {}

    void setLocation(int location) { mLocation = location; }

    const std::string &name() const { return mName; }

    virtual void setValue(const char *value) = 0;
    virtual void uploadValue() const = 0;

  protected:
    void upload(bool value) const;
    void upload(int value) const;
    void upload(float value) const;
    void upload(const glm::vec2 &value) const;
    void upload(const glm::vec3 &value) const;
    void upload(const glm::vec4 &value) const;
    void upload(const glm::mat4 &value) const;

    std::string mName;
    int mLocation;
  };

  template<class T> class CustomUniform : public CustomUniformBase {
  public:
    CustomUniform(const std::string &name, const T &initialValue) : CustomUniformBase(name), mValue(initialValue) {}
    virtual ~CustomUniform() {}

    void setValue(const char *value) override { mValue = *(reinterpret_cast<const T *>(value)); }
    void uploadValue() const override {
      assert(mLocation >= 0);
      CustomUniformBase::upload(mValue);
    }

  private:
    T mValue;
  };

}  // namespace wren

#endif  // CUSTOM_UNIFORM_HPP
