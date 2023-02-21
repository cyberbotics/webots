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

#ifndef SHADER_PROGRAM_HPP
#define SHADER_PROGRAM_HPP

#include "Constants.hpp"
#include "CustomUniform.hpp"
#include "GlUser.hpp"
#include "GlslLayout.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace wren {

  class Texture;

  // Defines a GLSL program created by compiling and linking a vertex shader and a fragment shader source.
  class ShaderProgram : public GlUser {
  public:
    // Encapsulate memory management
    static ShaderProgram *createShaderProgram() { return new ShaderProgram(); }
    static void deleteShaderProgram(ShaderProgram *program);

    void setVertexShaderPath(const char *vertexShaderPath) {
      assert(vertexShaderPath);
      mVertexShaderPath.assign(vertexShaderPath);
    }

    void setFragmentShaderPath(const char *fragmentShaderPath) {
      assert(fragmentShaderPath);
      mFragmentShaderPath.assign(fragmentShaderPath);
    }

    unsigned int glName() const { return mGlName; }

    // Lets the user declare which built-in uniforms need to be available in the shader
    void useUniform(WrGlslLayoutUniform uniform);
    void useUniformBuffer(WrGlslLayoutUniformBuffer uniformBuffer);

    // Create a uniform whose value can be set by the user
    template<class T> void createCustomUniform(const std::string &name, const T &initialValue) {
      mCustomUniforms.emplace(std::make_pair(name, new CustomUniform<T>(name, initialValue)));
    }

    template<class T> void setCustomUniformValue(const std::string &name, const T &value) {
      auto it = mCustomUniforms.find(name);
      assert(it != mCustomUniforms.end());
      it->second->setValue(reinterpret_cast<const char *>(&value));
    }

    // Internal, used by wren to modify the value of built-in uniforms
    int uniformLocation(WrGlslLayoutUniform uniform) const {
      if (!mGlName)
        return -1;

      assert(uniform >= 0 && uniform < mUniformLocations.size());
      return mUniformLocations[uniform];
    }

    bool hasVertexShadercompilationFailed() const { return mHasVertexShaderCompilationFailed; }
    bool hasFragmentShadercompilationFailed() const { return mHasFragmentShaderCompilationFailed; }
    const std::string &compilationLog() const { return mCompilationLog; }

    void setup();
    void bind() const;
    void release() const;

  private:
    static bool readFile(const std::string &path, std::string &contents);

    ShaderProgram();
    ~ShaderProgram();

    unsigned int compileShader(const std::string &path, unsigned int type);
    bool linkProgram();

    void prepareGl() override;
    void cleanupGl() override;

    unsigned int mGlName;

    std::string mVertexShaderPath;
    std::string mFragmentShaderPath;

    std::vector<WrGlslLayoutUniform> mUniforms;
    std::vector<WrGlslLayoutUniformBuffer> mUniformBuffers;
    std::vector<int> mUniformLocations;

    std::unordered_map<std::string, CustomUniformBase *> mCustomUniforms;

    bool mHasVertexShaderCompilationFailed;
    bool mHasFragmentShaderCompilationFailed;
    std::string mCompilationLog;
  };

}  // namespace wren

#endif  // SHADER_PROGRAM_HPP
