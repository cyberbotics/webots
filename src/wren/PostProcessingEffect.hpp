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

#ifndef POST_PROCESSING_EFFECT_HPP
#define POST_PROCESSING_EFFECT_HPP

#include "Constants.hpp"
#include "Texture.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace wren {

  class FrameBuffer;
  class StaticMesh;
  class ShaderProgram;
  class TextureRtt;
  class Viewport;

  // Applies post-processing effects in the order defined by its passes
  class PostProcessingEffect {
  public:
    class Pass {
    public:
      struct Connection {
        Connection(Pass *from, size_t outputIndex, Pass *to, size_t inputIndex) :
          mFrom(from),
          mOutputIndex(outputIndex),
          mTo(to),
          mInputIndex(inputIndex) {}

        Pass *mFrom;
        size_t mOutputIndex;
        Pass *mTo;
        size_t mInputIndex;
      };

      // Encapsulate memory management
      static Pass *createPass() { return new Pass(); }
      static void deletePass(Pass *pass) { delete pass; }

      void setName(const std::string &name) { mName = name; }
      void setProgram(ShaderProgram *program) { mProgram = program; }
      void setOutputSize(int width, int height) {
        mOutputWidth = width;
        mOutputHeight = height;
      }

      void setInputTextureCount(size_t count) {
        mInputTextures = std::vector<Texture *>(count, NULL);

        Texture::UsageParams params(Texture::DEFAULT_USAGE_PARAMS);
        params.mAreMipMapsEnabled = false;
        mInputTextureParams = std::vector<Texture::UsageParams>(count, params);
      }

      void setOutputTextureCount(size_t count) {
        mOutputTextureFormat = std::vector<WrTextureInternalFormat>(count, WR_TEXTURE_INTERNAL_FORMAT_RGBA8);
      }

      void setInputTextureWrapMode(size_t index, WrTextureWrapMode mode) {
        assert(index < mInputTextures.size());
        mInputTextureParams[index].mWrapS = mode;
        mInputTextureParams[index].mWrapT = mode;
      }

      void setInputTextureInterpolation(size_t index, bool enable) {
        assert(index < mInputTextures.size());
        mInputTextureParams[index].mIsInterpolationEnabled = enable;
      }

      void setOutputTextureFormat(size_t index, WrTextureInternalFormat format) {
        assert(index < mOutputTextureFormat.size());
        mOutputTextureFormat[index] = format;
      }

      void setInputTexture(size_t index, Texture *texture) {
        assert(index < mInputTextures.size());
        mInputTextures[index] = texture;
      }

      void setIterationCount(size_t count) { mIterationCount = count; }

      void addConnection(const Connection &connection) {
        assert(connection.mFrom == this || connection.mTo == this);
        if (connection.mFrom == this)
          assert(connection.mOutputIndex < mOutputTextureFormat.size());
        else {
          assert(connection.mInputIndex < mInputTextures.size());
          assert(mInputTextures[connection.mInputIndex] == NULL);
        }
        mConnections.push_back(connection);
      }

      void addInputOutputTexture(size_t indexInput, size_t indexOutput) {
        assert(indexInput < mInputTextures.size());
        assert(indexOutput < mOutputTextureFormat.size());
        assert(mInputTextures[indexInput] == NULL);
        mInputOutputTextures.push_back(InputOutputTexture(indexInput, indexOutput));
      }

      void setProgramParameter(const std::string &parameterName, const char *value) {
        assert(mProgram);
        mProgramParameters[parameterName] = value;
      }

      void setClearBeforeDraw(bool enable) { mClearBeforeDraw = enable; }

      void setAlphaBlending(bool enable) { mUseAlphaBlending = enable; }

      const std::string &name() const { return mName; }
      int outputWidth() const { return mOutputWidth; }
      int outputHeight() const { return mOutputHeight; }
      FrameBuffer *frameBuffer() const { return mFrameBuffer; }
      ShaderProgram *program() const { return mProgram; }
      const std::vector<Texture *> &inputTextures() const { return mInputTextures; }
      int inputTextureCount() const { return mInputTextures.size(); }
      int outputTextureCount() const { return mOutputTextureFormat.size(); }
      int inputOutputTextureCount() const { return mInputOutputTextures.size(); }

      void setup();
      void processConnections();
      void apply();

      TextureRtt *outputTexture(size_t index);

    private:
      struct InputOutputTexture {
        InputOutputTexture(size_t inputTextureIndex, size_t outputTextureIndexEven) :
          mTextureEven(NULL),
          mTextureOdd(NULL),
          mInputTextureIndex(inputTextureIndex),
          mOutputTextureIndexEven(outputTextureIndexEven),
          mOutputTextureIndexOdd(-1) {}

        TextureRtt *mTextureEven;
        TextureRtt *mTextureOdd;
        size_t mInputTextureIndex;
        size_t mOutputTextureIndexEven;
        size_t mOutputTextureIndexOdd;
      };

      Pass();
      ~Pass();

      void swapInputOutputTextures();

      std::string mName;

      FrameBuffer *mFrameBuffer;
      StaticMesh *mMesh;
      ShaderProgram *mProgram;

      int mIterationCount;
      int mOutputWidth;
      int mOutputHeight;

      // used for clearing output framebuffer before next draw
      bool mClearBeforeDraw;
      bool mUseAlphaBlending;

      std::vector<Texture *> mInputTextures;
      std::vector<Texture::UsageParams> mInputTextureParams;
      std::vector<WrTextureInternalFormat> mOutputTextureFormat;
      std::vector<InputOutputTexture> mInputOutputTextures;
      std::vector<Connection> mConnections;

      std::unordered_map<std::string, const char *> mProgramParameters;
    };

    // Encapsulate memory management
    static PostProcessingEffect *createPostProcessingEffect() { return new PostProcessingEffect(); }
    static void deletePostProcessingEffect(PostProcessingEffect *postProcessingEffect) { delete postProcessingEffect; }

    void appendPass(Pass *pass) { mPasses.push_back(pass); }
    void connect(Pass *from, int outputIndex, Pass *to, int inputIndex);

    void setInputFrameBuffer(FrameBuffer *frameBuffer) { mInputFrameBuffer = frameBuffer; }
    void setResultProgram(ShaderProgram *program) { mResultProgram = program; }
    void setResultFrameBuffer(FrameBuffer *frameBuffer) { mResultFrameBuffer = frameBuffer; }

    // CSS-like index to specify order of application
    int drawingIndex() const { return mDrawingIndex; }
    void setDrawingIndex(unsigned int index) { mDrawingIndex = index; }

    Pass *firstPass() const {
      if (mPasses.size())
        return mPasses.front();
      return NULL;
    }

    Pass *lastPass() const {
      if (mPasses.size())
        return mPasses.back();
      return NULL;
    }

    Pass *pass(const std::string &name) {
      for (Pass *p : mPasses) {
        if (p->name() == name)
          return p;
      }
      return NULL;
    }

    void setup();
    void apply();

    void printPasses() const;

  private:
    PostProcessingEffect();
    ~PostProcessingEffect();

    void renderToResultFrameBuffer();

    std::vector<Pass *> mPasses;
    ShaderProgram *mResultProgram;
    FrameBuffer *mInputFrameBuffer;
    FrameBuffer *mResultFrameBuffer;
    StaticMesh *mMesh;

    size_t mDrawingIndex;
  };

}  // namespace wren

#endif  // POST_PROCESSING_EFFECT_HPP
