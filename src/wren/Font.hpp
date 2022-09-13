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

#ifndef FONT_HPP
#define FONT_HPP

#include <ft2build.h>
#ifdef FT_FREETYPE_H
// cppcheck-suppress preprocessorErrorDirective
#include FT_FREETYPE_H
#endif

#include <wren/font.h>

namespace wren {

  class Font {
  public:
    static Font *createFont() { return new Font(); }
    static void deleteFont(Font *font) { delete font; }

    void setFontFace(const char *filename);
    void setFontSize(unsigned int size);
    unsigned char *generateCharBuffer(unsigned long character, bool antiAliasing, int *width, int *rows, int *verticalOffset,
                                      int *horizontalOffset, int *transparencyFactor, int *horizontalAdvance, int *pitch);

    unsigned int verticalSpace() const;
    int descender() const { return mFace->descender * (mFontSize / static_cast<float>(mFace->units_per_EM)); }
    void getBoundingBox(const char *text, int *width, int *height);
    unsigned int fontSize() const { return mFontSize; }
    WrFontError error() const { return mError; }

  private:
    Font();
    ~Font();

    FT_Library mLibrary;
    FT_Face mFace;
    bool mFaceIsInitialized;
    WrFontError mError;
    unsigned int mFontSize;
#ifdef _WIN32
    unsigned char *mFileBuffer;
#endif
  };

}  // namespace wren

#endif
