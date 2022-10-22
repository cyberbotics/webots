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

#include "Font.hpp"

#include <wren/font.h>
#ifdef _WIN32
#include <windows.h>
#endif

namespace wren {

  Font::Font() : mFaceIsInitialized(false), mError(WR_FONT_ERROR_NONE), mFontSize(0) {
    FT_Error fontError = FT_Init_FreeType(&mLibrary);
    if (fontError)
      mError = WR_FONT_ERROR_FREETYPE_LOADING;
#ifdef _WIN32
    mFileBuffer = NULL;
#endif
  }

  Font::~Font() {
    if (mFaceIsInitialized) {
      FT_Done_Face(mFace);
#ifdef _WIN32
      delete[] mFileBuffer;
#endif
    }
    FT_Done_FreeType(mLibrary);
  }

  unsigned char *Font::generateCharBuffer(unsigned long character, bool antiAliasing, int *width, int *rows,
                                          int *verticalOffset, int *horizontalOffset, int *transparencyFactor,
                                          int *horizontalAdvance, int *pitch) {
    FT_Error fontError;
    if (antiAliasing)
      fontError = FT_Load_Char(mFace, character, FT_LOAD_RENDER | FT_LOAD_TARGET_NORMAL);
    else
      fontError = FT_Load_Char(mFace, character, FT_LOAD_RENDER | FT_LOAD_TARGET_MONO);

    if (fontError) {
      mError = WR_FONT_ERROR_CHARACTER_LOADING;
      return NULL;
    }

    if (mFace->glyph->bitmap.pixel_mode == FT_PIXEL_MODE_GRAY) {
      *transparencyFactor = 255;
    } else
      *transparencyFactor = 1;

    *width = mFace->glyph->bitmap.width;
    *rows = mFace->glyph->bitmap.rows;
    *verticalOffset = mFace->glyph->bitmap_top;
    *horizontalOffset = mFace->glyph->bitmap_left;
    *horizontalAdvance = mFace->glyph->advance.x >> 6;
    *pitch = abs(mFace->glyph->bitmap.pitch);

    return mFace->glyph->bitmap.buffer;
  }

  void Font::setFontFace(const char *filename) {
    if (mFaceIsInitialized)
      FT_Done_Face(mFace);

#ifdef _WIN32
    // on Windows, we cannot call FT_New_Face directly as it doesn't support multi byte characters
    // thus, we have to convert the multi byte file name into a wchar_t array to open it
    // and load its contents into the memory before calling FT_Open_Face
    const int l = strlen(filename) + 1;  // include the final '\0'
    wchar_t *wfilename = new wchar_t[l];
    MultiByteToWideChar(CP_UTF8, 0, filename, -1, wfilename, l);
    FILE *fp = _wfopen(wfilename, L"rb");
    delete[] wfilename;
    if (fp == NULL) {
      mError = WR_FONT_ERROR_FONT_LOADING;
      return;
    }
    fseek(fp, 0, SEEK_END);
    const long size = ftell(fp);
    rewind(fp);
    delete[] mFileBuffer;
    mFileBuffer = new unsigned char[size + 1];
    fread(mFileBuffer, 1, size, fp);
    mFileBuffer[size] = '\0';
    fclose(fp);
    FT_Open_Args args;
    args.flags = FT_OPEN_MEMORY;
    args.memory_base = mFileBuffer;
    args.memory_size = size;
    FT_Error fontError = FT_Open_Face(mLibrary, &args, 0, &mFace);
#else
    FT_Error fontError = FT_New_Face(mLibrary, filename, 0, &mFace);
#endif

    if (fontError == FT_Err_Unknown_File_Format)
      mError = WR_FONT_ERROR_UNKNOWN_FILE_FORMAT;
    else if (fontError)
      mError = WR_FONT_ERROR_FONT_LOADING;
    else
      mFaceIsInitialized = true;
  }

  void Font::setFontSize(unsigned int size) {
    mFontSize = size;

    // Size is multiplied by 64 because FreeType measures font sizes in 1/64 of pixels
    FT_Error fontError = FT_Set_Char_Size(mFace, mFontSize << 6, 0, 72, 0);
    if (fontError)
      mError = WR_FONT_ERROR_FONT_SIZE;
  }

  unsigned int Font::verticalSpace() const { return mFace->size->metrics.height >> 6; }

  void Font::getBoundingBox(const char *text, int *width, int *height) {
    *height = 0;
    *width = 0;
    int max = 0;

    int l = strlen(text);
    wchar_t *wcharText = new wchar_t[l + 1];
#ifdef _WIN32  // mbstowcs doesn't work properly on Windows
    l = MultiByteToWideChar(CP_UTF8, 0, text, -1, wcharText, l + 1) - 1;
#else
    // cppcheck-suppress uninitdata
    l = mbstowcs(wcharText, text, l + 1);
#endif

    for (int i = 0; i < l; ++i) {
      if (wcharText[i] == L'\n') {
        *height += verticalSpace();
        max = 0;
      } else {
        if (FT_Load_Char(mFace, wcharText[i], FT_LOAD_RENDER | FT_LOAD_TARGET_NORMAL)) {
          mError = WR_FONT_ERROR_CHARACTER_LOADING;
          delete[] wcharText;
          return;
        }

        max += mFace->glyph->advance.x >> 6;
      }

      if (max > *width)
        *width = max;
    }

    // First line height
    *height += verticalSpace();

    delete[] wcharText;
  }

}  // namespace wren

// C interface implementation
WrFont *wr_font_new() {
  return reinterpret_cast<WrFont *>(wren::Font::createFont());
}

void wr_font_delete(WrFont *font) {
  wren::Font::deleteFont(reinterpret_cast<wren::Font *>(font));
}

void wr_font_set_face(WrFont *font, const char *filename) {
  reinterpret_cast<wren::Font *>(font)->setFontFace(filename);
}

void wr_font_set_size(WrFont *font, unsigned int size) {
  reinterpret_cast<wren::Font *>(font)->setFontSize(size);
}

void wr_font_get_bounding_box(WrFont *font, const char *text, int *width, int *height) {
  reinterpret_cast<wren::Font *>(font)->getBoundingBox(text, width, height);
}

WrFontError wr_font_get_error(WrFont *font) {
  return reinterpret_cast<wren::Font *>(font)->error();
}
