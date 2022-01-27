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

#include "WbDisplayFont.hpp"
#include "WbStandardPaths.hpp"

#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QTextStream>

#include <cassert>
#include <cmath>

void WbDisplayFont::loadFallbackFaces(unsigned int size) {
  static QStringList fontFilenames;
  if (fontFilenames.isEmpty()) {
    QString fallbackFontsFilename = WbStandardPaths::fontsPath() + "fallback_fonts.txt";
    QFile fallbackFontsFile(fallbackFontsFilename);
    if (fallbackFontsFile.open(QIODevice::ReadOnly)) {
      QTextStream in(&fallbackFontsFile);
      while (!in.atEnd()) {
        QString line = in.readLine();
        if (!line.startsWith("#") && !line.isEmpty()) {
          QString fontPath = WbStandardPaths::fontsPath() + QString(line) + ".ttf";
          if (QFileInfo(fontPath).exists())
            fontFilenames << fontPath;
        }
      }
      fallbackFontsFile.close();
    }
  }
  mFallbackFacesSize = size;

  foreach (const QString &fontFilename, fontFilenames) {
    FT_Face face;
    loadFace(&face, fontFilename, size);
    if (error().isEmpty())
      mFallbackFaces << face;
    else
      assert(0);
  }
}

void WbDisplayFont::cleanupFallbackFaces() {
  foreach (const FT_Face face, mFallbackFaces)
    FT_Done_Face(face);
  mFallbackFaces.clear();
}

WbDisplayFont::WbDisplayFont() : mFallbackFacesSize(0), mFaceIsInitialized(false), mError(""), mFontSize(0) {
  FT_Error error = FT_Init_FreeType(&mLibrary);
  if (error)
    mError = "An error occurred while initializing the freetype library.";
}

WbDisplayFont::~WbDisplayFont() {
  FT_Done_Face(mFace);
  cleanupFallbackFaces();
  FT_Done_FreeType(mLibrary);
}

unsigned char *WbDisplayFont::generateCharBuffer(unsigned long character, bool antiAliasing, int *width, int *rows,
                                                 int *verticalOffset, int *horizontalOffset, int *transparencyFactor,
                                                 int *horizontalAdvance, int *pitch) {
  unsigned char *result = NULL;
  FT_Int32 aliasingFlags = antiAliasing ? FT_LOAD_RENDER | FT_LOAD_TARGET_NORMAL : FT_LOAD_RENDER | FT_LOAD_TARGET_MONO;

  mFallbackFaces.prepend(mFace);

  foreach (const FT_Face face, mFallbackFaces) {
    unsigned int index = FT_Get_Char_Index(face, character);
    FT_Error error = FT_Load_Glyph(face, index, aliasingFlags);
    if (!error && index != 0) {  // found a glyph.
      *transparencyFactor = face->glyph->bitmap.pixel_mode == FT_PIXEL_MODE_GRAY ? 255 : 1;
      *width = face->glyph->bitmap.width;
      *rows = face->glyph->bitmap.rows;
      *verticalOffset = face->glyph->bitmap_top;
      *horizontalOffset = face->glyph->bitmap_left;
      *horizontalAdvance = face->glyph->linearHoriAdvance >> 16;
      *pitch = abs(face->glyph->bitmap.pitch);
      result = face->glyph->bitmap.buffer;
      break;
    }
  }

  mFallbackFaces.removeFirst();
  return result;
}

void WbDisplayFont::setFont(const QString &filename, unsigned int size) {
  mFontSize = size;
  if (mFaceIsInitialized)
    FT_Done_Face(mFace);
  loadFace(&mFace, filename, size);
  if (mFontSize != mFallbackFacesSize) {
    cleanupFallbackFaces();
    loadFallbackFaces(mFontSize);
  }
  mFaceIsInitialized = true;
}

void WbDisplayFont::loadFace(FT_Face *face, const QString &filename, unsigned int size) {
  mError = "";
  mFontSize = size;
  FT_Error error = FT_New_Face(mLibrary, filename.toUtf8().constData(), 0, face);
  if (error == FT_Err_Unknown_File_Format)
    mError = "The font file could be opened and read, but its font format is unsupported.";
  else if (error)
    mError = "The font file could not be opened or read, or is broken.";
  error = FT_Set_Char_Size(*face, size * 64, 0, 100, 0);
  if (error)
    mError = "An error occurred when setting the font size.";
}

unsigned int WbDisplayFont::verticalSpace() const {
  return mFace->glyph->linearVertAdvance >> 16;
}
