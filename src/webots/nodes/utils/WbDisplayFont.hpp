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

#ifndef WB_DISPLAY_FONT_HPP
#define WB_DISPLAY_FONT_HPP

#include <QtCore/QList>
#include <QtCore/QString>

#include <ft2build.h>
#ifdef FT_FREETYPE_H
// cppcheck-suppress preprocessorErrorDirective
#include FT_FREETYPE_H
#endif

class WbDisplayFont {
public:
  WbDisplayFont();
  virtual ~WbDisplayFont();

  void setFont(const QString &filename, unsigned int size);
  unsigned char *generateCharBuffer(unsigned long character, bool antiAliasing, int *width, int *rows, int *verticalOffset,
                                    int *horizontalOffset, int *transparencyFactor, int *horizontalAdvance, int *pitch);

  unsigned int verticalSpace() const;
  unsigned int fontSize() const { return mFontSize; }
  QString error() const { return mError; }

private:
  void loadFace(FT_Face *face, const QString &filename, unsigned int size);
  void loadFallbackFaces(unsigned int size);
  void cleanupFallbackFaces();

  QList<FT_Face> mFallbackFaces;
  unsigned int mFallbackFacesSize;

  FT_Library mLibrary;
  FT_Face mFace;
  bool mFaceIsInitialized;
  QString mError;
  unsigned int mFontSize;
#ifdef _WIN32
  QByteArrayList mFileBuffers;
#endif
};

#endif  // WB_DISPLAY_FONT_HPP
