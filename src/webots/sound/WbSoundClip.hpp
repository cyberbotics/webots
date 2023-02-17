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

#ifndef WB_SOUND_CLIP_HPP
#define WB_SOUND_CLIP_HPP

//
// Description: manage a sound buffer
//

#include <QtCore/QString>

class QIODevice;

class WbWaveFile;

class WbSoundClip {
public:
  WbSoundClip();
  virtual ~WbSoundClip();
  const QString &filename() const { return mFilename; }
  int side() const { return mSide; }
  double balance() const { return mBalance; }
  void load(const QString &filename, const QString &extension, QIODevice *device = NULL, double balance = 0, int side = 0);
  void load(const WbWaveFile *wave);
  unsigned int openALBuffer() const { return mBuffer; }

protected:
  QString mFilename;
  QIODevice *mDevice;
  unsigned int mBuffer;
  int mSide;  // 0: both sides, -1: left only, 1: right only
  double mBalance;
};

#endif
