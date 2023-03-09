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

#ifndef WB_WAVE_FILE_HPP
#define WB_WAVE_FILE_HPP

#include <QtCore/QIODevice>
#include <QtCore/QString>

class WbWaveFile {
public:
  explicit WbWaveFile(const QString &filename, QIODevice *device);
  WbWaveFile(qint16 *buffer, int bufferSize, int channelNumber, int bitsPerSample, int rate);
  virtual ~WbWaveFile();

  void loadFromFile(const QString &extension, int side = 0);

  void loadConvertedFile(int side);
  void loadConvertedFile(int side, const QString &filename);

  // balance: -1 = only left, +1 = only right
  void convertToMono(double balance = 0);

  // the API was designed to be able to easily add the following functions:
  // void loadFromBuffer(const unsigned char *buffer, int channels, int bitPerSample, int frequency);
  // void writeToFile();

  const qint16 *buffer() const { return mBuffer; }
  qint64 bufferSize() const { return mBufferSize; }
  int nChannels() const { return mNChannels; }
  int bitsPerSample() const { return mBitsPerSample; }
  int rate() const { return mRate; }
  QString filename() const { return mFilename; }

private:
  QString mFilename;
  QIODevice *mDevice;
  qint16 *mBuffer;
  qint64 mBufferSize;  // number of 'qint16' in the buffer

  int mNChannels;
  int mBitsPerSample;
  int mRate;

  bool mOwnBuffer;
};

#endif
