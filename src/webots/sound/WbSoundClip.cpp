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

#include "WbSoundClip.hpp"

#include "WbWaveFile.hpp"

#include <QtCore/QObject>

#include <AL/al.h>

WbSoundClip::WbSoundClip() : mFilename(), mBuffer(0), mSide(0), mBalance(0.0) {
}

WbSoundClip::~WbSoundClip() {
  if (mBuffer)
    alDeleteBuffers(1, &mBuffer);
}

void WbSoundClip::load(const QString &filename, double balance, int side) {
  WbWaveFile wave(filename);
  wave.loadFromFile(side);
  if (wave.nChannels() > 1)
    wave.convertToMono(balance);
  mFilename = wave.filename();
  mSide = side;
  mBalance = balance;
  load(&wave);
}

void WbSoundClip::load(const WbWaveFile *wave) {
  ALuint buffer = 0;

  try {
    ALenum format = 0;
    if (wave->nChannels() == 1 && wave->bitsPerSample() == 8)
      format = AL_FORMAT_MONO8;
    else if (wave->nChannels() == 1 && wave->bitsPerSample() == 16)
      format = AL_FORMAT_MONO16;
    else if (wave->nChannels() == 2 && wave->bitsPerSample() == 8)
      format = AL_FORMAT_STEREO8;
    else if (wave->nChannels() == 2 && wave->bitsPerSample() == 16)
      format = AL_FORMAT_STEREO16;
    else
      throw QObject::tr("Unknown WAVE format");

    ALsizei rate = wave->rate();
    ALsizei bufferSize = wave->bufferSize() * sizeof(qint16);

    alGenBuffers(1, &buffer);
    alBufferData(buffer, format, wave->buffer(), bufferSize, rate);

  } catch (const QString &e) {
    if (buffer != 0)
      alDeleteBuffers(1, &buffer);

    throw;  // throw up
  }

  mBuffer = buffer;
}
