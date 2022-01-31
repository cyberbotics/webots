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

#include "WbSoundSource.hpp"
#include "WbPreferences.hpp"

#include "WbSoundClip.hpp"
#include "WbVector3.hpp"

#include <AL/al.h>

WbSoundSource::WbSoundSource() {
  mOpenAL = WbPreferences::instance()->value("Sound/OpenAL").toBool();
  if (mOpenAL) {
    alGenSources(1, &mSource);
    alSourcei(mSource, AL_LOOPING, AL_FALSE);
  }
}

WbSoundSource::~WbSoundSource() {
  if (mOpenAL) {
    alSourceStop(mSource);
    alDeleteSources(1, &mSource);
  }
}

bool WbSoundSource::isPlaying() const {
  if (mOpenAL) {
    ALenum source_state;
    alGetSourcei(mSource, AL_SOURCE_STATE, &source_state);
    return (source_state == AL_PLAYING);
  } else
    return false;
}

bool WbSoundSource::isStopped() const {
  if (mOpenAL) {
    ALenum source_state;
    alGetSourcei(mSource, AL_SOURCE_STATE, &source_state);
    return (source_state == AL_STOPPED);
  } else
    return false;
}

bool WbSoundSource::isPaused() const {
  if (mOpenAL) {
    ALenum source_state;
    alGetSourcei(mSource, AL_SOURCE_STATE, &source_state);
    return (source_state == AL_PAUSED);
  } else
    return false;
}

void WbSoundSource::play() {
  if (mOpenAL)
    alSourcePlay(mSource);
}

void WbSoundSource::stop() {
  if (mOpenAL)
    alSourceStop(mSource);
}

void WbSoundSource::pause() {
  if (mOpenAL)
    alSourcePause(mSource);
}

void WbSoundSource::setLooping(bool loop) {
  if (mOpenAL) {
    if (loop)
      alSourcei(mSource, AL_LOOPING, AL_TRUE);
    else
      alSourcei(mSource, AL_LOOPING, AL_FALSE);
  }
}

void WbSoundSource::setSoundClip(const WbSoundClip *clip) {
  if (mOpenAL)
    alSourcei(mSource, AL_BUFFER, clip->openALBuffer());
}
void WbSoundSource::setPitch(double pitch) {
  if (mOpenAL)
    alSourcef(mSource, AL_PITCH, pitch);
}

void WbSoundSource::setGain(double gain) {
  if (mOpenAL)
    alSourcef(mSource, AL_GAIN, gain);
}

void WbSoundSource::setPosition(const WbVector3 &pos) {
  if (mOpenAL)
    alSource3f(mSource, AL_POSITION, pos.x(), pos.y(), pos.z());
}

void WbSoundSource::setVelocity(const WbVector3 &v) {
  if (mOpenAL)
    alSource3f(mSource, AL_VELOCITY, v.x(), v.y(), v.z());
}

void WbSoundSource::setDirection(const WbVector3 &dir) {
  if (mOpenAL)
    alSource3f(mSource, AL_DIRECTION, dir.x(), dir.y(), dir.z());
}
