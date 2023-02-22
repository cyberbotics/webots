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

#include "WbSoundSource.hpp"

#include "WbSoundClip.hpp"
#include "WbSoundEngine.hpp"
#include "WbVector3.hpp"

#ifdef __APPLE__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <OpenAL/al.h>
#else
#include <AL/al.h>
#endif

WbSoundSource::WbSoundSource() {
  if (!WbSoundEngine::openAL())
    return;
  alGenSources(1, &mSource);
  alSourcei(mSource, AL_LOOPING, AL_FALSE);
}

WbSoundSource::~WbSoundSource() {
  if (!WbSoundEngine::openAL())
    return;
  alSourceStop(mSource);
  alDeleteSources(1, &mSource);
}

bool WbSoundSource::isPlaying() const {
  if (!WbSoundEngine::openAL())
    return false;
  ALenum source_state;
  alGetSourcei(mSource, AL_SOURCE_STATE, &source_state);
  return (source_state == AL_PLAYING);
}

bool WbSoundSource::isStopped() const {
  if (!WbSoundEngine::openAL())
    return false;
  ALenum source_state;
  alGetSourcei(mSource, AL_SOURCE_STATE, &source_state);
  return (source_state == AL_STOPPED);
}

bool WbSoundSource::isPaused() const {
  if (!WbSoundEngine::openAL())
    return false;
  ALenum source_state;
  alGetSourcei(mSource, AL_SOURCE_STATE, &source_state);
  return (source_state == AL_PAUSED);
}

void WbSoundSource::play() {
  if (!WbSoundEngine::openAL())
    return;
  alSourcePlay(mSource);
}

void WbSoundSource::stop() {
  if (!WbSoundEngine::openAL())
    return;
  alSourceStop(mSource);
}

void WbSoundSource::pause() {
  if (!WbSoundEngine::openAL())
    return;
  alSourcePause(mSource);
}

void WbSoundSource::setLooping(bool loop) {
  if (!WbSoundEngine::openAL())
    return;
  if (loop)
    alSourcei(mSource, AL_LOOPING, AL_TRUE);
  else
    alSourcei(mSource, AL_LOOPING, AL_FALSE);
}

void WbSoundSource::setSoundClip(const WbSoundClip *clip) {
  if (!WbSoundEngine::openAL())
    return;
  alSourcei(mSource, AL_BUFFER, clip->openALBuffer());
}

void WbSoundSource::setPitch(double pitch) {
  if (!WbSoundEngine::openAL())
    return;
  alSourcef(mSource, AL_PITCH, pitch);
}

void WbSoundSource::setGain(double gain) {
  if (!WbSoundEngine::openAL())
    return;
  alSourcef(mSource, AL_GAIN, gain);
}

void WbSoundSource::setPosition(const WbVector3 &pos) {
  if (!WbSoundEngine::openAL())
    return;
  alSource3f(mSource, AL_POSITION, pos.x(), pos.y(), pos.z());
}

void WbSoundSource::setVelocity(const WbVector3 &v) {
  if (!WbSoundEngine::openAL())
    return;
  alSource3f(mSource, AL_VELOCITY, v.x(), v.y(), v.z());
}

void WbSoundSource::setDirection(const WbVector3 &dir) {
  if (!WbSoundEngine::openAL())
    return;
  alSource3f(mSource, AL_DIRECTION, dir.x(), dir.y(), dir.z());
}

#ifdef __APPLE__
#pragma GCC diagnostic pop
#endif
