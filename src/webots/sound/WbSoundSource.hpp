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

#ifndef WB_SOUND_SOURCE_HPP
#define WB_SOUND_SOURCE_HPP

//
// Description: manage a sound buffer
//

class WbSoundClip;
class WbVector3;

class WbSoundSource {
public:
  WbSoundSource();
  virtual ~WbSoundSource();

  bool isPlaying() const;
  bool isStopped() const;
  bool isPaused() const;

  void play();
  void stop();
  void pause();

  void setLooping(bool loop);
  void setSoundClip(const WbSoundClip *clip);
  void setPitch(double pitch);
  void setGain(double gain);

  void setPosition(const WbVector3 &pos);
  void setVelocity(const WbVector3 &v);
  void setDirection(const WbVector3 &dir);

private:
  unsigned int mSource;
};

#endif
