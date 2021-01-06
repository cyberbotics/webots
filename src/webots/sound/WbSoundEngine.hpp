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

#ifndef WB_SOUND_ENGINE_HPP
#define WB_SOUND_ENGINE_HPP

//
// Description: manage sounds
//

class WbWorld;
class QString;
class WbSoundClip;
class WbSoundSource;

namespace WbSoundEngine {
  void setWorld(WbWorld *world);
  void setMute(bool mute);
  void setVolume(int volume);
  void setPause(bool pause);
  void updateAfterPhysicsStep();
  void updateListener();
  WbSoundSource *createSource();
  void deleteSource(WbSoundSource *);
  void stopAllSources();
  // 0: both sides, -1: left only, 1: right only
  WbSoundClip *sound(const QString &filename, double balance = 0.0, int side = 0);
  WbSoundClip *soundFromText(const QString &text, const QString &engine, const QString &language);
  void clearAllMotorSoundSources();
  void clearAllContactSoundSources();
};  // namespace WbSoundEngine

#endif
