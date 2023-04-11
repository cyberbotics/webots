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

#include "WbMotorSoundManager.hpp"

#include "WbJoint.hpp"
#include "WbMotor.hpp"
#include "WbSolid.hpp"
#include "WbSoundEngine.hpp"
#include "WbSoundSource.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QMap>

#include <cassert>

static QMap<const WbMotor *, WbSoundSource *> gMotorSoundSources;

void WbMotorSoundManager::clearAllMotorSoundSources() {
  foreach (WbSoundSource *source, gMotorSoundSources)
    WbSoundEngine::deleteSource(source);
  gMotorSoundSources.clear();
}

void WbMotorSoundManager::update() {
  foreach (const WbMotor *motor, WbMotor::motors()) {
    const WbSoundClip *sound = motor->soundClip();
    if (!sound)
      continue;
    WbSoundSource *source = NULL;
    if (gMotorSoundSources.contains(motor))
      source = gMotorSoundSources[motor];
    else {
      source = WbSoundEngine::createSource();
      gMotorSoundSources[motor] = source;
    }
    assert(source);

    double velocityCoefficient = qBound(0.0, fabs(motor->currentVelocity() / motor->maxVelocity()), 1.0);

    if (velocityCoefficient > 0.1) {
      if (!source->isPlaying()) {
        source->setSoundClip(sound);
        source->play();
        source->setLooping(false);
      }

      const WbJoint *joint = motor->joint();
      if (joint) {
        const WbSolid *solid = joint->solidParent();
        if (solid) {
          source->setPosition(solid->position());
          source->setVelocity(solid->linearVelocity());
        }
      }

      static const double GAIN_INFLUENCY = 0.1;                                   // 10%
      double gain = 1.0 - GAIN_INFLUENCY + GAIN_INFLUENCY * velocityCoefficient;  // empirical

      static const double PITCH_INFLUENCY = 0.8;                                           // 80%
      double pitch = 1.0 - 0.5 * PITCH_INFLUENCY + PITCH_INFLUENCY * velocityCoefficient;  // empirical

      source->setPitch(pitch);
      source->setGain(gain);
    } else if (source->isPlaying())
      source->stop();
  }
}
