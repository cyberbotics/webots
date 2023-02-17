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

#include "WbContactSound.hpp"

#include "WbAffinePlane.hpp"
#include "WbContactProperties.hpp"
#include "WbSimulationState.hpp"
#include "WbSoundClip.hpp"
#include "WbSoundEngine.hpp"
#include "WbSoundSource.hpp"
#include "WbStandardPaths.hpp"
#include "WbUrl.hpp"

#include <ode/ode.h>

#include <QtCore/qmath.h>

WbContactSound::WbContactSound(const dGeomID &geom1, const dGeomID &geom2, const WbContactProperties *contactProperties) :
  mType(NONE),
  mGeom1(geom1),
  mGeom2(geom2),
  mBody1(dGeomGetBody(geom1)),
  mBody2(dGeomGetBody(geom2)),
  mEnergy(0.0),
  mDerivativeEnergy(0.0),
  mDoubleDerivativeEnergy(0.0),
  mMass1(0.0),
  mMass2(0.0) {
  dMass mass;
  if (mBody1) {
    dBodyGetMass(mBody1, &mass);
    mMass1 = mass.mass;
  }
  if (mBody2) {
    dBodyGetMass(mBody2, &mass);
    mMass2 = mass.mass;
  }

  mSource = WbSoundEngine::createSource();

  if (contactProperties) {
    mBumpSoundClip = contactProperties->bumpSoundClip();
    mRollSoundClip = contactProperties->rollSoundClip();
    mSlideSoundClip = contactProperties->slideSoundClip();
  } else {
    mBumpSoundClip = NULL;
    mRollSoundClip = NULL;
    mSlideSoundClip = NULL;
  }

  mContactTime = 0.001 * WbSimulationState::instance()->time();
  mUpdateTime = 0.001 * WbSimulationState::instance()->time();
}

WbContactSound::~WbContactSound() {
  WbSoundEngine::deleteSource(mSource);
}

// cppcheck-suppress constParameter
bool WbContactSound::doesGeomsMatch(const dGeomID &geom1, const dGeomID &geom2) const {
  return ((geom1 == mGeom1 && geom2 == mGeom2) || (geom1 == mGeom2 && geom2 == mGeom1));
}

void WbContactSound::newOdeContact(const dContactGeom &c) {
  double deltaTime = 0.001 * WbSimulationState::instance()->time() - mUpdateTime;

  WbVector3 newContactPosition(c.pos[0], c.pos[1], c.pos[2]);

  if (deltaTime > 0) {  // avoid a division by 0
    WbVector3 newContactVelocity = (newContactPosition - mContactPosition) / deltaTime;
    mContactVelocity = newContactVelocity;
  }

  mContactPosition = newContactPosition;

  dVector3 v1;
  if (mBody1)
    dBodyGetPointVel(mBody1, c.pos[0], c.pos[1], c.pos[2], v1);
  else
    v1[0] = v1[1] = v1[2] = 0;

  dVector3 v2;
  if (mBody2)
    dBodyGetPointVel(mBody2, c.pos[0], c.pos[1], c.pos[2], v2);
  else
    v2[0] = v2[1] = v2[2] = 0;

  dVector3 dv;
  dSubtractVectors3(dv, v1, v2);

  double newEnergy = 0.5 * (mMass1 + mMass2) * dCalcVectorLengthSquare3(dv);  // Ec = 1/2 * mass * velocity^2

  if (deltaTime > 0) {  // avoid a division by 0
    double newDerivativeEnergy = (newEnergy - mEnergy) / deltaTime;
    double newDoubleDerivativeEnergy = (newDerivativeEnergy - mDerivativeEnergy) / deltaTime;

    mDerivativeEnergy = newDerivativeEnergy;
    mDoubleDerivativeEnergy = newDoubleDerivativeEnergy;
  }

  mEnergy = newEnergy;

  mContactTime = mUpdateTime = 0.001 * WbSimulationState::instance()->time();

  updateVelocities();
}

void WbContactSound::finalizeContactUpdate() {
  double deltaTime = 0.001 * WbSimulationState::instance()->time() - mUpdateTime;
  if (deltaTime > 0.0) {  // no collision this time
    double newEnergy = 0.0;
    double newDerivativeEnergy = (newEnergy - mEnergy) / deltaTime;
    double newDoubleDerivativeEnergy = (newDerivativeEnergy - mDerivativeEnergy) / deltaTime;

    mEnergy = newEnergy;
    mDerivativeEnergy = newDerivativeEnergy;
    mDoubleDerivativeEnergy = newDoubleDerivativeEnergy;

    mContactVelocity.setXyz(0.0, 0.0, 0.0);
    updateVelocities();

    mUpdateTime = 0.001 * WbSimulationState::instance()->time();
  }
}

void WbContactSound::updateVelocities() {
  if (mBody1) {
    const dReal *lv = dBodyGetLinearVel(mBody1);
    mLinearVelocity1.setXyz(lv[0], lv[1], lv[2]);

    const dReal *av = dBodyGetAngularVel(mBody1);
    mAngularVelocity1.setXyz(av[0], av[1], av[2]);
  } else {
    mLinearVelocity1.setXyz(0.0, 0.0, 0.0);
    mAngularVelocity1.setXyz(0.0, 0.0, 0.0);
  }

  if (mBody2) {
    const dReal *lv = dBodyGetLinearVel(mBody2);
    mLinearVelocity2.setXyz(lv[0], lv[1], lv[2]);

    const dReal *av = dBodyGetAngularVel(mBody2);
    mAngularVelocity2.setXyz(av[0], av[1], av[2]);
  } else {
    mAngularVelocity2.setXyz(0.0, 0.0, 0.0);
    mAngularVelocity2.setXyz(0.0, 0.0, 0.0);
  }
}

void WbContactSound::updateSource() {
  mSource->setPosition(mContactPosition);
  mSource->setVelocity(mContactVelocity);

  if (mSource->isPlaying())
    mType = NONE;

  if (mBumpSoundClip && mEnergy > 0.000001 && fabs(mDoubleDerivativeEnergy) > 1.0) {
    if (mType != BUMP)
      mSource->stop();

    if (!mSource->isPlaying()) {
      double gain = qBound(0.0, qSqrt(100 * mEnergy), 1.0);  // empirical function

      if (gain > 0.05) {  // optimisation: don't play too small noises
        mSource->setSoundClip(mBumpSoundClip);
        mSource->play();
        mSource->setGain(gain);
        mSource->setPitch(1.0);
        mType = BUMP;
      }
    }
  }

  if (mType == BUMP)
    return;

  static const double MATCHING_VELOCITY_TOLERANCE = 0.01;  // 1 %
  double bodiesRelativeVelocities = (mLinearVelocity2 - mLinearVelocity1).length();
  double contactVelocity = mContactVelocity.length();

  if (bodiesRelativeVelocities > 0.01 && bodiesRelativeVelocities < (1.0 + MATCHING_VELOCITY_TOLERANCE) * contactVelocity &&
      bodiesRelativeVelocities > (1.0 - MATCHING_VELOCITY_TOLERANCE) * contactVelocity) {
    WbVector3 projection =
      WbAffinePlane(mContactVelocity, mContactPosition).vectorProjection(mAngularVelocity1 + mAngularVelocity2);
    double projectionLength = projection.length();

    bool significantEnergy = mEnergy > 0.1;
    bool significantProjection = projectionLength > 0.1;

    if (mSlideSoundClip && significantEnergy && !significantProjection) {  // SLIDE

      if (mType != SLIDE)
        mSource->stop();

      static const double SLIDE_PITCH_INFLUENCY = 0.4;                                  // 40%
      double gain = qSqrt(qBound(0.0, contactVelocity, 1.0));                           // empirical function
      double pitch = 1.0 - 0.5 * SLIDE_PITCH_INFLUENCY + SLIDE_PITCH_INFLUENCY * gain;  // empirical function
      mSource->setGain(gain);
      mSource->setPitch(pitch);

      if (!mSource->isPlaying()) {
        if (gain > 0.05) {
          mSource->setSoundClip(mSlideSoundClip);
          mSource->play();
          mType = SLIDE;
        }
      }

    } else if (mRollSoundClip && !significantEnergy && significantProjection) {  // ROLL

      if (mType != ROLL)
        mSource->stop();

      static const double ROLL_PITCH_INFLUENCY = 0.3;                                 // 30%
      double gain = qBound(0.0, projectionLength / (5.0 * M_PI), 1.0);                // empirical function
      double pitch = 1.0 - 0.5 * ROLL_PITCH_INFLUENCY + ROLL_PITCH_INFLUENCY * gain;  // empirical function
      mSource->setGain(gain);
      mSource->setPitch(pitch);

      if (!mSource->isPlaying()) {
        if (gain > 0.05) {
          mSource->setSoundClip(mRollSoundClip);
          mSource->play();
          mType = ROLL;
        }
      }
    }
  } else {
    mSource->stop();
    mType = NONE;
  }
}
