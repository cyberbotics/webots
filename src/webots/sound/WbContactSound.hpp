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

#ifndef WB_CONTACT_SOUND_HPP
#define WB_CONTACT_SOUND_HPP

//
// Description: manage a sound contact
//

#include <ode/contact.h>

#include <WbVector3.hpp>

class WbContactProperties;
class WbSoundClip;
class WbSoundSource;

class WbContactSound {
public:
  WbContactSound(const dGeomID &geom1, const dGeomID &geom2, const WbContactProperties *contactProperties);
  virtual ~WbContactSound();

  bool doesGeomsMatch(const dGeomID &geom1, const dGeomID &geom2) const;

  void newOdeContact(const dContactGeom &c);
  double lastContactTime() const { return mContactTime; }
  double lastUpdateTime() const { return mUpdateTime; }

  void finalizeContactUpdate();
  void updateSource();

private:
  void updateVelocities();

  enum Type { BUMP, ROLL, SLIDE, NONE };
  Type mType;

  dGeomID mGeom1;
  dGeomID mGeom2;
  dBodyID mBody1;
  dBodyID mBody2;

  double mContactTime;  // in seconds
  double mUpdateTime;   // in seconds

  double mEnergy;
  double mDerivativeEnergy;
  double mDoubleDerivativeEnergy;

  double mMass1;
  double mMass2;

  WbVector3 mContactPosition;
  WbVector3 mContactVelocity;

  WbVector3 mLinearVelocity1;
  WbVector3 mLinearVelocity2;
  WbVector3 mAngularVelocity1;
  WbVector3 mAngularVelocity2;

  WbSoundSource *mSource;

  const WbSoundClip *mBumpSoundClip;
  const WbSoundClip *mRollSoundClip;
  const WbSoundClip *mSlideSoundClip;
};

#endif
