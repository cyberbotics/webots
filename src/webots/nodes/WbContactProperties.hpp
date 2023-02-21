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

#ifndef WB_CONTACT_PROPERTIES_HPP
#define WB_CONTACT_PROPERTIES_HPP

#include "WbBaseNode.hpp"
#include "WbMFDouble.hpp"
#include "WbSFDouble.hpp"
#include "WbSFInt.hpp"
#include "WbSFString.hpp"
#include "WbSFVector2.hpp"
#include "WbSFVector3.hpp"

class WbSoundClip;
class WbDownloader;

class WbContactProperties : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbContactProperties(WbTokenizer *tokenizer = NULL);
  WbContactProperties(const WbContactProperties &other);
  explicit WbContactProperties(const WbNode &other);
  virtual ~WbContactProperties();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_CONTACT_PROPERTIES; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;

  // field accessors
  const QString &material1() const { return mMaterial1->value(); }
  const QString &material2() const { return mMaterial2->value(); }
  int coulombFrictionSize() const { return mCoulombFriction->size(); }
  double coulombFriction(int index) const { return mCoulombFriction->item(index); }
  WbVector2 frictionRotation() const { return mFrictionRotation->value(); }
  WbVector3 rollingFriction() const { return mRollingFriction->value(); }
  double bounce() const { return mBounce->value(); }
  double bounceVelocity() const { return mBounceVelocity->value(); }
  int forceDependentSlipSize() const { return mForceDependentSlip->size(); }
  double forceDependentSlip(int index) const { return mForceDependentSlip->item(index); }
  double softERP() const { return mSoftErp->value(); }
  double softCFM() const { return mSoftCfm->value(); }
  const WbSoundClip *bumpSoundClip() const { return mBumpSoundClip; }
  const WbSoundClip *rollSoundClip() const { return mRollSoundClip; }
  const WbSoundClip *slideSoundClip() const { return mSlideSoundClip; }
  int maxContactJoints() const { return mMaxContactJoints->value(); }

signals:
  void valuesChanged();
  void needToEnableBodies();

private:
  // user accessible fields
  WbSFString *mMaterial1;
  WbSFString *mMaterial2;
  WbMFDouble *mCoulombFriction;
  WbSFVector2 *mFrictionRotation;
  WbSFVector3 *mRollingFriction;
  WbSFDouble *mBounce;
  WbSFDouble *mBounceVelocity;
  WbMFDouble *mForceDependentSlip;
  WbSFDouble *mSoftCfm;
  WbSFDouble *mSoftErp;
  WbSFString *mBumpSound;
  WbSFString *mRollSound;
  WbSFString *mSlideSound;
  WbSFInt *mMaxContactJoints;
  const WbSoundClip *mBumpSoundClip;
  const WbSoundClip *mRollSoundClip;
  const WbSoundClip *mSlideSoundClip;
  WbDownloader *mDownloader[3];
  WbContactProperties &operator=(const WbContactProperties &);  // non copyable
  WbNode *clone() const override { return new WbContactProperties(*this); }
  void init();
  void downloadAsset(const QString &url, int index);
  void loadSound(int index, const QString &sound, const QString &name, const WbSoundClip **clip);

private slots:
  void updateCoulombFriction();
  void updateFrictionRotation();
  void updateRollingFriction();
  void updateBounce();
  void updateBounceVelocity();
  void updateSoftCfm();
  void updateSoftErp();
  void updateBumpSound();
  void updateRollSound();
  void updateSlideSound();
  void updateForceDependentSlip();
  void enableBodies();
  void updateMaxContactJoints();
};

#endif
