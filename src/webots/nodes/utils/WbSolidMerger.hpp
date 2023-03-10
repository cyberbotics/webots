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

#ifndef WB_SOLID_MERGER_HPP
#define WB_SOLID_MERGER_HPP

#include "WbOdeTypes.hpp"
#include "WbVector3.hpp"

#include <QtCore/QMap>
#include <QtCore/QObject>

class WbMatrix4;
class WbSolid;
struct dMass;

class WbSolidMerger : public QObject {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbSolidMerger(WbSolid *solid);
  virtual ~WbSolidMerger();

  WbSolid *solid() const { return mSolid; }
  dBodyID body() const { return mBody; }
  dSpaceID space() const { return mSpace; }
  dSpaceID reservedSpace();
  void removeExtraSpace();
  const WbVector3 &centerOfMass() const { return mCenterOfMass; }
  const WbVector3 &absoluteCenterOfMass() const { return mAbsoluteCenterOfMass; }
  const QMap<WbSolid *, dMass *> &mergedSolids() const;
  void appendSolid(WbSolid *solid);
  void attachGeomsToBody(dGeomID g);
  void removeSolid(WbSolid *solid);
  void mergeMass(WbSolid *const solid, bool subtract = true);
  void addGeomToSpace(dGeomID g);
  void setGeomAndBodyPositions(bool zeroVelocities = false, bool resetJoints = false);
  void setupOdeBody();
  void updateMasses();
  bool isSet() const;

  void setBodyArtificiallyDisabled(bool disabled);
  bool isBodyArtificiallyDisabled() const { return mBodyArtificiallyDisabled; }

public slots:
  void setOdeDamping();

private:
  WbSolidMerger(const WbSolidMerger &other);
  WbSolidMerger &operator=(const WbSolidMerger &other);
  WbSolid *mSolid;
  dSpaceID mSpace;
  WbVector3 mCenterOfMass;
  WbVector3 mAbsoluteCenterOfMass;
  void updateCenterOfMass();
  dMass *mOdeMass;
  dBodyID mBody;
  QMap<WbSolid *, dMass *> mMergedSolids;
  bool mBodyArtificiallyDisabled;

  void addMassToBody();
  void mergeMasses();
  void setGeomOffsetPositions();
  void subtractSolidMass(WbSolid *solid);
  void transformMass(WbSolid *const solid, const WbMatrix4 &m4) const;
  void transformMass(WbSolid *const solid) const;
  void transformMasses() const;
  void reserveSpace();
  WbMatrix4 inverseMatrix() const;

private slots:
  void setOdeAutoDisable();
};
#endif
