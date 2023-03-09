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

#ifndef WB_ODE_GEOM_DATA
#define WB_ODE_GEOM_DATA

class WbFluid;
class WbSolid;
class WbGeometry;

class WbOdeGeomData {
public:
  // constructors and destructor
  explicit WbOdeGeomData(WbSolid *solid, WbGeometry *geometry = NULL) :
    mFluid(NULL),
    mSolid(solid),
    mGeometry(geometry),
    mLastChangeTime(0.0),
    mEnableForContactPoint(false),
    mMagicNumber(0x7765626F7473LL) {}
  explicit WbOdeGeomData(WbFluid *fluid, WbGeometry *geometry = NULL) :
    mFluid(fluid),
    mSolid(NULL),
    mGeometry(geometry),
    mLastChangeTime(0.0),
    mEnableForContactPoint(false),
    mMagicNumber(0x7765626F7473LL) {}
  virtual ~WbOdeGeomData() {}

  // field accessors
  WbSolid *solid() const { return mSolid; }
  WbFluid *fluid() const { return mFluid; }
  WbGeometry *geometry() const { return mGeometry; }
  double lastChangeTime() const { return mLastChangeTime; }
  void setLastChangeTime(double time) { mLastChangeTime = time; }
  bool enableForContactPoint() const { return mEnableForContactPoint; }
  void setEnableForContactPoint(bool enable) { mEnableForContactPoint = enable; }
  const long long int &magicNumber() const { return mMagicNumber; }

private:
  WbFluid *mFluid;
  WbSolid *mSolid;
  WbGeometry *mGeometry;
  double mLastChangeTime;
  bool mEnableForContactPoint;
  const long long int mMagicNumber;  // this number allows to distinguish WbOdeGeomData from users' own data put into an ODE
                                     // dGeomID (in "webots" replace each character by its hex. ascii)
};

#endif
