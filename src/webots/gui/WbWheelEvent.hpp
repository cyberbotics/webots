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

#ifndef WB_WHEEL_EVENT_HPP
#define WB_WHEEL_EVENT_HPP

//
// Description: classes allowing to store data related with different kinds of mouse wheeling events
//               (used so far only for for wheeling event impacting the 3D view, i.e., WbWheelLiftSolidEvent)
//

#include "WbVector3.hpp"

class WbViewpoint;
class WbSolid;

// WbWheelEvent class
class WbWheelEvent {
public:
  virtual ~WbWheelEvent() {}
  virtual void apply(int delta) = 0;

protected:
  WbWheelEvent();
};

// WbWheelLiftSolidEvent class
class WbWheelLiftSolidEvent : public WbWheelEvent {
public:
  WbWheelLiftSolidEvent(WbViewpoint *viewpoint, WbSolid *selectedSolid);
  virtual ~WbWheelLiftSolidEvent();
  void apply(int delta) override;

private:
  WbViewpoint *mViewpoint;
  WbSolid *mSelectedSolid;
  double mScaleFactor;
  WbVector3 mUpWorldVector;
  WbVector3 mInitialTranslation;
};

#endif
