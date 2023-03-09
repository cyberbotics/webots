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

#ifndef WB_ODE_CONTACT
#define WB_ODE_CONTACT

#include <ode/ode.h>

class WbContactProperties;

class WbOdeContact {
public:
  WbOdeContact(const dContactGeom &geom, const WbContactProperties *contactProperties) :
    mContactGeom(geom),
    mContactProperties(contactProperties) {}
  WbOdeContact(const WbOdeContact &other) : mContactGeom(other.mContactGeom), mContactProperties(other.mContactProperties) {}
  WbOdeContact &operator=(const WbOdeContact &arg) {
    (dContactGeom) mContactGeom = arg.mContactGeom;
    mContactProperties = arg.mContactProperties;
    return *this;
  }
  const dContactGeom &contactGeom() const { return mContactGeom; }
  const WbContactProperties *contactProperties() const { return mContactProperties; }

private:
  const dContactGeom mContactGeom;
  const WbContactProperties *mContactProperties;
};

#endif
