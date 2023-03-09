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

#include "WbMassChecker.hpp"

#include "WbLog.hpp"
#include "WbSolid.hpp"

#include <QtCore/QCoreApplication>

#include <limits>

WbMassChecker *WbMassChecker::cInstance = NULL;

WbMassChecker *WbMassChecker::instance() {
  if (cInstance == NULL) {
    cInstance = new WbMassChecker;
    qAddPostRoutine(WbMassChecker::cleanup);
  }
  return cInstance;
}

void WbMassChecker::cleanup() {
  delete cInstance;
  cInstance = NULL;
}

void WbMassChecker::checkMasses() {
  double min = std::numeric_limits<double>::infinity();
  double max = 0.0;
  const WbSolid *heaviestSolid = NULL;
  const WbSolid *lightestSolid = NULL;

  foreach (const WbSolid *const solid, WbSolid::solids()) {
    double mass = solid->mass();

    if (mass <= 0.0)  // static object
      continue;

    if (mass > max) {
      max = mass;
      heaviestSolid = solid;
    }
    if (mass < min) {
      min = mass;
      lightestSolid = solid;
    }
  }

  if (lightestSolid != NULL && heaviestSolid != NULL) {
    // this threshold was determined on these 2 facts:
    // - issues have been detected on an Asimo example where the ratio is 1e6
    // - a regular humanoid model has a ratio of about 1e3 (body - phalange). A safety marge is kept
    const double ratioThreshold = 1e5;

    double ratio = max / min;
    if (ratio > ratioThreshold) {
      WbLog::warning(QObject::tr("The mass ratio between the heaviest solid ('%1' - mass = %2 kg) and the lightest solid ('%3' "
                                 "- mass = %4 kg) is huge. "
                                 "This is a source of physics instabilities.")
                       .arg(heaviestSolid->usefulName())
                       .arg(max)
                       .arg(lightestSolid->usefulName())
                       .arg(min));
      heaviestSolid->warn(QObject::tr("heaviest solid"));
      lightestSolid->warn(QObject::tr("lightest solid"));
    }
  }
}
