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

#include "WbContactSoundManager.hpp"

#include "WbContactSound.hpp"
#include "WbOdeContact.hpp"
#include "WbSimulationState.hpp"

#include <QtCore/QCoreApplication>

QList<WbContactSound *> gContactSounds;

static WbContactSound *findContactFromGeoms(const dGeomID &geom1, const dGeomID &geom2) {
  foreach (WbContactSound *contactSound, gContactSounds) {
    if (contactSound->doesGeomsMatch(geom1, geom2))
      return contactSound;
  }
  return NULL;
}

static void newOdeContact(const WbOdeContact &odeContact) {
  WbContactSound *contactSound = findContactFromGeoms(odeContact.contactGeom().g1, odeContact.contactGeom().g2);

  if (contactSound == NULL) {
    contactSound = new WbContactSound(odeContact.contactGeom().g1, odeContact.contactGeom().g2, odeContact.contactProperties());
    gContactSounds << contactSound;
  }

  contactSound->newOdeContact(odeContact.contactGeom());
}

static void removeObsoleteContacts() {
  double currentTime = 0.001 * WbSimulationState::instance()->time();

  QMutableListIterator<WbContactSound *> it(gContactSounds);
  while (it.hasNext()) {
    WbContactSound *contactSound = it.next();
    if (contactSound->lastContactTime() + 0.05 < currentTime) {
      it.remove();
      delete contactSound;
    }
  }
}

void WbContactSoundManager::clearAllContactSoundSources() {
  qDeleteAll(gContactSounds);
  gContactSounds.clear();
}

void WbContactSoundManager::update(const QList<WbOdeContact> &odeContacts) {
  foreach (const WbOdeContact &odeContact, odeContacts)
    newOdeContact(odeContact);

  foreach (WbContactSound *contactSound, gContactSounds)
    contactSound->finalizeContactUpdate();

  removeObsoleteContacts();

  foreach (WbContactSound *contactSound, gContactSounds)
    contactSound->updateSource();
}
