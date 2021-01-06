// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbSolidUtilities.hpp"

#include "WbBox.hpp"
#include "WbCapsule.hpp"
#include "WbCylinder.hpp"
#include "WbElevationGrid.hpp"
#include "WbIndexedFaceSet.hpp"
#include "WbSphere.hpp"
#include "WbTransform.hpp"

#include <ode/ode.h>

#include <assert.h>

// The ODE interface does not allow to simply cast a dGeomID in a dSpaceID although dSpace inherits from dGeom; this forces the
// use of a reinterpret_cast Note: this is possible because the internal stucture dxSpace inherits of dxGeom
dSpaceID WbSolidUtilities::dynamicCastInSpaceID(dGeomID g) {
  if (g == NULL || !dGeomIsSpace(g))
    return NULL;

  return reinterpret_cast<dSpaceID>(g);
}

void WbSolidUtilities::subtractInertiaMatrix(double *I, const double *J) {
  I[0] -= J[0];
  I[1] -= J[1];
  I[2] -= J[2];
  I[5] -= J[5];
  I[6] -= J[6];
  I[10] -= J[10];
  I[4] = I[1];
  I[8] = I[2];
  I[9] = I[6];
}

void WbSolidUtilities::setDefaultMass(dMass *m) {
  dMassSetParameters(m, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
}

// The mass is supposed to be homogeneously spread over the body boundingObject
void WbSolidUtilities::addMass(dMass *const mass, WbNode *const node, double density, bool warning) {
  const WbShape *const shape = dynamic_cast<WbShape *>(node);
  if (shape) {
    if (shape->geometry() == NULL)
      return;
    addMass(mass, shape->geometry(), density, warning);
    return;
  }

  dMass m;
  dMassSetZero(&m);

  // The WbTransform case must come before the WbGroup case
  const WbTransform *const transform = dynamic_cast<WbTransform *>(node);
  if (transform) {
    WbGeometry *geometry = transform->geometry();

    // Computes the total mass
    if (geometry && geometry->odeGeom())
      addMass(&m, geometry, density, warning);
    else
      // invalid geometry
      return;

    // Rotates the inertia matrix
    const WbRotation &r = transform->rotation();
    dMatrix3 m3;
    dRFromAxisAndAngle(m3, r.x(), r.y(), r.z(), r.angle());
    dMassRotate(&m, m3);

    // Translates the inertia matrix
    WbVector3 t = transform->translation();
    t *= transform->upperTransform()->absoluteScale().x();
    dMassTranslate(&m, t.x(), t.y(), t.z());
    dMassAdd(mass, &m);

    geometry->setOdeMass(&m);
    return;
  }

  // The case of a WbGroup which is NOT a WbTransform
  const WbGroup *const group = dynamic_cast<WbGroup *>(node);
  if (group) {
    WbMFNode::Iterator it(group->children());
    while (it.hasNext())
      addMass(&m, it.next(), density, warning);
    if (m.mass > 0.0)
      dMassAdd(mass, &m);
    return;
  }

  static const QString defaultValues(
    QObject::tr(": the corresponding added mass defaults to 1kg, the added inertia matrix defaults to the identity matrix."));

  WbSphere *const sphere = dynamic_cast<WbSphere *>(node);
  if (sphere) {
    const double radius = sphere->scaledRadius();
    if (radius <= 0.0) {
      setDefaultMass(&m);
      if (warning)
        sphere->parsingWarn(QObject::tr("Use a positive radius to allow proper mass settings") + defaultValues);
    } else
      dMassSetSphere(&m, density, radius);
    dMassAdd(mass, &m);
    // stores mass info in order to correct the global mass after the destruction of a bounding WbGeometry
    sphere->setOdeMass(&m);
    return;
  }

  WbCylinder *const cylinder = dynamic_cast<WbCylinder *>(node);
  if (cylinder) {
    const double radius = cylinder->scaledRadius();
    const double height = cylinder->scaledHeight();
    if (radius <= 0.0 || height <= 0.0) {
      setDefaultMass(&m);
      if (warning)
        cylinder->parsingWarn(QObject::tr("Use a positive radius and a positive height to allow proper mass settings") +
                              defaultValues);
    } else
      dMassSetCylinder(&m, density, 2, radius, height);  // 2 -> y-axis
    dMassAdd(mass, &m);
    cylinder->setOdeMass(&m);
    return;
  }

  WbCapsule *const capsule = dynamic_cast<WbCapsule *>(node);
  if (capsule) {
    const double radius = capsule->scaledRadius();
    const double height = capsule->scaledHeight();
    if (radius <= 0.0 || height <= 0.0) {
      setDefaultMass(&m);
      if (warning)
        capsule->parsingWarn(QObject::tr("Use a positive radius and a positive height to allow proper mass settings") +
                             defaultValues);
    } else
      dMassSetCapsule(&m, density, 2, radius, height);  // 2 -> y-axis
    dMassAdd(mass, &m);
    capsule->setOdeMass(&m);
    return;
  }

  WbBox *const box = dynamic_cast<WbBox *>(node);
  if (box) {
    const WbVector3 size = box->scaledSize();
    if (size.x() <= 0.0 || size.y() <= 0.0 || size.z() <= 0.0) {
      setDefaultMass(&m);
      if (warning)
        box->parsingWarn(QObject::tr("Use positive dimensions to allow proper mass settings") + defaultValues);
    } else
      dMassSetBox(&m, density, size.x(), size.y(), size.z());
    dMassAdd(mass, &m);
    box->setOdeMass(&m);
    return;
  }

  WbIndexedFaceSet *const ifs = dynamic_cast<WbIndexedFaceSet *>(node);
  if (ifs) {
    dGeomID g = ifs->odeGeom();
    // The trimesh failed to build, probably because of invalid faces
    if (g == NULL) {
      if (warning)
        ifs->parsingInfo(
          QObject::tr("The creation of the IndexedFaceSet physical boundaries failed because its geometry is not "
                      "suitable for representing a bounded closed volume") +
          defaultValues);
      setDefaultMass(&m);
      return;
    }

    assert(dGeomGetClass(g) == dTriMeshClass);

    // Computes the inertia matrix of the trimesh
    dMassSetTrimesh(&m, density, g);
    if (m.mass <= 0.0 || !dIsPositiveDefinite(m.I, 3)) {
      setDefaultMass(&m);
      if (warning)
        ifs->parsingWarn(
          QObject::tr("Mass properties computation failed for this IndexedFaceSet") + defaultValues +
          QObject::tr("Please check this geometry has no singularities and can suitably represent a bounded closed volume. "
                      "Note in particular that every triangle should appear only once with its 'outward' orientation."));
    }
    dMassAdd(mass, &m);
    ifs->setOdeMass(&m);

    return;
  }

  const WbElevationGrid *const elevationGrid = dynamic_cast<WbElevationGrid *>(node);
  if (elevationGrid) {
    elevationGrid->parsingWarn(QObject::tr("Webots cannot compute inertia for ElevationGrid objects."));
    return;
  }
}

// Recursive method that checks the validity of a boundingObject: USED FOR DEBUG ONLY
// (Note: such check is already done during the contruction process which issues appropriate warnings)
bool WbSolidUtilities::checkBoundingObject(WbNode *const node) {
  if (node == NULL)
    return false;

  const WbTransform *const transform = dynamic_cast<WbTransform *>(node);
  // cppcheck-suppress knownConditionTrueFalse
  if (transform) {
    WbNode *child = transform->child(0);
    if (child == NULL) {
      node->parsingWarn(
        QObject::tr("Invalid 'boundingObject' (a Transform has no 'geometry'): the inertia matrix cannot be calculated."));
      return false;
    }

    const WbGeometry *const geometry = dynamic_cast<WbGeometry *>(child);
    // cppcheck-suppress knownConditionTrueFalse
    if (geometry)
      return true;

    const WbShape *const shape = dynamic_cast<WbShape *>(child);
    if (shape == NULL || shape->geometry() == NULL) {
      node->parsingWarn(
        QObject::tr("Invalid 'boundingObject' (a Transform, or a Shape within a Transform, has no 'geometry'): the "
                    "inertia matrix cannot be calculated."));
      return false;
    }
  }

  const WbShape *const shape = dynamic_cast<WbShape *>(node);
  if (shape && shape->geometry() == NULL) {
    node->parsingWarn(QObject::tr(
      "Invalid 'boundingObject' (a Shape's 'geometry' field is not set): the inertia matrix cannot be calculated."));
    return false;
  }

  const WbGroup *const group = dynamic_cast<WbGroup *>(node);
  if (group) {
    const WbMFNode &children = group->children();
    const int size = children.size();
    if (size == 0) {
      node->parsingWarn(
        QObject::tr("Invalid 'boundingObject' (Group with no child set): the inertia matrix cannot be calculated."));
      return false;
    }
    for (int i = 0; i < size; ++i) {
      if (!checkBoundingObject(children.item(i)))
        return false;
    }
  }

  return true;
}

WbGeometry *WbSolidUtilities::geometry(WbNode *const node) {
  if (!node)
    return NULL;

  WbGeometry *const geometry = dynamic_cast<WbGeometry *>(node);
  // cppcheck-suppress knownConditionTrueFalse
  if (geometry)
    return geometry;

  const WbShape *const shape = dynamic_cast<WbShape *>(node);
  if (shape) {
    if (shape->geometry() == NULL)
      shape->parsingInfo(QObject::tr("Please specify 'geometry' (of Shape placed in 'boundingObject')."));

    return shape->geometry();
  }

  node->parsingWarn(QObject::tr("Ignoring illegal \"%1\" node in 'boundingObject'.").arg(node->usefulName()));
  return NULL;
}

bool WbSolidUtilities::isPermanentlyKinematic(const WbNode *node) {
  const WbBaseNode *const p = node ? static_cast<WbBaseNode *>(node->parentNode()) : NULL;
  return p && p->nodeType() == WB_NODE_PROPELLER;
}
