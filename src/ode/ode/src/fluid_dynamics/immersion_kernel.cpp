/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

/*

core immersion functions and data structures, plus part of the public API
for geometry objects

*/

#include <ode/fluid_dynamics/immersion.h>
#include <ode/fluid_dynamics/fluid_dynamics.h>
#include "immersion_kernel.h"
#include "immersion_std.h"
#include "immersion_transform.h"
#include <ode/matrix.h>
#include <ode/odemath.h>
#include "../collision_kernel.h"
#include "../collision_std.h"
#include "../collision_util.h"
#include "../collision_trimesh_internal.h"
#include "../collision_space_internal.h"

#ifdef _MSC_VER
#pragma warning(disable:4291)  // for VC++, no complaints about "no matching operator delete found"
#endif

//****************************************************************************
// dispatcher for the N^2 fluids immersion functions

// function pointers and modes for N^2 class immerser functions

static dImmerserFn *immersers[dGeomNumClasses][dGeomNumClasses];
static int immersers_initialized = 0;

//****************************************************************************
// helper functions for dImmerse()ing a space with another geom

// setImmerser() will refuse to write over a collider entry once it has
// been written.

static void setImmerser (int i, int j, dImmerserFn *fn)
{
  if (immersers[i][j] == 0) {
    immersers[i][j] = fn;
  }
}

static void setAllImmersers (int i, dImmerserFn *fn)
{
  for (int j = 0; j < dGeomNumClasses; ++j) {
    setImmerser (i,j,fn);
    setImmerser (j,i,fn);
  }
}

/*extern */void dInitImmersers()
{
  dIASSERT(!immersers_initialized);
  immersers_initialized = 1;

  memset (immersers,0,sizeof(immersers));

  setImmerser (dBoxClass, dBoxClass, &dImmerseBoxBox);
  setImmerser (dBoxClass, dCapsuleClass, &dImmerseBoxCapsule);
  setImmerser (dBoxClass, dCylinderClass, &dImmerseBoxCylinder);
  setImmerser (dBoxClass, dPlaneClass, &dImmerseBoxPlane);
  setImmerser (dBoxClass, dSphereClass, &dImmerseBoxSphere);

  setImmerser (dCapsuleClass, dBoxClass, &dImmerseCapsuleBox);
  setImmerser (dCapsuleClass, dCapsuleClass, &dImmerseCapsuleCapsule);
  setImmerser (dCapsuleClass, dCylinderClass, &dImmerseCapsuleCylinder);
  setImmerser (dCapsuleClass, dPlaneClass, &dImmerseCapsulePlane);
  setImmerser (dCapsuleClass, dSphereClass, &dImmerseCapsuleSphere);

  setImmerser (dCylinderClass, dBoxClass, &dImmerseCylinderBox);
  setImmerser (dCylinderClass, dCapsuleClass,&dImmerseCylinderCapsule);
  setImmerser (dCylinderClass, dCylinderClass,&dImmerseCylinderCylinder);
  setImmerser (dCylinderClass, dPlaneClass,&dImmerseCylinderPlane);

  setImmerser (dSphereClass, dBoxClass,&dImmerseSphereBox);
  setImmerser (dSphereClass, dCapsuleClass, &dImmerseSphereCapsule);
  setImmerser (dSphereClass, dCylinderClass, &dImmerseSphereCylinder);
  setImmerser (dSphereClass, dPlaneClass, &dImmerseSpherePlane);
  setImmerser (dSphereClass, dSphereClass, &dImmerseSphereSphere);

#if dTRIMESH_ENABLED
  setImmerser (dBoxClass, dTriMeshClass, &dImmerseBoxTrimesh);
  setImmerser (dCapsuleClass, dTriMeshClass, &dImmerseCapsuleTrimesh);
  setImmerser (dCylinderClass, dTriMeshClass, &dImmerseCylinderTrimesh);
  setImmerser (dCylinderClass, dSphereClass, &dImmerseCylinderSphere);
  setImmerser (dSphereClass, dTriMeshClass, &dImmerseSphereTrimesh);

  setImmerser (dTriMeshClass, dBoxClass, &dImmerseTrimeshBox);
  setImmerser (dTriMeshClass, dCapsuleClass, &dImmerseTrimeshCapsule);
  setImmerser (dTriMeshClass, dCylinderClass, &dImmerseTrimeshCylinder);
  setImmerser (dTriMeshClass, dPlaneClass, &dImmerseTrimeshPlane);
  setImmerser (dTriMeshClass, dSphereClass, &dImmerseTrimeshSphere);
  setImmerser (dTriMeshClass, dTriMeshClass, &dImmerseTrimeshTrimesh);
#endif

  setAllImmersers (dGeomTransformClass,&dImmerseTransform);
}

/*extern */void dFinitImmersers()
{
  immersers_initialized = 0;
}

void dSetImmerserOverride (int i, int j, dImmerserFn *fn)
{
 dIASSERT( immersers_initialized );
 dAASSERT( i < dGeomNumClasses );
 dAASSERT( j < dGeomNumClasses );

 immersers[i][j] = fn;
}

int dImmerse (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dAASSERT(o1 && o2 && immersion);
  dUASSERT(immersers_initialized,"Please call ODE initialization (dInitODE() or similar) before using the library");
  dUASSERT(o1->type >= 0 && o1->type < dGeomNumClasses,"bad o1 class number");
  dUASSERT(o2->type >= 0 && o2->type < dGeomNumClasses,"bad o2 class number");
  // Even though comparison for greater or equal to one is used in all the
  // other places, here it is more logical to check for greater than zero
  // because function does not require any specific number of immersion slots -
  // it must be just a positive.

  dUASSERT((flags & NUMI_MASK) > 0, "no immersions requested");

  // Extra precaution for zero immersion count in parameters
  if ((flags & NUMI_MASK) == 0) return 0;

  dUASSERT(o1 != o2, "dImmerse was called for a pair of identical dGeoms");
  dUASSERT((o1->body && o2->fluid) || (o1->fluid && o2->body), "One of the dImmerse arguments is neither attached to a body nor a fluid");

  o1->recomputePosr();
  o2->recomputePosr();

  dImmerserFn *fn = immersers[o1->type][o2->type];

  if (fn)
    return o1-> body ? (*fn) (o1,o2,flags,immersion) : (*fn) (o2, o1,flags,immersion);

  return 0;
}

void dGeomSetFluid (dxGeom *g, dxFluid *f)
{
  dAASSERT (g);
  CHECK_NOT_LOCKED (g->parent_space);

  if (f) {
    if (g->fluid != f) {
      g->fluidRemove();
      g->fluidAdd (f);
    }
    dGeomMoved (g);
  } else if (g->fluid) {
    g->fluidRemove();
  }
}

dFluidID dGeomGetFluid (dxGeom *g)
{
  dAASSERT (g);
  return g->fluid;
}

void dxGeom::fluidRemove()
{
  if (fluid) {
    // delete this geom from fluid list
    dxGeom **last = &fluid->geom, *g = fluid->geom;
    while (g) {
      if (g == this) {
        *last = g->fluid_next;
        break;
      }
      last = &g->fluid_next;
      g = g->fluid_next;
    }
    fluid = 0;
    fluid_next = 0;
  }
}

dReal dGeomGetArea(dxGeom *g) {
  dAASSERT (g);

  switch (dGeomGetClass(g)) {
  case dBoxClass:
    return dGeomBoxGetArea(g);
  case dCapsuleClass:
    return dGeomCapsuleGetArea(g);
  case dCylinderClass:
    return dGeomCylinderGetArea(g);
  case dSphereClass:
    return dGeomSphereGetArea(g);
  case dTriMeshClass:
    return dGeomTriMeshGetArea(g);
  default:
    return 0.0;
  }
}

dReal dGeomGetVolume(dxGeom *g) {
  dAASSERT (g);

  switch (dGeomGetClass(g)) {
  case dBoxClass:
    return dGeomBoxGetVolume(g);
  case dCapsuleClass:
    return dGeomCapsuleGetVolume(g);
  case dCylinderClass:
    return dGeomCylinderGetVolume(g);
  case dSphereClass:
    return dGeomSphereGetVolume(g);
  case dTriMeshClass:
    return dGeomTriMeshGetVolume(g);
  default:
    return 0.0;
  }
}

void dGeomGetImmersionPlane(dxGeom *g, dReal *p) {
  dAASSERT (g && p);

  switch (dGeomGetClass(g)) {
  case dBoxClass:
    dGeomBoxGetImmersionPlane(g, p);
    break;
  case dCapsuleClass:
    dGeomCapsuleGetImmersionPlane(g, p);
    break;
  case dCylinderClass:
    dGeomCylinderGetImmersionPlane(g, p);
    break;
  case dPlaneClass:
    dGeomPlaneGetParams(g, p);
    break;
  case dSphereClass:
    dGeomSphereGetImmersionPlane(g, p);
    break;
  case dTriMeshClass:
    dGeomTriMeshGetImmersionPlane(g, p);
    break;
  default:
    dSetZero(p, 4);
    p[2] = 1.0;
  }
}

int dGeomIsInside(dxGeom *g, dReal x, dReal y, dReal z) {
  dAASSERT (g);

  static const dReal ZERO_THRESHOLD = 1e-3;

  if (dGeomGetClass(g) == dGeomTransformClass) {
    dxGeomTransform *tr = (dxGeomTransform *) g;
    dxPosR *posrBackup = tr->obj->final_posr;
    tr->obj->final_posr = &tr->transform_posr;
    const int result = dGeomIsInside(tr->obj, x, y, z);
    tr->obj->final_posr = posrBackup;
    return result;
  }

  dReal depth = - dInfinity;

  switch (dGeomGetClass(g)) {
  case dBoxClass:
    depth = dGeomBoxPointDepth(g, x, y, z);
    break;
  case dCapsuleClass:
    depth = dGeomCapsulePointDepth(g, x, y, z);
    break;
  case dCylinderClass:
    depth = dGeomCylinderPointDepth(g, x, y, z);
    break;
  case dPlaneClass:
    depth = dGeomPlanePointDepth(g, x, y, z);
    break;
  case dSphereClass:
    depth = dGeomSpherePointDepth(g, x, y, z);
    break;
 case dTriMeshClass:
    depth = dGeomTriMeshPointDepth(g, x, y, z);
    break;
  }

  return depth > - ZERO_THRESHOLD ? 1 : 0;
}

int dGeomIsBelowImmersionPlane(dxGeom *g, dReal x, dReal y, dReal z) {
  dAASSERT (g);

  static const dReal ZERO_THRESHOLD = 1e-3;

  if (dGeomGetClass(g) == dGeomTransformClass) {
    dxGeomTransform *tr = (dxGeomTransform *) g;
    dxPosR *posrBackup = tr->obj->final_posr;
    tr->obj->final_posr = &tr->transform_posr;
    const int result = dGeomIsBelowImmersionPlane(tr->obj, x, y, z);
    tr->obj->final_posr = posrBackup;
    return result;
  }

  dVector4 plane;
  dGeomGetImmersionPlane(g, plane);
  int result = plane[3] - (x * plane[0] + y * plane[1] + z * plane[2]) > - ZERO_THRESHOLD  ? 1 : 0;

  return result;
}

int dGeomGetFlags(dxGeom *g) {
  dAASSERT (g);
  return g->gflags;
}
