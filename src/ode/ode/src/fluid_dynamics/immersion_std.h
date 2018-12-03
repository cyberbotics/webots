/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
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

#ifndef _ODE_IMMERSION_STD_H_
#define _ODE_IMMERSION_STD_H_

#include <ode/common.h>
#include "immersion_kernel.h"

// primitive immersion functions - these have the dImmerserFn interface, i.e.
// the same interface as dImmerse(). the first and second geom arguments must
// have the specified types.

int dImmerseBoxBox (dxGeom *o1, dxGeom *o2,
      int flags, dImmersionGeom *immersion);
int dImmerseBoxCylinder (dxGeom *o1, dxGeom *o2,
      int flags, dImmersionGeom *immersion);
int dImmerseBoxCapsule (dxGeom *o1, dxGeom *o2,
      int flags, dImmersionGeom *immersion);
int dImmerseBoxPlane (dxGeom *o1, dxGeom *o2,
      int flags, dImmersionGeom *immersion);
int dImmerseBoxSphere (dxGeom *o1, dxGeom *o2,
      int flags, dImmersionGeom *immersion);

int dImmerseCapsuleBox (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseCapsuleCapsule (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseCapsuleCylinder (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseCapsulePlane (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseCapsuleSphere (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);

int dImmerseCylinderBox (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseCylinderCapsule (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseCylinderCylinder (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseCylinderPlane (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseCylinderSphere (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);

int dImmerseSphereBox (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseSphereCapsule (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseSphereCylinder (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseSpherePlane (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseSphereSphere (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);

#if dTRIMESH_ENABLED
int dImmerseBoxTrimesh (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseCapsuleTrimesh (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseCylinderTrimesh (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseSphereTrimesh (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);

int dImmerseTrimeshBox (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseTrimeshCapsule (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseTrimeshCylinder (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseTrimeshPlane (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseTrimeshSphere (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
int dImmerseTrimeshTrimesh (dxGeom *o1, dxGeom *o2,
     int flags, dImmersionGeom *immersion);
#endif

#endif
