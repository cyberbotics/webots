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

standard ODE geometry primitives: public API and pairwise immersion functions.

the rule is that only the low level primitive immersion functions should set
dImmersionGeom::g1 and dImmersionGeom::g2.

*/

#include <ode/collision.h>
#include <ode/fluid_dynamics/immersion.h>
#include <ode/fluid_dynamics/fluid_dynamics.h>
#include <ode/odemath.h>
#include "../collision_kernel.h"
#include "../collision_std.h"
#include "../collision_trimesh_internal.h"
#include <ode/matrix.h>
#include "immersion_kernel.h"
#include "immersion_std.h"
#include "immersion_outline.h"

#ifdef _MSC_VER
#pragma warning(disable:4291)  // for VC++, no complaints about "no matching operator delete found"
#endif

dReal dGeomSphereGetVolume (dGeomID g)
 {
   dUASSERT (g && g->type == dSphereClass,"argument not a sphere");
   static const dReal K = 4.0 * M_PI / 3.0;

   dxSphere *s = (dxSphere*) g;
   const dReal r = s->radius;
   return K * r * r * r;
 }

 dReal dGeomSphereGetArea (dGeomID g)
 {
   dUASSERT (g && g->type == dSphereClass,"argument not a sphere");

   dxSphere *s = (dxSphere*) g;
   const dReal r = s->radius;
   return 4.0 * M_PI * r * r;
 }

 void dGeomSphereGetTangentPlane (dGeomID g, dReal x, dReal y, dReal z, dVector4 plane)
 {
   dUASSERT (g && g->type == dSphereClass,"argument not a sphere");

   dxSphere *s = (dxSphere*) g;
   const dReal *const center = s->final_posr->pos;
   dVector3 p = { x - center[0], y - center[1], z - center[2] };
   plane[0] =  p[0];
   plane[1] =  p[1];
   plane[2] =  p[2];
   plane[3] = x * p[0] + y * p[1] + z * p[2];
   make_sure_plane_normal_has_unit_length(plane);
 }

 void dGeomSphereGetImmersionPlane (dGeomID g, dVector4 plane)
 {
   dUASSERT (g && g->type == dSphereClass,"argument not a sphere");
   const dReal *const rotation = g->final_posr->R;
   const dReal *const center = g->final_posr->pos;
   const dReal r = ((dxSphere *)g)->radius;
   const dVector3 p = { r * rotation[0 + FLUID_PLANE_NORMAL] + center[0], r * rotation[4 + FLUID_PLANE_NORMAL] +
   center[1], r * rotation[8 + FLUID_PLANE_NORMAL] + center[2] };
   dGeomSphereGetTangentPlane(g, p[0], p[1], p[2], plane);
 }

//****************************************************************************
// pairwise immersion functions for standard geom types

int dImmerseSphere (const dxSphere *sphere, const dReal *fluidPlane, int flags,
      dImmersionGeom *immersion) {

  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT(sphere->body, "this sphere is not attached to a body");

  const dReal *const center = sphere->final_posr->pos;
  const dReal zI = fluidPlane[3] - dCalcVectorDot3 (center, fluidPlane);
  const dReal radius = sphere->radius;
  if (zI > -radius) {
    for (int i = 0; i < 3; ++i)
      immersion->projectedAreas[i] = 0.0;

    if (zI >= radius) {
      immersion->volume = dGeomSphereGetVolume(const_cast<dxSphere *>(sphere));
      if (immersion->volume <= VOLUME_ZERO_THRESHOLD)
        return 0;
      dReal area = dGeomSphereGetArea(const_cast<dxSphere *>(sphere));
      immersion->area = area;
      area *= 0.25;
      for (int i = 0; i < 3; ++i)
        immersion->projectedAreas[i] = area;
      dCopyVector3(immersion->buoyancyCenter, center);
      return 1;
    }

      const dReal temp1 = zI + radius;
      const dReal temp2 = zI - radius;
      const dReal temp3 = 2.0 * radius - zI;
      immersion->volume = (M_PI / 3.0) * temp1 * temp1 * temp3;
      if (immersion->volume <= VOLUME_ZERO_THRESHOLD)
        return 0;
      immersion->area = 2.0 * M_PI * radius * (zI + radius);
      dAddScaledVectors3(immersion->buoyancyCenter, center, fluidPlane, 1.0, - 0.75 * temp2 * temp2 / temp3);

    // Immersion outline
    if (immersion->outline && (flags & ~dxImmersionOutlineDisabled))  {
      dCurvedEdge ce;
      dPlaneSpace(fluidPlane, ce.e1, ce.e2);
      const double r = dSqrt(radius * radius - zI * zI);
      dScaleVector3(ce.e1, r);
      dScaleVector3(ce.e2, r);
      dAddScaledVectors3(ce.center, center, fluidPlane, 1.0, zI);
      ce.minAngle = 0.0;
      ce.maxAngle = 2.0 * M_PI;
      immersion->outline->appendCurvedEdge(ce);
    }

    return 1;
  }
  return 0;
}

int dImmerseSphereSphere (dxGeom *o1, dxGeom *o2, int flags,
      dImmersionGeom *immersion)
{

  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT(o1-> body && o2->fluid, "The first dImmerseSphereSphere argument is not attached to a body or the second is not attached to a fluid");

  dContactGeom c[1];
  const int collision = dCollideSphereSphere(o1, o2, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  const dxSphere *const sphere1 = (dxSphere*) o1;
  dxSphere *const sphere2 = (dxSphere*) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomSphereGetImmersionPlane(sphere2, fluidPlane);

  return dImmerseSphere (sphere1, fluidPlane, flags, immersion);
}

int dImmerseSphereBox (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first dImmerseSphereBox argument is not attached to a body or the second is not attached to a fluid");

  dContactGeom c[1];
  const int collision = dCollideSphereBox(o1, o2, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  const dxSphere *const sphere = (dxSphere *) o1;
  dxBox *const box = (dxBox *) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomBoxGetImmersionPlane(box, fluidPlane);

  return dImmerseSphere (sphere, fluidPlane, flags, immersion);
}

int dImmerseSphereCapsule (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseBoxCapsule has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideCapsuleSphere(o2, o1, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  dxSphere *const sphere = (dxSphere *) o1;
  dxCapsule *const capsule = (dxCapsule *) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomCapsuleGetImmersionPlane(capsule, fluidPlane);

  return dImmerseSphere (sphere, fluidPlane, flags, immersion);
}

int dImmerseSphereCylinder (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseSphereCylinder has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideCylinderSphere(o2, o1, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  dxSphere *const sphere = (dxSphere *) o1;
  dxCylinder *const cylinder = (dxCylinder *) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomCylinderGetImmersionPlane(cylinder, fluidPlane);

  return dImmerseSphere (sphere, fluidPlane, flags, immersion);
}

int dImmerseSphereTrimesh (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseBoxCylinder has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideSTL(o2, o1, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomCylinderGetImmersionPlane(o2, fluidPlane);

  return dImmerseSphere ((dxSphere *) o1, fluidPlane, flags, immersion);
}

int dImmerseSpherePlane (dxGeom *o1, dxGeom *o2, int flags,
      dImmersionGeom *immersion)
{

  dIASSERT ((flags & NUMI_MASK) >= 1);
  dIASSERT (o1->type == dSphereClass);
  dIASSERT (o2->type == dPlaneClass);
  dUASSERT(o1-> body && o2->fluid, "The first dImmerseSpherePlane argument is not attached to a body or the second is not attached to a fluid");

  immersion->g1 = o1;
  immersion->g2 = o2;

  dxSphere *const sphere = (dxSphere*) o1;
  dxPlane *const plane = (dxPlane*) o2;

  return dImmerseSphere (sphere, plane->p, flags, immersion);
}
