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

#include <ode/objects.h>
#include <ode/collision.h>
#include <ode/fluid_dynamics/fluid_dynamics.h>
#include <ode/fluid_dynamics/odemath_fluid_dynamics.h>
#include "immersion_revolution_data.h"
#include "../collision_kernel.h"
#include "../collision_std.h"
#include "../collision_trimesh_internal.h"
#include "immersion_kernel.h"
#include "immersion_outline.h"
#include "immersion_std.h"

#ifdef _MSC_VER
#pragma warning(disable:4291)  // for VC++, no complaints about "no matching operator delete found"
#endif

dReal dGeomCylinderGetVolume (dGeomID g)
 {
   dUASSERT (g && g->type == dCylinderClass,"argument not a cylinder");

   const dxCylinder *const c = (dxCylinder*) g;
   const dReal r = c->radius;
   const dReal h = c->lz;
   return M_PI * r * r * h;
 }

dReal dGeomCylinderGetArea (dGeomID g)
 {
   dUASSERT (g && g->type == dCylinderClass,"argument not a cylinder");

   const dxCylinder *const c = (dxCylinder*) g;
   const dReal r = c->radius;
   const dReal h = c->lz;

   return 2.0 * M_PI * r * (r + h);
 }

void dGeomCylinderGetTangentPlane (dGeomID g, dReal x, dReal y, dReal z, dVector4 plane)
 {
   dUASSERT (g && g->type == dCylinderClass,"argument not a cylinder");

   const dxCylinder *const c = (dxCylinder*) g;
   const dReal *const R = c->final_posr->R;
   const dReal *const center = c->final_posr->pos;
   dVector3 p = { x - center[0], y - center[1], z - center[2] };

   static const dReal ZERO_THRESHOLD = 1e-3;
   const dVector3 n = { R[2], R[6], R[10] };
   const dReal l = dCalcVectorDot3(n, p);
   dReal diskSign = 0.0;
   const dReal halfHeight = 0.5 * c->lz;
   if (dFabs(l - halfHeight) < ZERO_THRESHOLD)
     diskSign = 1.0;
   else if (dFabs(l + halfHeight) < ZERO_THRESHOLD)
     diskSign = -1.0;

   if (diskSign != 0.0) {
     plane[0] =  diskSign * n[0];
     plane[1] =  diskSign * n[1];
     plane[2] =  diskSign * n[2];
     plane[3] = x * plane[0] + y * plane[1] + z * plane[2];
     return;
   }

   dAddScaledVectors3(p, p, n, 1.0, -l);
   const dReal r = c->radius;
   if (dFabs(dCalcVectorLengthSquare3(p) - r * r) < ZERO_THRESHOLD) {
     plane[0] =  p[0];
     plane[1] =  p[1];
     plane[2] =  p[2];
     plane[3] = x * plane[0] + y * plane[1] + z * plane[2];
     make_sure_plane_normal_has_unit_length(plane);
     return;
   }

  dSetZero(plane, 4);
  plane[2] = 1.0;
 }

void dGeomCylinderGetDiskPlane (dGeomID g, int diskSign, dVector4 plane)
 {
   dUASSERT (g && g->type == dCylinderClass,"argument not a cylinder");
   dUASSERT(abs(diskSign) == 1, "Invalid face index: -1 or 1");

   const dxCylinder *const c = (dxCylinder*) g;
   const dReal *const center = c->final_posr->pos;
   const dReal *const R = c->final_posr->R;
   plane[0] =  diskSign * R[2];
   plane[1] =  diskSign * R[6];
   plane[2] =  diskSign * R[10];
   plane[3] = dCalcVectorDot3(plane, center) + 0.5 * diskSign * c->lz;
 }

void dGeomCylinderGetImmersionPlane (dGeomID g, dVector4 plane)
 {
   dGeomCylinderGetDiskPlane(g, 1, plane);
 }

dReal dGeomCylinderPointDepth (dGeomID g, dReal x, dReal y, dReal z)
 {
   dUASSERT (g && g->type == dCylinderClass,"argument not a cylinder");
   const dxCylinder *const cylinder = (dxCylinder *) g;
   const dReal *const R = cylinder->final_posr->R;
   dVector3 p = { x, y, z };
   dSubtractVectors3(p, p, cylinder->final_posr->pos);

   const dVector3 ez = { R[2], R[6], R[10] };
   const dReal pZ = dCalcVectorDot3(p, ez);
   const dReal distanceToTheCapPlanes = 0.5 * cylinder->lz - dFabs(pZ);

   dAddScaledVectors3(p, p, ez, 1.0, -pZ);
   const dReal distanceToTheBody =  cylinder->radius * cylinder->radius - dCalcVectorLengthSquare3(p);

   return distanceToTheCapPlanes < distanceToTheBody ? distanceToTheCapPlanes : distanceToTheBody;
 }

int dImmerseCylinder (const dxCylinder *cylinder, const dReal *fluidPlane, int flags,
      dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT(cylinder->body, "This cylinder has no body and hence cannot be tested for immersion into a fluid");

  sCylinderImmersionData data(cylinder, fluidPlane, flags, immersion);
  return data.performImmersionChecking();
}

int dImmerseCylinderBox (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseCylinderBox has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideCylinderBox(o1, o2, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  const dxCylinder *const cylinder = (dxCylinder *) o1;
  dxBox *const box = (dxBox *) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomBoxGetImmersionPlane(box, fluidPlane);
  return dImmerseCylinder (cylinder, fluidPlane, flags, immersion);
}

int dImmerseCylinderCapsule (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseCylinderCapsule has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideCylinderCapsule(o1, o2, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  const dxCylinder *const cylinder = (dxCylinder *) o1;
  dxCapsule *const capsule = (dxCapsule *) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomCapsuleGetImmersionPlane(capsule, fluidPlane);

  return dImmerseCylinder (cylinder, fluidPlane, flags, immersion);
}

int dImmerseCylinderCylinder (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseCylinderCylinder has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideCylinderCylinder(o1, o2, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  const dxCylinder *const cylinder1 = (dxCylinder *) o1;
  dxCylinder *const cylinder2 = (dxCylinder *) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomCylinderGetImmersionPlane(cylinder2, fluidPlane);

  return dImmerseCylinder (cylinder1, fluidPlane, flags, immersion);
}

int dImmerseCylinderSphere (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseCylinderSphere has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideCylinderSphere(o1, o2, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  const dxCylinder *const cylinder = (dxCylinder *) o1;
  dxSphere *const sphere = (dxSphere *) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomSphereGetImmersionPlane(sphere, fluidPlane);

  return dImmerseCylinder (cylinder, fluidPlane, flags, immersion);
}

int dImmerseCylinderTrimesh (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseCylinderTrimesh has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideCylinderTrimesh(o1, o2, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomTriMeshGetImmersionPlane(o2, fluidPlane);

  return dImmerseCylinder ((dxCylinder *) o1, fluidPlane, flags, immersion);
}

int dImmerseCylinderPlane (dxGeom *o1, dxGeom *o2, int flags,
      dImmersionGeom *immersion)
{

  dIASSERT ((flags & NUMI_MASK) >= 1);
  dIASSERT (o1->type == dCylinderClass);
  dIASSERT (o2->type == dPlaneClass);
  dUASSERT(o1-> body && o2->fluid, "The first argument of dImmerseCylinderPlane has no body or the second has no fluid");

  immersion->g1 = o1;
  immersion->g2 = o2;

  const dxCylinder *const cylinder = (dxCylinder*) o1;
  const dxPlane *const plane = (dxPlane*) o2;

  return dImmerseCylinder (cylinder, plane->p, flags, immersion);
}
