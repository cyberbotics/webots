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

#include <ode/fluid_dynamics/immersion.h>
#include <ode/fluid_dynamics/fluid_dynamics.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/odemath.h>
#include "config.h"
#include "../collision_std.h"
#include "immersion_transform.h"

#ifdef _MSC_VER
#pragma warning(disable:4291)  // for VC++, no complaints about "no matching operator delete found"
#endif

//****************************************************************************
// immerser function:
// this immerses a transformed geom in another geom. the other geom can
// also be a transformed geom, but this case is not handled specially.

int dImmerseTransform (dxGeom *o1, dxGeom *o2, int flags,
		       dImmersionGeom *immersion)
{
  dIASSERT (o1->type == dGeomTransformClass || o2->type == dGeomTransformClass);
  dxGeomTransform *tr[2] = { NULL, NULL };
  tr[0] = o1->type == dGeomTransformClass ? (dxGeomTransform*) o1 : NULL;
  tr[1] = o2->type == dGeomTransformClass ? (dxGeomTransform*) o2 : NULL;

  if (tr[0] && !tr[0]->obj) return 0;
  if (tr[1] && !tr[1]->obj) return 0;
  dUASSERT (tr[0] == NULL || tr[0]->obj->parent_space==0,
    "The encapsulated object of the first GeomTransform must not be in a space");
  dUASSERT (tr[1] == NULL || tr[1]->obj->parent_space==0,
    "The encapsulated object of the second GeomTransform must not be in a space");
  dUASSERT (tr[0] == NULL || tr[0]->obj->body==0,
    "The encapsulated object of the first GeomTransform must not be attached "
    "to a body");
  dUASSERT (tr[1] == NULL || tr[1]->obj->fluid==0,
    "The encapsulated object of the second GeomTransform must not be attached "
    "to a fluid");

  // backup the relative pos and R pointers of the encapsulated geom object,
  // and the body pointer
  dxPosR *posr_bak[2] = { NULL, NULL };
  dxBody *bodybak =  NULL;
  dxFluid *fluidbak = NULL;
  if (tr[0]) {
    posr_bak[0] = tr[0]->obj->final_posr;
    bodybak = tr[0]->obj->body;
  }
  if (tr[1]) {
    posr_bak[1] = tr[1]->obj->final_posr;
    fluidbak = tr[1]->obj->fluid;
  }
  // compute temporary pos and R for the encapsulated geom object.
  // note that final_pos and final_R are valid if no GEOM_AABB_BAD flag,
  // because computeFinalTx() will have already been called in
  // dxGeomTransform::computeAABB()

  for (int i = 0; i < 2; ++i) {
    if (tr[i]) {
      if (tr[i]->gflags & GEOM_AABB_BAD)
        tr[i]->computeFinalTx();
      tr[i]->obj->final_posr = &tr[i]->transform_posr;
    }
  }

  if (tr[0])
    tr[0]->obj->body = o1->body;
  if (tr[1])
    tr[1]->obj->fluid = o2->fluid;

  // do the immersion
  const int n = dImmerse (tr[0] ? tr[0]->obj : o1, tr[1] ? tr[1]->obj : o2, flags, immersion);

  // if required, adjust the 'g1' and 'g2' values in the generated immersions so that
  // they indicated the GeomTransform object instead of the encapsulated
  // object.
  if (tr[0] && tr[0]->infomode)
    immersion->g1 = o1;
  if (tr[1] && tr[1]->infomode)
    immersion->g2 = o2;

  // restore the pos, R and body

  if (tr[0]) {
    tr[0]->obj->final_posr = posr_bak[0];
    tr[0]->obj->body = bodybak;
  }

  if (tr[1]) {
    tr[1]->obj->final_posr = posr_bak[1];
    tr[1]->obj->fluid = fluidbak;
  }

  return n;
}
