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

#ifdef _MSC_VER
#pragma warning(disable:4291)  // for VC++, no complaints about "no matching operator delete found"
#endif

// this source file is mostly concerned with the data structures, not the
// numerics.

#include <ode/ode.h>
#include <ode/fluid_dynamics/fluid_dynamics.h>
#include <ode/fluid_dynamics/objects_fluid_dynamics.h>
#include <ode/fluid_dynamics/odemath_fluid_dynamics.h>
#include "objects_fluid_dynamics.h"
#include "immersion_outline.h"
#include "immersion_link/immersion_link.h"
#include "ode_MT/ode_MT.h"

#include "../collision_kernel.h"

// misc defines
#define ALLOCA dALLOCA16

#ifdef ODE_MT
#include <pthread.h>
#endif

extern void addObjectToList( dObject *obj, dObject **first );
extern void removeObjectFromList( dObject *obj);

//****************************************************************************
// fluids

dxFluid::dxFluid(dxWorld *w) :
    dObject(w)
{
  density = 1000.0;
  viscosity = 0.001;
}

dxWorld* dFluidGetWorld (dxFluid *f)
{
  dAASSERT (f);
  return f->world;
}

dxFluid *dFluidCreate_ST (dxWorld *w)
{
  dAASSERT (w);
  dxFluid *f = new dxFluid(w);
  f->flags = 0;
  f->geom = 0;
  f->firstimmersionlink = 0;

  addObjectToList (f,(dObject **) &w->firstfluid);
  w->nf++;

  f->flags |= w->fluid_flags;

  return f;
}

dxFluid *dFluidCreate (dxWorld *w)
{
#ifdef ODE_MT
    return dFluidCreate_MT(w, &dFluidCreate_ST);
#else
    return dFluidCreate_ST(w);
#endif
}

void dFluidDestroy_ST (dxFluid *f)
{
  dAASSERT (f);

  // all geoms that link to this fluid must be notified that the fluid is about
  // to disappear. note that the call to dGeomSetFluid(geom,0) will result in
  // dGeomGetFluidNext() returning 0 for the fluid, so we must get the next fluid
  // before setting the fluid to 0.
  dxGeom *next_geom = 0;
  for (dxGeom *geom = f->geom; geom; geom = next_geom) {
    next_geom = dGeomGetFluidNext (geom);
    dGeomSetFluid (geom,0);
  }

  dxImmersionLink *iml = f->firstimmersionlink;
  while (iml) {
    dxImmersionLink *next = iml->nextFluidImmersionLink;
    dImmersionLinkDestroy(iml);
    iml = next;
  }

  removeObjectFromList (f);
  --(f->world->nf);

  delete f;
}

void dFluidDestroy (dxFluid *f)
{
#ifdef ODE_MT
    dFluidDestroy_MT(f, &dFluidDestroy_ST);
#else
    dFluidDestroy_ST(f);
#endif
}

void dFluidSetData (dFluidID f, void *data)
{
  dAASSERT (f);
  f->userdata = data;
}

void *dFluidGetData (dFluidID f)
{
  dAASSERT (f);
  return f->userdata;
}

void dFluidSetDensity (dFluidID f, dReal d)
{
  dAASSERT (f);
  f->density = d;
}

dReal dFluidGetViscosity (dFluidID f)
{
  dAASSERT (f);
  return f->viscosity;
}

void dFluidSetViscosity (dFluidID f, dReal v)
{
  dAASSERT (f);
  f->viscosity = v;
}

dReal dFluidGetDensity (dFluidID f)
{
  dAASSERT (f);
  return f->density;
}

void dFluidSetStreamVel (dFluidID f, const dReal *velocity)
{
  dAASSERT (f);
  dCopyVector3(f->streamVelocity, velocity);
}

const dReal* dFluidGetStreamVel (dFluidID f)
{
  dAASSERT (f);
  return f->streamVelocity;
}

void dFluidEnable (dFluidID f)
{
  dAASSERT (f);
  f->flags &= ~dxFluidDisabled;
}

void dFluidDisable (dFluidID f)
{
  dAASSERT (f);
  f->flags |= dxFluidDisabled;
}

int dFluidIsEnabled (dFluidID f)
{
  dAASSERT (f);
  return ((f->flags & dxFluidDisabled) == 0);
}

dGeomID dFluidGetNextGeom(dGeomID geom)
{
        dAASSERT(geom);
        return dGeomGetFluidNext(geom);
}

dGeomID dFluidGetFirstGeom(dFluidID f)
{
        dAASSERT(f);
        return f->geom;
}

int dFluidDynamicsStep (dWorldID w)
{
  dUASSERT (w,"bad world argument");
  dxBody *body = NULL;
  for (body = w->firstbody; body; body = (dxBody*)body->next) {
    for (dxImmersionLink *immersionLink = body->firstimmersionlink; immersionLink; immersionLink = immersionLink->nextBodyImmersionLink) {
      const dImmersionGeom &img = immersionLink->immersion.geom;
      static const int artificalMove = GEOM_DIRTY | GEOM_POSR_BAD | GEOM_AABB_BAD;
      if ((dGeomGetFlags(img.g1) & artificalMove) | (dGeomGetFlags(img.g2) & artificalMove)) // the body or the fluid has been moved artificially
        continue;

      dVector3 av;
      dCopyVector3(av, dBodyGetAngularVel(body));
      dReal avLength = dCalcVectorLength3(av);
      const dReal *const lv = dBodyGetLinearVel(body);
      const dReal *const R = dBodyGetRotation(body);

      // Relative linear velocity
      dVector3 rlv;
      dCopyVector3(rlv, lv);
      dSubtractVectors3(rlv, rlv, dFluidGetStreamVel(immersionLink->fluid));
      dReal rlvLength = dCalcVectorLength3(rlv);
      const dImmersionSurfaceParameters &surface = immersionLink->immersion.surface;
      const dReal maxLv = surface.maxLinearVel;
      if (rlvLength > maxLv) {
        dNormalize3(rlv);
        dScaleVector3(rlv, maxLv);
        rlvLength = maxLv;
      }

      // Relative angular velocity
      dVector3 rav;
      dCopyVector3(rav, av);
      const dReal maxAv = surface.maxAngularVel;
      if (avLength > maxAv) {
        dNormalize3(av);
        dScaleVector3(av, maxAv);
        avLength = maxAv;
      }
      dMultiplyHelper1_331(rav, R, rav);

      //printf("linear speed %g  angular speed %g\n", rlvLength, avLength);
#ifndef dNODEBUG
      static const dReal TOLERANCE = 1e-6;
#endif
      //printf("immersed area %g full area %g\n", img.area, dGeomGetArea(img.g1));
      dIASSERT(img.area >= 0.0 && img.area <= (1.0 + TOLERANCE) * dGeomGetArea(img.g1));
      //printf("immersed volume %g full volume %g\n", img.volume, dGeomGetVolume(img.g1));
      dIASSERT(img.volume >= 0.0 && img.volume <= (1.0 + TOLERANCE) * dGeomGetVolume(img.g1));

      // Sums of all fluid forces and torques
      dVector3 force, torque;

      // Archimedes' thrust
      const dReal d = immersionLink->fluid->density;
      dVector3 archimedesThrust;
      dCopyScaledVector3(archimedesThrust, w->gravity, -img.volume * d);

      // Quadratic drags
      dVector3 quadraticDragForce = { 0.0, 0.0, 0.0 };
      dVector3 quadraticDragTorque = { 0.0, 0.0, 0.0 };

      const dReal *const c = surface.dragForceCoefficients;
      const dReal *const t = surface.dragTorqueCoefficients;
      if (surface.mode == dImmersionProjectedAreas) {
        dVector3 rlvTemp;
        dMultiplyHelper1_331(rlvTemp, R, rlv);
        for (int i = 0; i < 3; ++i) {
          const dReal signF = rlvTemp[i] > 0.0 ? -1.0 : 1.0;
          quadraticDragForce[i] = signF * d * c[i] * img.projectedAreas[i] * rlvTemp[i] * rlvTemp[i];
          const dReal dragTorqueWeight = t[i] * (img.projectedAreas[(i + 1) % 3] + img.projectedAreas[(i + 2) % 3]);
          const dReal signT = rav[i] > 0.0 ? -1.0 : 1.0;
          quadraticDragTorque[i] = signT * d * dragTorqueWeight * rav[i] * rav[i];
        }
        // back to absolute coordinates
        dMultiplyHelper0_331(quadraticDragForce, R, quadraticDragForce);
        dMultiplyHelper0_331(quadraticDragTorque, R, quadraticDragTorque);
      } else if (surface.mode == dImmersionImmersedArea){
        const dReal p = -d * img.area;
        dCopyScaledVector3(quadraticDragForce, rlv, p * c[0] * rlvLength);
        dCopyScaledVector3(quadraticDragTorque, av, p * t[0] * avLength);
      }

      // Viscous resistance (linear drags)
      dVector3 viscousResistanceForce, viscousResistanceTorque;
      const dReal p = - (img.area / dGeomGetArea(img.g1)) * immersionLink->fluid->viscosity;
      dCopyScaledVector3(viscousResistanceForce, rlv, p * surface.viscousResistanceForceCoefficient);
      dCopyScaledVector3(viscousResistanceTorque, av, p * surface.viscousResistanceTorqueCoefficient);

      /*
      printf("Archimedes' thrust % g %g %g \n", archimedesThrust[0], archimedesThrust[1], archimedesThrust[2]);
      printf("Quadratic drag force %g %g %g \n", quadraticDragForce[0], quadraticDragForce[1], quadraticDragForce[2]);
      printf("Quadratic drag torque %g %g %g \n", quadraticDragTorque[0], quadraticDragTorque[1], quadraticDragTorque[2]);
      printf("Viscous resistance force %g %g %g \n", viscousResistanceForce[0], viscousResistanceForce[1], viscousResistanceForce[2]);
      printf("Viscous resistance torque %g %g %g \n", viscousResistanceTorque[0], viscousResistanceTorque[1], viscousResistanceTorque[2]);
      printf("\n\n\n");
      */

      const dReal *const bc = img.buoyancyCenter;
      dIASSERT(dValidVector3(bc));

      /*
      const dReal *pos = dBodyGetPosition(body);
      const dReal *posg = dGeomGetPosition(img.g1);
      printf("body center %g %g %g\n", pos[0], pos[1], pos[2]);
      printf("geom (%p) center %g %g %g\n", img.g1, posg[0], posg[1], posg[2]);
      printf("buoyancy center %g %g %g\n\n", bc[0], bc[1], bc[2]);
      */

      dIASSERT(dGeomIsInside(img.g1, bc[0], bc[1], bc[2]));
      dIASSERT(dGeomIsBelowImmersionPlane(img.g2, bc[0], bc[1], bc[2]));
      dIASSERT(dValidVector3(archimedesThrust) && dValidVector3(quadraticDragForce) && dValidVector3(quadraticDragTorque));
      dIASSERT(dValidVector3(viscousResistanceForce) && dValidVector3(viscousResistanceTorque));

      dCopyVector3(force, archimedesThrust);
      dAddVectors3(force, force, quadraticDragForce);
      dAddVectors3(force, force, viscousResistanceForce);
      dBodyAddForceAtPos(body, force[0], force[1], force[2], bc[0], bc[1], bc[2]);
      dCopyVector3(torque, quadraticDragTorque);
      dAddVectors3(torque, torque, viscousResistanceTorque);
      dBodyAddTorque(body, torque[0], torque[1], torque[2]);
    }
  }

  return 1;
}

dImmersionLinkGroupID dImmersionLinkGroupCreate ()
{
    dxImmersionLinkGroup *const group = new dxImmersionLinkGroup;
    group->firstimmersionlink = 0;
    return group;
}

// Immersion group methods

void dImmersionLinkGroupDestroy (dImmersionLinkGroupID group)
{
    dAASSERT (group);
    dImmersionLinkGroupEmpty (group);
    delete group;
}

// remove the link from neighbour lists of all connected bodies

static void removeImmersionLinkReferencesFromAttachedMatters (dxImmersionLink *iml)
{
  dxBody *const body = iml->body;
  if (body) {
    dxImmersionLink *currentImmersionLink = body->firstimmersionlink;
    dxImmersionLink *previousImmersionLink = 0;
    // Rewind and moves forward to retrieve the previous immersion, if it exists
    while (currentImmersionLink && currentImmersionLink != iml) {
      previousImmersionLink = currentImmersionLink;
      currentImmersionLink = currentImmersionLink->nextBodyImmersionLink;
    }

    if (previousImmersionLink)
      previousImmersionLink->nextBodyImmersionLink = iml->nextBodyImmersionLink;
    else
      body->firstimmersionlink = iml->nextBodyImmersionLink;
  }

  dxFluid *const fluid = iml->fluid;
  if (fluid) {
    dxImmersionLink *currentImmersionLink = fluid->firstimmersionlink;
    dxImmersionLink *previousImmersionLink = 0;
    // Rewind and moves forward to retrieve the previous immersion, if it exists
    while (currentImmersionLink && currentImmersionLink != iml) {
      previousImmersionLink = currentImmersionLink;
      currentImmersionLink = currentImmersionLink->nextFluidImmersionLink;
    }

    if (previousImmersionLink)
      previousImmersionLink->nextFluidImmersionLink = iml->nextFluidImmersionLink;
    else
      fluid->firstimmersionlink = iml->nextFluidImmersionLink;
  }

  dxImmersionLinkGroup *const group = iml->group;
  if (group) {
    dxImmersionLink *currentImmersionLink = group->firstimmersionlink;
    dxImmersionLink *previousImmersionLink = 0;
    // Rewind and moves forward to retrieve the previous immersion, if it exists
    while (currentImmersionLink && currentImmersionLink != iml) {
      previousImmersionLink = currentImmersionLink;
      currentImmersionLink = currentImmersionLink->nextGroupImmersionLink;
    }

    if (previousImmersionLink)
      previousImmersionLink->nextGroupImmersionLink = iml->nextGroupImmersionLink;
    else
      group->firstimmersionlink = iml->nextGroupImmersionLink;
  }

  iml->body = 0;
  iml->fluid = 0;
  iml->group = 0;
  iml->nextBodyImmersionLink = 0;
  iml->nextFluidImmersionLink = 0;
  iml->nextGroupImmersionLink = 0;
}

void dImmersionLinkGroupEmpty (dImmersionLinkGroupID group)
{

  dAASSERT (group);
  dxImmersionLink *iml = group->firstimmersionlink;
  while (iml) {
    dxImmersionLink *next = iml->nextGroupImmersionLink;
    dImmersionLinkDestroy(iml);
    iml = next;
  }
  group->firstimmersionlink = 0;
}

dImmersionLinkID dImmersionLinkCreate(dWorldID w, dImmersionLinkGroupID group, const dImmersion *immersion)
{
    dxImmersionLink *iml;
    iml = (dxImmersionLink*) dAlloc(sizeof(dxImmersionLink));
    new(iml) dxImmersionLink(w);

    if (group)
      iml->group = group;

    iml->nextGroupImmersionLink = group->firstimmersionlink;
    group->firstimmersionlink = iml;

    iml->immersion = *immersion;

    return iml;
}

void dImmersionLinkDestroy (dxImmersionLink *iml)
{
    dAASSERT (iml);
    removeImmersionLinkReferencesFromAttachedMatters (iml);
    iml->~dxImmersionLink();
    dFree (iml, sizeof(dxImmersionLink));
}

void dImmersionLinkAttach (dxImmersionLink *immersionLink, dxBody *body, dxFluid *fluid)
{
  // check arguments
  dUASSERT (immersionLink,"bad immersion argument");

  // remove any existing body and fluid attachments
  if (immersionLink->body || immersionLink->fluid)
    removeImmersionLinkReferencesFromAttachedMatters (immersionLink);

  // attach to a new body and a new fluid
  immersionLink->body = body;
  immersionLink->fluid = fluid;
  immersionLink->nextBodyImmersionLink = body->firstimmersionlink;
  immersionLink->nextFluidImmersionLink = fluid->firstimmersionlink;
  body->firstimmersionlink = immersionLink;
  fluid->firstimmersionlink = immersionLink;
}

void dImmersionLinkEnable (dxImmersionLink *immersionLink)
{
  dAASSERT (immersionLink);
  immersionLink->enabled = true;
}

void dImmersionLinkDisable (dxImmersionLink *immersionLink)
{
  dAASSERT (immersionLink);
  immersionLink->enabled = false;
}

int dImmersionLinkIsEnabled (const dxImmersionLink *immersionLink)
{
  dAASSERT (immersionLink);
  return immersionLink->enabled;
}

void dImmersionLinkSetData (dxImmersionLink *immersionLink, void *data)
{
  dAASSERT (immersionLink);
  immersionLink->userdata = data;
}

void *dImmersionLinkGetData (const dxImmersionLink *immersionLink)
{
  dAASSERT (immersionLink);
  return immersionLink->userdata;
}

dBodyID dImmersionLinkGetBody (const dxImmersionLink *immersionLink)
{
  return immersionLink->body;
}

dFluidID dImmersionLinkGetFluid (const dxImmersionLink *immersionLink)
{
  return immersionLink->fluid;
}

dImmersionLinkID dLinkingImmersion (const dBodyID b, const dFluidID f)
{
  dAASSERT (f && b);
    // look through b's neighbour list for f
    for (dxImmersionLink *iml = b->firstimmersionlink; iml; iml = (dxImmersionLink*) iml->nextBodyImmersionLink) {
        if (iml->fluid == f) return iml;
    }

    return 0;
}

int dAreLinked (const dBodyID b, const dFluidID f)
{
  return dLinkingImmersion (b, f) != 0 ? 1 : 0;
}

int dBodyGetNumImmersionLinks (dBodyID b)
{
  dAASSERT (b);
  int count = 0;
  for (dxImmersionLink *iml = b->firstimmersionlink; iml; iml = (dxImmersionLink*)iml->nextBodyImmersionLink, ++count);
  return count;
}

dImmersionLinkID dBodyGetImmersionLink (dBodyID b, int index)
{
  dAASSERT (b);
  int i = 0;
  for (dxImmersionLink *iml = b->firstimmersionlink; iml ; iml = (dxImmersionLink*)iml->nextBodyImmersionLink, ++i) {
    if (i == index) return iml;
  }
  return 0;
}

int dFluidGetNumImmersionLinks (dFluidID f)
{
  dAASSERT (f);
  int count = 0;
  for (dxImmersionLink *iml = f->firstimmersionlink; iml; iml = (dxImmersionLink*)iml->nextFluidImmersionLink, ++count);
  return count;
}

dImmersionLinkID dFluidGetImmersionLink (dFluidID f, int index)
{
  dAASSERT (f);
  int i = 0;
  for (dxImmersionLink *iml = f->firstimmersionlink; iml ; iml = (dxImmersionLink*)iml->nextFluidImmersionLink, ++i) {
    if (i == index) return iml;
  }
  return 0;
}

dImmersionOutlineID dImmersionOutlineCreate () {
  return new dxImmersionOutline;
}

void dImmersionOutlineDestroy (dImmersionOutlineID io) {
  delete io;
}

int dImmersionOutlineGetStraightEdgesSize (dImmersionOutlineID io) {
  return io->straightEdgesSize();
}

void dImmersionOutlineGetStraightEdge (dImmersionOutlineID io, int index, dStraightEdge *se) {
  memcpy(se, &(io->straightEdge(index)), sizeof(dStraightEdge));
}

const dReal *dImmersionOutlineGetStraightEdgeEnd (dImmersionOutlineID io, int index) {
  return io->straightEdge(index).end;
}

const dReal *dImmersionOutlineGetStraightEdgeOrigin (dImmersionOutlineID io, int index) {
  return io->straightEdge(index).origin;
}

int dImmersionOutlineGetCurvedEdgesSize (dImmersionOutlineID io) {
  return io->curvedEdgesSize();
}

void dImmersionOutlineGetCurvedEdge (dImmersionOutlineID io, int index, dCurvedEdge *ce) {
  memcpy(ce, &(io->curvedEdge(index)), sizeof(dCurvedEdge));
}
