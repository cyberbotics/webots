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

#include <ode/common.h>
#include <ode/fluid_dynamics/immersion.h>
#include <ode/fluid_dynamics/fluid_dynamics.h>
#include <ode/fluid_dynamics/odemath_fluid_dynamics.h>
#include "immersion_revolution_data.h"
#include "../collision_kernel.h"
#include "../collision_std.h"
#include "../collision_trimesh_internal.h"
#include "immersion_std.h"
#include "immersion_kernel.h"
#include "immersion_outline.h"

#ifdef _MSC_VER
#pragma warning(disable:4291)  // for VC++, no complaints about "no matching operator delete found"
#endif

dReal dGeomCapsuleGetVolume (dGeomID g)
 {
   dUASSERT (g && g->type == dCapsuleClass,"argument not a capsule");

   const dxCapsule *const c = (dxCapsule*) g;
   const dReal r = c->radius;
   const dReal h = c->lz;
   return M_PI * r * r * (h + (4.0 / 3.0) * r);
 }

dReal dGeomCapsuleGetArea (dGeomID g)
 {
   dUASSERT (g && g->type == dCapsuleClass,"argument not a capsule");

   const dxCapsule *const c = (dxCapsule*) g;
   const dReal r = c->radius;
   const dReal h = c->lz;

   return 2.0 * M_PI * r * (h + 2.0 * r);
 }

void dGeomCapsuleGetTangentPlane (dGeomID g, dReal x, dReal y, dReal z, dVector4 plane)
 {
   dUASSERT (g && g->type == dCapsuleClass,"argument not a capsule");

   const dxCapsule *const c = (dxCapsule*) g;
   const dReal *const R = c->final_posr->R;
   const dReal *const center = c->final_posr->pos;

   static const dReal ZERO_THRESHOLD = 1e-3;
   const dVector3 n = { R[2], R[6], R[10] };
   const dVector3 p = { x, y, z};
   dVector3 delta;
   const dReal h = 0.5 * c->lz;
   dAddScaledVectors3(delta, center, n, 1.0, h);
   dSubtractVectors3(delta, p, delta);
   const dReal r = c->radius;

   if (dCalcVectorDot3(delta, n) >= 0.0 && dCalcVectorLengthSquare3(delta) - r * r < ZERO_THRESHOLD) {
     plane[0] = delta[0];
     plane[1] = delta[1];
     plane[2] = delta[2];
     plane[3] = x * plane[0] + y * plane[1] + z * plane[2];
     make_sure_plane_normal_has_unit_length(plane);
     return;
   }

   dAddScaledVectors3(delta, center, n, 1.0, -h);
   dSubtractVectors3(delta, p, delta);

   if (dCalcVectorDot3(delta, n) <= 0.0 && dCalcVectorLengthSquare3(delta) - r * r < ZERO_THRESHOLD) {
     plane[0] = delta[0];
     plane[1] = delta[1];
     plane[2] = delta[2];
     plane[3] = x * plane[0] + y * plane[1] + z * plane[2];
     make_sure_plane_normal_has_unit_length(plane);
     return;
   }

   dVector3 v = { x - center[0], y - center[1], z - center[2] };
   const dReal l = dCalcVectorDot3(v, n);
   dAddScaledVectors3(v, v, n, 1.0, -l);
   if (dFabs(l) <= h && dCalcVectorLengthSquare3(v) - r * r < ZERO_THRESHOLD) {
     plane[0] = v[0];
     plane[1] = v[1];
     plane[2] = v[2];
     plane[3] = x * plane[0] + y * plane[1] + z * plane[2];
     make_sure_plane_normal_has_unit_length(plane);
     return;
   }

  dSetZero(plane, 4);
  plane[2] = 1.0;
 }

void dGeomCapsuleGetImmersionPlane (dGeomID g, dVector4 plane) {
  dUASSERT (g && g->type == dCapsuleClass,"argument not a capsule");
  const dxCapsule *const capsule = (dxCapsule *) g;
  const dReal *const center = capsule->final_posr->pos;
  const dReal *const R = capsule->final_posr->R;
  const dReal k = 0.5 * capsule->lz + capsule->radius;
  dVector3 p = { center[0] + k * R[2], center[1] + k * R[6], center[2] + k * R[10] };
  dGeomCapsuleGetTangentPlane(g, p[0], p[1], p[2], plane);
}

dReal sCapsuleImmersionData::immersedHemisphereArea(dReal zIOverR, dReal tanTheta0, dReal sinTheta1) const {
  static const int n = 50;
  static const dReal singularityFactor = 0.9;

  const sHemisphereIntegrationData data(zIOverR, 1.0 / tanTheta0);
  const dReal end = asin(sinTheta1);
  const dReal singularityStart = singularityFactor * end;
  dReal area = simpson<sHemisphereIntegrationData, &sHemisphereIntegrationData::areaIntegrand>(0.0, singularityStart, n, data);
  area += simpson<sHemisphereIntegrationData, &sHemisphereIntegrationData::areaIntegrand>(singularityStart, end, n, data);
  return 2.0 * m_fRadiusSquare * area;
}

dReal sCapsuleImmersionData::normalizedImmersedHemisphereVolume(dReal zIOverR, dReal tanTheta0, dReal sinTheta1) const {
  static const int n = 50;
  static const dReal singularityFactor = 0.9;

  const dReal singularityStart = singularityFactor * sinTheta1;
  const sHemisphereIntegrationData data(zIOverR, 1.0 / tanTheta0);
  dReal volume = simpson<sHemisphereIntegrationData, &sHemisphereIntegrationData::volumeIntegrand>(0.0, singularityStart, n, data);
  volume += simpson<sHemisphereIntegrationData, &sHemisphereIntegrationData::volumeIntegrand>(singularityStart, sinTheta1, n, data);
  return volume;
}

dReal sCapsuleImmersionData::immersedHemisphereXBuoyancyCenter(dReal zIOverR, dReal tanTheta0, dReal sinTheta1, dReal volume) const {
  static const int n = 100;
  const sHemisphereIntegrationData data(zIOverR, 1.0 / tanTheta0);
  const dReal xb = simpson<sHemisphereIntegrationData, &sHemisphereIntegrationData::xCenterOfMassIntegrand>(0.0, sinTheta1, n, data);
  return (2.0 / 3.0) * m_fRadius * xb / volume;
}

dReal sCapsuleImmersionData::immersedHemisphereZBuoyancyCenter(dReal zIOverR, dReal tanTheta0, dReal sinTheta1, dReal volume) const {
  static const int n = 100;
  const sHemisphereIntegrationData data(zIOverR, 1.0 / tanTheta0);
  const dReal zb = simpson<sHemisphereIntegrationData, &sHemisphereIntegrationData::zCenterOfMassIntegrand>(0.0, sinTheta1, n, data);
  return m_fRadius * zb / volume;
}

int sCapsuleImmersionData::oneFullyImmersedHemisphere(dReal sign) {
  dVector3 center;
  dAddScaledVectors3(center, m_vCenter, m_vEz, 1.0, sign * m_fHalfHeight);
  dVector3 immersedHemisphereBuoyancyCenter;
  dAddScaledVectors3(immersedHemisphereBuoyancyCenter, center, m_vEz, 1.0 , sign * (3.0 / 8.0) * m_fRadius);

  addImmersionData(immersedHemisphereBuoyancyCenter, (2.0 * M_PI / 3.0) * m_fRadiusSquare * m_fRadius, 2.0 * M_PI * m_fRadiusSquare);

  return m_gImmersion->volume > VOLUME_ZERO_THRESHOLD ? 1 : 0;
}

int sCapsuleImmersionData::oneImmersedSphericalCap(dReal sign) {
  const dReal zI = sign < 0.0 ? m_fDepthMinus : m_fDepthPlus;
  dVector3 center;
  dAddScaledVectors3(center, m_vCenter, m_vEz, 1.0, sign * m_fHalfHeight);
  const bool complement = sign * m_fCosinusTheta > 0.0;

  const dReal temp1 = zI + m_fRadius;
  const dReal temp2 = zI - m_fRadius;
  const dReal temp3 = 2.0 * m_fRadius - zI;
  dReal immersedCapVolume = (M_PI / 3.0) * temp1 * temp1 * temp3;
  dReal immersedCapArea = 2.0 * M_PI * m_fRadius * temp1;
  dReal gammaZ = - 0.75 * temp2 * temp2 / temp3;
  if (complement)
    gammaZ *= -1.0;
  dVector3 immersedCapBuoyancyCenter;
  dAddScaledVectors3(immersedCapBuoyancyCenter, center, m_vN, 1.0 , gammaZ);
  if (complement) {
    const dReal vol = immersedCapVolume;
    const dReal hemisphereFullVolume = (2.0 / 3.0) * M_PI * m_fRadius * m_fRadiusSquare;
    immersedCapVolume = hemisphereFullVolume - immersedCapVolume;
    immersedCapArea = 2.0 * M_PI * m_fRadiusSquare - immersedCapArea;
    dVector3 hemisphereCenterOfMass;
    dAddScaledVectors3(hemisphereCenterOfMass, center, m_vEz, 1.0, sign * (3.0 / 8.0) * m_fRadius);
    dAddScaledVectors3(immersedCapBuoyancyCenter, hemisphereCenterOfMass, immersedCapBuoyancyCenter, hemisphereFullVolume / immersedCapVolume, - vol / immersedCapVolume);
  }

  addImmersionData(immersedCapBuoyancyCenter, immersedCapVolume, immersedCapArea);

   // Immersion outline
  if (m_gImmersion->outline && (m_iFlags & ~dxImmersionOutlineDisabled)) {
    dCurvedEdge ce;
    dAddScaledVectors3(ce.center, center, m_vN, 1.0, zI);
    const dReal r = dSqrt(m_fRadiusSquare - zI * zI);
    dPlaneSpace(m_vN, ce.e1, ce.e2);
    dScaleVector3(ce.e1, r);
    dScaleVector3(ce.e2, r);
    ce.minAngle = 0.0;
    ce.maxAngle = 2.0 * M_PI;
    m_gImmersion->outline->appendCurvedEdge(ce);
  }

  return m_gImmersion->volume > VOLUME_ZERO_THRESHOLD ? 1 : 0;
}

int sCapsuleImmersionData::twoOrthogonalPartiallyImmersedHemispheres() {
  const dReal zI = 0.5 * (m_fDepthPlus + m_fDepthMinus);
  const dReal temp1 = zI + m_fRadius;
  const dReal temp2 = zI - m_fRadius;
  const dReal temp3 = 2.0 * m_fRadius - zI;
  const dReal gammaZ = -0.75 * temp2 * temp2 / temp3;
  dVector3 immersedHemispheresBuoyancyCenter;
  dAddScaledVectors3(immersedHemispheresBuoyancyCenter, m_vCenter, m_vN, 1.0, gammaZ);
  addImmersionData(immersedHemispheresBuoyancyCenter, (M_PI / 3.0) * temp1 * temp1 * temp3, 2.0 * M_PI * m_fRadius * temp1);

  // Immersion outline
  if (m_gImmersion->outline && (m_iFlags & ~dxImmersionOutlineDisabled)) {
    dCurvedEdge ceMinus = {{0,0,0},{0,0,0},{0,0,0},0,0}, cePlus = {{0,0,0},{0,0,0},{0,0,0},0,0};
    dVector3 center;
    dAddScaledVectors3(center, m_vCenter, m_vN, 1.0, zI);
    dAddScaledVectors3(ceMinus.center, center, m_vEz, 1.0, -m_fHalfHeight);
    dAddScaledVectors3(cePlus.center, center, m_vEz, 1.0, m_fHalfHeight);
    const dReal r = dSqrt(m_fRadiusSquare - zI * zI);
    dCalcVectorCross3(m_vEx, m_vN, m_vEz);
    dCopyScaledVector3(ceMinus.e1, m_vEz, r);
    dCopyScaledVector3(ceMinus.e2, m_vEx, r);
    dCopyVector3(cePlus.e1, ceMinus.e1);
    dCopyVector3(cePlus.e2, ceMinus.e2);
    ceMinus.minAngle = M_PI_2;
    ceMinus.maxAngle = 3.0 * M_PI_2;
    cePlus.minAngle = - M_PI_2;
    cePlus.maxAngle = M_PI_2;
    m_gImmersion->outline->appendCurvedEdge(ceMinus);
    m_gImmersion->outline->appendCurvedEdge(cePlus);
  }

  return m_gImmersion->volume > VOLUME_ZERO_THRESHOLD ? 1 : 0;
}

int sCapsuleImmersionData::onePartiallyImmersedHemisphereDisk(dReal sign) {
  //MyTest test;
  //test.run();

  dScaleVector3(m_vEz, sign);
  dScaleVector3(m_vEy, sign);
  m_fCosinusTheta *= sign;
  const dReal zI = sign > 0.0 ? m_fDepthPlus / m_fCosinusTheta : m_fDepthMinus / m_fCosinusTheta;

  dReal nX, nZ;
  makeStandardBasis(nX, nZ, false);
  const dReal tanTheta = -nX / nZ;
  dVector3 center;
  dAddScaledVectors3(center, m_vCenter, m_vEz, 1.0, m_fHalfHeight);

  const dReal cosinusThetaSquare = m_fCosinusTheta * m_fCosinusTheta;
  const dReal quaterOfDeltaX = m_fRadiusSquare / cosinusThetaSquare - zI * zI;
  const dReal deltaX = dSqrt(quaterOfDeltaX);
  const dReal zR = cosinusThetaSquare * (zI + dFabs(tanTheta) * deltaX);
  const dReal sinTheta1 = zR / m_fRadius;
  const dReal zIOverR = zI / m_fRadius;

  const dReal area = immersedHemisphereArea(zIOverR, tanTheta, sinTheta1);
  dIASSERT(area >= 0.0 && area <= 2.0 * M_PI * m_fRadiusSquare);
  dReal normalizedVolume = normalizedImmersedHemisphereVolume(zIOverR, tanTheta, sinTheta1);
  dIASSERT(normalizedVolume >= 0.0 && normalizedVolume <= (2.0 / 3.0) * M_PI);
  const dReal rbc[2] = { immersedHemisphereXBuoyancyCenter(zIOverR, tanTheta, sinTheta1, normalizedVolume),
    immersedHemisphereZBuoyancyCenter(zIOverR, tanTheta, sinTheta1, normalizedVolume) };
  dIASSERT(dFabs(rbc[0]) <= m_fRadius && rbc[1] >= 0.0 && rbc[1] <= m_fRadius);

  dVector3 buoyancyCenter;
  dAddScaledVectors3(buoyancyCenter, m_vEx, m_vEz, rbc[0], rbc[1]);
  dAddVectors3(buoyancyCenter, buoyancyCenter, center);
  addImmersionData(buoyancyCenter, m_fRadius * m_fRadiusSquare * normalizedVolume, area);

  if (tanTheta * (zR - zI) < 0.0) {
    const dReal h = m_fRadius - zR;
    const dReal sphericalSegmentVolume = (M_PI / 3.0) * h * h * (3.0 * m_fRadius - h);
    const dReal sphericalSegmentArea = 2.0 * M_PI * m_fRadius * h;
    const dReal temp = m_fRadius + zR;
    dVector3 sphericalSegmentBuoyancyCenter;
    dAddScaledVectors3(sphericalSegmentBuoyancyCenter, center, m_vEz, 1.0, 0.75 * temp * temp / (m_fRadius + temp));
    addImmersionData(sphericalSegmentBuoyancyCenter, sphericalSegmentVolume, sphericalSegmentArea);
  }

     // Immersion outline
  if (m_gImmersion->outline && (m_iFlags & ~dxImmersionOutlineDisabled)) {
    const dReal halfSumX = - tanTheta * cosinusThetaSquare * zI;
    const dReal halfSumZ = zI * cosinusThetaSquare;
    dCurvedEdge ce = {{0,0,0},{0,0,0},{0,0,0},0,0};
    dAddScaledVectors3(ce.center, m_vEx, m_vEz, halfSumX, halfSumZ);
    dAddVectors3(ce.center, ce.center, center);

    dVector3 x0;
    dAddScaledVectors3(x0, center, m_vEx, 1.0, - zI / tanTheta);
    dSubtractVectors3(x0, x0, ce.center);
    dCalcVectorCross3(ce.e1, m_vEy, m_vN);
    const dReal x = dCalcVectorDot3(x0, ce.e1);
    const dReal r = dFabs(m_fCosinusTheta) * deltaX;
    dCopyScaledVector3(ce.e2, m_vEy, r);
    dScaleVector3(ce.e1, r);

    dReal cosMaxAngle = x / r;
    if (dFabs(cosMaxAngle) > 1.0)
      cosMaxAngle = cosMaxAngle > 0.0 ? 1.0 : -1.0;
    ce.maxAngle = acos(cosMaxAngle);
    ce.minAngle = - ce.maxAngle;
    m_gImmersion->outline->appendCurvedEdge(ce);
  }

  // Restores original setting
  dScaleVector3(m_vEy, sign);
  dScaleVector3(m_vEz, sign);
  m_fCosinusTheta *= sign;

  return m_gImmersion->volume > VOLUME_ZERO_THRESHOLD ? 1 : 0;
}

int sCapsuleImmersionData::performImmersionChecking() {
  const bool lowerHemisphereIsFullyImmersed = m_fCosinusTheta < 0.0 ? m_fDepthMinus > m_fRadius : m_bLowerDiskIsFullyImmersed;
  const bool lowerHemisphereIsPartiallyImmersed = lowerHemisphereIsFullyImmersed || (m_fCosinusTheta < 0.0 ? m_bLowerDiskIsPartiallyImmersed > 0.0 : m_fDepthMinus > -m_fRadius);
  const bool upperHemisphereIsFullyImmersed = m_fCosinusTheta > 0.0 ? m_fDepthPlus > m_fRadius : m_bUpperDiskIsFullyImmersed;
  const bool upperHemisphereIsPartiallyImmersed = upperHemisphereIsFullyImmersed || (m_fCosinusTheta > 0.0 ? m_bUpperDiskIsPartiallyImmersed : m_fDepthPlus > -m_fRadius);

  if (!lowerHemisphereIsPartiallyImmersed && !upperHemisphereIsPartiallyImmersed)
    return 0;

  if (lowerHemisphereIsFullyImmersed && upperHemisphereIsFullyImmersed) {
    m_gImmersion->volume = M_PI * m_fRadiusSquare * (m_fHeight + (4.0 / 3.0) * m_fRadius);
    m_gImmersion->area = 2.0 * M_PI * m_fRadius * (2.0 * m_fRadius + m_fHeight);
    m_gImmersion->projectedAreas[0] = m_fRadius * m_fHeight + M_PI * m_fRadiusSquare;
    m_gImmersion->projectedAreas[1] = m_gImmersion->projectedAreas[0];
    m_gImmersion->projectedAreas[2] = M_PI * m_fRadiusSquare;
    dCopyVector3(m_gImmersion->buoyancyCenter, m_vCenter);
    return m_gImmersion->volume > VOLUME_ZERO_THRESHOLD ? 1 : 0;
  }

  int result = sRevolutionImmersionData::performImmersionChecking();

  if (m_bOrthogonal)
    return twoOrthogonalPartiallyImmersedHemispheres();

  const bool lowerSphericalCapImmersion = lowerHemisphereIsPartiallyImmersed && !lowerHemisphereIsFullyImmersed && (m_bLowerDiskIsFullyImmersed || !m_bLowerDiskIsPartiallyImmersed);
  const bool upperSphericalCapImmersion = upperHemisphereIsPartiallyImmersed && !upperHemisphereIsFullyImmersed && (m_bUpperDiskIsFullyImmersed || !m_bUpperDiskIsPartiallyImmersed);

  if (lowerHemisphereIsFullyImmersed || upperHemisphereIsFullyImmersed)
    result += oneFullyImmersedHemisphere(lowerHemisphereIsFullyImmersed ? -1.0 : 1.0);

  if (lowerSphericalCapImmersion || upperSphericalCapImmersion)
    result += oneImmersedSphericalCap(lowerSphericalCapImmersion ? -1.0 : 1.0);

  if (m_bLowerDiskIsPartiallyImmersed && !m_bLowerDiskIsFullyImmersed)
    result += onePartiallyImmersedHemisphereDisk(-1.0);

  if (m_bUpperDiskIsPartiallyImmersed && !m_bUpperDiskIsFullyImmersed)
    result += onePartiallyImmersedHemisphereDisk(1.0);

  return result > 0 ? 1 : 0;
}

int dImmerseCapsule (const dxCapsule *capsule, const dReal *fluidPlane, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT(capsule->body, "This capsule has no body and hence cannot be tested for immersion into a fluid");

  sCapsuleImmersionData data(capsule, fluidPlane, flags, immersion);
  return data.performImmersionChecking();
}

int dImmerseCapsuleBox (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseCapsuleBox has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideCapsuleBox(o1, o2, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  const dxCapsule *const capsule = (dxCapsule *) o1;
  dxBox *const box = (dxBox *) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomBoxGetImmersionPlane(box, fluidPlane);
  return dImmerseCapsule (capsule, fluidPlane, flags, immersion);
}

int dImmerseCapsuleCapsule (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseCapsuleCapsule has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideCapsuleCapsule(o1, o2, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  const dxCapsule *const capsule1 = (dxCapsule *) o1;
  dxCapsule *const capsule2 = (dxCapsule *) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomCapsuleGetImmersionPlane(capsule2, fluidPlane);

  return dImmerseCapsule (capsule1, fluidPlane, flags, immersion);
}

int dImmerseCapsuleCylinder (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseCapsuleCylinder has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideCylinderCapsule(o2, o1, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  const dxCapsule *const capsule1 = (dxCapsule *) o1;
  dxCylinder *const cylinder = (dxCylinder *) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomCylinderGetImmersionPlane(cylinder, fluidPlane);

  return dImmerseCapsule (capsule1, fluidPlane, flags, immersion);
}

int dImmerseCapsuleSphere (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseCapsuleSphere has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideCapsuleSphere(o1, o2, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  const dxCapsule *const capsule = (dxCapsule *) o1;
  dxSphere *const sphere = (dxSphere *) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  const dReal *const rotation = sphere->final_posr->R;
  const dReal *const center = sphere->final_posr->pos;
  const dReal r = sphere->radius;
  const dVector3 p = { r * rotation[0 + FLUID_PLANE_NORMAL] + center[0], r * rotation[4 + FLUID_PLANE_NORMAL] +
    center[1], r * rotation[8 + FLUID_PLANE_NORMAL] + center[2] };
  dVector4 fluidPlane;
  dGeomSphereGetTangentPlane(sphere, p[0], p[1], p[2], fluidPlane);

  return dImmerseCapsule (capsule, fluidPlane, flags, immersion);
}

int dImmerseCapsuleTrimesh (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseCapsuleTrimesh has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideCCTL(o2, o1, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomTriMeshGetImmersionPlane(o2, fluidPlane);

  return dImmerseCapsule ((dxCapsule *) o1, fluidPlane, flags, immersion);
}

int dImmerseCapsulePlane (dxGeom *o1, dxGeom *o2, int flags,
      dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dIASSERT (o1->type == dCapsuleClass);
  dIASSERT (o2->type == dPlaneClass);
  dUASSERT(o1-> body && o2->fluid, "The first argument of dImmerseCapsulePlane has no body or the second has no fluid");

  immersion->g1 = o1;
  immersion->g2 = o2;

  const dxCapsule *const capsule = (dxCapsule*) o1;
  const dxPlane *const plane = (dxPlane*) o2;

  return dImmerseCapsule (capsule, plane->p, flags, immersion);
}
