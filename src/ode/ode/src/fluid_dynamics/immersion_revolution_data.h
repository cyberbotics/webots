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

#ifndef _ODE_REVOLUTION_DATA_H_
#define _ODE_REVOLUTION_DATA_H_

/*

Data used to cache cylinder and capsule parameters during immersion computations

*/

#include <ode/objects.h>

#include <ode/common.h>
#include <ode/odemath.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/fluid_dynamics/immersion.h>
#include "../collision_kernel.h"
#include "../collision_std.h"

struct sRevolutionImmersionData
{
  void makeStandardBasis(dReal &nX, dReal &nZ, bool makeTanThetaPositive);
  virtual int performImmersionChecking();
  int oneFullyImmersedDisk(dReal sign);
  int onePartiallyImmersedDisk(dReal sign);
  int twoPartiallyImmersedDisks();
  void initBasisVectors() {
    m_vEz[0] = m_mRotation[2];
    m_vEz[1] = m_mRotation[6];
    m_vEz[2] = m_mRotation[10];

    dCalcVectorCross3(m_vEy, m_vEz, m_vN);
  }
  inline void initDisksData();

  const dxGeom *m_gRevolutionGeom;
  int m_iFlags;
  dImmersionGeom *m_gImmersion;

  // cylinder parameters
  bool m_bComplement, m_bOrthogonal;
  const dReal *m_mRotation;
  const dReal *m_vCenter;
  bool m_bUpperDiskIsPartiallyImmersed, m_bLowerDiskIsPartiallyImmersed;
  bool m_bUpperDiskIsFullyImmersed, m_bLowerDiskIsFullyImmersed;
  dVector3 m_vN, m_vEy, m_vEx, m_vEz;
  dReal m_fHeight, m_fHalfHeight;
  dReal m_fSinusTheta, m_fCosinusTheta;
  dReal m_fRadius, m_fRadiusSquare;
  dReal m_fCenterDepth, m_fDepthPlus, m_fDepthMinus;

  sRevolutionImmersionData(const dxGeom *geom, const dReal *fluidPlane, int flags, dImmersionGeom *immersion):
    m_gRevolutionGeom(geom), m_iFlags(flags), m_gImmersion(immersion), m_bComplement(false), m_bOrthogonal(false),
    m_mRotation(m_gRevolutionGeom->final_posr->R),
    m_vCenter(m_gRevolutionGeom->final_posr->pos),
    m_bUpperDiskIsPartiallyImmersed(false), m_bLowerDiskIsPartiallyImmersed(false),
    m_bUpperDiskIsFullyImmersed(false), m_bLowerDiskIsFullyImmersed(false)
    {
      dUASSERT(m_gRevolutionGeom && (m_gRevolutionGeom->type == dCylinderClass || m_gRevolutionGeom->type == dCapsuleClass),
        "argument not a cylinder nor a capsule");
      dCopyVector3(m_vN, fluidPlane);
      m_vN[3] = fluidPlane[3];
      m_gImmersion->volume = 0.0;
      m_gImmersion->area = 0.0;
      m_gImmersion->projectedAreas[0] = 0.0;
      m_gImmersion->projectedAreas[1] = 0.0;
      m_gImmersion->projectedAreas[2] = 0.0;
    }
};

void sRevolutionImmersionData::initDisksData() {
  const dReal sinusThetaSquare = dCalcVectorLengthSquare3(m_vEy);
  m_fSinusTheta = dSqrt(sinusThetaSquare);
  m_fCenterDepth = m_vN[3] - dCalcVectorDot3(m_vCenter, m_vN);
  m_fCosinusTheta = dCalcVectorDot3(m_vEz, m_vN);

  const dReal ch = m_fCosinusTheta * m_fHalfHeight;
  const dReal sr = m_fSinusTheta * m_fRadius;

  m_fDepthPlus = m_fCenterDepth - ch;
  m_bUpperDiskIsPartiallyImmersed = m_fDepthPlus > -sr;
  m_bUpperDiskIsFullyImmersed = m_fDepthPlus > sr;

  m_fDepthMinus = m_fCenterDepth + ch;
  m_bLowerDiskIsPartiallyImmersed = m_fDepthMinus > -sr;
  m_bLowerDiskIsFullyImmersed = m_fDepthMinus > sr;

  m_fRadiusSquare = m_fRadius * m_fRadius;
}

struct sCylinderImmersionData: sRevolutionImmersionData
{
  sCylinderImmersionData(const dxGeom *geom, const dReal *fluidPlane, int flags, dImmersionGeom *immersion):
    sRevolutionImmersionData(geom, fluidPlane, flags, immersion),
    m_gCylinder((dxCylinder *) m_gRevolutionGeom)
  {
    dUASSERT(m_gRevolutionGeom && m_gRevolutionGeom->type == dCylinderClass, "argument not a cylinder");
    m_fHeight = m_gCylinder->lz;
    m_fHalfHeight = 0.5 * m_fHeight;
    m_fRadius = m_gCylinder->radius;
    initBasisVectors();
    initDisksData();
  }

  const dxCylinder *m_gCylinder;
};

struct sCapsuleImmersionData: sRevolutionImmersionData
{

  sCapsuleImmersionData(const dxGeom *geom, const dReal *fluidPlane, int flags, dImmersionGeom *immersion):
    sRevolutionImmersionData(geom, fluidPlane, flags, immersion),
    m_gCapsule((dxCapsule *) m_gRevolutionGeom)
  {
    dUASSERT(m_gRevolutionGeom && m_gRevolutionGeom->type == dCapsuleClass, "argument not a capsule");
    m_fHeight = m_gCapsule->lz;
    m_fHalfHeight = 0.5 * m_fHeight;
    m_fRadius = m_gCapsule->radius;
    initBasisVectors();
    initDisksData();
  }

  virtual int performImmersionChecking();
  int oneFullyImmersedHemisphere(dReal sign);
  int oneImmersedSphericalCap(dReal sign);
  int twoOrthogonalPartiallyImmersedHemispheres();
  int onePartiallyImmersedHemisphereDisk(dReal sign);
  inline void addImmersionData(const dVector3 addedCenter, dReal addedVolume, dReal addedArea);

  dReal immersedHemisphereArea(dReal zIOverR, dReal tanTheta0, dReal sinTheta1) const;
  dReal normalizedImmersedHemisphereVolume(dReal zIOverR, dReal tanTheta0, dReal sinTheta1) const;
  dReal immersedHemisphereVolume(dReal zIOverR, dReal tanTheta0, dReal sinTheta1) const {
    return m_fRadiusSquare * m_fRadius * normalizedImmersedHemisphereVolume(zIOverR, tanTheta0, sinTheta1);
  }
  dReal immersedHemisphereXBuoyancyCenter(dReal zIOverR, dReal tanTheta0, dReal sinTheta1, dReal volume) const;
  dReal immersedHemisphereZBuoyancyCenter(dReal zIOverR, dReal tanTheta0, dReal sinTheta1, dReal volume) const;

  const dxCapsule *m_gCapsule;
};

void sCapsuleImmersionData::addImmersionData(const dVector3 addedCenter, dReal addedVolume, dReal addedArea) {
  const dReal previousVolume = m_gImmersion->volume;
  m_gImmersion->volume += addedVolume;
  m_gImmersion->area += addedArea;
  dReal *const b = m_gImmersion->buoyancyCenter;
  dAddScaledVectors3(b, b, addedCenter, previousVolume / m_gImmersion->volume, addedVolume / m_gImmersion->volume);
}

struct sHemisphereIntegrationData
{
  sHemisphereIntegrationData(dReal zIOverR, dReal invTanTheta0) : m_fZIOverR(zIOverR), m_fInvTanTheta0(invTanTheta0) {}

  dReal m_fZIOverR, m_fInvTanTheta0;

  inline dReal areaIntegrand(dReal theta) const;
  inline dReal volumeIntegrand(dReal z) const;
  inline dReal xCenterOfMassIntegrand(dReal z) const;
  inline dReal zCenterOfMassIntegrand(dReal z) const;
};

dReal sHemisphereIntegrationData::areaIntegrand(dReal theta) const {
  const dReal cosinusTheta = cos(theta);
  const dReal sinusTheta = dSqrt(1.0 - cosinusTheta * cosinusTheta);
  dReal cosinusPhi = m_fInvTanTheta0 * (sinusTheta - m_fZIOverR) / cosinusTheta;
  if (dFabs(cosinusPhi) > 1.0)
    cosinusPhi = cosinusPhi > 0.0 ? 1.0 : -1.0;
  return cosinusTheta * acos(cosinusPhi);
}

dReal sHemisphereIntegrationData::volumeIntegrand(dReal z) const {
  const dReal rhoSquare = 1.0 - z * z;
  const dReal rho = dSqrt(rhoSquare);
  const dReal x = (z - m_fZIOverR) * m_fInvTanTheta0;
  dReal cosinusPhi = x / rho;
  if (dFabs(cosinusPhi) > 1.0)
    cosinusPhi = cosinusPhi > 0.0 ? 1.0 : -1.0;
  const dReal phi = acos(cosinusPhi);
  const dReal temp = rhoSquare - x * x;
  return temp > 0.0 ? rhoSquare * phi - x * dSqrt(temp) : rhoSquare * phi;
}

dReal sHemisphereIntegrationData::xCenterOfMassIntegrand(dReal z) const {
  const dReal rhoSquare = 1.0 - z * z;
  const dReal x = (z - m_fZIOverR) * m_fInvTanTheta0;
  const dReal temp = rhoSquare - x * x;
  return temp > 0.0 ? temp * dSqrt(temp) : 0.0;
}

dReal sHemisphereIntegrationData::zCenterOfMassIntegrand(dReal z) const {
  return z * volumeIntegrand(z);
}

ODE_PURE_INLINE dReal circularSegmentArea(dReal phi, dReal sinusPhi, dReal cosinusPhi) {
  return phi - sinusPhi * cosinusPhi;
}

ODE_PURE_INLINE dReal cylinderSectionVolume(dReal phi, dReal sinusPhi, dReal cosinusPhi) {
  return sinusPhi * (1.0 - sinusPhi * sinusPhi / 3.0) - phi * cosinusPhi;
}

ODE_PURE_INLINE void computeCylinderSectionGamma(dReal gamma[2], dReal phi, dReal sinusPhi, dReal cosinusPhi) {
  const dReal sinusPhiCosinusPhi = sinusPhi * cosinusPhi;
  const dReal cosinus2Phi = 2.0 * cosinusPhi * cosinusPhi - 1.0;
  const dReal sinus4PhiOver4 = sinusPhiCosinusPhi * cosinus2Phi;
  gamma[0] = 3.0 * phi - 4.0 * sinusPhiCosinusPhi + sinus4PhiOver4;
  gamma[1] = 2.0 * sinusPhiCosinusPhi - 2.0 * phi * cosinus2Phi - phi + sinus4PhiOver4;
}

struct MyTest {
  void run();
  dReal f1(dReal x) const { return dSqrt(1.0  -  x * x); }
  dReal f2(dReal x) const { return acos(x); }
  dReal f3(dReal x) const { return sin(x); }
};

#endif
