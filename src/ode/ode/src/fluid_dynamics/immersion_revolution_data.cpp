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
#include "immersion_revolution_data.h"

#include <ode/objects.h>
#include <ode/common.h>
#include <ode/fluid_dynamics/fluid_dynamics.h>
#include <ode/fluid_dynamics/odemath_fluid_dynamics.h>
#include <ode/fluid_dynamics/immersion.h>
#include "../collision_kernel.h"
#include "../collision_std.h"

#include "immersion_kernel.h"
#include "immersion_outline.h"
#include "immersion_std.h"

#ifdef _MSC_VER
#pragma warning(disable:4291)  // for VC++, no complaints about "no matching operator delete found"
#endif

int sRevolutionImmersionData::oneFullyImmersedDisk(dReal sign) {
  const dReal depth = sign > 0.0 ? m_fDepthPlus : m_fDepthMinus;
  dReal nX, nZ;
  dScaleVector3(m_vEz, -sign);
  dScaleVector3(m_vEy, -sign);
  m_fCosinusTheta *= -sign;
  const dReal zI = m_fCenterDepth / m_fCosinusTheta;

  const bool parallel = m_fSinusTheta < 0.001;

  if (parallel) {
    m_gImmersion->volume = M_PI * m_fRadiusSquare * depth;
    m_gImmersion->area = 2.0 * M_PI * m_fRadius * (m_fRadius + depth);
    m_gImmersion->projectedAreas[0] = m_fRadius * depth;
    m_gImmersion->projectedAreas[1] = m_fRadius * depth;
    m_gImmersion->projectedAreas[2] = M_PI * m_fRadiusSquare;
    dAddScaledVectors3(m_gImmersion->buoyancyCenter, m_vCenter, m_vEz, 1.0, 0.5 * (zI - m_fHalfHeight));
  } else {
    makeStandardBasis(nX, nZ, true);

    m_gImmersion->volume = M_PI * m_fRadiusSquare * (zI + m_fHalfHeight);
    m_gImmersion->area = M_PI * m_fRadius * (zI + m_fHeight + m_fRadius);

    const dReal tanTheta = - nX / nZ;
    const dReal hPrime = zI - m_fRadius * tanTheta + m_fHalfHeight;
    const dVector3 gamma0 = { 0.0, 0.0, - m_fHalfHeight + 0.5 * hPrime };
    const dVector3 gamma1 = { 0.25 * m_fRadius, 0.0, zI - 3.0 * (m_fRadius * tanTheta) / 8.0 };
    const dReal temp = M_PI * m_fRadiusSquare;
    const dReal V0 = temp * hPrime; // volume of a subcylinder
    const dReal V1 = temp * tanTheta * m_fRadius; // volume of a half-subcylinder
    dVector3 b;
    dAddScaledVectors3(b, gamma0, gamma1, V0 / m_gImmersion->volume, V1 / m_gImmersion->volume);

    // Absolute coordinates
    dAddScaledVectors3(m_gImmersion->buoyancyCenter, m_vEx, m_vEz, b[0], b[2]);
    dAddVectors3(m_gImmersion->buoyancyCenter, m_gImmersion->buoyancyCenter, m_vCenter);
  }
  // Immersion outline
  if (m_gImmersion->outline && (m_iFlags & ~dxImmersionOutlineDisabled))  {
    dCurvedEdge ce;
    if (parallel) {
      ce.e1[0] =  m_fRadius * m_mRotation[0];
      ce.e1[1] =  m_fRadius * m_mRotation[4];
      ce.e1[2] =  m_fRadius * m_mRotation[8];
      ce.e2[0] =  m_fRadius * m_mRotation[1];
      ce.e2[1] =  m_fRadius * m_mRotation[5];
      ce.e2[2] =  m_fRadius * m_mRotation[9];
    } else {
      dAddScaledVectors3(ce.e1, m_vEx, m_vN, 1.0, -nX);
      dNormalize3(ce.e1);
      dScaleVector3(ce.e1, m_fRadius / m_fCosinusTheta);
      dCopyScaledVector3(ce.e2, m_vEy, m_fRadius);
    }

    dAddScaledVectors3(ce.center, m_vCenter, m_vEz, 1.0, zI);
    ce.minAngle = 0.0;
    ce.maxAngle = 2.0 * M_PI;
    m_gImmersion->outline->appendCurvedEdge(ce);
  }

  dScaleVector3(m_vEz, -sign);
  return m_gImmersion->volume > VOLUME_ZERO_THRESHOLD ? 1 : 0;
}

void sRevolutionImmersionData::makeStandardBasis(dReal &nX, dReal &nZ, bool makeTanThetaPositive) {
  dNormalize3(m_vEy);
  dCalcVectorCross3(m_vEx, m_vEy, m_vEz);
  nX = dCalcVectorDot3(m_vN, m_vEx);
  nZ = dCalcVectorDot3(m_vN, m_vEz);
  const dReal z = makeTanThetaPositive ? nZ : 1.0;
  const dReal signX = nX * z > 0.0 ? -1.0 : 1.0;
  dScaleVector3(m_vEx, signX);
  dScaleVector3(m_vEy, signX);
  nX *= signX;
}

int sRevolutionImmersionData::onePartiallyImmersedDisk(dReal sign) {
  const dReal complementSign = m_bComplement ? -1.0 : 1.0;
  dScaleVector3(m_vN, complementSign);
  sign = -sign;
  dScaleVector3(m_vEz, sign);
  dScaleVector3(m_vEy, complementSign * sign);
  m_fCosinusTheta *= sign;
  const dReal zI = m_fCenterDepth / m_fCosinusTheta;

  dReal nX, nZ;
  makeStandardBasis(nX, nZ, true);

  const dReal tanTheta = -nX / nZ;
  const dReal x = -(m_fHalfHeight + zI) / tanTheta;
  const dReal cosinusPhi = x / m_fRadius;
  const dReal phi = acos(cosinusPhi);
  const dReal sinusPhi = dSqrt(1.0 - cosinusPhi * cosinusPhi);

  const dReal t = cylinderSectionVolume(phi, sinusPhi, cosinusPhi);
  m_gImmersion->volume = m_fRadius * m_fRadiusSquare * tanTheta * t;
  m_gImmersion->area = m_fRadiusSquare * (2.0 * tanTheta * (sinusPhi - phi * cosinusPhi) + phi - sinusPhi * cosinusPhi);
  const bool geomIsCylinder = dGeomGetClass(const_cast<dxGeom *>(m_gRevolutionGeom)) == dCylinderClass;
  if (geomIsCylinder)
    m_gImmersion->area += m_fRadiusSquare * (phi - sinusPhi * cosinusPhi);

  dReal gamma[2];
  computeCylinderSectionGamma(gamma, phi, sinusPhi, cosinusPhi);
  dReal b[2] = { m_fRadius * gamma[0] / (12.0 * t), m_fRadius * tanTheta * gamma[1] / (8.0 * t) + zI };

  // Absolute coordinates
  dAddScaledVectors3(m_gImmersion->buoyancyCenter, m_vEx, m_vEz, b[0], b[1]);

  // Immersion outline
  if (m_gImmersion->outline && (m_iFlags & ~dxImmersionOutlineDisabled)) {
    dCurvedEdge ce;
    dAddScaledVectors3(ce.e1, m_vEx, m_vN, 1.0, -nX);
    dNormalize3(ce.e1);
    dVector3 ue1;
    dCopyVector3(ue1, ce.e1);
    dScaleVector3(ce.e1, m_fRadius / dFabs(m_fCosinusTheta));
    dCopyScaledVector3(ce.e2, m_vEy, m_fRadius);

    dAddScaledVectors3(ce.center, m_vCenter, m_vEz, 1.0, zI);

    ce.minAngle = - phi;
    ce.maxAngle =  phi;
    m_gImmersion->outline->appendCurvedEdge(ce);

    if (dGeomGetClass(const_cast<dxGeom *>(m_gRevolutionGeom)) == dCylinderClass) {
      dStraightEdge se = {{0,0,0},{0,0,0}};
      dAddScaledVectors3(se.origin, m_vEx, m_vEz, x, -m_fHalfHeight);
      dAddVectors3(se.origin, se.origin, m_vCenter);
      dCopyVector3(se.end, se.origin);
      const dReal rSinusPhi = m_fRadius * sinusPhi;
      dAddScaledVectors3(se.origin, se.origin, m_vEy, 1.0, rSinusPhi);
      dAddScaledVectors3(se.end, se.end, m_vEy, 1.0, -rSinusPhi);

      m_gImmersion->outline->appendStraightEdge(se);
    }
  }

  if (m_bComplement) {
    const dReal volume = M_PI * m_fRadiusSquare * m_fHeight - m_gImmersion->volume;
    dCopyScaledVector3(m_gImmersion->buoyancyCenter, m_gImmersion->buoyancyCenter, -m_gImmersion->volume / volume);
    m_gImmersion->volume = volume;
    m_gImmersion->area = 2.0 * M_PI * m_fRadius * m_fHeight - m_gImmersion->area;
    if (geomIsCylinder)
      m_gImmersion->area +=  2.0 * M_PI * m_fRadiusSquare;
  }

  dAddVectors3(m_gImmersion->buoyancyCenter, m_gImmersion->buoyancyCenter, m_vCenter);
  // Restores original setting
  dScaleVector3(m_vN, complementSign);
  dScaleVector3(m_vEy, sign);
  dScaleVector3(m_vEz, sign);
  m_fCosinusTheta *= sign;

  return m_gImmersion->volume > VOLUME_ZERO_THRESHOLD ? 1 : 0;
}

int sRevolutionImmersionData::twoPartiallyImmersedDisks() {
  m_bOrthogonal = dFabs(m_fCosinusTheta) < 0.001;
  dReal nX, nZ;
  makeStandardBasis(nX, nZ, false);

  const dReal tanTheta = m_bOrthogonal ? NAN : -nX / nZ;
  const dReal zI = m_bOrthogonal ? NAN : m_fCenterDepth / m_fCosinusTheta;

  const dReal xPlus = m_bOrthogonal ? -m_fCenterDepth : (m_fHalfHeight - zI) / tanTheta;
  const dReal cosinusPhiPlus = xPlus / m_fRadius;
  const dReal phiPlus = acos(cosinusPhiPlus);
  const dReal sinusPhiPlus = dSqrt(1.0 - cosinusPhiPlus * cosinusPhiPlus);
  const dReal normalizedAreaPlus = circularSegmentArea(phiPlus, sinusPhiPlus, cosinusPhiPlus);

  const bool geomIsCylinder = dGeomGetClass(const_cast<dxGeom *>(m_gRevolutionGeom)) == dCylinderClass;

  if (m_bOrthogonal) {
    const dReal temp1 = m_fRadiusSquare * normalizedAreaPlus;
    m_gImmersion->volume = m_fHeight * temp1;
    m_gImmersion->area = 2.0 * m_fHeight * m_fRadius * phiPlus;
    if (geomIsCylinder)
      m_gImmersion->area += 2.0 * temp1;
    const dReal gammaX = 2.0 * m_fRadius * sinusPhiPlus * sinusPhiPlus * sinusPhiPlus / (3.0 * normalizedAreaPlus);
    dAddScaledVectors3(m_gImmersion->buoyancyCenter, m_vCenter, m_vN, 1.0, -gammaX);
  }

  const dReal xMinus = m_bOrthogonal ? xPlus : -(m_fHalfHeight + zI) / tanTheta;
  const dReal cosinusPhiMinus = xMinus / m_fRadius;
  const dReal phiMinus = m_bOrthogonal ? phiPlus : acos(cosinusPhiMinus);
  const dReal sinusPhiMinus = m_bOrthogonal ? sinusPhiPlus : dSqrt(1.0 - cosinusPhiMinus * cosinusPhiMinus);
  const dReal normalizedAreaMinus = circularSegmentArea(phiMinus, sinusPhiMinus, cosinusPhiMinus);

  if (!m_bOrthogonal){
    const dReal temp2 = sinusPhiPlus - sinusPhiMinus - phiPlus * cosinusPhiPlus + phiMinus * cosinusPhiMinus;
    m_gImmersion->area = -2.0 * m_fRadiusSquare * tanTheta * temp2;
    if (geomIsCylinder)
      m_gImmersion->area += m_fRadiusSquare * (normalizedAreaPlus + normalizedAreaMinus);
    const dReal t = cylinderSectionVolume(phiMinus, sinusPhiMinus, cosinusPhiMinus) - cylinderSectionVolume(phiPlus, sinusPhiPlus, cosinusPhiPlus);
    m_gImmersion->volume = m_fRadius * m_fRadiusSquare * tanTheta * t;

    dReal gammaPlus[2];
    computeCylinderSectionGamma(gammaPlus, phiPlus, sinusPhiPlus, cosinusPhiPlus);
    dReal gammaMinus[2];
    computeCylinderSectionGamma(gammaMinus, phiMinus, sinusPhiMinus, cosinusPhiMinus);
    const dReal delta[2] = { gammaMinus[0] - gammaPlus[0], gammaMinus[1] - gammaPlus[1] };
    const dVector3 b = { m_fRadius * delta[0] / (12.0 * t), 0.0, m_fRadius * tanTheta * delta[1] / (8.0 * t) + zI };

    // Absolute coordinates
    dAddScaledVectors3(m_gImmersion->buoyancyCenter, m_vEx, m_vEz, b[0], b[2]);
    dAddVectors3(m_gImmersion->buoyancyCenter, m_gImmersion->buoyancyCenter, m_vCenter);
  }

  // Immersion outline
  if (m_gImmersion->outline && (m_iFlags & ~dxImmersionOutlineDisabled)) {
    dStraightEdge sePlus = {{0,0,0},{0,0,0}}, seMinus = {{0,0,0},{0,0,0}};
    dAddScaledVectors3(sePlus.origin, m_vEx, m_vEz, xPlus, m_fHalfHeight);
    dAddVectors3(sePlus.origin, sePlus.origin, m_vCenter);
    dCopyVector3(sePlus.end, sePlus.origin);
    dAddScaledVectors3(seMinus.origin, m_vEx, m_vEz, xMinus, -m_fHalfHeight);
    dAddVectors3(seMinus.origin, seMinus.origin, m_vCenter);
    dCopyVector3(seMinus.end, seMinus.origin);
    const dReal rSinusPhiPlus = m_fRadius * sinusPhiPlus;
    const dReal rSinusPhiMinus = m_fRadius * sinusPhiMinus;
    dAddScaledVectors3(sePlus.origin, sePlus.origin, m_vEy, 1.0, rSinusPhiPlus);
    dAddScaledVectors3(sePlus.end, sePlus.end, m_vEy, 1.0, -rSinusPhiPlus);
    dAddScaledVectors3(seMinus.origin, seMinus.origin, m_vEy, 1.0, rSinusPhiMinus);
    dAddScaledVectors3(seMinus.end, seMinus.end, m_vEy, 1.0, -rSinusPhiMinus);

    if (dGeomGetClass(const_cast<dxGeom *>(m_gRevolutionGeom)) == dCylinderClass) {
      m_gImmersion->outline->appendStraightEdge(sePlus);
      m_gImmersion->outline->appendStraightEdge(seMinus);
    }

    if (!m_bOrthogonal) {
      dCurvedEdge cePlus = {{0,0,0},{0,0,0},{0,0,0},0,0}, ceMinus = {{0,0,0},{0,0,0},{0,0,0},0,0};
      dAddScaledVectors3(cePlus.e1, m_vEx, m_vN, 1.0, -nX);
      dNormalize3(cePlus.e1);
      dScaleVector3(cePlus.e1, m_fRadius / dFabs(m_fCosinusTheta));
      dCopyScaledVector3(cePlus.e2, m_vEy, m_fRadius);
      dAddScaledVectors3(cePlus.center, m_vCenter, m_vEz, 1.0, zI);
      dCopyVector3(ceMinus.e1, cePlus.e1);
      dCopyVector3(ceMinus.e2, cePlus.e2);
      dCopyVector3(ceMinus.center, cePlus.center);
      const dReal phiMax = phiMinus < phiPlus ? phiPlus : phiMinus;
      const dReal phiMin = phiMinus > phiPlus ? phiPlus : phiMinus;
      cePlus.minAngle = phiMin;
      cePlus.maxAngle = phiMax;
      ceMinus.minAngle = - phiMax;
      ceMinus.maxAngle = - phiMin;
      m_gImmersion->outline->appendCurvedEdge(cePlus);
      m_gImmersion->outline->appendCurvedEdge(ceMinus);
    } else {
      dStraightEdge seXPlus = {{0,0,0},{0,0,0}}, seXMinus = {{0,0,0},{0,0,0}};
      dCopyVector3(seXPlus.origin, sePlus.origin);
      dCopyVector3(seXPlus.end, seMinus.origin);
      dCopyVector3(seXMinus.origin, sePlus.end);
      dCopyVector3(seXMinus.end, seMinus.end);
      m_gImmersion->outline->appendStraightEdge(seXPlus);
      m_gImmersion->outline->appendStraightEdge(seXMinus);
    }
  }

  return m_gImmersion->volume > VOLUME_ZERO_THRESHOLD ? 1 : 0;
}

int sRevolutionImmersionData::performImmersionChecking() {
  if (!m_bUpperDiskIsPartiallyImmersed && !m_bLowerDiskIsPartiallyImmersed)
    return 0;

  if (m_bUpperDiskIsFullyImmersed && m_bLowerDiskIsFullyImmersed) {
    m_gImmersion->volume = M_PI * m_fRadiusSquare * m_fHeight;
    m_gImmersion->area = 2.0 * M_PI * m_fRadius * (m_fRadius + m_fHeight);
    m_gImmersion->projectedAreas[0] = m_fRadius * m_fHeight;
    m_gImmersion->projectedAreas[1] = m_gImmersion->projectedAreas[0];
    m_gImmersion->projectedAreas[2] = M_PI * m_fRadiusSquare;
    dCopyVector3(m_gImmersion->buoyancyCenter, m_vCenter);
    return m_gImmersion->volume > VOLUME_ZERO_THRESHOLD ? 1 : 0;
  }

  if (m_bUpperDiskIsFullyImmersed && !m_bLowerDiskIsPartiallyImmersed)
    return oneFullyImmersedDisk(1.0);
  if (m_bLowerDiskIsFullyImmersed && !m_bUpperDiskIsPartiallyImmersed)
    return oneFullyImmersedDisk(-1.0);

  m_bComplement = m_bUpperDiskIsPartiallyImmersed && m_bLowerDiskIsFullyImmersed;
  if ((m_bUpperDiskIsPartiallyImmersed && !m_bLowerDiskIsPartiallyImmersed) || m_bComplement)
    return onePartiallyImmersedDisk(1.0);

  m_bComplement = m_bLowerDiskIsPartiallyImmersed && m_bUpperDiskIsFullyImmersed;
  if ((m_bLowerDiskIsPartiallyImmersed && !m_bUpperDiskIsPartiallyImmersed) || m_bComplement)
    return onePartiallyImmersedDisk(-1.0);

  return twoPartiallyImmersedDisks();
}

void MyTest::run() {
  static const dReal error = 1e-5;
  static const dReal singularityFactor = 0.95;
  int n = 1;

  dReal result1, result2;
  do {
    result1 = simpson<MyTest, &MyTest::f1>(0.0, singularityFactor, n, *this);
    result2 = simpson<MyTest, &MyTest::f1>(singularityFactor, 1.0, n, *this);
     ++n;
  } while (n < 1000 && fabs(result1 + result2 - M_PI_2 / 2.0) > error);

  printf("test sqrt(1- x * x): %d\n", 2 * (n -1));
  n = 1;

  do {
    result1 = simpson<MyTest, &MyTest::f2>(0.0, singularityFactor, n, *this);
    result2 = simpson<MyTest, &MyTest::f2>(singularityFactor, 1.0, n, *this);
     ++n;
  } while (n < 1000 && fabs(result1 + result2 - 1.0) > error);

  printf("test acos(x): %d\n", 2 * (n -1));
  n = 1;

  while (n < 1000 && fabs(simpson<MyTest, &MyTest::f3>(0.0, M_PI_2, n, *this) - 1.0) > error)
    ++n;

  printf("test sin(x): %d\n", n);
  n = 1;

  int N = 24;
  int half = N % 2 ? -1 : N / 2;
  for (int i = 1; i < N; ++i) {
    if (i == half)
      continue;
    dReal theta = i * M_PI / N;
    const dReal temp1 = sin(theta);
    const dReal temp2 = 1.0 - sin(theta);
    sHemisphereIntegrationData data(0.0, 1.0 / tan(theta));
    const dReal thetaPrime = i > N / 2 ? M_PI - theta : theta;

    const dReal sphericalCapArea = i > N / 2 ? M_PI * temp2 : 0.0;

    do {
      result1 = simpson<sHemisphereIntegrationData, &sHemisphereIntegrationData::areaIntegrand>(0.0, singularityFactor * thetaPrime, n, data);
      result2 = simpson<sHemisphereIntegrationData, &sHemisphereIntegrationData::areaIntegrand>(singularityFactor * thetaPrime, thetaPrime, n, data);
      ++n;
    } while (n < 1000 && fabs(result1 + result2 + sphericalCapArea - theta) > error);
    printf("test area %d Pi / %d: %d\n", i, N, 2 * (n - 1));
    n = 1;

    const dReal sphericalCapVolume = i > N / 2 ? M_PI * temp2 * temp2 * (2.0 + temp1) / 3.0 : 0.0;

    do {
      result1 = simpson<sHemisphereIntegrationData, &sHemisphereIntegrationData::volumeIntegrand>(0.0, singularityFactor * temp1, n, data);
      result2 = simpson<sHemisphereIntegrationData, &sHemisphereIntegrationData::volumeIntegrand>(singularityFactor * temp1, temp1, n, data);
      ++n;
    } while (n < 1000 && fabs(result1 + result2 + sphericalCapVolume - (2.0 / 3.0) * theta) > error);
    printf("test volume %d Pi / %d: %d\n",i, N, 2 * (n - 1));
    n = 1;
  }

}
