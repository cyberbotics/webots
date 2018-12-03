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

#include <iostream>
#include <ode/common.h>
#include "config.h"
#include <ode/collision.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/odemath.h>
#include "collision_kernel.h"
#include "collision_std.h"
#include "collision_util.h"

#define dMessage(...) do {} while(0)

#ifdef _MSC_VER
#pragma warning(disable:4291)  // for VC++, no complaints about "no matching operator delete found"
#endif

struct sZBrentData {
  dReal coeffA[6], coeffB[2], coeffC[3];
  dReal objectiveFunction(dReal t, dReal &distCN) const;
  dReal differential(dReal t) const;
  dReal differential2(dReal t) const;
};

// Data that passed through the collider's functions
struct sCylinderRevolutionData
{
 sCylinderRevolutionData(dxCylinder *cylinder1, dxGeom *revolutionGeom2, int flags, dContactGeom *contact, int skip):
    m_gCylinder1(cylinder1), m_gRevolutionGeom2(revolutionGeom2),
    m_iFlags(flags), m_gContact(contact), m_iSkip(skip), m_nNumberOfContacts(0),
    m_vCenter1(dGeomGetPosition(m_gCylinder1)),
    m_vCenter2(dGeomGetPosition(m_gRevolutionGeom2)),
    m_mRotation1(dGeomGetRotation(m_gCylinder1)),
    m_mRotation2(dGeomGetRotation(m_gRevolutionGeom2))
 {
    dUASSERT (cylinder1 && cylinder1->type == dCylinderClass,"first argument not a cylinder");
    dSubtractVectors3(m_vDelta, m_vCenter2, m_vCenter1);
 }
  inline void init();
  inline void initWData(dReal &radiusMinusDistance, dReal &t1, dReal &t2);
  inline void initProjectedLengths(dReal &ch1, dReal &ch2, dReal &sr1, dReal &sr2);
  inline void initOrthogonalCase(dReal &d12, dReal &d21, dReal &rho1, dReal &rho2);
  inline void initSquares();

  virtual int performCollisionChecking() = 0;
  virtual bool centersPenetration(const dReal *h1n1, const dReal *h2n2);
  void capCentersPenetration(const dReal *h1n1, const dReal *h2n2, dReal ch1, dReal ch2);

  virtual void parallelAxesIntersectionTest(dReal ch1, dReal ch2, dReal hSum1) = 0;
  inline void generatorAgainstGeneratorIntersectionTest(dReal t1, dReal t2, dReal radiusMinusDistance);
  void computeCoefficents(sZBrentData &data, dReal r1, dReal delta1, dReal delta2, dReal t2,
    dReal h1Square, dReal h1Delta1, dReal epsilon, dReal start);
  dReal squaredDistanceFromLinetoCircle(dReal r1, const dReal *R1, const dReal *R2,
    dReal delta1, dReal delta2, dReal t2, dReal h1Square, dReal ch1, dReal h1Delta1, dReal epsilon, dReal &distCN, dReal &tLine);
  void computeContactPointAndNormalCAG(const dVector3 c1, const dVector3 c2, const dVector3 h1n1,
    const dReal *R1, const dReal *R2, dReal r1, dReal distCN, dReal dmin, dReal tmin, dReal epsilon1, dReal sign);
  bool capAgainstGeneratorOneSidedTest(const dVector3 h1n1, const dVector3 h2n2, const dVector3 c1, const dVector3 c2,
    dReal r1, dReal r2, dReal h1, dReal h2, const dReal *R1, const dReal *R2, dReal delta1, dReal delta2,
    dReal t1, dReal t2, dReal ch1, dReal sign, int dcac[2][2]);

  void capsCollisionFallBack(const dVector3 c1, const dVector3 c2, const dVector3 h1n1,
    const dReal *R1, const dReal *R2,
    dReal r1, dReal r2, dReal h1, dReal delta1, dReal distCN, dReal tmin, dReal hmin,
    dReal epsilon1, dReal sign);

  // ODE structs
  dGeomID m_gCylinder1;
  dGeomID m_gRevolutionGeom2;
  int m_iFlags;
  dContactGeom *m_gContact;
  int m_iSkip;
  int m_nNumberOfContacts;

  // cylinder parameters
  dReal m_fRadius1, m_fRadius2, m_fRadiusSum;
  dReal m_fHeight1, m_fHeight2;
  dReal m_fH1, m_fH2; // half heights
  const dReal *m_vCenter1, *m_vCenter2;
  dVector3 m_vDelta;
  const dReal *m_mRotation1, *m_mRotation2;
  dReal m_fCosinus; // cosinus of the angle between the cylinders axes
  dReal m_fSinus, m_fInvSinus;
  dReal m_fDet, m_fDelta1, m_fDelta2, m_fD, m_fDPlus;
  dVector3 m_vW; // cross-product of the cylinders' z-unit vectors

  dReal m_fRadius1Square, m_fRadius2Square;
  dReal m_fRadiusSumSquare;
  dReal m_fDeltaSquare;
  dReal m_fH1Square, m_fH2Square;
  dReal m_fSigma;

  static const dReal thresholdSquaredSinus;
  static const dReal thresholdRadius;
  static const dReal thresholdSquaredRadius;
  static const dReal thresholdCosinus1;
};

void sCylinderRevolutionData::init() {
  m_fRadiusSum = m_fRadius1 + m_fRadius2;
  m_fH1 = REAL(0.5) * m_fHeight1;
  m_fH2 = REAL(0.5) * m_fHeight2;
  m_fDelta1 = dCalcVectorDot3_14(m_vDelta, m_mRotation1 + 2);
  m_fDelta2 = dCalcVectorDot3_14(m_vDelta, m_mRotation2 + 2);
  m_fCosinus = dCalcVectorDot3_44(m_mRotation1 + 2, m_mRotation2 + 2);
  // m_fCosinus = cosinus(n1,n2); det = 1 - c^2 = ||n1 cross n2||^2 = || w ||^2 = sinus(n1,n2)^2 = s^2;
  const dReal cSquare = m_fCosinus * m_fCosinus;
  m_fDet = REAL(1.0) - cSquare;
  m_fSinus = m_fDet > 0.0 ? dSqrt(m_fDet) : 0.0; // norm of w = n1 cross n2; it is also equal to |sin(n1,n2)|
}

void sCylinderRevolutionData::initWData(dReal &radiusMinusDistance, dReal &t1, dReal &t2) {
  const dReal invDet = REAL(1.0) / m_fDet;
  t1 = (m_fDelta1 - m_fCosinus * m_fDelta2) * invDet;
  t2 = m_fCosinus * t1 - m_fDelta2;
  dCalcVectorCross3_144(m_vW, m_mRotation1 + 2, m_mRotation2 + 2);
  m_fInvSinus = REAL(1.0) / m_fSinus;
  m_fD = dCalcVectorDot3(m_vDelta, m_vW) * m_fInvSinus;// signed distance between axes //TODO : choose a better notation
  m_fDPlus = dFabs(m_fD);
  radiusMinusDistance = m_fRadiusSum - m_fDPlus;
}

void sCylinderRevolutionData::initProjectedLengths(dReal &ch1, dReal &ch2, dReal &sr1, dReal &sr2) {
  ch1 = m_fCosinus * m_fH1;
  ch2 = m_fCosinus * m_fH2;
  sr1 = m_fSinus * m_fRadius1;
  sr2 = m_fSinus * m_fRadius2;
}

void sCylinderRevolutionData::generatorAgainstGeneratorIntersectionTest(dReal radiusMinusDistance, dReal t1, dReal t2) {
  if (dFabs(t1) <= m_fH1 && dFabs(t2) <= m_fH2) {
    // Origin on Cylinder 1 axis
    const dVector3 A1 = { m_vCenter1[0] + t1 * m_mRotation1[2], m_vCenter1[1] + t1 * m_mRotation1[6], m_vCenter1[2] + t1 * m_mRotation1[10] };
    const dVector3 u = {  m_fInvSinus * m_vW[0] , m_fInvSinus * m_vW[1] , m_fInvSinus * m_vW[2]  };
    dReal dPlus = REAL(0.5) * (m_fDPlus + m_fRadius1 - m_fRadius2);
    radiusMinusDistance *= REAL(0.5);
    dReal sign = m_fD > 0.0 ? REAL(1.0) : REAL(-1.0);
    dPlus *= sign;
    dMessage(1,"Deep point of type GAC detected : depth = radiusMinusDistance = %f", radiusMinusDistance);
    dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
    dAddScaledVectors3(contact->pos, A1, u, 1.0, dPlus);
    dCopyScaledVector3(contact->normal, u, -sign);
    contact->depth = radiusMinusDistance;
    ++m_nNumberOfContacts;
  }
}

void sCylinderRevolutionData::initOrthogonalCase(dReal &d12, dReal &d21, dReal &rho1, dReal &rho2) {
  rho2 = dFabs(m_fDelta2) - m_fH2;
  d12 = m_fRadius1 - rho2;
  rho1 = dFabs(m_fDelta1) - m_fH1;
  d21 = m_fRadius2 - rho1;

  if (m_fD < 0.0)
    dNegateVector3(m_vW);
}

void sCylinderRevolutionData::initSquares() {
  m_fRadiusSumSquare = m_fRadiusSum * m_fRadiusSum;
  m_fH1Square = m_fH2 * m_fH2;
  m_fH2Square = m_fH2 * m_fH2;
  m_fRadius1Square = m_fRadius1 * m_fRadius1;
  m_fRadius2Square = m_fRadius2 * m_fRadius2;
  m_fDeltaSquare = dCalcVectorLengthSquare3(m_vDelta);
  m_fSigma = m_fDeltaSquare + m_fH1Square + m_fH2Square;
}

// Data that passed through the collider's functions
struct sCylinderCylinderData: public sCylinderRevolutionData
{
  sCylinderCylinderData(dxCylinder *cylinder1, dxCylinder *cylinder2, int flags, dContactGeom *contact, int skip):
    sCylinderRevolutionData(cylinder1, cylinder2, flags, contact, skip),
    m_gCylinder2(cylinder2)
  {
    dGeomCylinderGetParams(m_gCylinder1, &m_fRadius1, &m_fHeight1);
    dGeomCylinderGetParams(m_gCylinder2, &m_fRadius2, &m_fHeight2);
    init();
  }
  virtual int performCollisionChecking();
  virtual void parallelAxesIntersectionTest(dReal ch1, dReal ch2, dReal hSum1);
  void computeContactPointAndNormalForSAD(dReal r, dReal dh, const dVector3 u);
  void computeContactPointsAndNormalsParallelAxes(dReal dr, dVector3 u);
  void computeContactPointsParallelAxesGAG(const dVector3 c2, const dReal *n,
    dReal r2, dReal h2, dReal dr, dVector3 u, dReal revertU);
  void computeContactPointsParallelAxesCAC(const dReal *R1, const dReal *R2, const dVector3 c2, dReal r2,
    dReal h1, dReal h2, dReal ch2, dReal delta1, dReal epsilon1, dReal epsilon2);
  void computeContactPointsParallelAxesCornersH(dReal epsilon, dReal dh, dReal dr, dVector3 u);
  void computeContactPointsParallelAxesCornersV(dReal r, dReal ch1, dReal epsilon, const dVector3 u, const dVector3 v);
  void orthogonalAxesIntersectionTest(const dVector3 A1, const dReal *R1, const dReal *R2,  dReal r1, dReal r2,
    dReal h1, dReal h2, dReal delta1, dReal delta2,
    dReal t1, dReal t2, dReal rho1, dReal rho2,
    dReal d12, dReal d21, dReal epsilon1, dReal epsilon2,
    dReal epsilon, bool &exit);

  void pickPointOnIntersectionLine(const dVector3 h1n1, dReal epsilon1, dReal p2, dReal &q, dVector3 O1, dVector3 I);
  void computeContactPointIC(dReal m, const dVector3 u, const dVector3 I);
  void computeRadialNormal(dReal depth , const dVector3 normal, dReal normalLength);
  void computeVerticalNormal(dReal depth, const dReal *R, dReal sign);
  void computeNormalIC(dReal epsilon1, dReal epsilon2, const dVector3 normal, dReal normalLength,
  dReal normalDepth, dReal depth, dReal depth1, dReal depth2);

  void slightlyInclinedCapsTest(dReal sr1, dReal sr2, dReal ch1, dReal ch2, dReal ch1h2,
    const dVector3 h1n1, const dVector3 h2n2, dReal t1, dReal t2, int dcac[2][2]);
  bool slightlyInclinedCaps(dReal sr1, dReal sr2, dReal ch1, dReal ch2, const dVector3 h1n1, const dVector3 h2n2, dReal t1, dReal t2,
    dReal p1, dReal p2, dReal epsilon1, dReal epsilon2, dReal O1O2square,  bool &needMoreContacts);
  void computeContactPointAndNormalIC(dReal sr1, dReal sr2, dReal ch1, dReal ch2, const dVector3 h1n1,
    dReal p1, dReal p2, dReal d1, dReal d2, dReal diff12, dReal epsilon1, dReal epsilon2);
  void computeContactPointAndNormalDCACV1(const dVector3 h1n1, dReal epsilon1, dReal epsilon2, dReal depth, dReal revertRadius1);
  void computeContactPointAndNormalDCACW1(const dVector3 h1n1, dReal epsilon1, dReal epsilon2, dReal depth, dReal revertRadius1);
  void computeContactPointAndNormalDCACV2(const dVector3 h2n2, dReal epsilon1, dReal epsilon2, dReal depth, dReal revertRadius2);
  void computeContactPointAndNormalDCACW2(const dReal *R1, const dVector3 h2n2,
    dReal epsilon1, dReal epsilon2, dReal depth, dReal revertRadius2);

  void stronglyInclinedCapsTest(dReal sr1, dReal sr2, dReal ch1, dReal ch2, dReal ch1h2,
    const dVector3 h1n1, const dVector3 h2n2, dReal t1, dReal t2);
  bool stronglyInclinedCaps(dReal sr1, dReal sr2, dReal ch1, dReal ch2, const dVector3 h1n1, const dVector3 h2n2,
    dReal p1, dReal p2, dReal epsilon1, dReal epsilon2, dReal O1O2square);
  void computeContactPointAndNormalPDV1(dReal sr1, dReal p2, const dVector3 h1n1, const dVector3 h2n2,
    dReal epsilon1, dReal epsilon2);
  void computeContactPointAndNormalPDV2(dReal sr2, dReal p1, const dVector3 h1n1, const dVector3 h2n2,
    dReal epsilon1, dReal epsilon2);
  void computeContactPointAndNormalCACV1(const dVector3 h1n1,  const dVector3 h2n2,
    dReal epsilon1, dReal epsilon2, dReal depth, dReal revertRadius1);
  void computeContactPointAndNormalCACV2(const dVector3 h1n1, const dVector3 h2n2,
    dReal epsilon1, dReal epsilon2, dReal depth, dReal revertRadius2);

  bool capAgainstGeneratorFullTest(const dVector3 h1n1,const dVector3 h2n2, dReal t1, dReal t2,
    dReal ch1, dReal ch2, int dcac[2][2]);

  // ODE geom
  dGeomID m_gCylinder2;
};

// Data that passed through the collider's functions
struct sCylinderCapsuleData: sCylinderRevolutionData
{
  sCylinderCapsuleData(dxCylinder *cylinder, dxCapsule *capsule, int flags, dContactGeom *contact, int skip):
    sCylinderRevolutionData(cylinder, capsule, flags, contact, skip),
    m_gCapsule(capsule)
  {
    dGeomCylinderGetParams(m_gCylinder1, &m_fRadius1, &m_fHeight1);
    dGeomCapsuleGetParams(m_gCapsule, &m_fRadius2, &m_fHeight2);
    init();
  }
  virtual int performCollisionChecking();
  virtual bool centersPenetration(const dReal *h1n1, const dReal *h2n2);
  virtual void parallelAxesIntersectionTest(dReal ch1, dReal ch2, dReal hSum1);
  void computeContactPointsAndNormalsParallelAxes(dReal dr, dVector3 u);
  bool capSphereAgainstGeneratorOrthogonalAxes(const dVector3 A1, dReal epsilon2);
  bool capSphereAgainstCapDiskOrthogonalAxes(dReal epsilon1, dReal epsilon2);
  bool capDiskAgainstGeneratorOrthogonalAxes(const dVector3 A2, dReal t1, dReal rho1, dReal rho2,
    dReal d12, dReal d21, dReal epsilon1, dReal epsilon2);

  void capSphereAgainstGenerator(dReal epsilon2, const dVector3 h2n2, dReal squaredDistanceToCylinderAxis);
  void capDiskAgainstCapSphere(const dVector3 h1n1, const dVector3 h2n2, dReal epsilon1, dReal epsilon2);
  void computeContactPointAndNormalForSAD(dReal r, dReal dh, const dVector3 u);
  // ODE geom
  dGeomID m_gCapsule;
};

#endif
