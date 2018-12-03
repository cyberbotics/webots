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

#include <iostream>
#include <ode/common.h>
#include "config.h"
#include <ode/collision.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/odemath.h>
#include "cylinder_revolution_data.h"
#include "collision_kernel.h"
#include "collision_std.h"
#include "collision_util.h"

using namespace std;

const dReal sCylinderRevolutionData::thresholdSquaredSinus = REAL(1e-4);
const dReal sCylinderRevolutionData::thresholdRadius = REAL(1e-1);
const dReal sCylinderRevolutionData::thresholdSquaredRadius = REAL(0.9);
const dReal sCylinderRevolutionData::thresholdCosinus1 = REAL(1e-4);

////////////////////////////////////
//---------------------------------
// Preparing Cap Against Generator
//---------------------------------
////////////////////////////////////

// Computes the coefficients of the distance squared function and its first two derivatives
//-----------------------------------------------------------------------------------------
void sCylinderRevolutionData::computeCoefficents(sZBrentData &data, dReal r1, dReal delta1, dReal delta2, dReal t2,
    dReal h1Square, dReal h1Delta1, dReal epsilon, dReal start) {

  // Coefficients of f(t) = MP^2, f'(t) and f''(t)
  //---------------------------------------------
  data.coeffA[0] = r1 * r1 +  h1Square + m_fDeltaSquare - epsilon * h1Delta1;
  data.coeffA[1] = - 2.0 * start;
  data.coeffA[2] = - 2.0 * r1;
  data.coeffA[3] = m_fDeltaSquare - delta1 * delta1;
  data.coeffA[4] = - 2.0 * m_fDet * t2;
  data.coeffA[5] = m_fDet;

  data.coeffB[0] = 0.5 * data.coeffA[2] * data.coeffA[4];
  data.coeffB[1] = data.coeffA[2] * data.coeffA[5];

  data.coeffC[0] =  - 0.5 * data.coeffB[0] * data.coeffA[4];
  data.coeffC[1] =  - data.coeffB[0] * data.coeffA[5] - 0.5 * data.coeffB[1] * data.coeffA[4];
  data.coeffC[2] =  - data.coeffB[1] * data.coeffA[5];
}

// Computes the squared distance function f(t) = PM^2 and its first two derivatives
//------------------------------------------------------------------------------------------------------------
dReal sZBrentData::objectiveFunction(dReal t, dReal &distCN) const
{
  distCN = dSqrt(coeffA[3] + t * (coeffA[4] + t * coeffA[5]));
  return coeffA[0] + t * (coeffA[1] + t) + coeffA[2] * distCN;
}

//--------------------------------------------------------------------------------------------------------------
dReal sZBrentData::differential(dReal t) const
{
  const dReal invSinusSqrtDenom = REAL(1.0) / sqrt(coeffA[3] + t * (coeffA[4] + t * coeffA[5]));
  return coeffA[1] + 2.0 * t + (coeffB[0] + t * coeffB[1]) * invSinusSqrtDenom;
}

//---------------------------------------------------------------------------------------------------------------
dReal sZBrentData::differential2(dReal t) const
{
  const dReal denom = coeffA[3] + t * (coeffA[4] + t * coeffA[5]);
  const dReal invSinusSqrtDenom = REAL(1.0) / sqrt(denom);
  return REAL(2.0) + invSinusSqrtDenom * (coeffB[1] + (coeffC[0] + t * (coeffC[1] + t * coeffC[2])) / denom);
}
//-----------------------------------------------------------------------------------------------------------------

bool extractQuadraticRoots(const dReal coeff[2], dReal x[2])
{
  const dReal p = - 0.5 * coeff[0] / coeff[1];
  dReal det = p * p - REAL(1.0) / coeff[1];
  if (det < 0.0)
    return false;

  det = dSqrt(det);
  x[0] = p - det;
  x[1] = p + det;

  return true;
}

// Root finding Brent's method (from 'Numerical Recipes')
//-------------------------------------------------------

static inline void zbrent(const sZBrentData &data, dReal x1, dReal x2, dReal tol, dReal &root)
{
  static const int maxIter = 15;

  int iter;
  dReal a = x1, b = x2, d, min1, min2;
  dReal c = a;
  dReal e = d = b - a;
  dReal fa = data.differential(a);
  dReal fb = data.differential(b);
  root = b;
  if (fb * fa > 0.0) return;

  dReal p, q, r, s, tol1, xm;
  dReal fc = fb;
  for (iter = 1; iter <= maxIter; ++iter) {
    if (fb * fc > 0.0) {
      c = a;
      fc = fa;
      e = d = b - a;
    }

    if (dFabs(fc) < dFabs(fb)) {
      a = b;
      b = c;
      c = a;
      fa = fb;
      fb = fc;
      fc = fa;
    }

    tol1 = REAL(2.0) * dEpsilon * dFabs(b) + REAL(0.5) * tol;
    xm = REAL(0.5) * (c - b);
    if (dFabs(xm) <= tol1 || fb == 0.0) {
      root = b;
      return;
    }

    if (dFabs(e) >= tol1 && dFabs(fa) > dFabs(fb)) {
      s = fb / fa;
      if (a == c) {
        p = 2.0 * xm * s;
        q = 1.0 - s;
      }
      else {
        q = fa / fc;
        r = fb / fc;
        p = s * (2.0 * xm * q * (q - r) - (b - a) * (r - 1.0));
        q = (q - 1.0) * (r - 1.0) * (s - 1.0);
      }

      if (p > 0.0)
        q = -q;

      p = dFabs(p);
      min1 = 3.0 * xm * q - dFabs(tol1 * q);
      min2 = dFabs(e * q);
      if (2.0 * p < (min1 < min2 ? min1 : min2)) {
        e = d;
        d = p / q;
      }
      else {
        d = xm;
        e = d;
      }
    }
    else {
      d = xm;
      e = d;
    }

    a = b;
    fa = fb;
    if (dFabs(d) > tol1)
      b += d;
    else
      b += (xm > 0.0 ? dFabs(tol1) : -dFabs(tol1));
    fb = data.differential(b);
  }
  return;
}

// Compute the minimal distance between the boundary circle of a cylinder's cap and the axis of another cylinder (from David Vranek, 2002)
//----------------------------------------------------------------------------------------------------------------------------------------

dReal sCylinderRevolutionData::squaredDistanceFromLinetoCircle(dReal r1, const dReal *R1, const dReal *R2,
  dReal delta1, dReal delta2, dReal t2, dReal h1Square, dReal ch1, dReal h1Delta1, dReal epsilon, dReal &distCN, dReal &tLine)
{
  static const dReal numericalPrecision = REAL(1e-4);
  dReal start = - delta2 + epsilon * ch1;
  sZBrentData data;
  computeCoefficents(data, r1, delta1, delta2, t2, h1Square, h1Delta1, epsilon, start);

  // Bracketing interval for the search of critical points of MP^2 = f(t) via Brent's method
  //---------------------------------------------------------------------------------------
  const dReal startMin = start - r1;// TODO : improve the pinching
  const dReal startMax = start + r1;

  // Find the first root
  zbrent(data, startMin, startMax, numericalPrecision, tLine);

  // Polish up the root
  tLine -= data.differential(tLine) / data.differential2(tLine);
  dReal minDist = data.objectiveFunction(tLine, distCN);

  // if tLine is the local maximum, find the remaining minima
  if (data.differential2(tLine) < 0.0) {
    dReal t1, t2, fMin1, fMin2;
    zbrent(data, startMin, tLine - numericalPrecision, numericalPrecision, t1);
    fMin1 = data.objectiveFunction(t1, distCN);

    zbrent(data, tLine + numericalPrecision, startMax, numericalPrecision, t2);
    fMin2 = data.objectiveFunction(t2, distCN);

    if (fMin1 < fMin2) {
      tLine = t1;
      return fMin1;
    } else {
      tLine = t2;
      return fMin2;
    }
  } else {
    // Compute coefficients of a polynomial p4
    dReal fPoly4Term3 = 2.0 * data.coeffA[1];
    dReal fPoly4Term2 = - data.coeffA[2] * data.coeffA[2] * data.coeffA[5] + 2.0 * (data.coeffA[0] - minDist) +
      data.coeffA[1] * data.coeffA[1];

    // Deflate polynomial p4->p2
    dReal afPoly2[2], afRoot[2];
    afPoly2[1] = fPoly4Term2 + (3.0 * tLine + 2.0 * fPoly4Term3) * tLine;
    afPoly2[0] = fPoly4Term3 + 2.0 * tLine;

    // Bracket the second mininmum
    if (extractQuadraticRoots(afPoly2, afRoot)) {
      // Sort roots
      if (afRoot[0] > afRoot[1])
        swap(afRoot[0], afRoot[1]);

      // Find the global minimum
      zbrent(data, afRoot[0], afRoot[1], numericalPrecision, tLine);
      minDist = data.objectiveFunction(tLine, distCN);
    }
  }

  return minDist;
}

// Computes the contact point and its normal in the case 'Cap Against Generator' (CAG)
//------------------------------------------------------------------------------------

//   C = C_1^{+ or -} is the center of a cap of Cylinder 1;
//   C2 is the center of Cylinder 2;
//   P (on Cylinder 1's boundary circle) and M (on Cylinder 2's axis) are the points separated by the minimal distance;
//   N is the orthogonal projection of M on the plane Pi_1 containing the cap of Cylinder 1;

void sCylinderRevolutionData::computeContactPointAndNormalCAG(
  const dVector3 c1, const dVector3 c2, const dVector3 h1n1,
  const dReal *R1, const dReal *R2, dReal r1,
  dReal distCN, dReal dmin, dReal tmin, dReal epsilon1, dReal sign)
{
  const dVector3 C = { c1[0] + epsilon1 * h1n1[0] , c1[1] + epsilon1 * h1n1[1] , c1[2] + epsilon1 * h1n1[2] };
  const dVector3 C2C = { C[0] - c2[0], C[1] - c2[1], C[2] - c2[2] };
  const dVector3 CM = { tmin * R2[2] - C2C[0], tmin * R2[6] - C2C[1], tmin * R2[10] - C2C[2] };
  const dReal x = dCalcVectorDot3_14(CM, R1 + 2);
  const dVector3 CN = { CM[0] - x * R1[2], CM[1] - x * R1[6], CM[2] - x * R1[10] };
  const dReal y = r1 / distCN;
  const dVector3 CP = { y * CN[0], y * CN[1], y * CN[2] };
  dVector3 MP = { CP[0] - CM[0], CP[1] - CM[1], CP[2] - CM[2] };

  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  dAddVectors3(contact->pos, CP, C);
  contact->depth = dmin;
  dMessage(1,"Cap against Generator Case : depth=%f", dmin);
  dNormalize3(MP);
  dCopyScaledVector3(contact->normal, MP, sign);
}

// Computes the contact point and its normal when the CAG test returns a point too close to a cap
//--------------------------------------------------------------------------------------------

// the normal is orthogonal to a cylinder's cap

void sCylinderRevolutionData::capsCollisionFallBack(
  const dVector3 c1, const dVector3 c2, const dVector3 h1n1, const dReal *R1, const dReal *R2,
  dReal r1, dReal r2, dReal h1, dReal delta1, dReal distCN, dReal tmin, dReal hmin,
  dReal epsilon1, dReal sign)
{
  const dVector3 C = { c1[0] + epsilon1 * h1n1[0] , c1[1] + epsilon1 * h1n1[1] , c1[2] + epsilon1 * h1n1[2] };
  const dVector3 C2C = { C[0] - c2[0], C[1] - c2[1], C[2] - c2[2] };
  const dVector3 CM = { tmin * R2[2] - C2C[0], tmin * R2[6] - C2C[1], tmin * R2[10] - C2C[2] };
  //dReal x = dDOT14(CM,R1 + 2);
  const dReal x = - epsilon1 * h1 + delta1 + m_fCosinus * tmin;
  const dVector3 CN = { CM[0] - x * R1[2], CM[1] - x * R1[6], CM[2] - x * R1[10] };
  const dReal y = r1 / distCN;
  const dVector3 CP = { y * CN[0], y * CN[1], y * CN[2] };

  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  dAddVectors3(contact->pos, CP, C);
  contact->depth = hmin;
  dMessage(1,"Cap against Generator Case (fallback): depth = %f", hmin);
  dReal epsilon2 = tmin > 0.0 ? REAL(1.0) : REAL(-1.0);
  epsilon2 *= sign;
  contact->normal[0] = epsilon2 * R2[2];
  contact->normal[1] = epsilon2 * R2[6];
  contact->normal[2] = epsilon2 * R2[10];
}

//----------------------------------------------------------------------------------------------------------------
//                         Cap Against Generator Main Program
//----------------------------------------------------------------------------------------------------------------

//---------------------------- This is the slowest test ----------------------------------------------------------

//  the iterative Brent's method is used to compute
//  the squared distance from a boundary circle to a cylinder's axis

//----------------------------------------------------------------------------------------------------------------
// It returns a single contact point; the normal has the direction of the line joining the pair of closest points,
// one on the corresponding boundary circle, the other on the corresponding axis
//----------------------------------------------------------------------------------------------------------------

// One-sided test : Cylinder 1 Against Cylinder / Capsule 2

bool sCylinderRevolutionData::capAgainstGeneratorOneSidedTest(
    const dVector3 h1n1, const dVector3 h2n2, const dVector3 c1, const dVector3 c2,
    dReal r1, dReal r2, dReal h1, dReal h2,
    const dReal *R1, const dReal *R2, dReal delta1, dReal delta2,
    dReal t1, dReal t2, dReal ch1, dReal sign, int dcac[2][2])
{

  // Which cap of Cylinder 1 is the closest to the axis of Cylinder 2 / Capsule ?
  //-----------------------------------------------------------------------------

  const dReal h1Square = h1 * h1;
  const dReal r2Square = r2 * r2;
  const dReal h1Delta1 = 2.0 * h1 * delta1;

  const dReal sigma = m_fDet * h1Square - delta2 * delta2 + m_fDeltaSquare;
  const dReal sigma1 = 2.0 * m_fDet * m_fH1 * t1;
  const dReal dPlusSquare  = sigma - sigma1; // squared distance from upper disk center to the capsule / cylinder 2 axis
  const dReal dMinusSquare = sigma + sigma1; // squared distance from lower disk center to the capsule / cylinder 2 axis
  const int cylinderIndex = sign > 0.0 ? 0 : 1;

  if (dPlusSquare <= dMinusSquare) {//the upper cap is the closest one
    dMessage(1, "upper cap is closest\n");
    if (dPlusSquare <= m_fRadiusSumSquare && (dcac == NULL || dPlusSquare > thresholdSquaredRadius * r2Square)) {
      const dReal depth = r1 + h2 - dFabs(ch1 - delta2);
      dMessage(1, "depth %g\n", depth);
      if (depth > 0.0) {
        dReal tmin, distCN;
        // Heavy computations are here
        const dReal squaredMinDist = squaredDistanceFromLinetoCircle(r1, R1, R2, delta1, delta2, t2, h1Square, ch1, h1Delta1, REAL(1.0), distCN, tmin);
        const dReal hmin = h2 - dFabs(tmin);
        const dReal minDist = dSqrt(squaredMinDist);
        const dReal dmin = r2 - minDist;
        if (dmin >= 0.0 && hmin >= 0.0) {
          if (dcac == NULL || dmin <= hmin || (dcac[cylinderIndex][1] == 0 && tmin * m_fCosinus > 0.0)) {
            computeContactPointAndNormalCAG(c1, c2, h1n1, R1, R2, r1, distCN, dmin, tmin, REAL(1.0), sign);
            ++m_nNumberOfContacts;
            return true;
          } else if (dcac) {
            dMessage(1, "Cap against Generator: fallback to CAC");
            capsCollisionFallBack(c1, c2, h1n1, R1, R2, r1, r2, h1, delta1, distCN, tmin, hmin, REAL(1.0), sign);
            ++m_nNumberOfContacts;
            return true;
          }
        }
      }
    }
  } else {//the lower cap is the closest one
    if (dMinusSquare <= m_fRadiusSumSquare && (dcac == NULL || dMinusSquare > thresholdSquaredRadius * r2Square)) {
      dMessage(1, "lower cap is closest\n");
      const dReal depth = r1 + h2 - dFabs(ch1 + delta2);
      dMessage(1, "depth %g\n", depth);
      if (depth > 0.0) {
        dReal tmin, distCN;
        const dReal squaredMinDist = squaredDistanceFromLinetoCircle(r1, R1, R2, delta1, delta2, t2, h1Square, ch1, h1Delta1, REAL(-1.0), distCN, tmin);
        const dReal hmin = h2 - dFabs(tmin);
        const dReal minDist = dSqrt(squaredMinDist);
        const dReal dmin = r2 - minDist;
        dMessage(1, "hmin %g dmin %g\n", hmin, dmin);
        if ((dmin >= 0.0 && hmin >= 0.0) || (dcac && dcac[cylinderIndex][0] == 0 && tmin * m_fCosinus < 0.0)) {
          if (dcac == NULL || dmin <= hmin || dcac[cylinderIndex][0] == 0) {
            computeContactPointAndNormalCAG(c1, c2, h1n1, R1, R2, r1, distCN, dmin, tmin , REAL(-1.0), sign);
            ++m_nNumberOfContacts;
            return true;
          } else if (dcac){
            dMessage(1, "Cap against Generator: fallback to CAC");
            capsCollisionFallBack(c1, c2, h1n1, R1, R2, r1, r2, h1, delta1, distCN, tmin, hmin, REAL(-1.0), sign);
            ++m_nNumberOfContacts;
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool sCylinderRevolutionData::centersPenetration(const dReal *h1n1, const dReal *h2n2) {
  dReal radialDepth = m_fRadius2Square - m_fDeltaSquare + m_fDelta2 * m_fDelta2;
  radialDepth = radialDepth > 0.0 ? dSqrt(radialDepth) : 0.0;
  dReal verticalDepth = m_fH2 - dFabs(m_fDelta2);
  bool radialNormal = verticalDepth > radialDepth;
  dReal depth = radialNormal ? radialDepth : verticalDepth;

  // Cylinder 1's center lies inside cylinder 2
  if (depth > 0.0) {
    dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
    dCopyVector3(m_gContact->pos, m_vCenter1);
    dReal *const n = contact->normal;
    const dVector3 n2 = { m_mRotation2[2], m_mRotation2[6], m_mRotation2[10] };
    if (radialNormal) {
      dAddScaledVectors3(n, m_vDelta, n2, 1.0, -m_fDelta2);
      dNormalize3(n);
    } else
      dCopyScaledVector3(n, n2, m_fDelta2 > 0.0 ? -1.0 : 1.0);

     contact->depth = depth;
     ++m_nNumberOfContacts;
    return true;
  }

  radialDepth = m_fRadius1Square - m_fDeltaSquare + m_fDelta1 * m_fDelta1;
  radialDepth = radialDepth > 0.0 ? dSqrt(radialDepth) : 0.0;
  verticalDepth = m_fH1 - dFabs(m_fDelta1);
  radialNormal = verticalDepth > radialDepth;
  depth = radialNormal ? radialDepth : verticalDepth;

  // Cylinder 2's center lies inside cylinder 1
  if (depth > 0.0) {
    dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
    dCopyVector3(m_gContact->pos, m_vCenter2);
    dReal *const n = contact->normal;
    const dVector3 n1 = { m_mRotation1[2], m_mRotation1[6], m_mRotation1[10] };
    if (radialNormal) {
      dAddScaledVectors3(n, m_vDelta, n1, -1.0, m_fDelta1);
      dNormalize3(n);
    } else
      dCopyScaledVector3(n, n1, m_fDelta1 > 0.0 ? -1.0 : 1.0);
    contact->depth = depth;
    ++m_nNumberOfContacts;
    return true;
  }

  return false;
}

void sCylinderRevolutionData::capCentersPenetration(const dReal *h1n1, const dReal *h2n2, dReal ch1, dReal ch2) {
  const dVector3 n1 = { m_mRotation1[2], m_mRotation1[6], m_mRotation1[10] };
  const dVector3 n2 = { m_mRotation2[2], m_mRotation2[6], m_mRotation2[10] };
  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  dReal *const n = contact->normal;

  dReal temp1 = m_fDeltaSquare + m_fH1Square;
  dReal temp2 = ch1 - m_fDelta2;
  dReal radialDepth = m_fRadius2Square - temp1 + 2.0 * m_fH1 * m_fDelta1 + temp2 * temp2;
  radialDepth = radialDepth > 0.0 ? dSqrt(radialDepth) : 0.0;
  dReal verticalDepth = m_fH2 - dFabs(temp2);
  bool radialNormal = verticalDepth > radialDepth;
  dReal depth = radialNormal ? radialDepth : verticalDepth;

  // One of the cap centers of cylinder 1 lies inside cylinder 2
  if (depth > 0.0) {
  dAddVectors3(m_gContact->pos, m_vCenter1, h1n1);
  if (radialNormal) {
    dSubtractVectors3(n, h1n1, m_vDelta);
    dAddScaledVectors3(n, n, n2, 1.0, - temp2);
    dNormalize3(n);
  } else
    dCopyScaledVector3(n, n2, temp2 > 0.0 ? 1.0 : -1.0);

   contact->depth = depth;
   ++m_nNumberOfContacts;
   return;
  }

  temp2 = - ch1 - m_fDelta2;
  radialDepth = m_fRadius2Square - temp1 - 2.0 * m_fH1 * m_fDelta1 + temp2 * temp2;
  radialDepth = radialDepth > 0.0 ? dSqrt(radialDepth) : 0.0;
  verticalDepth = m_fH2 - dFabs(temp2);
  radialNormal = verticalDepth > radialDepth;
  depth = radialNormal ? radialDepth : verticalDepth;

  if (depth > 0.0) {
    dSubtractVectors3(m_gContact->pos, m_vCenter1, h1n1);
    if (radialNormal) {
      dAddVectors3(n, m_vDelta, h1n1);
      dNegateVector3(n);
      dAddScaledVectors3(n, n, n2, -1.0, temp2);
      dNormalize3(n);
    } else
      dCopyScaledVector3(n, n2, temp2 > 0.0 ? 1.0 : -1.0);

    contact->depth = depth;
    ++m_nNumberOfContacts;
    return;
  }

  temp1 = m_fDeltaSquare + m_fH2Square;
  temp2 = ch2 + m_fDelta1;
  radialDepth = m_fRadius1Square - temp1 - 2.0 * m_fH2 * m_fDelta2 + temp2 * temp2;
  radialDepth = radialDepth > 0.0 ? dSqrt(radialDepth) : 0.0;
  verticalDepth = m_fH1 - dFabs(temp2);
  radialNormal = verticalDepth > radialDepth ? true : false;
  depth = radialNormal ? radialDepth : verticalDepth;

  // One of the cap centers of cylinder 2 lies inside cylinder 1
  if (depth > 0.0) {
    dAddVectors3(m_gContact->pos, m_vCenter2, h2n2);
    if (radialNormal) {
      dAddVectors3(n, m_vDelta, h2n2);
      dAddScaledVectors3(n, n, n1, -1.0, temp2);
      dNormalize3(n);
    } else
      dCopyScaledVector3(n, n1, temp2 > 0.0 ? -1.0 : 1.0);

    contact->depth = depth;
    ++m_nNumberOfContacts;
    return;
  }

  temp2 = - ch2 + m_fDelta1;
  radialDepth = m_fRadius2Square - temp1 + 2.0 * m_fH2 * m_fDelta2 + temp2 * temp2;
  radialDepth = radialDepth > 0.0 ? dSqrt(radialDepth) : 0.0;
  verticalDepth = m_fH2 - dFabs(temp2);
  radialNormal = verticalDepth > radialDepth;
  depth = radialNormal ? radialDepth : verticalDepth;

  if (depth > 0.0) {
    dSubtractVectors3(m_gContact->pos, m_vCenter2, h2n2);
    if (radialNormal) {
      dSubtractVectors3(n, m_vDelta, h2n2);
      dNegateVector3(n);
      dAddScaledVectors3(n, n, n1, -1.0, temp2);
      dNormalize3(n);
    } else
      dCopyScaledVector3(n, n1, temp2 > 0.0 ? -1.0 : 1.0);

    ++m_nNumberOfContacts;
    contact->depth = depth;
  }
}
