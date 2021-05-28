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

standard ODE geometry primitives: public API and pairwise collision functions.

the rule is that only the low level primitive collision functions should set
dContactGeom::g1 and dContactGeom::g2.

*/

// Cylinder-Cylinder Collision
// Jan 2014

#include <iostream>
#include <ode/common.h>
#include <ode/collision.h>
#include <ode/rotation.h>
#include "config.h"
#include "matrix.h"
#include "odemath.h"
#include "cylinder_revolution_data.h"
#include "collision_kernel.h"
#include "collision_std.h"
#include "collision_util.h"
#include "ode_MT/ode_MT.h"

#include <cassert>

using namespace std;

#define dMAX(A,B)  ((A)>(B) ? (A) : (B))

// flat cylinder public API

dxCylinder::dxCylinder (dSpaceID space, dReal _radius, dReal _length) :
dxGeom (space,1)
{
    dAASSERT (_radius >= 0 && _length >= 0);
    type = dCylinderClass;
    radius = _radius;
    lz = _length;
    updateZeroSizedFlag(!_radius || !_length);
}

void dxCylinder::computeAABB()
{
    const dMatrix3& R = final_posr->R;
    const dVector3& pos = final_posr->pos;

    dReal dOneMinusR2Square = (dReal)(REAL(1.0) - R[2]*R[2]);
    dReal xrange = dFabs(R[2]*lz*REAL(0.5)) + radius * dSqrt(dMAX(REAL(0.0), dOneMinusR2Square));
    dReal dOneMinusR6Square = (dReal)(REAL(1.0) - R[6]*R[6]);
    dReal yrange = dFabs(R[6]*lz*REAL(0.5)) + radius * dSqrt(dMAX(REAL(0.0), dOneMinusR6Square));
    dReal dOneMinusR10Square = (dReal)(REAL(1.0) - R[10]*R[10]);
    dReal zrange = dFabs(R[10]*lz*REAL(0.5)) + radius * dSqrt(dMAX(REAL(0.0), dOneMinusR10Square));

    aabb[0] = pos[0] - xrange;
    aabb[1] = pos[0] + xrange;
    aabb[2] = pos[1] - yrange;
    aabb[3] = pos[1] + yrange;
    aabb[4] = pos[2] - zrange;
    aabb[5] = pos[2] + zrange;
}

dGeomID dCreateCylinder (dSpaceID space, dReal radius, dReal length)
{
    return new dxCylinder (space,radius,length);
}

void dGeomCylinderSetParams_ST (dGeomID cylinder, dReal radius, dReal length)
{
    dUASSERT (cylinder && cylinder->type == dCylinderClass,"argument not a ccylinder");
    dAASSERT (radius >= 0 && length >= 0);
    dxCylinder *c = (dxCylinder*) cylinder;
    c->radius = radius;
    c->lz = length;
    c->updateZeroSizedFlag(!radius || !length);
    dGeomMoved (cylinder);
}

void dGeomCylinderSetParams (dGeomID cylinder, dReal radius, dReal length)
{
#ifdef ODE_MT
  dGeomCylinderSetParams_MT(cylinder, radius, length, &dGeomCylinderSetParams_ST);
#else
  dGeomCylinderSetParams_ST(cylinder, radius, length);
#endif
}

void dGeomCylinderGetParams (dGeomID cylinder, dReal *radius, dReal *length)
{
    dUASSERT (cylinder && cylinder->type == dCylinderClass,"argument not a ccylinder");
    dxCylinder *c = (dxCylinder*) cylinder;
    *radius = c->radius;
    *length = c->lz;
}

//**************************** Cyberbotics version of dCylCyl ************************************************

static const int sufficientNumber = 4;

 // Two-sided test

bool sCylinderCylinderData::capAgainstGeneratorFullTest(const dVector3 h1n1,const dVector3 h2n2, dReal t1, dReal t2,
  dReal ch1, dReal ch2, int dcac[2][2])
 {
  const bool test =
    capAgainstGeneratorOneSidedTest(h1n1, h2n2, m_vCenter1, m_vCenter2, m_fRadius1, m_fRadius2,
      m_fH1, m_fH2, m_mRotation1, m_mRotation2, m_fDelta1, m_fDelta2, t1, t2, ch1, REAL(1.0), dcac)
    ||
    capAgainstGeneratorOneSidedTest(h2n2, h1n1, m_vCenter2, m_vCenter1, m_fRadius2, m_fRadius1,
      m_fH2, m_fH1, m_mRotation2, m_mRotation1, -m_fDelta2, -m_fDelta1, t2, t1, ch2, REAL(-1.0), dcac);

  return test;
}

/////////////////////////////
//--------------------------
// Preparing Cap Against Cap
//---------------------------
/////////////////////////////

// Compute a contact point and its normal when two caps collide in a non-degenerate way (Cap Against Cap)
//-------------------------------------------------------------------------------------------------------

void sCylinderCylinderData::computeContactPointAndNormalCACV1(const dVector3 h1n1,  const dVector3 h2n2,
    dReal epsilon1, dReal epsilon2, dReal depth, dReal revertRadius1)
{
  dReal x = revertRadius1 * m_fRadius1 * m_fInvSinus;
  const dReal y = - x;
  x *= m_fCosinus;

  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  contact->pos[0] = m_vCenter1[0] + epsilon1 * h1n1[0] + x * m_mRotation1[2]  + y * m_mRotation2[2];
  contact->pos[1] = m_vCenter1[1] + epsilon1 * h1n1[1] + x * m_mRotation1[6]  + y * m_mRotation2[6];
  contact->pos[2] = m_vCenter1[2] + epsilon1 * h1n1[2] + x * m_mRotation1[10] + y * m_mRotation2[10];
  contact->normal[0] = epsilon2 * m_mRotation2[2];
  contact->normal[1] = epsilon2 * m_mRotation2[6];
  contact->normal[2] = epsilon2 * m_mRotation2[10];
  contact->depth = depth;
}

void sCylinderCylinderData::computeContactPointAndNormalCACV2(const dVector3 h1n1, const dVector3 h2n2,
    dReal epsilon1, dReal epsilon2, dReal depth, dReal revertRadius2)
{
  const dReal x = revertRadius2 * m_fRadius2 * m_fInvSinus;
  const dReal y = - m_fCosinus * x;

  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  contact->pos[0] = m_vCenter2[0] + epsilon2 * h2n2[0] + x * m_mRotation1[2]  + y * m_mRotation2[2];
  contact->pos[1] = m_vCenter2[1] + epsilon2 * h2n2[1] + x * m_mRotation1[6]  + y * m_mRotation2[6];
  contact->pos[2] = m_vCenter2[2] + epsilon2 * h2n2[2] + x * m_mRotation1[10] + y * m_mRotation2[10];
  contact->normal[0] = -epsilon1 * m_mRotation1[2];
  contact->normal[1] = -epsilon1 * m_mRotation1[6];
  contact->normal[2] = -epsilon1 * m_mRotation1[10];
  contact->depth = depth;
}

//-----------------------------------------------------------------------------------------------------------------
//                                     Degenerate Cap Against Cap
//-----------------------------------------------------------------------------------------------------------------

// Compute a contact point and its normal when two caps collide in a degenerate way (Degenerate Cap Against Cap)
//--------------------------------------------------------------------------------------------------------------

void sCylinderCylinderData::computeContactPointAndNormalDCACV1(const dVector3 h1n1, dReal epsilon1, dReal epsilon2, dReal depth, dReal revertRadius1)
{
  dReal x = revertRadius1 * m_fRadius1 * m_fInvSinus;
  dReal y = - x;
  x *= m_fCosinus;

  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  contact->pos[0] = m_vCenter1[0] + epsilon1 * h1n1[0] + x * m_mRotation1[2]  + y * m_mRotation2[2];
  contact->pos[1] = m_vCenter1[1] + epsilon1 * h1n1[1] + x * m_mRotation1[6]  + y * m_mRotation2[6];
  contact->pos[2] = m_vCenter1[2] + epsilon1 * h1n1[2] + x * m_mRotation1[10] + y * m_mRotation2[10];
  dMessage(1, "V1 pos %g %g %g\n", contact->pos[0], contact->pos[1], contact->pos[2]);
  contact->normal[0] = epsilon2 * m_mRotation2[2];
  contact->normal[1] = epsilon2 * m_mRotation2[6];
  contact->normal[2] = epsilon2 * m_mRotation2[10];
  contact->depth = depth;
}

void sCylinderCylinderData::computeContactPointAndNormalDCACW1(const dVector3 h1n1, dReal epsilon1, dReal epsilon2, dReal depth, dReal revertRadius1)
{
  const dReal x = revertRadius1 * m_fRadius1 * m_fInvSinus;

  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  contact->pos[0] = m_vCenter1[0] + epsilon1 * h1n1[0] + x * m_vW[0];
  contact->pos[1] = m_vCenter1[1] + epsilon1 * h1n1[1] + x * m_vW[1];
  contact->pos[2] = m_vCenter1[2] + epsilon1 * h1n1[2] + x * m_vW[2];
  dMessage(1, "W1 pos %g %g %g (w %g, %g, %g)\n", contact->pos[0], contact->pos[1], contact->pos[2], m_vW[0], m_vW[1], m_vW[2]);
  contact->normal[0] = epsilon2 * m_mRotation2[2];
  contact->normal[1] = epsilon2 * m_mRotation2[6];
  contact->normal[2] = epsilon2 * m_mRotation2[10];
  contact->depth = depth;
}

void sCylinderCylinderData::computeContactPointAndNormalDCACV2(const dVector3 h2n2, dReal epsilon1, dReal epsilon2, dReal depth, dReal revertRadius2)
{
  const dReal x = revertRadius2 * m_fRadius2 * m_fInvSinus;
  const dReal y = - m_fCosinus * x;
  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  contact->pos[0] = m_vCenter2[0] + epsilon2 * h2n2[0] + x * m_mRotation1[2]  + y * m_mRotation2[2];
  contact->pos[1] = m_vCenter2[1] + epsilon2 * h2n2[1] + x * m_mRotation1[6]  + y * m_mRotation2[6];
  contact->pos[2] = m_vCenter2[2] + epsilon2 * h2n2[2] + x * m_mRotation1[10] + y * m_mRotation2[10];
  dMessage(1, "V2 pos %g %g %g\n", contact->pos[0], contact->pos[1], contact->pos[2]);
  contact->normal[0] = - epsilon1 * m_mRotation1[2];
  contact->normal[1] = - epsilon1 * m_mRotation1[6];
  contact->normal[2] = - epsilon1 * m_mRotation1[10];
  contact->depth = depth;
}

void sCylinderCylinderData::computeContactPointAndNormalDCACW2(const dReal *R1, const dVector3 h2n2,
    dReal epsilon1, dReal epsilon2, dReal depth, dReal revertRadius2)
{
    dMessage(1, "epsilon2 %g h2n2 %g %g %g\n", epsilon2, h2n2[0], h2n2[1], h2n2[2]);
    const dReal x = revertRadius2 * m_fRadius2 * m_fInvSinus;
    dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
    contact->pos[0] = m_vCenter2[0] + epsilon2 * h2n2[0] + x * m_vW[0];
    contact->pos[1] = m_vCenter2[1] + epsilon2 * h2n2[1] + x * m_vW[1];
    contact->pos[2] = m_vCenter2[2] + epsilon2 * h2n2[2] + x * m_vW[2];
    contact->normal[0] = - epsilon1 * R1[2];
    contact->normal[1] = - epsilon1 * R1[6];
    contact->normal[2] = - epsilon1 * R1[10];
    contact->depth = depth;
}
//---------------------------------------------------------------------------------------------------------------------------
//                                                   Penetrating Disks
//---------------------------------------------------------------------------------------------------------------------------

// Returns a single contact point and its normal, for a collision between caps with penetrating discs (Penetrating Disks Case)
//----------------------------------------------------------------------------------------------------------------------------
//                                                    D1 penetrates D2
//----------------------------------------------------------------------------------------------------------------------------
void sCylinderCylinderData::computeContactPointAndNormalPDV1(dReal sr1, dReal p2, const dVector3 h1n1, const dVector3 h2n2,
    dReal epsilon1, dReal epsilon2)
{
      //-------------------
      // Points V1+ and V1-
      //-------------------

      dReal scalar = - p2 - sr1;
      dReal depth = m_fH2 - dFabs(epsilon2 * m_fH2 + scalar);

      if (depth > 0.0) {
        //V1+
        dMessage(1,"Caps Intersection Test, PD Case V1- : success ");
        computeContactPointAndNormalCACV1(h1n1, h2n2, epsilon1, epsilon2, depth, 1.0);
      } else {
        //V1-
        scalar = - p2 + sr1;
        depth = m_fH2 - dFabs(epsilon2 * m_fH2 + scalar);
        dMessage(1,"Caps Intersection Test, PD Case V1- : success ");
        computeContactPointAndNormalCACV1(h1n1, h2n2, epsilon1, epsilon2, depth, -1.0);
      }
      return;
}

//----------------------------------------------------------------------------------------------------------------------------
//                                                    D2 penetrates D1
//----------------------------------------------------------------------------------------------------------------------------
void sCylinderCylinderData::computeContactPointAndNormalPDV2(dReal sr2, dReal p1, const dVector3 h1n1, const dVector3 h2n2,
  dReal epsilon1, dReal epsilon2) {
      //-------------------
      // Points V2+ and V2-
      //-------------------
      dReal scalar = p1 + sr2;
      dReal depth = m_fH1 - dFabs(epsilon1 * m_fH1 + scalar);

      if (depth > 0.0) {
        //V2+
        dMessage(1,"Caps Intersection Test, PD Case V2+ : success ");
        computeContactPointAndNormalCACV2(h1n1, h2n2, epsilon1, epsilon2, depth, 1.0);
      } else {
        //V2-
        scalar = p1 - sr2;
        depth = m_fH1 - dFabs(epsilon1 * m_fH1 + scalar);
        dMessage(1,"Caps Intersection Test, PD Case V2- : success ");
        computeContactPointAndNormalCACV2(h1n1, h2n2, epsilon1, epsilon2, depth, -1.0);
      }
      return;
}
//-----------------------------------------------------------------------------------------------------------------------------
//                                                  Interlacing Circles
//-----------------------------------------------------------------------------------------------------------------------------
// Compute 1 contact point and its normal when two caps collide and their boundary circles interlace (Interlacing Circles Case)
//-----------------------------------------------------------------------------------------------------------------------------
// the normal is either orthogonal to one cap (vertical) or directed along the intersection line (horizontal),
// according to where shallowest intersection point lies

// Returns a point I on the intersection line L = Pi1 cap Pi2 of the caps' planes
//-------------------------------------------------------------------------------
void sCylinderCylinderData::pickPointOnIntersectionLine(const dVector3 h1n1, dReal epsilon1, dReal p2, dReal &q, dVector3 O1, dVector3 I) {
  const dReal a0 = dFabs(m_vW[0]);
  const dReal a1 = dFabs(m_vW[1]);
  const dReal a2 = dFabs(m_vW[2]);
  O1[0] = m_vCenter1[0] + epsilon1 * h1n1[0];
  O1[1] = m_vCenter1[1] + epsilon1 * h1n1[1];
  O1[2] = m_vCenter1[2] + epsilon1 * h1n1[2];
  dVector3 O1I;
  dReal p2inv_w;
  if (a1 > a0) {
    if (a2 > a1) { // a2 is the largest w-coordinate
      p2inv_w = p2 / m_vW[2];
      O1I[0] = - p2inv_w * m_mRotation1[6];
      O1I[1] = p2inv_w * m_mRotation1[2];
      O1I[2] = 0;
      q = m_vW[0] * O1I[0] + m_vW[1] * O1I[1];
    } else {  // a1 is the largest w-coordinate
      p2inv_w = p2 / m_vW[1];
      O1I[0] = p2inv_w * m_mRotation1[10];
      O1I[1] = 0.0;
      O1I[2] = -p2inv_w * m_mRotation1[2];
      q = m_vW[0] * O1I[0] + m_vW[2] * O1I[2];
    }
  } else {
    if (a2 > a0) { // a2 is the largest w-coordinate
      p2inv_w = p2 / m_vW[2];
      O1I[0] = - p2inv_w * m_mRotation1[6];
      O1I[1] = p2inv_w * m_mRotation1[2];
      O1I[2] = 0.0;
      q = m_vW[0] * O1I[0] + m_vW[1] * O1I[1];
    } else { // a0 is the largest w-coordinate
      p2inv_w = p2 / m_vW[0];
      O1I[0] = 0.0;
      O1I[1] = - p2inv_w * m_mRotation1[10];
      O1I[2] = p2inv_w * m_mRotation1[6];
      q = m_vW[1] * O1I[1] + m_vW[2] * O1I[2];
    }
  }
  I[0] = O1I[0] + O1[0];
  I[1] = O1I[1] + O1[1];
  I[2] = O1I[2] + O1[2];
}

// Returns a contact point C for a collision between caps with interlacing boundary circles
//------------------------------------------------------------------------------------------
void sCylinderCylinderData::computeContactPointIC(dReal m, const dVector3 u, const dVector3 I) {
  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  dAddScaledVectors3(contact->pos, u, I, m, 1.0);
}

// Returns an horizontal normal for a collision between caps with interlacing boundary circles
//---------------------------------------------------------------------------------------------
void sCylinderCylinderData::computeRadialNormal(dReal depth , const dVector3 normal, dReal normalLength){
  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  contact->depth = depth;
  dCopyScaledVector3(contact->normal, normal, 1.0 / normalLength);
}

// Returns a vertical normal for a collision between caps with interlacing boundary circles
//------------------------------------------------------------------------------------------
void sCylinderCylinderData::computeVerticalNormal(dReal depth, const dReal *R, dReal sign) {
  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  contact->depth = depth;
  contact->normal[0] = sign * R[2];
  contact->normal[1] = sign * R[6];
  contact->normal[2] = sign * R[10];
}

// Returns a normal, either 'vertical' or 'horizontal', for a collision between caps with interlacing boundary circles
//---------------------------------------------------------------------------------------------------------------------
void sCylinderCylinderData::computeNormalIC(dReal epsilon1, dReal epsilon2, const dVector3 normal, dReal normalLength,
  dReal normalDepth, dReal depth, dReal depth1, dReal depth2) {

  if (depth1 >= depth) {
    if (depth2 > depth1) { // depth is the smallest
      computeRadialNormal(normalDepth, normal, normalLength);
      dMessage(1,"small radial intersection : code R1, depth = %f",depth);
    } else {
      if (depth2 >= depth) {
        //depth is the smallest
        computeRadialNormal(normalDepth, normal, normalLength);
        dMessage(1,"small radial intersection : code H2");
      } else {
        //depth2 is the smallest
        computeVerticalNormal(depth2, m_mRotation2, epsilon2);
        dMessage(1,"small vertical intersection : code V1, depth = %f", depth2);
      }
    }
  } else {
    if (depth2 >= depth1) {
      //depth1 is the smallest
      computeVerticalNormal(depth1, m_mRotation1, -epsilon1);
      dMessage(1,"small vertical intersection : code V2, depth = %f",depth1);
    } else {
      //depth2 is the smallest
      computeVerticalNormal(depth2, m_mRotation2, epsilon2);
      dMessage(1,"small vertical intersection : code V1, depth =%f ", depth2);
    }
  }
}

//                         IC Main Program
//  Returns a contact point and its normal for a collision between caps with interlacing boundary circles
//--------------------------------------------------------------------------------------------------------
void sCylinderCylinderData::computeContactPointAndNormalIC(
    dReal sr1, dReal sr2, dReal ch1, dReal ch2, const dVector3 h1n1,
    dReal p1, dReal p2,
    dReal d1, dReal d2, dReal diff12, dReal epsilon1, dReal epsilon2)
{
  dVector3 O1,I;
  dReal q;// q = O1I dotproduct w
  pickPointOnIntersectionLine(h1n1, epsilon1, p2, q, O1, I);  // A particular point I is choosen.

  dReal temp = m_fD - m_fInvSinus * q;
  dReal m = m_fD <= 0.0 ? d2 + temp : - d2 + temp;
  m *= m_fInvSinus;
  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  dAddScaledVectors3(contact->pos, m_vW, I, m, 1.0);

  //depth
  const dVector3 normal = { O1[0] - contact->pos[0], O1[1] - contact->pos[1], O1[2] - contact->pos[2] };
  const dReal normalLength = dSqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
  const dReal normalDepth = m_fRadius1 - normalLength;
  const dReal radialDepth = d1 + d2 - dFabs(m_fD);

  const dReal signC = m_fCosinus > 0.0 ? dReal(1.0) : dReal(-1.0);
  if (epsilon1 * epsilon2 * signC > 0.0) { // two lower caps or two upper caps intersect in a degenerate way
    dMessage(1, "IC: generating radial normal");
    computeRadialNormal(normalDepth, normal, normalLength);
    return;
  }
  // First, we determine which point on the cap of Cylinder 2 is the deepest intersection point (D2 penetrates D1)
  dReal temp1 = epsilon2 * ch2 + m_fDelta1;
  dReal temp2 = sr2;
  dReal depth = m_fH1 - dFabs(temp1 - temp2);

  dReal verticalDepth1 = depth >= 0.0 ? depth : m_fH1 - dFabs(temp1 + temp2);
  if (verticalDepth1 <= 0.0)
    verticalDepth1 = dInfinity;

  // Similarly, we determine which point on the cap of Cylinder 1 is the deepest intersection point (D1 penetrates D2)
  temp1 = epsilon1 * ch1 - m_fDelta2;
  temp2 = sr1;
  depth = m_fH2 - dFabs(temp1 + temp2);
  dReal verticalDepth2 = depth >= 0.0 ? depth : m_fH2 - dFabs(temp1 - temp2);
  if (verticalDepth2 <= 0.0)
    verticalDepth2 = dInfinity;

  if (radialDepth <= 0.0)
    depth = dInfinity;
  computeNormalIC(epsilon1, epsilon2, normal, normalLength, normalDepth, radialDepth, verticalDepth1, verticalDepth2);
}

//-------------------------------------------------------------------------------------------------------------------------------------
//                                                       Cap Against Cap : Main Program
//--------------------------------------------------------------------------------------------------------------------------------------
//                                                        Detects collision between caps
//--------------------------------------------------------------------------------------------------------------------------------------
// It returns up to 4 contact points located on a boundary circle; the unit normal vector is either u = Normalized(n1 cross n2), n1, n2

bool sCylinderCylinderData::slightlyInclinedCaps(dReal sr1, dReal sr2, dReal ch1, dReal ch2, const dVector3 h1n1, const dVector3 h2n2,
    dReal t1, dReal t2, dReal p1, dReal p2, dReal epsilon1, dReal epsilon2,
    dReal O1O2square,  bool &needMoreContacts)
{
  bool success = false;

  // First, we find the intersection point(s) of the discs D1 and D2 with the line L
  dReal q1 = dFabs(p1) * m_fInvSinus;
  dReal q2 = dFabs(p2) * m_fInvSinus;
  dReal temp1 = m_fRadius1 - q2;
  dReal temp2 = m_fRadius2 - q1;

  if (temp1 >= 0.0 && temp2 >= 0.0) { // Boundary circles both intersect L
    dMessage(1,"Both caps intersect with L");
    dReal d1 = dSqrt(temp1 * (m_fRadius1 + q2));
    dReal d2 = dSqrt(temp2 * (m_fRadius2 + q1));
    if (d1 + d2 < m_fDPlus)
      return false;// The caps are disjoint, so collision fails

    dReal diff12 = d1 - d2;
    if (dFabs(diff12) < m_fDPlus) {
      // The boundary circles interlace (IC)
      //------------------------------------
      dMessage(1,"Boundary circles interlace : success ");
      computeContactPointAndNormalIC(sr1, sr2, ch1, ch2, h1n1, p1, p2, d1, d2, diff12, epsilon1, epsilon2);
      ++m_nNumberOfContacts;
      success = true;
      if (m_nNumberOfContacts >= sufficientNumber || m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
        needMoreContacts = false;
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
        return success;
    }
  }
  // The discs D1 and D2 are possibly close to be parallel and may penetrate each other in a degenerate position
  //------------------------------------------------------------------------------------------------------------
  const dReal cSign = m_fCosinus > 0.0 ? REAL(1.0) : REAL(-1.0);
  if (- epsilon1 * epsilon2 * cSign < 0.0)// these caps are very unlikely to intersect
    return success;

  //First, we test D1 against D2
  //-----------------------------

  //-----------
  // Point V1+
  //-----------
  dReal scalar = - p2 - sr1;
  dReal depth = m_fH2 - dFabs(epsilon2 * m_fH2 + scalar);
  temp1 = m_fRadius1Square + O1O2square;
  temp2 = 2.0 * sr1 * (t2 - epsilon2 * m_fH2);

  dReal scalarSquare;
  dReal squaredDistanceToAxis;

  if (depth > 0.0) {
    scalarSquare = scalar * scalar;
    squaredDistanceToAxis = temp1 - temp2 - scalarSquare;
    if (squaredDistanceToAxis < m_fRadius2Square) {
      computeContactPointAndNormalDCACV1(h1n1, epsilon1, epsilon2, depth, 1.0);
      ++m_nNumberOfContacts;
      dMessage(1,"V1+");
      if (m_nNumberOfContacts >= sufficientNumber || m_nNumberOfContacts >= (m_iFlags & NUMC_MASK)) {
        needMoreContacts = false;
        return true;
      }
    }
  }

  //-----------
  // Point W1+
  //-----------
  dReal scalarSquareW = p2 * p2;
  dReal depthW = m_fH2 - dFabs(epsilon2 * m_fH2 - p2);
  dReal temp2W = 2.0 * m_fRadius1 * m_fD;

  bool depthPositive = depthW > 0.0;
  if (depthPositive) {
    squaredDistanceToAxis =  temp1 - temp2W - scalarSquareW;
    if (squaredDistanceToAxis < m_fRadius2Square) {
      computeContactPointAndNormalDCACW1(h1n1, epsilon1, epsilon2, depthW, 1.0);
      ++m_nNumberOfContacts;
      dMessage(1,"W1+");
      if (m_nNumberOfContacts >= sufficientNumber || m_nNumberOfContacts >= (m_iFlags & NUMC_MASK)){
        needMoreContacts = false;
        return true;
      }
    }
  }

  //-----------
  // Point V1-
  //-----------
  scalar = - p2 + sr1;
  depth = m_fH2 - dFabs(epsilon2 * m_fH2 + scalar);

  if (depth > 0.0) {
    scalarSquare = scalar * scalar;
    squaredDistanceToAxis =  temp1 + temp2 - scalarSquare;
    if (squaredDistanceToAxis < m_fRadius2Square){
      computeContactPointAndNormalDCACV1(h1n1, epsilon1, epsilon2, depth, -1.0);
      ++m_nNumberOfContacts;
      dMessage(1,"V1-");
      if (m_nNumberOfContacts >= sufficientNumber || m_nNumberOfContacts >= (m_iFlags & NUMC_MASK)){
        needMoreContacts = false;
        return true;
      }
    }
  }

  //-----------
  // Points W1-
  //-----------
  if (depthPositive) {
    squaredDistanceToAxis =  temp1 + temp2W - scalarSquareW;
    if(squaredDistanceToAxis < m_fRadius2Square){
      computeContactPointAndNormalDCACW1(h1n1, epsilon1, epsilon2, depthW, -1.0);
      ++m_nNumberOfContacts;
      dMessage(1,"W1-");
      if (m_nNumberOfContacts >= sufficientNumber || m_nNumberOfContacts >= (m_iFlags & NUMC_MASK)){
         needMoreContacts = false;
         return true;
      }
    }
  }

  //Now we test D2 against D1
  //-------------------------

  //------------
  // Points V2+
  //------------
  scalar = p1 + sr2;
  depth = m_fH1 - dFabs(epsilon1 * m_fH1 + scalar);
  temp1 = m_fRadius2Square + O1O2square;
  temp2 = 2.0 * sr2 * (t1 - epsilon1 * m_fH1);

  if (depth > 0.0) {
    scalarSquare = scalar*scalar;
    squaredDistanceToAxis = temp1 + temp2 - scalarSquare;
    if (squaredDistanceToAxis < m_fRadius1Square){
      computeContactPointAndNormalDCACV2(h2n2, epsilon1, epsilon2, depth, 1.0);
      ++m_nNumberOfContacts;
      success = true;
      dMessage(1,"V2+");
      if (m_nNumberOfContacts >= sufficientNumber || m_nNumberOfContacts >= (m_iFlags & NUMC_MASK)){
        needMoreContacts = false;
        return true;
      }
    }
  }

  //------------
  // Points W2+
  //------------
  depthW = m_fH1 - dFabs(epsilon1 * m_fH1 + p1);
  scalarSquareW = p1 * p1;
  temp2W = 2.0 * m_fRadius2 * m_fD;
  depthPositive = depthW > 0.0;

  if (depthPositive) {
    squaredDistanceToAxis = temp1 + temp2W - scalarSquareW;
    if (squaredDistanceToAxis < m_fRadius1Square){
      computeContactPointAndNormalDCACW2(m_mRotation2, h2n2, epsilon1, epsilon2, depthW, 1.0);
      ++m_nNumberOfContacts;
      success = true;
      dMessage(1,"W2+");
      if (m_nNumberOfContacts >= sufficientNumber || m_nNumberOfContacts >= (m_iFlags & NUMC_MASK)){
         needMoreContacts = false;
         return true;
      }
    }
  }

  //------------
  // Points V2-
  //------------
  scalar = p1 - sr2;
  depth  = m_fH1 - dFabs(epsilon1 * m_fH1 + scalar);
  if (depth > 0.0) {
    scalarSquare = scalar * scalar;
    squaredDistanceToAxis = temp1 - temp2 - scalarSquare;
    if (squaredDistanceToAxis < m_fRadius1Square) {
      computeContactPointAndNormalDCACV2(h2n2, epsilon1, epsilon2, depth, -1.0);
      ++m_nNumberOfContacts;
      success = true;
      dMessage(1,"V2-");
      if (m_nNumberOfContacts >= sufficientNumber || m_nNumberOfContacts >= (m_iFlags & NUMC_MASK)){
        needMoreContacts = false;
        return true;
      }
    }
  }

  //-----------
  // Points W2-
  //-----------
  if (depthPositive) {
    squaredDistanceToAxis =  temp1 - temp2W - scalarSquareW;
    if (squaredDistanceToAxis < m_fRadius1Square){
      computeContactPointAndNormalDCACW2(m_mRotation1, h2n2, epsilon1, epsilon2, depthW, -1.0);
      ++m_nNumberOfContacts;
      success = true;
      dMessage(1,"W2-");
      if (m_nNumberOfContacts >= sufficientNumber || m_nNumberOfContacts >= (m_iFlags & NUMC_MASK)){
         needMoreContacts = false;
         return true;
      }
    }
  }

  return success;
}

bool sCylinderCylinderData::stronglyInclinedCaps(dReal sr1, dReal sr2, dReal ch1, dReal ch2, const dVector3 h1n1, const dVector3 h2n2,
    dReal p1, dReal p2, dReal epsilon1, dReal epsilon2, dReal O1O2square)
{
  // First, we find the intersection point(s) of the discs D1 and D2 with the line L
  dReal q1 = dFabs(p1) * m_fInvSinus;
  dReal q2 = dFabs(p2) * m_fInvSinus;
  dReal temp1 = m_fRadius1 - q2;
  dReal temp2 = m_fRadius2 - q1;

  if (temp1 >= 0.0 && temp2 >= 0.0) { // Boundary circles both intersect L
  //dMessage(1,"Caps both intersect with L");
    dReal d1 = sqrt(temp1 * (m_fRadius1 + q2));
    dReal d2 = sqrt(temp2 * (m_fRadius2 + q1));
    if (d1 + d2 < m_fDPlus)
      return true;// The caps are disjoint, so collision fails

    dReal diff12 = d1 - d2;
    if (dFabs(diff12) < m_fDPlus) {
      // The boundary circles interlace (IC)
      //------------------------------------
      //dMessage(1,"Boundary circles interlace : success ");
      computeContactPointAndNormalIC(sr1, sr2, ch1, ch2, h1n1, p1, p2, d1, d2, diff12, epsilon1, epsilon2);
      ++m_nNumberOfContacts;
      return (m_nNumberOfContacts < sufficientNumber) && (m_nNumberOfContacts < (m_iFlags & NUMC_MASK));
    } else if (-diff12 > m_fDPlus) {
      dMessage(1,"PDV1 : success ");
      computeContactPointAndNormalPDV1(sr1, p2, h1n1, h2n2, epsilon1, epsilon2);
      ++m_nNumberOfContacts;
      return (m_nNumberOfContacts < sufficientNumber) && (m_nNumberOfContacts < (m_iFlags & NUMC_MASK));
    } else {
      dMessage(1,"PDV2 : success ");
      computeContactPointAndNormalPDV2(sr2, p1, h1n1, h2n2, epsilon1, epsilon2);
      ++m_nNumberOfContacts;
    }
  }

  return (m_nNumberOfContacts < sufficientNumber) && (m_nNumberOfContacts < (m_iFlags & NUMC_MASK));
}

void sCylinderCylinderData::stronglyInclinedCapsTest(dReal sr1, dReal sr2, dReal ch1, dReal ch2, dReal ch1h2,
    const dVector3 h1n1, const dVector3 h2n2, dReal t1, dReal t2)
{
  bool needMoreContacts = true;
  const dReal h1Delta1 = m_fHeight1 * m_fDelta1;
  const dReal h2Delta2 = m_fHeight2 * m_fDelta2;

  dReal p1 = - m_fH1   + m_fDelta1 + ch2;
  dReal p2 = - ch1  + m_fDelta2 + m_fH2;
  dReal p1Plus = dFabs(p1);
  dReal p2Plus = dFabs(p2);

  dReal squaredDistanceBetweenCenters = m_fSigma - h1Delta1 + h2Delta2 - ch1h2;

  #define CAPS_ARE_NOT_TOO_FAR_AWAY(expr1,expr2,compare1,compareh1, compare2, compareh2) \
        (p1Plus <= sr2 || (p1 compare1 0.0 && p1 compareh1 (expr1))) && (p2Plus <= sr1 || (p2 compare2 0.0 && p2 compareh2 (expr2)) ) && \
        (squaredDistanceBetweenCenters <= m_fRadiusSumSquare)

  #define INITIALIZE_DISTANCE_AND_DEPTH_PARAMATERS(eh1, eh2, ech1, ech2, eh1Delta1, eh2Delta2, ech1h2) \
          p1 = (eh1)   + m_fDelta1 + (ech2); \
          p2 = (ech1)  + m_fDelta2 + (eh2); \
          p1Plus = dFabs(p1); \
          p2Plus = dFabs(p2); \
          squaredDistanceBetweenCenters = m_fSigma + (eh1Delta1) + (eh2Delta2) + (ech1h2);

  if (CAPS_ARE_NOT_TOO_FAR_AWAY(-m_fH1, m_fH2,<,>, >,<)) {
    needMoreContacts = needMoreContacts && stronglyInclinedCaps(sr1, sr2, ch1, ch2, h1n1, h2n2, p1, p2, REAL(1.0), REAL(1.0),
      squaredDistanceBetweenCenters);
  }

  if (needMoreContacts) {
    INITIALIZE_DISTANCE_AND_DEPTH_PARAMATERS(-m_fH1, -m_fH2, -ch1, -ch2, -h1Delta1, -h2Delta2, ch1h2)
    if (CAPS_ARE_NOT_TOO_FAR_AWAY(-m_fH1, -m_fH2,< , >, <,>))
      needMoreContacts = needMoreContacts && stronglyInclinedCaps(sr1, sr2, ch1, ch2, h1n1, h2n2, p1, p2, REAL(1.0), REAL(-1.0),
        squaredDistanceBetweenCenters);

    if (needMoreContacts) {
      INITIALIZE_DISTANCE_AND_DEPTH_PARAMATERS(m_fH1, m_fH2, ch1, ch2, h1Delta1, h2Delta2, ch1h2)
      if (CAPS_ARE_NOT_TOO_FAR_AWAY(m_fH1, m_fH2, >,<, >,<)) {
        needMoreContacts = needMoreContacts && stronglyInclinedCaps(sr1, sr2, ch1, ch2, h1n1, h2n2, p1, p2, REAL(-1.0), REAL(1.0),
          squaredDistanceBetweenCenters);
      }

       if (needMoreContacts){
        INITIALIZE_DISTANCE_AND_DEPTH_PARAMATERS(m_fH1, -m_fH2, ch1, -ch2, h1Delta1, -h2Delta2, -ch1h2)
        if (CAPS_ARE_NOT_TOO_FAR_AWAY(m_fH1, -m_fH2,>,<, <,>)) {
          needMoreContacts = needMoreContacts && stronglyInclinedCaps(sr1, sr2, ch1, ch2, h1n1, h2n2, p1, p2, REAL(-1.0), REAL(-1.0),
            squaredDistanceBetweenCenters);
        }
      }
    }
  }
  #undef CAPS_ARE_NOT_TOO_FAR_AWAY
  #undef INITIALIZE_DISTANCE_AND_DEPTH_PARAMATERS
}

void sCylinderCylinderData::slightlyInclinedCapsTest(
    dReal sr1, dReal sr2, dReal ch1, dReal ch2, dReal ch1h2,
    const dVector3 h1n1, const dVector3 h2n2,
    dReal t1, dReal t2, int dcac[2][2] // numbers of intersection of the upper (resp. lower) cap of Cylinder 1 (resp. Cylinder 2)
)
{
  bool needMoreContacts = true;
  const dReal cSign = m_fCosinus > 0.0 ? REAL(1.0) : REAL(-1.0);
  const dReal h1Delta1 = m_fHeight1 * m_fDelta1;
  const dReal h2Delta2 = m_fHeight2 * m_fDelta2;

  dReal p1 = - m_fH1 + m_fDelta1 + ch2;
  dReal p2 = - ch1 + m_fDelta2 + m_fH2;
  dReal p1Plus = dFabs(p1);
  dReal p2Plus = dFabs(p2);

  dReal squaredDistanceBetweenCenters = m_fSigma - h1Delta1 + h2Delta2 - ch1h2;

  #define CAPS_ARE_NOT_TOO_FAR_AWAY(expr1, expr2, compare1, compareh1, compare2, compareh2) \
        (p1Plus <= sr2 || (p1 compare1 0.0 && p1 compareh1 (expr1))) && (p2Plus <= sr1 || (p2 compare2 0.0 && p2 compareh2 (expr2)) ) && \
        (squaredDistanceBetweenCenters <= m_fRadiusSumSquare)
  #define INITIALIZE_DISTANCE_AND_DEPTH_PARAMATERS(eh1, eh2, ech1, ech2, eh1Delta1, eh2Delta2, ech1h2) \
          p1 = (eh1)   + m_fDelta1 + (ech2); \
          p2 = (ech1)  + m_fDelta2 + (eh2); \
          p1Plus = dFabs(p1); \
          p2Plus = dFabs(p2); \
          squaredDistanceBetweenCenters = m_fSigma + (eh1Delta1) + (eh2Delta2) + (ech1h2);

  if (CAPS_ARE_NOT_TOO_FAR_AWAY(-m_fH1, m_fH2,<,>, >,<)) {
    const bool success = slightlyInclinedCaps(sr1, sr2, ch1, ch2, h1n1, h2n2, t1, t2, p1, p2, REAL(1.0), REAL(1.0),
      squaredDistanceBetweenCenters, needMoreContacts);
    if (success && cSign > 0.0) {
      ++(dcac[0][0]);
      ++(dcac[1][1]);
    }
  }

  if (needMoreContacts) {
    INITIALIZE_DISTANCE_AND_DEPTH_PARAMATERS(-m_fH1, -m_fH2, -ch1, -ch2, -h1Delta1, -h2Delta2, ch1h2)
    if (CAPS_ARE_NOT_TOO_FAR_AWAY(-m_fH1, -m_fH2,<,>, <,>)) {
      const bool success = slightlyInclinedCaps(sr1, sr2, ch1, ch2, h1n1, h2n2, t1, t2, p1, p2, REAL(1.0), REAL(-1.0),
        squaredDistanceBetweenCenters, needMoreContacts);
      if (success && cSign < 0.0) {
        ++(dcac[0][0]);
        ++(dcac[1][0]);
      }
    }

    if (needMoreContacts) {
      INITIALIZE_DISTANCE_AND_DEPTH_PARAMATERS(m_fH1, m_fH2, ch1, ch2, h1Delta1, h2Delta2, ch1h2)
      if (CAPS_ARE_NOT_TOO_FAR_AWAY(m_fH1, m_fH2,>,<, >,<)) {
        const bool success = slightlyInclinedCaps(sr1, sr2, ch1, ch2, h1n1, h2n2, t1, t2, p1, p2, REAL(-1.0), REAL(1.0),
          squaredDistanceBetweenCenters, needMoreContacts);
        if (success && cSign < 0.0) {
          ++(dcac[0][1]);
          ++(dcac[1][1]);
        }
      }

      if (needMoreContacts) {
        INITIALIZE_DISTANCE_AND_DEPTH_PARAMATERS(m_fH1, -m_fH2, ch1, -ch2, h1Delta1, -h2Delta2, -ch1h2)
        if (CAPS_ARE_NOT_TOO_FAR_AWAY(m_fH1, -m_fH2,>,<, <,>)) {
          const bool success = slightlyInclinedCaps(sr1, sr2, ch1, ch2, h1n1, h2n2, t1, t2, p1, p2, REAL(-1.0), REAL(-1.0),
            squaredDistanceBetweenCenters, needMoreContacts);
          if (success && cSign > 0.0) {
            ++(dcac[0][1]);
            ++(dcac[1][0]);
          }
        }
      }
    }
  }
  #undef CAPS_ARE_NOT_TOO_FAR_AWAY
  #undef INITIALIZE_DISTANCE_AND_DEPTH_PARAMATERS

  //dMessage(1, "%d %d %d %d", dcac[0][0], dcac[0][1], dcac[1][0], dcac[1][1]);
}

//////////////////////////////////////
//------------------------------------
// Preparing Global Parallel Axes Test
//------------------------------------
//////////////////////////////////////

// Parallel axes : Generator against Generator (GAG)
//              Case I_1
//              --------
//  A cylinder contains a generator of the other
//-------------------------------------------------------------
//  3 contact points are chosen on the generator, all have the same normal -u
void sCylinderCylinderData::computeContactPointsParallelAxesGAG(const dVector3 c2, const dReal *n,
    dReal r2, dReal h2, dReal dr, dVector3 u, dReal revertU)
{
  const dVector3 v = { - r2 * u[0] + c2[0], - r2 * u[1] + c2[1], - r2 * u[2] + c2[2] };
  const dVector3 v1 = { h2 * n[2], h2 * n[6], h2 * n[10] };

  dContactGeom *contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  dAddVectors3(contact->pos, v, v1);
  contact->depth = dr;
  dCopyScaledVector3(contact->normal, u, revertU);
  ++m_nNumberOfContacts;
  if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK) )
    return;

  contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  dSubtractVectors3(contact->pos, v, v1);
  contact->depth = dr;
  dCopyScaledVector3(contact->normal, u, revertU);
  ++m_nNumberOfContacts;
  if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK) )
    return;

  contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  dCopyVector3(contact->pos, v);
  contact->depth = dr;
  dCopyScaledVector3(contact->normal, u, revertU);
  ++m_nNumberOfContacts;
}

// Parallel axes : Cap against Cap (CAC)
//            Case I_2
//            --------
//  A cap of a cylinder contains a cap of the other
//------------------------------------------------------------------------
//  up to 8 contact points are chosen on the small cap, all have the same normal : -epsilon*n

void sCylinderCylinderData::computeContactPointsParallelAxesCAC(
    const dReal *R1, const dReal *R2,
    const dVector3 c2, dReal r2,
    dReal h1, dReal h2,
    dReal ch2, dReal delta1,
    dReal epsilon1, dReal epsilon2)
  {
    const dReal t  = - epsilon2 * h2;
    const dReal tc = - epsilon2 * ch2;
    const dVector3 v = { t * R2[2]  + c2[0], t * R2[6]  + c2[1], t * R2[10] + c2[2] };
    const dVector3 v1 = { r2 * R2[0], r2 * R2[4], r2 * R2[8] };

    const dVector3 normal = { -epsilon1 * R1[2], -epsilon1 * R1[6], -epsilon1 * R1[10] };
    const dReal temp = delta1 + tc;
    const dReal cosinus2 = dCalcVectorDot3_44(R1 + 2, R2);
    const dReal cosinus2r2 = cosinus2 * r2;
    dReal depth = h1 - dFabs(temp + cosinus2r2);

    if (depth > 0.0) {
      dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
      dAddVectors3(contact->pos, v, v1);
      dCopyVector3(contact->normal, normal);
      contact->depth = depth;
      ++m_nNumberOfContacts;
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK) )
        return;
    }

    const dVector3 v2 = { r2 * R2[1], r2 * R2[5], r2 * R2[9] };
    const dVector3 v3Plus = { M_SQRT1_2 * (v1[0] + v2[0]), M_SQRT1_2 * (v1[1] + v2[1]), M_SQRT1_2 * (v1[2] + v2[2]) };
    const dReal sinus2  = dCalcVectorDot3_44(R1 + 2, R2 + 1);
    const dReal r2PythagorPlusPlus = M_SQRT1_2 * r2 * (cosinus2 + sinus2);
    depth = h1 - dFabs(temp + r2PythagorPlusPlus);

    if (depth > 0.0) {
      dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
      dAddVectors3(contact->pos, v, v3Plus);
      dCopyVector3(contact->normal, normal);
      contact->depth = depth;
      ++m_nNumberOfContacts;
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
        return;
    }

    const dReal sinus2r2 = sinus2 * r2;
    depth = h1 - dFabs(temp + sinus2r2);
    if (depth > 0.0) {
      dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
      dAddVectors3(contact->pos, v, v2);
      dCopyVector3(contact->normal, normal);
      contact->depth = depth;
      ++m_nNumberOfContacts;
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
        return;
    }

    const dVector3 v3Minus = { M_SQRT1_2 * (v1[0] - v2[0]), M_SQRT1_2 * (v1[1] - v2[1]), M_SQRT1_2 * (v1[2] - v2[2]) };
    const dReal r2PythagorMinusPlus = M_SQRT1_2 * r2 *(-cosinus2 + sinus2);
    depth = h1 - dFabs(temp + r2PythagorMinusPlus);
    if (depth > 0.0) {
      dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
      dSubtractVectors3(contact->pos, v, v3Minus);
      dCopyVector3(contact->normal, normal);
      contact->depth = depth;
      ++m_nNumberOfContacts;
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
        return;
    }

    depth = h1 - dFabs(temp - cosinus2r2);
    if (depth > 0.0) {
      dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
      dSubtractVectors3(contact->pos, v, v1);
      dCopyVector3(contact->normal, normal);
      contact->depth = depth;
      ++m_nNumberOfContacts;
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
        return;
    }

    depth = h1 - dFabs(temp - r2PythagorPlusPlus);
    if (depth > 0.0) {
      dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
      dSubtractVectors3(contact->pos, v, v3Plus);
      dCopyVector3(contact->normal, normal);
      contact->depth = depth;
      ++m_nNumberOfContacts;
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
        return;
    }

    depth = h1 - dFabs(temp - sinus2r2);
    if (depth > 0.0) {
      dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
      dSubtractVectors3(contact->pos, v, v2);
      dCopyVector3(contact->normal, normal);
      contact->depth = depth;
      ++m_nNumberOfContacts;
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
        return;
    }

    depth = h1 - dFabs(temp - r2PythagorMinusPlus);
    if (depth > 0.0) {
      dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
      dAddVectors3(contact->pos, v, v3Minus);
      dCopyVector3(contact->normal, normal);
      contact->depth = depth;
      ++m_nNumberOfContacts;
    }
}

//           Parallel axes : Corners collide
//                    Case I_3
//                    --------
//  Corners collide with a small horizontal overlap
//----------------------------------------------------
//  3 contact points are chosen in the overlap zone,
//  they share the same radial normal u

void sCylinderCylinderData::computeContactPointsParallelAxesCornersH(dReal epsilon, dReal dh, dReal dr, dVector3 u) {
  const dReal t = epsilon * m_fH2;
  dNegateVector3(u);

  dContactGeom *const contact0 = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  contact0->pos[0] = m_fRadius2 * u[0] - t * m_mRotation1[2]  + m_vCenter2[0];
  contact0->pos[1] = m_fRadius2 * u[1] - t * m_mRotation1[6]  + m_vCenter2[1];
  contact0->pos[2] = m_fRadius2 * u[2] - t * m_mRotation1[10] + m_vCenter2[2];
  contact0->depth = dr;
  dCopyVector3(contact0->normal, u);
  ++m_nNumberOfContacts;
  if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
    return;

  dContactGeom *const contact1 = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  const dReal s = epsilon * dh;
  contact1->pos[0] = s * m_mRotation1[2]  + contact0->pos[0];
  contact1->pos[1] = s * m_mRotation1[6]  + contact0->pos[1];
  contact1->pos[2] = s * m_mRotation1[10] + contact0->pos[2];
  contact1->depth = dr;
  dCopyVector3(contact1->normal, u);
  ++m_nNumberOfContacts;
  if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
    return;

  dContactGeom *const contact2 = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  contact2->pos[0] = REAL(0.5) * (contact1->pos[0] + contact0->pos[0]);
  contact2->pos[1] = REAL(0.5) * (contact1->pos[1] + contact0->pos[1]);
  contact2->pos[2] = REAL(0.5) * (contact1->pos[2] + contact0->pos[2]);
  contact2->depth = dr;
  dCopyVector3(contact2->normal, u);
  ++m_nNumberOfContacts;
}

//           Parallel axes : case I_4
//  Corners collide with a resting contact
//----------------------------------------------------
//  up to 8 contact points are chosen in the overlap zone
// All have a normal orthogonal to the caps, namely -epsilon*n

void sCylinderCylinderData::computeContactPointsParallelAxesCornersV(dReal r, dReal ch1, dReal epsilon, const dVector3 u, const dVector3 v)
{
  const dReal t = epsilon * m_fH1;
  const dReal ct = epsilon * ch1;

  const dVector3 c1 = { t * m_mRotation1[2] + m_vCenter1[0], t * m_mRotation1[6] + m_vCenter1[1], t * m_mRotation1[10] + m_vCenter1[2] };
  const dVector3 v1 = { m_fRadius1 * u[0], m_fRadius1 * u[1], m_fRadius1 * u[2] };
  const dVector3 normal = { - epsilon * m_mRotation1[2], - epsilon * m_mRotation1[6], - epsilon * m_mRotation1[10] };

  const dReal temp = - m_fDelta2 + ct;
  const dReal scalarU = dCalcVectorDot3_14(u, m_mRotation2 + 2);
  dReal depth = m_fH2 - dFabs(temp + m_fRadius1 * scalarU);
  if (depth > 0.0) {
    dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
    dAddVectors3(contact->pos, c1, v1);
    contact->depth = depth;
    dCopyVector3(contact->normal, normal);
    ++m_nNumberOfContacts;
    if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
      return;
  }

  const dReal cosinusTheta1 = REAL(0.5) * (r * r + m_fRadius1 * m_fRadius1 - m_fRadius2 * m_fRadius2) / (m_fRadius1 * r);
  const dReal sinusTheta1 = dSqrt(REAL(1.0) - cosinusTheta1 * cosinusTheta1);
  const dReal cosinusHalfTheta1 = dSqrt(REAL(0.5)*(cosinusTheta1 + REAL(1.0)));
  const dReal sinusHalfTheta1 = sinusTheta1 / ( 2.0 * cosinusHalfTheta1);

  dReal m1 = cosinusHalfTheta1 * m_fRadius1;
  dReal m2 = sinusHalfTheta1 * m_fRadius1;

  const dVector3 O2 = { m1 * u[0] + c1[0], m1 * u[1] + c1[1], m1 * u[2] + c1[2] };
  const dVector3 v2 = { m2 * v[0], m2 * v[1], m2 * v[2] };

  const dReal scalarV = dCalcVectorDot3_14(v, m_mRotation2 + 2);
  const dReal tempO2 = temp + m1 * scalarU;
  const dReal nu2 = m2 * scalarV;
  depth = m_fH2 - dFabs(tempO2 + nu2);

  if (depth > 0.0) {
    dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
    dAddVectors3(contact->pos, O2, v2);
    contact->depth = depth;
    dCopyVector3(contact->normal, normal);
    ++m_nNumberOfContacts;
    if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
      return;
  }

  const dReal cr1 = cosinusTheta1 * m_fRadius1;
  const dReal sr1 = sinusTheta1 * m_fRadius1;

  const dVector3 O1 = { cr1 * u[0] + c1[0], cr1 * u[1] + c1[1], cr1 * u[2] + c1[2] };
  const dVector3 v1Prime = { sr1 * v[0], sr1 * v[1], sr1 * v[2] };

  const dReal tempO1 = temp + cr1 * scalarU;
  const dReal sr1ScalarV = sr1 * scalarV;
  depth = m_fH2 - dFabs(tempO1 + sr1ScalarV);
  if (depth > 0.0) {
    dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
    dAddVectors3(contact->pos, O1, v1Prime);
    contact->depth  = depth;
    dCopyVector3(contact->normal, normal);
    ++m_nNumberOfContacts;
    if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
      return;
  }

  const dReal sinusTheta2 = sr1 / m_fRadius2;
  const dReal cosinusTheta2 = sqrt(REAL(1.0) - sinusTheta2 * sinusTheta2);
  const dReal sinusHalfTheta2 = sqrt(REAL(0.5) * (REAL(1.0) - cosinusTheta2));

  m1 = r - m_fRadius2 * sinusTheta2 / (2.0 * sinusHalfTheta2);
  m2 = m_fRadius2 * dSqrt(REAL(0.5) * (REAL(1.0) - cosinusTheta2));

  const dVector3 O3 = { m1 * u[0] + c1[0], m1 * u[1] + c1[1], m1 * u[2] + c1[2] };
  const dVector3 v3 = { m2 * v[0], m2 * v[1], m2 * v[2] };
  const dReal nu3 = m2 * scalarV;
  const dReal tempO3 = temp + m1 * scalarU;
  depth = m_fH2 - dFabs(tempO3 + nu3);
  if (depth > 0.0) {
    dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
    dAddVectors3(contact->pos, O3, v3);
    contact->depth = depth;
    dCopyVector3(contact->normal, normal);
    ++m_nNumberOfContacts;
    if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
      return;
  }

  const dReal rr2 = r - m_fRadius2;
  depth = m_fH2 - dFabs(temp + rr2 * scalarU);
  if (depth > 0.0) {
    dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
    dAddScaledVectors3(contact->pos, c1, u, 1.0, rr2);
    contact->depth = depth;
    dCopyVector3(contact->normal, normal);
    ++m_nNumberOfContacts;
    if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
      return;
  }

  depth = m_fH2 - dFabs(tempO3 - nu3);
  if (depth > 0.0) {
    dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
    dSubtractVectors3(contact->pos, O3, v3);
    contact->depth = depth;
    dCopyVector3(contact->normal, normal);
    ++m_nNumberOfContacts;
    if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
      return;
  }

  depth = m_fH2 - dFabs(tempO1 - sr1ScalarV);
  if (depth > 0.0) {
    dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
    dSubtractVectors3(contact->pos, O1, v1Prime);
    contact->depth = depth;
    dCopyVector3(contact->normal, normal);
    ++m_nNumberOfContacts;
    if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
      return;
  }

  depth = m_fH2 - dFabs(tempO2 - nu2);
  if (depth > 0.0) {
    dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
    dSubtractVectors3(contact->pos, O2, v2);
    contact->depth = depth;
    dCopyVector3(contact->normal, normal);
    ++m_nNumberOfContacts;
  }
}

//     Parallel Axes Global Test
//     --------------------------

// The function below handles the case of parallel axes
//-----------------------------------------------------
/* It returns 3 contact points if the cylinders collide along generators (Generator Against Generator),
 *    8 contact points if one cap entirely or partially rests on another one (a Cap Rests over a Cap);
 *    3 contact points for a small overlap of corners;
 *  in each case, contact points share the SAME depth and the SAME normal */

void sCylinderCylinderData::parallelAxesIntersectionTest(dReal ch1, dReal ch2, dReal hSum1) {
  // We compute the unit vector u = u1 whose direction is orthogonal to the axes of both cylinders
  dVector3 u = { m_vDelta[0] - m_fDelta1 * m_mRotation1[2], m_vDelta[1] - m_fDelta1 * m_mRotation1[6], m_vDelta[2] - m_fDelta1 * m_mRotation1[10] };

  dReal r = dCalcVectorLengthSquare3(u);// r is the squared distance separating the axes
  if (r <= 0.0)
    return;

  const dReal l = dRecipSqrt(r);
  dNormalize3(u);
  //we compute the coordinates r and h of the center c2 of Cylinder 2 in the frame (c1; u1,n1), c1 being the center of Cylinder 1
  r *= l;

  dReal dr = m_fRadiusSum - r;
  //Exit test
  if (dr < 0.0)
    return;

  const dReal delta1Plus = dFabs(m_fDelta1);
  dReal dh = dFabs(delta1Plus - hSum1);
  dr = dFabs(m_fCosinus * dr);

  //dMessage(1, "dr %g, dh %g delta1Plus %g hSum1 %g", dr, dh, delta1Plus, hSum1);

  //First, we store the sign of delta1
  const dReal epsilon1 = m_fDelta1 > 0.0 ? REAL(1.0) : REAL(-1.0);
  const dReal epsilon2 = m_fDelta2 > 0.0 ? REAL(1.0) : REAL(-1.0);
  const dReal h12 = m_fH1 - m_fH2;

  // I1 : a generator of a cylinder is contained in the other
  // 3 contact points are chosen on the generator, all have the same normal -u
  if (delta1Plus <= h12) {
     dMessage(1,"Parallel axes I1\n");
     computeContactPointsParallelAxesGAG(m_vCenter2, m_mRotation1, m_fRadius2, m_fH2, dr, u, -1.0);
     return;
  }

  if (delta1Plus <= -h12) {
    dNegateVector3(u);
    computeContactPointsParallelAxesGAG(m_vCenter1, m_mRotation1, m_fRadius1, m_fH1, dr, u, 1.0);
    return;
  }

  // Case I2 : A cap of a cylinder contains a cap of the other one (Cap Against Cap)
  // up to 8 contact points are selected on the small cap, all have the same normal.
  const dReal r12 = m_fRadius1 - m_fRadius2;
  if (r <= r12) {
    dMessage(1,"Parallel axes I2: r <= r12\n");
    computeContactPointsParallelAxesCAC(m_mRotation1, m_mRotation2, m_vCenter2, m_fRadius2, m_fH1, m_fH2, ch2, m_fDelta1, epsilon1, epsilon2);
    return;
  }

  if (r <= -r12) {
    dMessage(1,"Parallel axes I2: r <= r21\n");
    computeContactPointsParallelAxesCAC(m_mRotation2, m_mRotation1, m_vCenter1, m_fRadius1, m_fH2, m_fH1, ch1,-m_fDelta2, epsilon2, -epsilon1);
    return;
  }

  // Case I3 : corners collide with a small horizontal overlap
  // We select three 3 contact points, namely the ends and the middle point of the overlap interval; all have the same normal
  if (dr <= dh) {
   dMessage(1,"Parallel axes I3\n");
   computeContactPointsParallelAxesCornersH(epsilon1, dh, dr, u);
   return;
  }
  // Case I4 : corners have resting contacts
  // up to 8 contact points are selected on the overlaping caps
  else {
    dVector3 v;
    dCalcVectorCross3_141(v, m_mRotation1 + 2, u);
    dMessage(1,"Parallel axes I4\n");
    computeContactPointsParallelAxesCornersV(r, ch1, epsilon1, u, v);
    return;
  }

  dMessage(1,"Parallel axes: default case");
  dAASSERT(false);
}

////////////////////////////////////
//----------------------------------
//   Orthogonal Axes Global Test
//----------------------------------
////////////////////////////////////

// The function below handles the case of orthogonal axes
//-------------------------------------------------------

void sCylinderCylinderData::orthogonalAxesIntersectionTest(
    const dVector3 A1,
    const dReal *R1, const dReal *R2,
    dReal r1, dReal r2, dReal h1, dReal h2,
    dReal delta1, dReal delta2,
    dReal t1, dReal t2,
    dReal rho1, dReal rho2,
    dReal d12, dReal d21,
    dReal epsilon1, dReal epsilon2, dReal epsilon,
    bool &exit)
{
//                          Case I1
//                          -------
// A cap of the second cylinder contains a part of a generator of the first (CCG)
// Returns up to 3 contact points
//-------------------------------
  dReal r2d = r2 - m_fDPlus;
  if (r2d >= 0.0) {
    dReal zeta2 = dSqrt(r2d * (r2 + m_fDPlus));
    dReal alpha, omega;
    dReal temp1 =  h1 - delta1;
    dReal temp2 = -h1 - delta1;

    if (-zeta2 > temp1 || zeta2 < temp2)
      return;

    alpha = temp1 > zeta2 ? zeta2 : temp1;
    omega = temp2 < -zeta2 ? -zeta2 : temp2;

    dMessage(1,"Orthogonal axes, case I1");
    const dVector3 normal = { -epsilon * epsilon2 * R2[2], -epsilon * epsilon2 * R2[6], -epsilon * epsilon2 * R2[10] };

    const dReal er1 = epsilon2 * r1;
    const dVector3 v = { er1 * R2[2] + A1[0], er1 * R2[6]  + A1[1], er1 * R2[10] + A1[2] };
    const dVector3 v1 = { alpha * R1[2], alpha * R1[6], alpha * R1[10] };

    const dReal temp = t2 + er1;
    const dReal calpha = m_fCosinus * alpha;
    dReal depth = h2 - dFabs(temp + calpha);
    if (depth > 0.0) {
      dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
      dAddVectors3(contact->pos, v, v1);
      dCopyVector3(contact->normal, normal);
      contact->depth = depth;
      exit = true;
      ++m_nNumberOfContacts;
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
        return;
    }

    const dVector3 v2 = { omega * R1[2], omega * R1[6], omega * R1[10] };
    const dReal comega = m_fCosinus * omega;
    depth = h2 - dFabs(temp + comega);

    if (depth > 0.0) {
      dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
      dAddVectors3(contact->pos, v, v2);
      dCopyVector3(contact->normal, normal);
      contact->depth = depth;
      exit = true;
      ++m_nNumberOfContacts;
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK) )
        return;
    }

    depth = h2 - dFabs(temp + (calpha + comega) * REAL(0.5));
    if (depth > 0.0) {
      dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
      const dVector3 v3 = { REAL(0.5) * (v1[0] + v2[0]), REAL(0.5) * (v1[1] + v2[1]), REAL(0.5) * (v1[2] + v2[2]) };
      dAddVectors3(contact->pos, v, v3);
      dCopyVector3(contact->normal, normal);
      contact->depth = depth;
      exit = true;
      ++m_nNumberOfContacts;
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK) )
        return;
    }
    return;
  } else {
    //Two exit tests
    dReal temp = d12 * (r1 + rho2);
    if (temp < 0.0)
      return;
    const dReal gamma12 = dSqrt(temp);
    dReal r2d = r2 - m_fDPlus;
    const dReal lambda = r2d + gamma12;
    if (lambda < -thresholdRadius * r2) {
      exit = true;
      return;
    }
    temp = d21 * (r2 + rho1);
    if (temp >= 0.0) {
      const dReal gamma21 = dSqrt(temp);
      const dReal lambda = r1 - m_fDPlus + gamma21;
      if (lambda < -thresholdRadius * r1) {
        return;
      }
    }
    if (rho1 <= 0.0) {
      const dReal kappa = dSqrt(rho2 * rho2 + r2d * r2d);
      const dReal depth = r1 - kappa;
      if (depth < 0.0) {
        exit = true;
        return;
      }
      dMessage(1,"Orthogonal axes : collision case I2\n");
      // Case I2
      // A boundary circle of the second cylinder hits the interior of a generator belonging to the first cylinder
      //----------------------------------------------------------------------------------------------------------
      // returns 1 contact point
      const dReal erho2 = epsilon2 * rho2;
      r2d = -r2d;
      const dVector3 u = { m_fInvSinus * m_vW[0], m_fInvSinus * m_vW[1], m_fInvSinus * m_vW[2] };
      const dVector3 v0 = { r2d * u[0], r2d * u[1], r2d * u[2] };
      const dVector3 v1 = { erho2 * R2[2], erho2 * R2[6], erho2 * R2[10] };
      const dVector3 v2 = { v0[0] + v1[0], v0[1] + v1[1], v0[2] + v1[2] };

      dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
      dAddVectors3(contact->pos, v2, A1);
      contact->depth = depth;
      dCopyScaledVector3(contact->normal, v2, -epsilon / kappa);
      exit = true;
      ++m_nNumberOfContacts;
    } else { // end of case I2
      //rho1 > 0.0
      const dReal gamma12d = gamma12 - m_fDPlus;
      const dReal nu = dSqrt(rho1 * rho1 + gamma12d * gamma12d);
      const dReal depth = r2 - nu;
      if (depth < 0.0)
        return;
      // Case I3
      //---------
      // The caps collide
      // returns 1 contact point
      dMessage(1,"Orthogonal axes : collision case I3, r1 = %f, r2 = %f \n",r1,r2);
      const dReal erho1 = epsilon1 * rho1;
      const dReal erho2 = epsilon2 * rho2;
      const dReal x = - erho1 * R1[2];
      const dReal y = - erho1 * R1[6];
      const dReal z = - erho1 * R1[10];
      const dVector3 u = { m_fInvSinus * m_vW[0], m_fInvSinus * m_vW[1], m_fInvSinus * m_vW[2] };
      dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
      contact->pos[0] = gamma12 * u[0] + x + erho2 * R2[2]  + A1[0];
      contact->pos[1] = gamma12 * u[1] + y + erho2 * R2[6]  + A1[1];
      contact->pos[2] = gamma12 * u[2] + z + erho2 * R2[10] + A1[2];
      contact->depth = depth;
      const dReal epsilonInvNu = epsilon / nu;
      contact->normal[0] = epsilonInvNu * (gamma12d * u[0] + x);
      contact->normal[1] = epsilonInvNu * (gamma12d * u[1] + y);
      contact->normal[2] = epsilonInvNu * (gamma12d * u[2] + z);
      exit = true;
      ++m_nNumberOfContacts;
    }// end else (rho1 < 0.0)
  }// end if - else r2d >= 0.0
}
