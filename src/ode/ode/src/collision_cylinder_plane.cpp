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
 * Cylinder-Plane collider by Luc Guyot (Cyberbotics Ltd.)
 * July 25th, 2013
 */

#include <ode/collision.h>
#include <ode/rotation.h>
#include <ode/objects.h>
#include "config.h"
#include "matrix.h"
#include "odemath.h"
#include "collision_kernel.h"	// for dxGeom
#include "collision_util.h"

// Data that passed through the collider's functions
struct sCylinderPlaneData
{
  sCylinderPlaneData(dxGeom *cylinder, dxGeom *plane, int flags, dContactGeom *contact, int skip):
    m_gCylinder(cylinder), m_gPlane(plane), m_iFlags(flags), m_gContact(contact), m_iSkip(skip), m_nNumberOfContacts(0)
  {
  }

  void _cldInitCylinderPlane();
  int capAgainstPlane(const dReal *capCenter);
  void setContact(const dReal *contactPosition, dReal depth);
  int performCollisionChecking();

  // cylinder parameters
  dVector3 m_vUpperCapCenter;
  dVector3 m_vLowerCapCenter;
  dVector3 m_vCrossProduct;
  dVector3 m_vMostInclinedRadius;
  bool m_bParallel;
  bool m_bSlightlyInclined;

  // plane parameters
  dVector4 m_vPlaneEquation;
  dReal *m_vPlaneNormal;

  // ODE stuff
  dGeomID m_gCylinder;
  dGeomID m_gPlane;
  int m_iFlags;
  dContactGeom *m_gContact;
  int m_iSkip;
  int m_nNumberOfContacts;
};

// initialize collision data
void sCylinderPlaneData::_cldInitCylinderPlane()
{
  // Get cylinder's dimensions and disk centers
  dReal halfHeight, radius;
  dGeomCylinderGetParams(m_gCylinder, &radius, &halfHeight);
  halfHeight *= REAL(0.5);
  const dReal *const R = m_gCylinder->final_posr->R;
  const dVector3 cylinderAxis = { R[2], R[6], R[10], 0.0 };
  const dVector3 halfHeightCylinderAxis = { halfHeight * R[2], halfHeight * R[6], halfHeight * R[10], 0.0 };

  const dReal *const cylinderCenter = m_gCylinder->final_posr->pos;
  dAddVectors3(m_vUpperCapCenter, cylinderCenter, halfHeightCylinderAxis);
  dSubtractVectors3(m_vLowerCapCenter, cylinderCenter, halfHeightCylinderAxis);

  // Get plane equation
  dGeomPlaneGetParams(m_gPlane, m_vPlaneEquation);
  m_vPlaneNormal = m_vPlaneEquation;

  // Compute mixed data
  dCalcVectorCross3(m_vCrossProduct, cylinderAxis, m_vPlaneNormal);
  const dReal sinusSquare = dCalcVectorLengthSquare3(m_vCrossProduct);

  // The plane normal and the cylinder axis are considered as 'parallel' if the angle between them does not exceed ~ pi/100 radians
  static const dReal PARALLEL_THRESHOLD = REAL(0.001);
  m_bParallel =  sinusSquare < PARALLEL_THRESHOLD;
  static const dReal SLIGHTLY_INCLINED = REAL(0.1);
  m_bSlightlyInclined = sinusSquare < SLIGHTLY_INCLINED;

  if (m_bParallel) {
    m_vCrossProduct[0] = radius * R[0];
    m_vCrossProduct[1] = radius * R[4];
    m_vCrossProduct[2] = radius * R[8];
    m_vMostInclinedRadius[0] = radius * R[1];
    m_vMostInclinedRadius[1] = radius * R[5];
    m_vMostInclinedRadius[2] = radius * R[9];
  } else {
    dScaleVector3(m_vCrossProduct, radius / dSqrt(sinusSquare));
    const dReal cosinus = dCalcVectorDot3(m_vPlaneNormal, cylinderAxis);
    dAddScaledVectors3(m_vMostInclinedRadius, cylinderAxis, m_vPlaneNormal, cosinus, - 1.0);
    dScaleVector3(m_vMostInclinedRadius, radius / dCalcVectorLength3(m_vMostInclinedRadius));
  }
}

int dCollideCylinderPlane(dxGeom *cylinder, dxGeom *plane, int flags, dContactGeom *contact, int skip)
{
  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (cylinder->type == dCylinderClass);
  dIASSERT (plane->type == dPlaneClass);
  dIASSERT ((flags & NUMC_MASK) >= 1);

  sCylinderPlaneData cData(cylinder, plane, flags, contact, skip);

  return cData.performCollisionChecking();
}

int sCylinderPlaneData::performCollisionChecking() {
  _cldInitCylinderPlane();
  if (capAgainstPlane(m_vUpperCapCenter) > 0)
    return m_nNumberOfContacts;

  capAgainstPlane(m_vLowerCapCenter);
  return m_nNumberOfContacts;
}

int sCylinderPlaneData::capAgainstPlane(const dReal *capCenter) {
  // Search for the deepest contact point
  dVector3 contactPosition;
  dAddVectors3(contactPosition, capCenter, m_vMostInclinedRadius);
  const double d = m_vPlaneEquation[3];
  dReal depth =  d - dCalcVectorDot3(m_vPlaneNormal, contactPosition);
  //dMessage(1,"depth 1 = %g\n", depth);
  if (depth > 0.0)
  {
    setContact(contactPosition, depth);
    if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
      return m_nNumberOfContacts; // enough contacts
  }

  if (m_bSlightlyInclined == false)
    return 0;

  dAddVectors3(contactPosition, capCenter, m_vCrossProduct);
  depth =  d - dCalcVectorDot3(m_vPlaneNormal, contactPosition);
  //dMessage(1,"depth 2 = %g\n", depth);
  if (depth > 0.0)
  {
    setContact(contactPosition, depth);
    if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
      return m_nNumberOfContacts; // enough contacts
  }

  dSubtractVectors3(contactPosition, capCenter, m_vMostInclinedRadius);
  depth =  d - dCalcVectorDot3(m_vPlaneNormal, contactPosition);
  //dMessage(1,"depth 3 = %g\n", depth);
  if (depth > 0.0)
  {
    setContact(contactPosition, depth);
    if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK) )
      return m_nNumberOfContacts; // enough contacts
  }

  dSubtractVectors3(contactPosition, capCenter, m_vCrossProduct);
  depth =  d - dCalcVectorDot3(m_vPlaneNormal, contactPosition);
  //dMessage(1,"depth 4 = %g\n", depth);
  if (depth > 0.0) {
    setContact(contactPosition, depth);
    if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK) )
      return m_nNumberOfContacts; // enough contacts
  }

  return 0;
}

void sCylinderPlaneData::setContact(const dReal *contactPosition, dReal depth) {
  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  dCopyVector3(contact->pos, contactPosition);
  dCopyVector3(contact->normal, m_vPlaneNormal);
  contact->depth = depth;
  contact->g1 = m_gCylinder;
  contact->g2 = m_gPlane;
  contact->side1 = -1;
  contact->side2 = -1;
  ++m_nNumberOfContacts;
}
