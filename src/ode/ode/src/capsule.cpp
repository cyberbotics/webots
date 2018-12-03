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

#include <ode/common.h>
#include <ode/collision.h>
#include <ode/rotation.h>
#include "ode_MT/ode_MT.h"
#include "config.h"
#include "cylinder_revolution_data.h"
#include "matrix.h"
#include "odemath.h"
#include "collision_kernel.h"
#include "collision_std.h"
#include "collision_util.h"

//****************************************************************************
// capped cylinder public API

dxCapsule::dxCapsule (dSpaceID space, dReal _radius, dReal _length) :
dxGeom (space,1)
{
    dAASSERT (_radius >= 0 && _length >= 0);
    type = dCapsuleClass;
    radius = _radius;
    lz = _length;
    updateZeroSizedFlag(!_radius/* || !_length -- zero length capsule is not a zero sized capsule*/);
}

void dxCapsule::computeAABB()
{
    const dMatrix3& R = final_posr->R;
    const dVector3& pos = final_posr->pos;

    dReal xrange = dFabs(R[2]  * lz) * REAL(0.5) + radius;
    dReal yrange = dFabs(R[6]  * lz) * REAL(0.5) + radius;
    dReal zrange = dFabs(R[10] * lz) * REAL(0.5) + radius;
    aabb[0] = pos[0] - xrange;
    aabb[1] = pos[0] + xrange;
    aabb[2] = pos[1] - yrange;
    aabb[3] = pos[1] + yrange;
    aabb[4] = pos[2] - zrange;
    aabb[5] = pos[2] + zrange;
}

dGeomID dCreateCapsule (dSpaceID space, dReal radius, dReal length)
{
    return new dxCapsule (space,radius,length);
}

void dGeomCapsuleSetParams_ST (dGeomID g, dReal radius, dReal length)
{
    dUASSERT (g && g->type == dCapsuleClass,"argument not a ccylinder");
    dAASSERT (radius >= 0 && length >= 0);
    dxCapsule *c = (dxCapsule*) g;
    c->radius = radius;
    c->lz = length;
    c->updateZeroSizedFlag(!radius/* || !length -- zero length capsule is not a zero sized capsule*/);
    dGeomMoved (g);
}

void dGeomCapsuleSetParams (dGeomID g, dReal radius, dReal length)
{
#ifdef ODE_MT
  dGeomCapsuleSetParams_MT(g, radius, length, &dGeomCapsuleSetParams_ST);
#else
  dGeomCapsuleSetParams_ST(g, radius, length);
#endif
}

void dGeomCapsuleGetParams (dGeomID g, dReal *radius, dReal *length)
{
    dUASSERT (g && g->type == dCapsuleClass,"argument not a ccylinder");
    dxCapsule *c = (dxCapsule*) g;
    *radius = c->radius;
    *length = c->lz;
}

dReal dGeomCapsulePointDepth (dGeomID g, dReal x, dReal y, dReal z)
{
    dUASSERT (g && g->type == dCapsuleClass,"argument not a ccylinder");
    g->recomputePosr();
    dxCapsule *c = (dxCapsule*) g;

    const dReal* R = g->final_posr->R;
    const dReal* pos = g->final_posr->pos;

    dVector3 a;
    a[0] = x - pos[0];
    a[1] = y - pos[1];
    a[2] = z - pos[2];
    dReal beta = dCalcVectorDot3_14(a,R+2);
    dReal lz2 = c->lz*REAL(0.5);
    if (beta < -lz2) beta = -lz2;
    else if (beta > lz2) beta = lz2;
    a[0] = c->final_posr->pos[0] + beta*R[0*4+2];
    a[1] = c->final_posr->pos[1] + beta*R[1*4+2];
    a[2] = c->final_posr->pos[2] + beta*R[2*4+2];
    return c->radius -
        dSqrt ((x-a[0])*(x-a[0]) + (y-a[1])*(y-a[1]) + (z-a[2])*(z-a[2]));
}

int dCollideCapsuleSphere (dxGeom *o1, dxGeom *o2, int flags,
                           dContactGeom *contact, int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dCapsuleClass);
    dIASSERT (o2->type == dSphereClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    dxCapsule *ccyl = (dxCapsule*) o1;
    dxSphere *sphere = (dxSphere*) o2;

    contact->g1 = o1;
    contact->g2 = o2;
    contact->side1 = -1;
    contact->side2 = -1;

    // find the point on the cylinder axis that is closest to the sphere
    dReal alpha =
        o1->final_posr->R[2]  * (o2->final_posr->pos[0] - o1->final_posr->pos[0]) +
        o1->final_posr->R[6]  * (o2->final_posr->pos[1] - o1->final_posr->pos[1]) +
        o1->final_posr->R[10] * (o2->final_posr->pos[2] - o1->final_posr->pos[2]);
    dReal lz2 = ccyl->lz * REAL(0.5);
    if (alpha > lz2) alpha = lz2;
    if (alpha < -lz2) alpha = -lz2;

    // collide the spheres
    dVector3 p;
    p[0] = o1->final_posr->pos[0] + alpha * o1->final_posr->R[2];
    p[1] = o1->final_posr->pos[1] + alpha * o1->final_posr->R[6];
    p[2] = o1->final_posr->pos[2] + alpha * o1->final_posr->R[10];
    return dCollideSpheres (p,ccyl->radius,o2->final_posr->pos,sphere->radius,contact);
}

int dCollideCapsuleBox (dxGeom *o1, dxGeom *o2, int flags,
                        dContactGeom *contact, int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dCapsuleClass);
    dIASSERT (o2->type == dBoxClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    dxCapsule *cyl = (dxCapsule*) o1;
    dxBox *box = (dxBox*) o2;

    contact->g1 = o1;
    contact->g2 = o2;
    contact->side1 = -1;
    contact->side2 = -1;

    // get p1,p2 = cylinder axis endpoints, get radius
    dVector3 p1,p2;
    dReal clen = cyl->lz * REAL(0.5);
    p1[0] = o1->final_posr->pos[0] + clen * o1->final_posr->R[2];
    p1[1] = o1->final_posr->pos[1] + clen * o1->final_posr->R[6];
    p1[2] = o1->final_posr->pos[2] + clen * o1->final_posr->R[10];
    p2[0] = o1->final_posr->pos[0] - clen * o1->final_posr->R[2];
    p2[1] = o1->final_posr->pos[1] - clen * o1->final_posr->R[6];
    p2[2] = o1->final_posr->pos[2] - clen * o1->final_posr->R[10];
    dReal radius = cyl->radius;

    // copy out box center, rotation matrix, and side array
    dReal *c = o2->final_posr->pos;
    dReal *R = o2->final_posr->R;
    const dReal *side = box->side;

    // get the closest point between the cylinder axis and the box
    dVector3 pl,pb;
    dClosestLineBoxPoints (p1,p2,c,R,side,pl,pb);

    // if the capsule is penetrated further than radius
    //  then pl and pb are equal (up to mindist) -> unknown normal
    // use normal vector of closest box surface
#ifdef dSINGLE
    dReal mindist = REAL(1e-6);
#else
    dReal mindist = REAL(1e-15);
#endif
    if (dCalcPointsDistance3(pl, pb)<mindist) {
        // consider capsule as box
        dVector3 normal;
        dReal depth;
        int code;
        // WARNING! rad2 is declared as #define in Microsoft headers (as well as psh2, chx2, grp2, frm2, rct2, ico2, stc2, lst2, cmb2, edt2, scr2). Avoid abbreviations!
        /* dReal rad2 = radius*REAL(2.0); */ dReal radiusMul2 = radius * REAL(2.0);
        const dVector3 capboxside = {radiusMul2, radiusMul2, cyl->lz + radiusMul2};
        int num = dBoxBox (c, R, side,
            o1->final_posr->pos, o1->final_posr->R, capboxside,
            normal, &depth, &code, flags, contact, skip);

        for (int i=0; i<num; i++) {
            dContactGeom *currContact = CONTACT(contact,i*skip);
            currContact->normal[0] = normal[0];
            currContact->normal[1] = normal[1];
            currContact->normal[2] = normal[2];
            currContact->g1 = o1;
            currContact->g2 = o2;
            currContact->side1 = -1;
            currContact->side2 = -1;
        }
        return num;
    } else {
        // generate contact point
        return dCollideSpheres (pl,radius,pb,0,contact);
    }
}

int dCollideCapsuleCapsule (dxGeom *o1, dxGeom *o2,
                            int flags, dContactGeom *contact, int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dCapsuleClass);
    dIASSERT (o2->type == dCapsuleClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    int i;
    const dReal tolerance = REAL(1e-5);

    dxCapsule *cyl1 = (dxCapsule*) o1;
    dxCapsule *cyl2 = (dxCapsule*) o2;

    contact->g1 = o1;
    contact->g2 = o2;
    contact->side1 = -1;
    contact->side2 = -1;

    // copy out some variables, for convenience
    dReal lz1 = cyl1->lz * REAL(0.5);
    dReal lz2 = cyl2->lz * REAL(0.5);
    dReal *pos1 = o1->final_posr->pos;
    dReal *pos2 = o2->final_posr->pos;
    dReal axis1[3],axis2[3];
    axis1[0] = o1->final_posr->R[2];
    axis1[1] = o1->final_posr->R[6];
    axis1[2] = o1->final_posr->R[10];
    axis2[0] = o2->final_posr->R[2];
    axis2[1] = o2->final_posr->R[6];
    axis2[2] = o2->final_posr->R[10];

    // if the cylinder axes are close to parallel, we'll try to detect up to
    // two contact points along the body of the cylinder. if we can't find any
    // points then we'll fall back to the closest-points algorithm. note that
    // we are not treating this special case for reasons of degeneracy, but
    // because we want two contact points in some situations. the closet-points
    // algorithm is robust in all casts, but it can return only one contact.

    dVector3 sphere1,sphere2;
    dReal a1a2 = dCalcVectorDot3 (axis1,axis2);
    dReal det = REAL(1.0)-a1a2*a1a2;
    if (det < tolerance) {
        // the cylinder axes (almost) parallel, so we will generate up to two
        // contacts. alpha1 and alpha2 (line position parameters) are related by:
        //       alpha2 =   alpha1 + (pos1-pos2)'*axis1   (if axis1==axis2)
        //    or alpha2 = -(alpha1 + (pos1-pos2)'*axis1)  (if axis1==-axis2)
        // first compute where the two cylinders overlap in alpha1 space:
        if (a1a2 < 0) {
            axis2[0] = -axis2[0];
            axis2[1] = -axis2[1];
            axis2[2] = -axis2[2];
        }
        dReal q[3];
        for (i=0; i<3; i++) q[i] = pos1[i]-pos2[i];
        dReal k = dCalcVectorDot3 (axis1,q);
        dReal a1lo = -lz1;
        dReal a1hi = lz1;
        dReal a2lo = -lz2 - k;
        dReal a2hi = lz2 - k;
        dReal lo = (a1lo > a2lo) ? a1lo : a2lo;
        dReal hi = (a1hi < a2hi) ? a1hi : a2hi;
        if (lo <= hi) {
            int num_contacts = flags & NUMC_MASK;
            if (num_contacts >= 2 && lo < hi) {
                // generate up to two contacts. if one of those contacts is
                // not made, fall back on the one-contact strategy.
                for (i=0; i<3; i++) sphere1[i] = pos1[i] + lo*axis1[i];
                for (i=0; i<3; i++) sphere2[i] = pos2[i] + (lo+k)*axis2[i];
                int n1 = dCollideSpheres (sphere1,cyl1->radius,
                    sphere2,cyl2->radius,contact);
                if (n1) {
                    for (i=0; i<3; i++) sphere1[i] = pos1[i] + hi*axis1[i];
                    for (i=0; i<3; i++) sphere2[i] = pos2[i] + (hi+k)*axis2[i];
                    dContactGeom *c2 = CONTACT(contact,skip);
                    int n2 = dCollideSpheres (sphere1,cyl1->radius,
                        sphere2,cyl2->radius, c2);
                    if (n2) {
                        c2->g1 = o1;
                        c2->g2 = o2;
                        c2->side1 = -1;
                        c2->side2 = -1;
                        return 2;
                    }
                }
            }

            // just one contact to generate, so put it in the middle of
            // the range
            dReal alpha1 = (lo + hi) * REAL(0.5);
            dReal alpha2 = alpha1 + k;
            for (i=0; i<3; i++) sphere1[i] = pos1[i] + alpha1*axis1[i];
            for (i=0; i<3; i++) sphere2[i] = pos2[i] + alpha2*axis2[i];
            return dCollideSpheres (sphere1,cyl1->radius,
                sphere2,cyl2->radius,contact);
        }
    }

    // use the closest point algorithm
    dVector3 a1,a2,b1,b2;
    a1[0] = o1->final_posr->pos[0] + axis1[0]*lz1;
    a1[1] = o1->final_posr->pos[1] + axis1[1]*lz1;
    a1[2] = o1->final_posr->pos[2] + axis1[2]*lz1;
    a2[0] = o1->final_posr->pos[0] - axis1[0]*lz1;
    a2[1] = o1->final_posr->pos[1] - axis1[1]*lz1;
    a2[2] = o1->final_posr->pos[2] - axis1[2]*lz1;
    b1[0] = o2->final_posr->pos[0] + axis2[0]*lz2;
    b1[1] = o2->final_posr->pos[1] + axis2[1]*lz2;
    b1[2] = o2->final_posr->pos[2] + axis2[2]*lz2;
    b2[0] = o2->final_posr->pos[0] - axis2[0]*lz2;
    b2[1] = o2->final_posr->pos[1] - axis2[1]*lz2;
    b2[2] = o2->final_posr->pos[2] - axis2[2]*lz2;

    dClosestLineSegmentPoints (a1,a2,b1,b2,sphere1,sphere2);
    return dCollideSpheres (sphere1,cyl1->radius,sphere2,cyl2->radius,contact);
}

int dCollideCapsulePlane (dxGeom *o1, dxGeom *o2, int flags,
                          dContactGeom *contact, int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dCapsuleClass);
    dIASSERT (o2->type == dPlaneClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    dxCapsule *ccyl = (dxCapsule*) o1;
    dxPlane *plane = (dxPlane*) o2;

    // collide the deepest capping sphere with the plane
    dReal sign = (dCalcVectorDot3_14 (plane->p,o1->final_posr->R+2) > 0) ? REAL(-1.0) : REAL(1.0);
    dVector3 p;
    p[0] = o1->final_posr->pos[0] + o1->final_posr->R[2]  * ccyl->lz * REAL(0.5) * sign;
    p[1] = o1->final_posr->pos[1] + o1->final_posr->R[6]  * ccyl->lz * REAL(0.5) * sign;
    p[2] = o1->final_posr->pos[2] + o1->final_posr->R[10] * ccyl->lz * REAL(0.5) * sign;

    dReal k = dCalcVectorDot3 (p,plane->p);
    dReal depth = plane->p[3] - k + ccyl->radius;
    if (depth < 0) return 0;
    contact->normal[0] = plane->p[0];
    contact->normal[1] = plane->p[1];
    contact->normal[2] = plane->p[2];
    contact->pos[0] = p[0] - plane->p[0] * ccyl->radius;
    contact->pos[1] = p[1] - plane->p[1] * ccyl->radius;
    contact->pos[2] = p[2] - plane->p[2] * ccyl->radius;
    contact->depth = depth;

    int ncontacts = 1;
    if ((flags & NUMC_MASK) >= 2) {
        // collide the other capping sphere with the plane
        p[0] = o1->final_posr->pos[0] - o1->final_posr->R[2]  * ccyl->lz * REAL(0.5) * sign;
        p[1] = o1->final_posr->pos[1] - o1->final_posr->R[6]  * ccyl->lz * REAL(0.5) * sign;
        p[2] = o1->final_posr->pos[2] - o1->final_posr->R[10] * ccyl->lz * REAL(0.5) * sign;

        k = dCalcVectorDot3 (p,plane->p);
        depth = plane->p[3] - k + ccyl->radius;
        if (depth >= 0) {
            dContactGeom *c2 = CONTACT(contact,skip);
            c2->normal[0] = plane->p[0];
            c2->normal[1] = plane->p[1];
            c2->normal[2] = plane->p[2];
            c2->pos[0] = p[0] - plane->p[0] * ccyl->radius;
            c2->pos[1] = p[1] - plane->p[1] * ccyl->radius;
            c2->pos[2] = p[2] - plane->p[2] * ccyl->radius;
            c2->depth = depth;
            ncontacts = 2;
        }
    }

    for (int i=0; i < ncontacts; i++) {
        dContactGeom *currContact = CONTACT(contact,i*skip);
        currContact->g1 = o1;
        currContact->g2 = o2;
        currContact->side1 = -1;
        currContact->side2 = -1;
    }
    return ncontacts;
}

/////////////////////////////////////////
//---------------------------------------
// Preparing performCollisionChecking
//---------------------------------------
/////////////////////////////////////////

//   Cap Sphere Against Generator
//   ----------------------------
// This function handles the case of cap sphere hitting a cylinder's generator
// returns 1 contact point

void sCylinderCapsuleData::capSphereAgainstGenerator(dReal epsilon2, const dVector3 h2n2, dReal squaredDistanceToCylinderAxis)
{
    const dVector3 O2 = { epsilon2 * h2n2[0] + m_vCenter2[0] , epsilon2 * h2n2[1] + m_vCenter2[1], epsilon2 * h2n2[2] + m_vCenter2[2] };
    dVector3 normal = { - O2[0] + m_vCenter1[0], - O2[1] + m_vCenter1[1], - O2[2] + m_vCenter1[2]};
    const dReal distanceToCylinderAxis = dSqrt(squaredDistanceToCylinderAxis);
    dNormalize3(normal);

    dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
    dAddScaledVectors3(contact->pos, O2, normal, 1.0, m_fRadius2);
    dCopyVector3(contact->normal, normal);
    contact->depth = m_fRadiusSum - distanceToCylinderAxis;
    ++m_nNumberOfContacts;
}

////////////////////////////////////////
//--------------------------------------
//  Preparing Parallel Axes Global Test
//--------------------------------------
////////////////////////////////////////

//           Parallel axes
//              Case I_1
//              --------
//     A cap sphere hits a cap disk
//----------------------------------------------------
//       returns 1 contact point

void sCylinderCapsuleData::computeContactPointAndNormalForSAD(dReal r, dReal dh, const dVector3 u){
  const dReal epsilon1 = m_fDelta1 > 0.0 ? REAL(1.0) : REAL(-1.0);
  const dReal epsilon2 = m_fDelta2 > 0.0 ? REAL(1.0) : REAL(-1.0);
  const dReal eh1 =   epsilon1 * m_fH1;
  const dReal eh2 = - epsilon2 * m_fH2;

  const dVector3 v1 = { eh2 * m_mRotation2[2] + m_vCenter2[0], eh2 * m_mRotation2[6] + m_vCenter2[1], eh2 * m_mRotation2[10] + m_vCenter2[2] };
  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);

  if (r > m_fRadius1) {
    const dVector3 v = { eh1 * m_mRotation1[2] + m_fRadius1 * u[0] + m_vCenter1[0], eh1 * m_mRotation1[6] + m_fRadius1 * u[1] + m_vCenter1[1], eh1 * m_mRotation1[10] + m_fRadius1 * u[2] + m_vCenter1[2] };
    dCopyVector3(contact->pos, v);
    dVector3 n = { v[0] - v1[0], v[1] - v1[1], v[2] - v1[2] };
    dReal lengthN = dCalcVectorLength3(n);
    contact->depth = m_fRadius2 - lengthN;
    dCopyScaledVector3(contact->normal, n, 1.0 / lengthN);
  } else {
    contact->pos[0] =  v1[0] - epsilon2 * m_fRadius2 * m_mRotation2[2];
    contact->pos[1] =  v1[1] - epsilon2 * m_fRadius2 * m_mRotation2[6];
    contact->pos[2] =  v1[2] - epsilon2 * m_fRadius2 * m_mRotation2[10];
    contact->depth = m_fRadius2 + dh;
    contact->normal[0] = - epsilon1 * m_mRotation1[2];
    contact->normal[1] = - epsilon1 * m_mRotation1[6];
    contact->normal[2] = - epsilon1 * m_mRotation1[10];
  }

  ++m_nNumberOfContacts;
}

//           Parallel axes : case I_2
//  Cylinders'bodies with a small horizontal overlap
//----------------------------------------------------
//  3 contact points are chosen in the overlap zone

void sCylinderCapsuleData::computeContactPointsAndNormalsParallelAxes(dReal dr, dVector3 u) {
  dReal temp = m_fH2 + m_fDelta1;
  const dReal alpha = m_fH1 > temp ? temp : m_fH1;
  temp = - m_fH2 + m_fDelta1;
  const dReal beta = - m_fH1 < temp ? temp : - m_fH1;
  const dVector3 v = { m_fRadius1 * u[0] + m_vCenter1[0], m_fRadius1 * u[1] + m_vCenter1[1], m_fRadius1 * u[2] + m_vCenter1[2] };
  dNegateVector3(u);

  //contact points
  dContactGeom *const contact0 = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  contact0->pos[0] = v[0] + alpha * m_mRotation1[2];
  contact0->pos[1] = v[1] + alpha * m_mRotation1[6];
  contact0->pos[2] = v[2] + alpha * m_mRotation1[10];
  contact0->depth = dr;
  dCopyVector3(contact0->normal, u);
  ++m_nNumberOfContacts;
  if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
    return;

  dContactGeom *const contact1 = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  contact1->pos[0] = v[0] + beta * m_mRotation1[2];
  contact1->pos[1] = v[1] + beta * m_mRotation1[6];
  contact1->pos[2] = v[2] + beta * m_mRotation1[10];
  dCopyVector3(contact1->normal, u);
  contact1->depth = dr;
  ++m_nNumberOfContacts;
  if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
    return;

  dContactGeom *const contact2 = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  dAddScaledVectors3(contact2->pos, contact0->pos, contact1->pos, 0.5, 0.5);
  contact2->depth = dr;
  dCopyVector3(contact2->normal, u);
  ++m_nNumberOfContacts;
}

//-------------------------------------------------------------------------------------
//                                  Parallel Axes : Main Program
//                                  ----------------------------
//--------------------------------------------------------------------------------------
// The function below handles the case of parallel axes for a Cylinder-Capsule collision
//--------------------------------------------------------------------------------------
// It returns 3 contact points if the body cylinders collide along a common generator (Genarator Against Generator),
// contact points share the SAME depth and the SAME normal

void sCylinderCapsuleData::parallelAxesIntersectionTest(dReal hSum1, dReal ch1, dReal ch2) {
  // We compute the unit vector u = u1 whose direction is orthogonal to the axes of both cylinders
  dVector3 u = { m_vDelta[0] - m_fDelta1 * m_mRotation1[2], m_vDelta[1] - m_fDelta1 * m_mRotation1[6], m_vDelta[2] - m_fDelta1 * m_mRotation1[10] };
  dReal r = dCalcVectorLengthSquare3(u);
  dSafeNormalize3(u);

#ifdef dSINGLE
  dReal mindist = REAL(1e-9);
#else
  dReal mindist = REAL(1e-18);
#endif
  if (r < mindist) {
    // return one contact
    dContactGeom *const c = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
    c->normal[0] = u[0];
    c->normal[1] = u[1];
    c->normal[2] = u[2];
    c->depth  = m_fRadius1;
    c->pos[0] = m_vCenter1[0];
    c->pos[1] = m_vCenter1[1];
    c->pos[2] = m_vCenter1[2];
    ++m_nNumberOfContacts;
    return;
  }

  dReal l = dRecipSqrt(r);

  //we compute the coordinates r and h of the center c2 of Cylinder 2 in the frame (c1; u1,n1), c1 being the center of Cylinder 1
  r *= l;
  //Relevant values for the Intersection Test
  const dReal dr = m_fRadiusSum - r;
  const dReal delta1Plus = dFabs(m_fDelta1);
  const dReal hSum = m_fH1 + m_fH2;
  const dReal dh = hSum - delta1Plus;

  // Exit Test
  //----------
  //dMessage(1,"Axes are parallel");
  if (dr < 0.0)
    return;

  // Case I1 : A cap sphere hits a cap disk
  // (Similar to Sphere-Cylinder collision)
  if (dh < 0.0) {
    computeContactPointAndNormalForSAD(r, dh, u);
    return;
  }

  // Case I2 : the cylinders'bodies collide with a small horizontal overlap
  // We pick 3 contact points in the overlap zone, all have the same radial normal -u
  if (dr <= dh)
    computeContactPointsAndNormalsParallelAxes(dr, u);
}

/////////////////////////////////////////
//--------------------------------------
//  Preparing Orthogonal Axes Global Test
//--------------------------------------
/////////////////////////////////////////

// The function below handles the collision of a Cap Sphere with a cylinder Generator when axes are orthogonal
//------------------------------------------------------------------------------------------------------------
// returns 1 contact point

bool sCylinderCapsuleData::capSphereAgainstGeneratorOrthogonalAxes(const dVector3 A1, dReal epsilon2) {
  const dReal eh2 = - epsilon2 * m_fH2;
  //center of the possibly colliding cap sphere
  const dVector3 C = { eh2 * m_mRotation2[2] + m_vCenter2[0], eh2 * m_mRotation2[6] + m_vCenter2[1], eh2 * m_mRotation2[10] + m_vCenter2[2] };
  //radial direction
  dVector3 n = { A1[0] - C[0], A1[1] - C[1], A1[2] - C[2] };
  const dReal squaredDistanceToCylinderAxis = dCalcVectorLengthSquare3(n);

  if (squaredDistanceToCylinderAxis > m_fRadiusSumSquare)
    return false;

  const dReal distanceToCylinderAxis = dSqrt(squaredDistanceToCylinderAxis);
  dScaleVector3(n, 1.0 / distanceToCylinderAxis);

  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  dAddScaledVectors3(contact->pos, A1, n, 1.0, -m_fRadius1);
  dCopyVector3(contact->normal, n);
  contact->depth = m_fRadiusSum - distanceToCylinderAxis;
  ++m_nNumberOfContacts;
  return true;
}

// The function below handles the collision of a Cap Sphere with a Cap Disk when axes are orthogonal
//---------------------------------------------------------------------------------------------------
// returns 1 contact point

bool sCylinderCapsuleData::capSphereAgainstCapDiskOrthogonalAxes(dReal epsilon1, dReal epsilon2) {
  const dReal eh1 =  epsilon1 * m_fH1;
  const dReal eh2 =  epsilon2 * m_fH2;
  //centers of the potential colliding cap disk and cap sphere
  const dVector3 C1 = { eh1 * m_mRotation1[2] + m_vCenter1[0], eh1 * m_mRotation1[6] + m_vCenter1[1], eh1 * m_mRotation1[10] + m_vCenter1[2] };
  const dVector3 C2 = { eh2 * m_mRotation2[2] + m_vCenter2[0], eh2 * m_mRotation2[6] + m_vCenter2[1], eh2 * m_mRotation2[10] + m_vCenter2[2] };
  //radial direction
  dVector3 C1C2 = { C2[0] - C1[0], C2[1] - C1[1], C2[2] - C1[2] };
  //projection of C1C2 on the cap's plane
  const dReal scalar = dCalcVectorDot3_14(C1C2, m_mRotation1 + 2);
  C1C2[0] -= scalar * m_mRotation1[2];
  C1C2[1] -= scalar * m_mRotation1[6];
  C1C2[2] -= scalar * m_mRotation1[10];
  dNormalize3(C1C2);
  //contact
  const dVector3 pos = { C1[0] + m_fRadius1 * C1C2[0], C1[1] + m_fRadius1 * C1C2[1], C1[2] + m_fRadius1 * C1C2[2] };

  dVector3 C2P = { pos[0] - C2[0], pos[1] - C2[1], pos[2] - C2[2] };
  const dReal C2PSquare = dCalcVectorLengthSquare3(C2P);
  if (C2PSquare > m_fRadius2 * m_fRadius2)
    return false;

  //normal
  dNormalize3(C2P);
  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  dCopyVector3(contact->pos, pos);
  dCopyVector3(contact->normal, C2P);
  contact->depth = m_fRadius2 - dSqrt(C2PSquare);
  ++m_nNumberOfContacts;
  return true;
}

// The function below handles the collision of a Cap Sphere with a Cap Disk in the general case
//----------------------------------------------------------------------------------------------
// returns 1 contact point

void sCylinderCapsuleData::capDiskAgainstCapSphere(const dVector3 h1n1, const dVector3 h2n2, dReal epsilon1, dReal epsilon2) {
  //centers of the potential colliding cap disk and cap sphere
  const dVector3 C1 = { epsilon1 * h1n1[0] + m_vCenter1[0], epsilon1 * h1n1[1] + m_vCenter1[1], epsilon1 * h1n1[2] + m_vCenter1[2] };
  const dVector3 C2 = { epsilon2 * h2n2[0] + m_vCenter2[0], epsilon2 * h2n2[1] + m_vCenter2[1], epsilon2 * h2n2[2] + m_vCenter2[2] };
  //radial direction
  dVector3 C1C2 = { C2[0] - C1[0], C2[1] - C1[1], C2[2] - C1[2] };
  //projection of C1C2 on the cap's plane
  const dReal scalar = dCalcVectorDot3_14(C1C2, m_mRotation1 + 2);
  C1C2[0] -= scalar * m_mRotation1[2];
  C1C2[1] -= scalar * m_mRotation1[6];
  C1C2[2] -= scalar * m_mRotation1[10];

  const dReal epsScalar = epsilon1 * scalar;
  if (epsScalar >= 0.0) {
    const dReal depth = m_fRadius2 - epsScalar;
    const dReal C1C2Square = dCalcVectorLengthSquare3(C1C2);
    if (depth > 0.0 && C1C2Square < m_fRadius1Square) {// the cap sphere hits the interior of the cap disk
      dMessage(1,"the cap sphere hits the interior of the cap disk, depth = %f",depth);
      dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
      const dVector3 n = { - epsilon1 * m_mRotation1[2], - epsilon1 * m_mRotation1[6], - epsilon1 * m_mRotation1[10] };
      dAddScaledVectors3(contact->pos, C2, n, 1.0, m_fRadius2);
      dCopyVector3(contact->normal, n);
      dMessage(1,"CAC interior, depth = %f", depth);
      contact->depth = depth;
      ++m_nNumberOfContacts;
    }
  } else {
    dNormalize3(C1C2);
    dVector3 pos;
    dAddScaledVectors3(pos, C1, C1C2, 1.0, m_fRadius1);
    dVector3 C2P = { pos[0] - C2[0], pos[1] - C2[1], pos[2] - C2[2] };
    const dReal C2PSquare = dCalcVectorLengthSquare3(C2P);
    dMessage(1, "C2PSquare %g  radius2 square %g\n", C2PSquare, m_fRadius2Square);
    if (C2PSquare > m_fRadius2Square)
      return;
    dMessage(1,"the cap sphere hits the boundary of the cap disk");
    dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
    dCopyVector3(contact->pos, pos);
    dNormalize3(C2P);
    dCopyVector3(contact->normal, C2P);
    contact->depth = m_fRadius2 - dSqrt(C2PSquare);
    ++m_nNumberOfContacts;
  }
}

//-------------------------------------------------------------------------------------
//                                 Orthogonal Axes : Main Program
//                                 ------------------------------
//--------------------------------------------------------------------------------------

// The function below handles the collision of a Cap Disk with a Generator when axes are orthogonal
//-------------------------------------------------------------------------------------------------

// returns 1 contact point if the boundary circle hits the interior of generator
// returns 3 contact points if the disk contains a part (.i.e. a segment) of a generator

bool sCylinderCapsuleData::capDiskAgainstGeneratorOrthogonalAxes(const dVector3 A2, dReal t1, dReal rho1, dReal rho2,
  dReal d12, dReal d21, dReal epsilon1, dReal epsilon2)
{
  bool exit = false;
  dReal r1d = m_fRadius1 - m_fD;

// Case I1: a cap disk of the cylinder contains a part of a generator of the capsule's body (CCG)
//------------------------------------------------------------------------------------------------
// Returns 3 contact points

  if (r1d >= 0.0) {
    dReal zeta1 = dSqrt(r1d * (m_fRadius1 + m_fD));
    dReal temp1 =  m_fH2 + m_fDelta2;
    dReal temp2 = -m_fH2 + m_fDelta2;

    if (-zeta1 > temp1 || zeta1 < temp2)
      return false;

    const dReal alpha = temp1 > zeta1 ? zeta1 : temp1;
    const dReal omega = (temp2 < - zeta1) ? - zeta1 : temp2;

    //dMessage(1,"Orthogonal axes, case I1");
    const dVector3 normal = { -epsilon1 * m_mRotation1[2], -epsilon1 * m_mRotation1[6], -epsilon1 * m_mRotation1[10] };
    const dReal er2 = - epsilon1 * m_fRadius2;
    const dVector3 v = { er2 * m_mRotation1[2]  + A2[0], er2 * m_mRotation1[6]  + A2[1], er2 * m_mRotation1[10] + A2[2] };
    const dVector3 v1 = { alpha * m_mRotation2[2], alpha * m_mRotation2[6], alpha * m_mRotation2[10]};
    const dReal calpha = m_fCosinus * alpha;
    const dReal temp = t1 + er2;
    dReal depth = m_fH1 - dFabs(temp + calpha);

    dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);

    if (depth > 0.0) {
      //dMessage(1,"point A, depth = %f",depth);
      dAddVectors3(contact->pos, v, v1);
      dCopyVector3(contact->normal, normal);
      contact->depth = depth;
      ++m_nNumberOfContacts;
      exit = true;
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
        return true;
    }

    const dVector3 v2 = { omega * m_mRotation2[2], omega * m_mRotation2[6], omega * m_mRotation2[10]};
    const dReal comega = m_fCosinus * omega;
    depth = m_fH1 - dFabs(temp + comega);

    if (depth > 0.0) {
      dAddVectors3(contact->pos, v, v2);
      dCopyVector3(contact->normal, normal);
      contact->depth = depth;
      ++m_nNumberOfContacts;
      exit = true;
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
        return true;
     }

    depth = m_fH1 - dFabs(temp + (calpha + comega) * REAL(0.5));
    if (depth > 0.0) {
      const dVector3 v3 = { REAL(0.5) * (v1[0] + v2[0]), REAL(0.5) * (v1[1] + v2[1]), REAL(0.5) * (v1[2] + v2[2]) };
      dAddVectors3(contact->pos, v, v3);
      dCopyVector3(contact->normal, normal);
      contact->depth = depth;
      ++m_nNumberOfContacts;
      exit = true;
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
        return true;
    }
     return false;
  } else {
    //Two Exit tests

    dReal gamma = dSqrt(dFabs(d21 * (m_fRadius2 + rho1)));
    dReal lambda = r1d + gamma;
    if (lambda < - thresholdRadius * m_fRadius2)
      return true;

    gamma = dSqrt(d12 * (m_fRadius1 + rho2));
    lambda = m_fRadius2 - m_fD + gamma;
    if (lambda < - thresholdRadius * m_fRadius1)
      return false;

    if (rho2 <= 0.0) {
      const dReal kappa = dSqrt(rho1 * rho1 + r1d * r1d);
      const dReal depth = m_fRadius2 - kappa;
      if (depth <= 0.0)
        return true;

      //dMessage(1,"Orthogonal axes : collision case I2\n");
      // Case I2
      // A boundary circle of the cylinder hits the interior of some capsule's generator
      //---------------------------------------------------------------------------------
      //dMessage(1,"Orthogonal axes, case I2");
      // returns 1 contact point
      const dVector3 u = { m_fInvSinus * m_vW[0], m_fInvSinus * m_vW[1], m_fInvSinus * m_vW[2] };
      const dVector3 v = { r1d * u[0], r1d * u[1], r1d * u[2] };
      const dReal erho1 = - epsilon1 * rho1;
      const dVector3 v1 = { erho1 * m_mRotation1[2], erho1 * m_mRotation1[6], erho1 * m_mRotation1[10] };
      const dVector3 v2 = { v[0] + v1[0], v[1] + v1[1], v[2] + v1[2] };
      const dReal invKappa = REAL(1.0) / kappa;
      const dVector3 v3 = { invKappa * v2[0], invKappa * v2[1], invKappa * v2[2] };
      dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
      dAddVectors3(contact->pos, v2, A2);
      contact->depth = depth;
      dCopyVector3(contact->normal, v3);
      exit = true;
      ++m_nNumberOfContacts;
    }// end of case I2
  }

  return exit;
   //dMessage(1,"Orthogonal axes : default case");
}

//********************************************************
//--------------------------------------------------------
//--------------------------------------------------------
// Main Cylinder-Capsule Collision Detection Program
//--------------------------------------------------------
//--------------------------------------------------------
//********************************************************

//---------- The code below has been adapted from dCollideCapsuleCapsule() by Yvan Bourquin and Luc Guyot - www.cyberbotics.com ---------
int sCylinderCapsuleData::performCollisionChecking()
{

  //---------------------------------------------------------------------
  //                      Three Fast Exit Tests
  //           Axis Separation using n1 cross n2, n1 and n2
  //---------------------------------------------------------------------

  // Are the infinite cylinders intersecting?
  // To answer this question, we need to compute the distance separating their axes
  //-------------------------------------------------------------------------------

  // Two first (fast exit) Axis Separating Tests before entering more complex tests
  //------------------------------------------------------------------------------
  dReal ch1, ch2, sr1, sr2;
  initProjectedLengths(ch1, ch2, sr1, sr2);
  const dReal hSum1 = m_fH1 + dFabs(ch2) + m_fRadius2;
  const dReal hSum2 = dFabs(ch1) + sr1 + m_fH2 + m_fRadius2;
  if (dFabs(m_fDelta1) > hSum1 || dFabs(m_fDelta2) > hSum2){
    dMessage(1,"Separating Axes Tests : Quick exit\n");
    return 0;
  }

  // Are cylinders'axes parallel?
  //---------------------------------
  // If yes, the intersection problem is fairly simple and we use some AABB-like algorithm
  //-------------------------------------------------------------------------------------------
  if (m_fDet < thresholdSquaredSinus) {
    dMessage(1,"Axes are parallel : det = %f", m_fDet);
    parallelAxesIntersectionTest(ch1, ch2, hSum1);
    return m_nNumberOfContacts;
  }

  // Are the infinite cylinders intersecting ?
  // We compute the distance separating their axes to answer this question

  dReal radiusMinusDistance, t1, t2;
  initWData(radiusMinusDistance, t1, t2);

  // Third Separating Axis Test
  //---------------------------
 if (radiusMinusDistance < 0.0) {
    dMessage(1,"The infinite cylinders are disjoints");
    return 0;
  }

  // From now on, the two infinite cylinders intersect

  initSquares();

  const dVector3 h1n1 = { m_fH1 * m_mRotation1[2], m_fH1 * m_mRotation1[6], m_fH1 * m_mRotation1[10] };
  const dVector3 h2n2 = { m_fH2 * m_mRotation2[2], m_fH2 * m_mRotation2[6], m_fH2 * m_mRotation2[10] };

  if (centersPenetration(h1n1, h2n2))
    return m_nNumberOfContacts;

  //                A Generator hits another one (Generator Against Generator)
  // Look for the middle point Omega of the segment with minimal length joining the cylinders' axes
  //---------------------------------------- (Fast Test) -------------------------------------------
  //------------------------------------------------------------------------------------------------

  generatorAgainstGeneratorIntersectionTest(radiusMinusDistance, t1, t2);
  if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
    return m_nNumberOfContacts;

// Are the axes orthogonal?
//-------------------------
// If yes, we must handle the special case of a cap disk containing (a part of) a generator of the capsule's body
//---------------------------------------------------------------------------------------------------------------

    /* Now, the bounded cylinders, or the caps, are PROBABLY intersecting in the following way:

     - Cap against Generator with a single contact point ('corner' point located on a boundary circle)
     - Cap against Generator along a common segment
     - Cap disk against Cap sphere
   */

    // The axes beeing othogonal, we proceed a specific test
    //------------------------------------------------------

    const dReal cPlus = dFabs(m_fCosinus);
    m_fRadiusSumSquare = m_fRadiusSum * m_fRadiusSum;

    if (cPlus < thresholdCosinus1) {
      //dMessage(1,"Axes are orthogonal");
      if (m_nNumberOfContacts > 0)
        return m_nNumberOfContacts;
      //--------------------
      // Axes are orthogonal
      //--------------------

      dReal d12, d21, rho1, rho2;
      initOrthogonalCase(d12, d21, rho1, rho2);
      const dReal epsilon1 = m_fDelta1 >= 0.0 ? REAL(1.0) : REAL(-1.0);
      const dReal epsilon2 = m_fDelta2 >= 0.0 ? REAL(1.0) : REAL(-1.0);

      if (rho1 <= 0.0) {
        //dMessage(1,"Orthogonal axes : rho1 is negative");
        const dVector3 A1 = { m_vCenter1[0] + t1 * m_mRotation1[2], m_vCenter1[1] + t1 * m_mRotation1[6], m_vCenter1[2] + t1 * m_mRotation1[10] };
        if (capSphereAgainstGeneratorOrthogonalAxes(A1, epsilon2) || m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
          return m_nNumberOfContacts;
      } else {
        //dMessage(5,"Orthogonal axes : rho1 is positive");
        d12 -= m_fRadius2;
        if (capSphereAgainstCapDiskOrthogonalAxes(epsilon1, -epsilon2) || m_nNumberOfContacts >= (m_iFlags & NUMC_MASK)){
          //dMessage(5,"Orthogonal axes : CS against CD : success");
          return m_nNumberOfContacts;
        } else {
          //dMessage(5,"Orthogonal axes : d12 >= d21");
          const dVector3 A2 = { m_vCenter2[0] - m_fDelta2 * m_mRotation2[2], m_vCenter2[1] - m_fDelta2 * m_mRotation2[6], m_vCenter2[2] - m_fDelta2 * m_mRotation2[10] };// Origin on the capsule's axis
          if (capDiskAgainstGeneratorOrthogonalAxes(A2, t1, rho1, rho2, d12, d21, epsilon1, epsilon2) || m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
            return m_nNumberOfContacts;
        }
      }
      return m_nNumberOfContacts;//No contact from orthogonal test
    }// End of Orthogonal Axes Case

  // From now on, the axes are neither parallel nor orthogonal
  //----------------------------------------------------------

// A cap sphere hits a generator (Sphere Against Generator)
//----------------------------------------------------------
//----------------------------------------------------------

 bool goThroughCAC = true;

//returns 1 contact point

  const dReal temp1 = m_fHeight2 * t2 * m_fDet;
  const dReal temp2 = m_fDet * m_fH2Square - m_fDelta1 * m_fDelta1 + m_fDeltaSquare;
  dReal squaredDistanceToCylinderAxis = temp2 -  temp1;

  if (squaredDistanceToCylinderAxis < m_fRadiusSumSquare){
    const dReal depth = m_fH1 - dFabs(m_fDelta1 + ch2);
    if (depth > 0.0) {
      dMessage(1,"SAG1");
      capSphereAgainstGenerator(REAL(1.0), h2n2, squaredDistanceToCylinderAxis);
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
        return m_nNumberOfContacts;
    }
  } else {
    squaredDistanceToCylinderAxis = temp2 + temp1;
    if (squaredDistanceToCylinderAxis < m_fRadiusSumSquare){
      const dReal depth = m_fH1 - dFabs(m_fDelta1 - ch2);
      if (depth > 0.0){
        dMessage(1,"SAG2");
        capSphereAgainstGenerator(REAL(-1.0), h2n2, squaredDistanceToCylinderAxis);
        if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
          return m_nNumberOfContacts;
      }
    } else
      goThroughCAC = false;
  }
//        Are caps colliding? (Cap Against Cap)
//-----------------------------------------------------------
//-----------------------------------------------------------

// We test the upper/lower cap of the cylinder against the upper/lower cap of the capsule
//----------------------------------------------------------------------------------------

  const dReal h1Delta1 = m_fHeight1 * m_fDelta1;

  if (goThroughCAC) {
    const dReal ch1h2 = ch2 * m_fHeight1;
    const dReal h2Delta2 = m_fHeight2 * m_fDelta2;
    dReal squaredDistanceBetweenCenters = m_fSigma - h1Delta1 + h2Delta2 - ch1h2;

    if (squaredDistanceBetweenCenters <= m_fRadiusSumSquare) {// caps may be too far away from each other
      dMessage(1,"CAC case UU");
      capDiskAgainstCapSphere(h1n1, h2n2, REAL(1.0), REAL(1.0));
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
          return m_nNumberOfContacts;
    }
    squaredDistanceBetweenCenters = m_fSigma - h1Delta1 - h2Delta2 + ch1h2;
    if (squaredDistanceBetweenCenters <= m_fRadiusSumSquare) {
      dMessage(1,"CAC case UL");
      capDiskAgainstCapSphere(h1n1, h2n2, REAL(1.0), REAL(-1.0));
      if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
          return m_nNumberOfContacts;
    }
    if (m_nNumberOfContacts < 2) {
      squaredDistanceBetweenCenters = m_fSigma + h1Delta1 + h2Delta2 + ch1h2;
      if (squaredDistanceBetweenCenters <= m_fRadiusSumSquare) {
        dMessage(1,"CAC case LU");
        capDiskAgainstCapSphere(h1n1, h2n2, REAL(-1.0), REAL(1.0));
        if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
          return m_nNumberOfContacts;
      }
      if (m_nNumberOfContacts < 2) {
        squaredDistanceBetweenCenters = m_fSigma + h1Delta1 - h2Delta2 - ch1h2;
        if (squaredDistanceBetweenCenters <= m_fRadiusSumSquare) {
          dMessage(1,"CAC case LL");
          capDiskAgainstCapSphere(h1n1, h2n2, REAL(-1.0), REAL(-1.0));
          if (m_nNumberOfContacts >= (m_iFlags & NUMC_MASK))
            return m_nNumberOfContacts;
        }
      } else
        return m_nNumberOfContacts;
    } else
      return m_nNumberOfContacts;
  }

  //          Final case : a Cap Disk collides with a Generator of the capsule's body (CAG)
  //---------------------------------------------------------------------------------------
  //--------------------- This is the slowest test ----------------------------------------
  //    the iterative Brent's method is used to compute
  //    the squared distance from a boundary circle to a cylinder's axis

    capAgainstGeneratorOneSidedTest(h1n1, h2n2, m_vCenter1, m_vCenter2, m_fRadius1, m_fRadius2, m_fH1, m_fH2,
      m_mRotation1, m_mRotation2 , m_fDelta1, m_fDelta2, t1, t2, ch1, REAL(1.0), NULL);

    if (m_nNumberOfContacts > 0.0)
      return m_nNumberOfContacts;

    capCentersPenetration(h1n1, h2n2, ch1, ch2);

    return m_nNumberOfContacts;
}

int dCollideCylinderCapsule(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip) {
  dIASSERT (skip >= (int)sizeof(dContactGeom));
  dIASSERT (o1->type == dCylinderClass);
  dIASSERT (o2->type == dCapsuleClass);
  dIASSERT ((flags & NUMC_MASK) >= 1);

  sCylinderCapsuleData cData(static_cast<dxCylinder *>(o1), static_cast<dxCapsule *>(o2), flags, contact, skip);
  const int num = cData.performCollisionChecking();

  for (int i = 0; i < num; ++i) {
    dContactGeom *const cg = CONTACT(contact, i * skip);
    cg->g1 = const_cast<dxGeom *>(o1);
    cg->g2 = const_cast<dxGeom *>(o2);
    cg->side1 = -1;
    cg->side2 = -1;
  }

  return num;
}

bool sCylinderCapsuleData::centersPenetration(const dReal *h1n1, const dReal *h2n2) {
  if (sCylinderRevolutionData::centersPenetration(h1n1, h2n2))
    return true;

  dContactGeom *const contact = SAFECONTACT(m_iFlags, m_gContact, m_nNumberOfContacts, m_iSkip);
  dReal *const n = contact->normal;

  // The center of the cylinder lies in one of the capsule caps
  dReal temp = m_fRadius2Square - m_fDeltaSquare - m_fH2Square;
  dReal depth = temp - 2.0 * m_fH2 * m_fDelta2;

  if (depth > 0.0) {
    dCopyVector3(m_gContact->pos, m_vCenter1);
    dAddVectors3(n, m_vCenter2, h2n2);
    dSubtractVectors3(n, m_vCenter1, n);
    dNormalize3(n);
    contact->depth = dSqrt(depth);
    ++m_nNumberOfContacts;
     return true;
  }

  depth = temp + 2.0 * m_fH2 * m_fDelta2;
  if (depth > 0.0) {
    dCopyVector3(m_gContact->pos, m_vCenter1);
    dSubtractVectors3(n, m_vCenter2, h2n2);
    dSubtractVectors3(n, m_vCenter1, n);
    dNormalize3(n);
    contact->depth = dSqrt(depth);
    ++m_nNumberOfContacts;
     return true;
  }

  return false;
}
