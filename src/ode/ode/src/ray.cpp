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
#include "config.h"
#include "matrix.h"
#include "odemath.h"
#include "collision_kernel.h"
#include "collision_std.h"
#include "collision_util.h"

#ifdef _MSC_VER
#pragma warning(disable:4291)  // for VC++, no complaints about "no matching operator delete found"
#endif

//****************************************************************************
// ray public API

dxRay::dxRay (dSpaceID space, dReal _length) : dxGeom (space,1)
{
    type = dRayClass;
    length = _length;
}

void dxRay::computeAABB()
{
    dVector3 e;
    e[0] = final_posr->pos[0] + final_posr->R[0*4+2]*length;
    e[1] = final_posr->pos[1] + final_posr->R[1*4+2]*length;
    e[2] = final_posr->pos[2] + final_posr->R[2*4+2]*length;

    if (final_posr->pos[0] < e[0]){
        aabb[0] = final_posr->pos[0];
        aabb[1] = e[0];
    }
    else{
        aabb[0] = e[0];
        aabb[1] = final_posr->pos[0];
    }

    if (final_posr->pos[1] < e[1]){
        aabb[2] = final_posr->pos[1];
        aabb[3] = e[1];
    }
    else{
        aabb[2] = e[1];
        aabb[3] = final_posr->pos[1];
    }

    if (final_posr->pos[2] < e[2]){
        aabb[4] = final_posr->pos[2];
        aabb[5] = e[2];
    }
    else{
        aabb[4] = e[2];
        aabb[5] = final_posr->pos[2];
    }
}

dGeomID dCreateRay (dSpaceID space, dReal length)
{
    return new dxRay (space,length);
}

void dGeomRaySetLength (dGeomID g, dReal length)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    dxRay *r = (dxRay*) g;
    r->length = length;
    dGeomMoved (g);
}

dReal dGeomRayGetLength (dGeomID g)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    dxRay *r = (dxRay*) g;
    return r->length;
}

void dGeomRaySet (dGeomID g, dReal px, dReal py, dReal pz,
                  dReal dx, dReal dy, dReal dz)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    g->recomputePosr();
    dReal* rot = g->final_posr->R;
    dReal* pos = g->final_posr->pos;
    dVector3 n;
    pos[0] = px;
    pos[1] = py;
    pos[2] = pz;

    n[0] = dx;
    n[1] = dy;
    n[2] = dz;
    dNormalize3(n);
    rot[0*4+2] = n[0];
    rot[1*4+2] = n[1];
    rot[2*4+2] = n[2];
    dGeomMoved (g);
}

void dGeomRayGet (dGeomID g, dVector3 start, dVector3 dir)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    g->recomputePosr();
    start[0] = g->final_posr->pos[0];
    start[1] = g->final_posr->pos[1];
    start[2] = g->final_posr->pos[2];
    dir[0] = g->final_posr->R[0*4+2];
    dir[1] = g->final_posr->R[1*4+2];
    dir[2] = g->final_posr->R[2*4+2];
}

void dGeomRaySetParams (dxGeom *g, int FirstContact, int BackfaceCull)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");

    dGeomRaySetFirstContact(g, FirstContact);
    dGeomRaySetBackfaceCull(g, BackfaceCull);
}

void dGeomRayGetParams (dxGeom *g, int *FirstContact, int *BackfaceCull)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");

    (*FirstContact) = ((g->gflags & RAY_FIRSTCONTACT) != 0);
    (*BackfaceCull) = ((g->gflags & RAY_BACKFACECULL) != 0);
}

// set/get backface culling flag
void dGeomRaySetBackfaceCull (dxGeom *g, int backfaceCull)
{

    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    if (backfaceCull) {
        g->gflags |= RAY_BACKFACECULL;
    } else {
        g->gflags &= ~RAY_BACKFACECULL;
    }
}

int dGeomRayGetBackfaceCull (dxGeom *g)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    return ((g->gflags & RAY_BACKFACECULL) != 0);
}

// set/get first contact flag
void dGeomRaySetFirstContact (dxGeom *g, int firstContact)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    if (firstContact) {
        g->gflags |= RAY_FIRSTCONTACT;
    } else {
        g->gflags &= ~RAY_FIRSTCONTACT;
    }
}

int dGeomRayGetFirstContact (dxGeom *g)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    return ((g->gflags & RAY_FIRSTCONTACT) != 0);
}

void dGeomRaySetClosestHit (dxGeom *g, int closestHit)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    if (closestHit){
        g->gflags |= RAY_CLOSEST_HIT;
    }
    else g->gflags &= ~RAY_CLOSEST_HIT;
}

int dGeomRayGetClosestHit (dxGeom *g)
{
    dUASSERT (g && g->type == dRayClass,"argument not a ray");
    return ((g->gflags & RAY_CLOSEST_HIT) != 0);
}

// if mode==1 then use the sphere exit contact, not the entry contact

static int ray_sphere_helper (dxRay *ray, dVector3 sphere_pos, dReal radius,
                              dContactGeom *contact, int mode)
{
    dVector3 q;
    q[0] = ray->final_posr->pos[0] - sphere_pos[0];
    q[1] = ray->final_posr->pos[1] - sphere_pos[1];
    q[2] = ray->final_posr->pos[2] - sphere_pos[2];
    dReal B = dCalcVectorDot3_14(q,ray->final_posr->R+2);
    dReal C = dCalcVectorDot3(q,q) - radius*radius;
    // note: if C <= 0 then the start of the ray is inside the sphere
    dReal k = B*B - C;
    if (k < 0) return 0;
    k = dSqrt(k);
    dReal alpha;
    if (mode && C >= 0) {
        alpha = -B + k;
        if (alpha < 0) return 0;
    }
    else {
        alpha = -B - k;
        if (alpha < 0) {
            alpha = -B + k;
            if (alpha < 0) return 0;
        }
    }
    if (alpha > ray->length) return 0;
    contact->pos[0] = ray->final_posr->pos[0] + alpha*ray->final_posr->R[0*4+2];
    contact->pos[1] = ray->final_posr->pos[1] + alpha*ray->final_posr->R[1*4+2];
    contact->pos[2] = ray->final_posr->pos[2] + alpha*ray->final_posr->R[2*4+2];
    dReal nsign = (C < 0 || mode) ? REAL(-1.0) : REAL(1.0);
    contact->normal[0] = nsign*(contact->pos[0] - sphere_pos[0]);
    contact->normal[1] = nsign*(contact->pos[1] - sphere_pos[1]);
    contact->normal[2] = nsign*(contact->pos[2] - sphere_pos[2]);
    dNormalize3 (contact->normal);
    contact->depth = alpha;
    return 1;
}

int dCollideRaySphere (dxGeom *o1, dxGeom *o2, int flags,
                       dContactGeom *contact, int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dRayClass);
    dIASSERT (o2->type == dSphereClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    dxRay *ray = (dxRay*) o1;
    dxSphere *sphere = (dxSphere*) o2;
    contact->g1 = ray;
    contact->g2 = sphere;
    contact->side1 = -1;
    contact->side2 = -1;
    return ray_sphere_helper (ray,sphere->final_posr->pos,sphere->radius,contact,0);
}

int dCollideRayBox (dxGeom *o1, dxGeom *o2, int flags,
                    dContactGeom *contact, int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dRayClass);
    dIASSERT (o2->type == dBoxClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    dxRay *ray = (dxRay*) o1;
    dxBox *box = (dxBox*) o2;

    contact->g1 = ray;
    contact->g2 = box;
    contact->side1 = -1;
    contact->side2 = -1;

    int i;

    // compute the start and delta of the ray relative to the box.
    // we will do all subsequent computations in this box-relative coordinate
    // system. we have to do a translation and rotation for each point.
    dVector3 tmp,s,v;
    tmp[0] = ray->final_posr->pos[0] - box->final_posr->pos[0];
    tmp[1] = ray->final_posr->pos[1] - box->final_posr->pos[1];
    tmp[2] = ray->final_posr->pos[2] - box->final_posr->pos[2];
    dMultiply1_331 (s,box->final_posr->R,tmp);
    tmp[0] = ray->final_posr->R[0*4+2];
    tmp[1] = ray->final_posr->R[1*4+2];
    tmp[2] = ray->final_posr->R[2*4+2];
    dMultiply1_331 (v,box->final_posr->R,tmp);

    // mirror the line so that v has all components >= 0
    dVector3 sign;
    for (i=0; i<3; i++) {
        if (v[i] < 0) {
            s[i] = -s[i];
            v[i] = -v[i];
            sign[i] = 1;
        }
        else sign[i] = -1;
    }

    // compute the half-sides of the box
    dReal h[3];
    h[0] = REAL(0.5) * box->side[0];
    h[1] = REAL(0.5) * box->side[1];
    h[2] = REAL(0.5) * box->side[2];

    // do a few early exit tests
    if ((s[0] < -h[0] && v[0] <= 0) || s[0] >  h[0] ||
        (s[1] < -h[1] && v[1] <= 0) || s[1] >  h[1] ||
        (s[2] < -h[2] && v[2] <= 0) || s[2] >  h[2] ||
        (v[0] == 0 && v[1] == 0 && v[2] == 0)) {
            return 0;
    }

    // compute the t=[lo..hi] range for where s+v*t intersects the box
    dReal lo = -dInfinity;
    dReal hi = dInfinity;
    int nlo = 0, nhi = 0;
    for (i=0; i<3; i++) {
        if (v[i] != 0) {
            dReal k = (-h[i] - s[i])/v[i];
            if (k > lo) {
                lo = k;
                nlo = i;
            }
            k = (h[i] - s[i])/v[i];
            if (k < hi) {
                hi = k;
                nhi = i;
            }
        }
    }

    // check if the ray intersects
    if (lo > hi) return 0;
    dReal alpha;
    int n;
    if (lo >= 0) {
        alpha = lo;
        n = nlo;
    }
    else {
        alpha = hi;
        n = nhi;
    }
    if (alpha < 0 || alpha > ray->length) return 0;
    contact->pos[0] = ray->final_posr->pos[0] + alpha*ray->final_posr->R[0*4+2];
    contact->pos[1] = ray->final_posr->pos[1] + alpha*ray->final_posr->R[1*4+2];
    contact->pos[2] = ray->final_posr->pos[2] + alpha*ray->final_posr->R[2*4+2];
    contact->normal[0] = box->final_posr->R[0*4+n] * sign[n];
    contact->normal[1] = box->final_posr->R[1*4+n] * sign[n];
    contact->normal[2] = box->final_posr->R[2*4+n] * sign[n];
    contact->depth = alpha;
    return 1;
}

int dCollideRayCapsule (dxGeom *o1, dxGeom *o2,
                        int flags, dContactGeom *contact, int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dRayClass);
    dIASSERT (o2->type == dCapsuleClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    dxRay *ray = (dxRay*) o1;
    dxCapsule *ccyl = (dxCapsule*) o2;

    contact->g1 = ray;
    contact->g2 = ccyl;
    contact->side1 = -1;
    contact->side2 = -1;

    dReal lz2 = ccyl->lz * REAL(0.5);

    // compute some useful info
    dVector3 cs,q,r;
    dReal C,k;
    cs[0] = ray->final_posr->pos[0] - ccyl->final_posr->pos[0];
    cs[1] = ray->final_posr->pos[1] - ccyl->final_posr->pos[1];
    cs[2] = ray->final_posr->pos[2] - ccyl->final_posr->pos[2];
    k = dCalcVectorDot3_41(ccyl->final_posr->R+2,cs);	// position of ray start along ccyl axis
    q[0] = k*ccyl->final_posr->R[0*4+2] - cs[0];
    q[1] = k*ccyl->final_posr->R[1*4+2] - cs[1];
    q[2] = k*ccyl->final_posr->R[2*4+2] - cs[2];
    C = dCalcVectorDot3(q,q) - ccyl->radius*ccyl->radius;
    // if C < 0 then ray start position within infinite extension of cylinder

    // see if ray start position is inside the capped cylinder
    int inside_ccyl = 0;
    if (C < 0) {
        if (k < -lz2) k = -lz2;
        else if (k > lz2) k = lz2;
        r[0] = ccyl->final_posr->pos[0] + k*ccyl->final_posr->R[0*4+2];
        r[1] = ccyl->final_posr->pos[1] + k*ccyl->final_posr->R[1*4+2];
        r[2] = ccyl->final_posr->pos[2] + k*ccyl->final_posr->R[2*4+2];
        if ((ray->final_posr->pos[0]-r[0])*(ray->final_posr->pos[0]-r[0]) +
            (ray->final_posr->pos[1]-r[1])*(ray->final_posr->pos[1]-r[1]) +
            (ray->final_posr->pos[2]-r[2])*(ray->final_posr->pos[2]-r[2]) < ccyl->radius*ccyl->radius) {
                inside_ccyl = 1;
        }
    }

    // compute ray collision with infinite cylinder, except for the case where
    // the ray is outside the capped cylinder but within the infinite cylinder
    // (it that case the ray can only hit endcaps)
    if (!inside_ccyl && C < 0) {
        // set k to cap position to check
        if (k < 0) k = -lz2; else k = lz2;
    }
    else {
        dReal uv = dCalcVectorDot3_44(ccyl->final_posr->R+2,ray->final_posr->R+2);
        r[0] = uv*ccyl->final_posr->R[0*4+2] - ray->final_posr->R[0*4+2];
        r[1] = uv*ccyl->final_posr->R[1*4+2] - ray->final_posr->R[1*4+2];
        r[2] = uv*ccyl->final_posr->R[2*4+2] - ray->final_posr->R[2*4+2];
        dReal A = dCalcVectorDot3(r,r);
        dReal B = 2*dCalcVectorDot3(q,r);
        k = B*B-4*A*C;
        if (k < 0) {
            // the ray does not intersect the infinite cylinder, but if the ray is
            // inside and parallel to the cylinder axis it may intersect the end
            // caps. set k to cap position to check.
            if (!inside_ccyl) return 0;
            if (uv < 0) k = -lz2; else k = lz2;
        }
        else {
            k = dSqrt(k);
            A = dRecip (2*A);
            dReal alpha = (-B-k)*A;
            if (alpha < 0) {
                alpha = (-B+k)*A;
                if (alpha < 0) return 0;
            }
            if (alpha > ray->length) return 0;

            // the ray intersects the infinite cylinder. check to see if the
            // intersection point is between the caps
            contact->pos[0] = ray->final_posr->pos[0] + alpha*ray->final_posr->R[0*4+2];
            contact->pos[1] = ray->final_posr->pos[1] + alpha*ray->final_posr->R[1*4+2];
            contact->pos[2] = ray->final_posr->pos[2] + alpha*ray->final_posr->R[2*4+2];
            q[0] = contact->pos[0] - ccyl->final_posr->pos[0];
            q[1] = contact->pos[1] - ccyl->final_posr->pos[1];
            q[2] = contact->pos[2] - ccyl->final_posr->pos[2];
            k = dCalcVectorDot3_14(q,ccyl->final_posr->R+2);
            dReal nsign = inside_ccyl ? REAL(-1.0) : REAL(1.0);
            if (k >= -lz2 && k <= lz2) {
                contact->normal[0] = nsign * (contact->pos[0] -
                    (ccyl->final_posr->pos[0] + k*ccyl->final_posr->R[0*4+2]));
                contact->normal[1] = nsign * (contact->pos[1] -
                    (ccyl->final_posr->pos[1] + k*ccyl->final_posr->R[1*4+2]));
                contact->normal[2] = nsign * (contact->pos[2] -
                    (ccyl->final_posr->pos[2] + k*ccyl->final_posr->R[2*4+2]));
                dNormalize3 (contact->normal);
                contact->depth = alpha;
                return 1;
            }

            // the infinite cylinder intersection point is not between the caps.
            // set k to cap position to check.
            if (k < 0) k = -lz2; else k = lz2;
        }
    }

    // check for ray intersection with the caps. k must indicate the cap
    // position to check
    q[0] = ccyl->final_posr->pos[0] + k*ccyl->final_posr->R[0*4+2];
    q[1] = ccyl->final_posr->pos[1] + k*ccyl->final_posr->R[1*4+2];
    q[2] = ccyl->final_posr->pos[2] + k*ccyl->final_posr->R[2*4+2];
    return ray_sphere_helper (ray,q,ccyl->radius,contact, inside_ccyl);
}

int dCollideRayPlane (dxGeom *o1, dxGeom *o2, int flags,
                      dContactGeom *contact, int skip)
{
    dIASSERT (skip >= (int)sizeof(dContactGeom));
    dIASSERT (o1->type == dRayClass);
    dIASSERT (o2->type == dPlaneClass);
    dIASSERT ((flags & NUMC_MASK) >= 1);

    dxRay *ray = (dxRay*) o1;
    dxPlane *plane = (dxPlane*) o2;

    dReal alpha = plane->p[3] - dCalcVectorDot3 (plane->p,ray->final_posr->pos);
    // note: if alpha > 0 the starting point is below the plane
    dReal nsign = (alpha > 0) ? REAL(-1.0) : REAL(1.0);
    dReal k = dCalcVectorDot3_14(plane->p,ray->final_posr->R+2);
    if (k==0) return 0;		// ray parallel to plane
    alpha /= k;
    if (alpha < 0 || alpha > ray->length) return 0;
    contact->pos[0] = ray->final_posr->pos[0] + alpha*ray->final_posr->R[0*4+2];
    contact->pos[1] = ray->final_posr->pos[1] + alpha*ray->final_posr->R[1*4+2];
    contact->pos[2] = ray->final_posr->pos[2] + alpha*ray->final_posr->R[2*4+2];
    contact->normal[0] = nsign*plane->p[0];
    contact->normal[1] = nsign*plane->p[1];
    contact->normal[2] = nsign*plane->p[2];
    contact->depth = alpha;
    contact->g1 = ray;
    contact->g2 = plane;
    contact->side1 = -1;
    contact->side2 = -1;
    return 1;
}

// Ray - Cylinder collider by David Walters (June 2006)
// Revised by Luc Guyot (December 2012)
int dCollideRayCylinder( dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip )
{
	dIASSERT( skip >= (int)sizeof( dContactGeom ) );
	dIASSERT( o1->type == dRayClass );
	dIASSERT( o2->type == dCylinderClass );
	dIASSERT( (flags & NUMC_MASK) >= 1 );

	dxRay* ray = (dxRay*)( o1 );
	dxCylinder* cyl = (dxCylinder*)( o2 );

	// Fill in contact information.
	contact->g1 = ray;
	contact->g2 = cyl;
	contact->side1 = -1;
	contact->side2 = -1;

  const dReal height = cyl->lz;
  const dReal h = height * REAL( 0.5 );
  const dReal r = cyl->radius;
  const dReal *cylinderFrame =  cyl->final_posr->R;
  const dReal *rayFrame =  ray->final_posr->R;
  const dReal length = ray->length;

  // Let u := cyl->final_posr->R+2, i.e. the cylinder z-unit vector
  const dVector3 u = { cylinderFrame[2], cylinderFrame[6], cylinderFrame[10] };

  // Let v := cyl->final_posr->R+2, i.e. the normalized direction vector of the ray
  const dVector3 v = { rayFrame[2], rayFrame[6], rayFrame[10] };

  // Let C be the cylinder center
  const dReal *C = cyl->final_posr->pos;

  // Let O be the ray origin
  const dReal *O  = ray->final_posr->pos;

  const dVector3 CO = { O[0] - C[0], O[1] - C[1], O[2] - C[2] };

  // Cosinus of the angle (u, v)
  const dReal c = dCalcVectorDot3(u, v);
  const bool cosinusIsNegative = c < 0.0;
  const bool cosinusIsNotZero = c != 0.0;

  // The ray has at most two contact points with the cylinder, only the first one is returned
  int totalNumberOfContacts = 0;

  ////////////////////////////
  // Intersection with caps //
  ////////////////////////////

  // M1 = O + t1 * v and M2 = O + t2 * v are the two possible intersection points of the ray with the cylinder's caps
  dReal t1 = -1.0, t2 = -1.0, t = -1.0;
  const dReal rSquare = r * r;

  // Upper cap
  // Let C1 be the center of top's cylinder disk
  bool hitTopCap = false;
  const dVector3 OC1 = { -CO[0] + h * u[0], -CO[1] + h * u[1], -CO[2] + h * u[2] };
  dReal scalarProduct = dCalcVectorDot3(OC1, u);
  if (cosinusIsNotZero && cosinusIsNegative == (scalarProduct < 0.0)) { // t > 0
    t = scalarProduct / c;
    const dReal difference = dCalcVectorLengthSquare3(OC1) + t * (t - 2.0 * dCalcVectorDot3(OC1, v)) - rSquare; // C1M1 * C1M1 - r * r
    hitTopCap = difference < 0.0 && t < length;
    if (hitTopCap) {  // the contact point lies inside the disk
      t1 = t;
      totalNumberOfContacts++;
    }
  }

  // Lower cap
  // Let C2 be the center of bottom's cylinder disk
  const dVector3 OC2 = { -CO[0] - h * u[0], -CO[1] - h * u[1], -CO[2] - h * u[2]};
  scalarProduct -= height;
  bool hitBottomCap = false;
  if (cosinusIsNotZero && cosinusIsNegative == (scalarProduct < 0.0)) { // t > 0
    t = scalarProduct / c;
    const dReal difference = dCalcVectorLengthSquare3(OC2) + t * (t - 2.0 * dCalcVectorDot3(OC2, v)) - rSquare; // C2M2 * C2M2 - r * r
    hitBottomCap = difference < 0.0 && t < length;
    if (hitBottomCap) { // the contact point lies inside the disk
      t2 = t;
      totalNumberOfContacts++;
    }
  }

  if (hitTopCap && hitBottomCap) { // Already two contacts, we return the first one starting from ray's origin
    dReal sign = 1.0;
    if (t1 < t2)
      t = t1;
    else {
      t = t2;
      sign = -1.0;
    }
    // position
    dAddScaledVectors3(contact->pos, O, v, 1.0, t);
    // normal
    contact->normal[0] = sign * u[0];
    contact->normal[1] = sign * u[1];
    contact->normal[2] = sign * u[2];
    // depth
    contact->depth = t; // Same as the ray-box contact: depth means distance from the ray's origin to contact point
    return 1;
  }

  /////////////////////////////////////////////
  // Intersection with cylinder's generators //
  /////////////////////////////////////////////

  // Distance between infinite ray and infinite cylinder axes
  dVector3 w;
  dCalcVectorCross3(w, u, v);
  dReal d = dCalcVectorDot3(CO, w); // d = s * distance between infinite lines, with s = sinus(u, v) = sqrt (1 - c * c);
  d *= d;
  const dReal sinusSquare = 1 - c * c;
  const dReal k = rSquare * sinusSquare - d;
  if (k <= 0.0) // ray's axis is too far away from cylinder's axis to intersect with it
    return 0;

  //
  // Collision with cylinder's generators ?
  //

  // Quadratic formula to get intersection distance t
  // The point M = O + tv belongs to some cylinder's generator iff ||CM cross u|| = radius, which is equivalent to:
  // A * t * t + B * t + D = 0
  // where A = s * s, s = sinus(u,v), B = -2 * (CO cross u) . (u cross v), D = (CO cross u) . (CO cross u) - radius * radius
  // NOTE: Delta := B * B - 4 A * D = 4 * k;

  dVector3 COcrossU;
  dCalcVectorCross3(COcrossU, CO, u);
  const dReal halfB = -dCalcVectorDot3(COcrossU, w);
  const dReal delta = dSqrt(k);

  // Compute distances from the ray origin to the two possible contact points, say M1 = O + alpha1 * v and M2 = O + alpha2 * v;
  const dReal signOfB = (halfB > 0.0) ? 1.0 : -1.0;
  const dReal q = -(halfB + signOfB * delta); // takes the sign of b into account so as to reduce round off error for small sinus(u, v)
  dReal numerator = - halfB - delta;
  dReal alpha1 = -1.0;
  if (numerator > 0.0) {
    const dReal D = dCalcVectorLengthSquare3(COcrossU) - rSquare;
    alpha1 = D / q;
  }
  numerator =  - halfB + delta;
  dReal alpha2 = (numerator > 0.0) ? q / sinusSquare : -1.0;

  bool firstGeneratorContact  = (alpha1 >= 0.0 && alpha1 <= length);
  bool secondGeneratorContact = (alpha2 >= 0.0 && alpha2 <= length);

  dVector3 CM1 = { 0.0, 0.0, 0.0 };
  dReal proj1 = 0.0;
  if (firstGeneratorContact) {
    dAddScaledVectors3(CM1, CO, v, 1.0, alpha1);
    proj1 = dCalcVectorDot3(CM1, u);
    firstGeneratorContact = dFabs(proj1) < h;
    if (firstGeneratorContact)
      totalNumberOfContacts++;
  }

  dVector3 CM2 = { 0.0, 0.0, 0.0 };
  dReal proj2 = 0.0;
  if (secondGeneratorContact) {
    dAddScaledVectors3(CM2, CO, v, 1.0, alpha2);
    proj2 = dCalcVectorDot3(CM2, u);
    secondGeneratorContact = dFabs(proj2) < h;
    if (secondGeneratorContact)
      totalNumberOfContacts++;
  }

  // No valid contact
  if (totalNumberOfContacts == 0)
    return 0;

  // Two contacts with generators, we return the first one
  if (firstGeneratorContact && secondGeneratorContact) {
    if (alpha1 < alpha2) {
      t = alpha1;
      // normal
      dAddScaledVectors3(contact->normal, CM1, u, 1.0, -proj1);
    } else {
      t = alpha2;
      // normal
      dAddScaledVectors3(contact->normal, CM2, u, 1.0, -proj2);
    }
    // position
    dAddScaledVectors3(contact->pos, O, v, 1.0, t);
    // depth
    contact->depth = t;
    // normal
    dNormalize3( contact->normal );
    return 1;
  }

  t = hitTopCap ? t1 : t2;
  dReal alpha = firstGeneratorContact ? alpha1 : alpha2;
  const bool hitCapFirst = (hitTopCap || hitBottomCap) && (totalNumberOfContacts == 1 || t <= alpha);

  if (hitCapFirst) {
    // the ray hits a cap (and possibly a generator afterwards)
    // position
    dAddScaledVectors3(contact->pos, O, v, 1.0, t);
    // depth
    contact->depth = t;
    // normal
    const dReal sign = hitTopCap ? 1.0 : -1.0;
    contact->normal[0] = sign * u[0];
    contact->normal[1] = sign * u[1];
    contact->normal[2] = sign * u[2];
    return 1;
    } else {
      // the ray hits a generator
      // position
      dAddScaledVectors3(contact->pos, O, v, 1.0, alpha);
      // depth
      contact->depth = alpha;
      // normal
      if (firstGeneratorContact)
        dAddScaledVectors3(contact->normal, CM1, u, 1.0, -proj1);
      else
        dAddScaledVectors3(contact->normal, CM2, u, 1.0, -proj2);

      dNormalize3(contact->normal);
      return 1;
   }
}

/*
// Ray-Cylinder collider by Joseph Cooper (2011)
int dCollideRayCylinder( dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip )
{
    dIASSERT( skip >= (int)sizeof( dContactGeom ) );
    dIASSERT( o1->type == dRayClass );
    dIASSERT( o2->type == dCylinderClass );
    dIASSERT( (flags & NUMC_MASK) >= 1 );

    dxRay* ray = (dxRay*)( o1 );
    dxCylinder* cyl = (dxCylinder*)( o2 );

    // Fill in contact information.
    contact->g1 = ray;
    contact->g2 = cyl;
    contact->side1 = -1;
    contact->side2 = -1;

    const dReal half_length = cyl->lz * REAL( 0.5 );

    / * Possible collision cases:
     *  Ray origin between/outside caps
     *  Ray origin within/outside radius
     *  Ray direction left/right/perpendicular
     *  Ray direction parallel/perpendicular/other
     *
     *  Ray origin cases (ignoring origin on surface)
     *
     *  A          B
     *     /-\-----------\
     *  C (   )    D      )
     *     \_/___________/
     *
     *  Cases A and D can collide with caps or cylinder
     *  Case C can only collide with the caps
     *  Case B can only collide with the cylinder
     *  Case D will produce inverted normals
     *  If the ray is perpendicular, only check the cylinder
     *  If the ray is parallel to cylinder axis,
     *  we can only check caps
     *  If the ray points right,
     *    Case A,C Check left cap
     *    Case  D  Check right cap
     *  If the ray points left
     *    Case A,C Check right cap
     *    Case  D  Check left cap
     *  Case B, check only first possible cylinder collision
     *  Case D, check only second possible cylinder collision
     * /
    // Find the ray in the cylinder coordinate frame:
    dVector3 tmp;
    dVector3 pos;  // Ray origin in cylinder frame
    dVector3 dir;  // Ray direction in cylinder frame
    // Translate ray start by inverse cyl
    dSubtractVectors3(tmp,ray->final_posr->pos,cyl->final_posr->pos);
    // Rotate ray start by inverse cyl
    dMultiply1_331(pos,cyl->final_posr->R,tmp);

    // Get the ray's direction
    tmp[0] = ray->final_posr->R[2];
    tmp[1] = ray->final_posr->R[6];
    tmp[2] = ray->final_posr->R[10];
    // Rotate the ray direction by inverse cyl
    dMultiply1_331(dir,cyl->final_posr->R,tmp);

    // Is the ray origin inside of the (extended) cylinder?
    dReal r2 = cyl->radius*cyl->radius;
    dReal C = pos[0]*pos[0] + pos[1]*pos[1] - r2;

    // Find the different cases
    // Is ray parallel to the cylinder length?
    int parallel = (dir[0]==0 && dir[1]==0);
    // Is ray perpendicular to the cylinder length?
    int perpendicular = (dir[2]==0);
    // Is ray origin within the radius of the caps?
    int inRadius = (C<=0);
    // Is ray origin between the top and bottom caps?
    int inCaps   = (dFabs(pos[2])<=half_length);

    int checkCaps = (!perpendicular && (!inCaps || inRadius));
    int checkCyl  = (!parallel && (!inRadius || inCaps));
    int flipNormals = (inCaps&&inRadius);

    dReal tt=-dInfinity; // Depth to intersection
    dVector3 tmpNorm = {dNaN, dNaN, dNaN}; // ensure we don't leak garbage

    if (checkCaps) {
        // Make it so we only need to check one cap
        int flipDir = 0;
        // Wish c had logical xor...
        if ((dir[2]<0 && flipNormals) || (dir[2]>0 && !flipNormals)) {
            flipDir = 1;
            dir[2]=-dir[2];
            pos[2]=-pos[2];
        }
        // The cap is half the cylinder's length
        // from the cylinder's origin
        // We only checkCaps if dir[2]!=0
        tt = (half_length-pos[2])/dir[2];
        if (tt>=0 && tt<=ray->length) {
            tmp[0] = pos[0] + tt*dir[0];
            tmp[1] = pos[1] + tt*dir[1];
            // Ensure collision point is within cap circle
            if (tmp[0]*tmp[0] + tmp[1]*tmp[1] <= r2) {
                // Successful collision
                tmp[2] = (flipDir)?-half_length:half_length;
                tmpNorm[0]=0;
                tmpNorm[1]=0;
                tmpNorm[2]=(flipDir!=flipNormals)?-1:1;
                checkCyl = 0;  // Short circuit cylinder check
            } else {
                // Ray hits cap plane outside of cap circle
                tt=-dInfinity; // No collision yet
            }
        } else {
            // The cap plane is beyond (or behind) the ray length
            tt=-dInfinity; // No collision yet
        }
        if (flipDir) {
            // Flip back
            dir[2]=-dir[2];
            pos[2]=-pos[2];
        }
    }
    if (checkCyl) {
        // Compute quadratic formula for parametric ray equation
        dReal A =    dir[0]*dir[0] + dir[1]*dir[1];
        dReal B = 2*(pos[0]*dir[0] + pos[1]*dir[1]);
        // Already computed C

        dReal k = B*B - 4*A*C;
        // Check collision with infinite cylinder
        // k<0 means the ray passes outside the cylinder
        // k==0 means ray is tangent to cylinder (or parallel)
        //
        //  Our quadratic formula: tt = (-B +- sqrt(k))/(2*A)
        //
        // A must be positive (otherwise we wouldn't be checking
        // cylinder because ray is parallel)
        //    if (k<0) ray doesn't collide with sphere
        //    if (B > sqrt(k)) then both times are negative
        //         -- don't calculate
        //    if (B<-sqrt(k)) then both times are positive (Case A or B)
        //         -- only calculate first, if first isn't valid
        //         -- second can't be without first going through a cap
        //    otherwise (fabs(B)<=sqrt(k)) then C<=0 (ray-origin inside/on cylinder)
        //         -- only calculate second collision
        if (k>=0 && (B<0 || B*B<=k)) {
            k = dSqrt(k);
            A = dRecip(2*A);
            if (dFabs(B)<=k) {
                tt = (-B + k)*A; // Second solution
                // If ray origin is on surface and pointed out, we
                // can get a tt=0 solution...
            } else {
                tt = (-B - k)*A; // First solution
            }
            if (tt<=ray->length) {
                tmp[2] = pos[2] + tt*dir[2];
                if (dFabs(tmp[2])<=half_length) {
                    // Valid solution
                    tmp[0] = pos[0] + tt*dir[0];
                    tmp[1] = pos[1] + tt*dir[1];
                    tmpNorm[0] = tmp[0]/cyl->radius;
                    tmpNorm[1] = tmp[1]/cyl->radius;
                    tmpNorm[2] = 0;
                    if (flipNormals) {
                        // Ray origin was inside cylinder
                        tmpNorm[0] = -tmpNorm[0];
                        tmpNorm[1] = -tmpNorm[1];
                    }
                } else {
                    // Ray hits cylinder outside of caps
                    tt=-dInfinity;
                }
            } else {
                // Ray doesn't reach the cylinder
                tt=-dInfinity;
            }
        }
    }

    if (tt>0) {
        contact->depth = tt;
        // Transform the point back to world coordinates
        tmpNorm[3]=0;
        tmp[3] = 0;
        dMultiply0_331(contact->normal,cyl->final_posr->R,tmpNorm);
        dMultiply0_331(contact->pos,cyl->final_posr->R,tmp);
        contact->pos[0]+=cyl->final_posr->pos[0];
        contact->pos[1]+=cyl->final_posr->pos[1];
        contact->pos[2]+=cyl->final_posr->pos[2];

        return 1;
    }
    // No contact with anything.
    return 0;
}
*/