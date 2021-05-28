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

#include <ode/collision.h>
#include <ccd/ccd.h>
#include "ccdcustom/vec3.h"
#include "ccdcustom/quat.h"
#include "config.h"
#include "odemath.h"
#include "collision_libccd.h"
#include "collision_trimesh_internal.h"
#include "collision_std.h"
#include "collision_util.h"
#include "error.h"

struct _ccd_obj_t {
    ccd_vec3_t pos;
    ccd_quat_t rot, rot_inv;
};
typedef struct _ccd_obj_t ccd_obj_t;

struct _ccd_box_t {
    ccd_obj_t o;
    ccd_real_t dim[3];
};
typedef struct _ccd_box_t ccd_box_t;

struct _ccd_cap_t {
    ccd_obj_t o;
    ccd_real_t radius;
    ccd_vec3_t axis;
    ccd_vec3_t p1;
    ccd_vec3_t p2;
};
typedef struct _ccd_cap_t ccd_cap_t;

struct _ccd_cyl_t {
    ccd_obj_t o;
    ccd_real_t radius;
    ccd_vec3_t axis;
    ccd_vec3_t p1;
    ccd_vec3_t p2;
};
typedef struct _ccd_cyl_t ccd_cyl_t;

struct _ccd_sphere_t {
    ccd_obj_t o;
    ccd_real_t radius;
};
typedef struct _ccd_sphere_t ccd_sphere_t;

struct _ccd_convex_t {
    ccd_obj_t o;
    dxConvex *convex;
};
typedef struct _ccd_convex_t ccd_convex_t;

struct _ccd_triangle_t {
    ccd_obj_t o;
    ccd_vec3_t vertices[3];
};
typedef struct _ccd_triangle_t ccd_triangle_t;

/** Transforms geom to ccd struct */
static void ccdGeomToObj(const dGeomID g, ccd_obj_t *);
static void ccdGeomToBox(const dGeomID g, ccd_box_t *);
static void ccdGeomToCap(const dGeomID g, ccd_cap_t *);
static void ccdGeomToCyl(const dGeomID g, ccd_cyl_t *);
static void ccdGeomToSphere(const dGeomID g, ccd_sphere_t *);
static void ccdGeomToConvex(const dGeomID g, ccd_convex_t *);

/** Support functions */
static void ccdSupportBox(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);
static void ccdSupportCap(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);
static void ccdSupportCyl(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);
static void ccdSupportSphere(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);
static void ccdSupportConvex(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);

/** Center function */
static void ccdCenter(const void *obj, ccd_vec3_t *c);

/** General collide function */
static int ccdCollide(dGeomID o1, dGeomID o2, int flags,
    dContactGeom *contact, int skip,
    void *obj1, ccd_support_fn supp1, ccd_center_fn cen1,
    void *obj2, ccd_support_fn supp2, ccd_center_fn cen2);

static int collideCylCyl(dxGeom *o1, dxGeom *o2, ccd_cyl_t* cyl1, ccd_cyl_t* cyl2, int flags, dContactGeom *contacts, int skip);
static bool testAndPrepareDiscContactForAngle(dReal angle, dReal radius, dReal length, dReal lSum, ccd_cyl_t *priCyl, ccd_cyl_t *secCyl, ccd_vec3_t &p, dReal &out_depth);
// Adds a contact between 2 cylinders
static int addCylCylContact(dxGeom *o1, dxGeom *o2, ccd_vec3_t* axis, dContactGeom *contacts, ccd_vec3_t* p, dReal normaldir, dReal depth, int j, int flags, int skip);

static 
void ccdGeomToObj(const dGeomID g, ccd_obj_t *o)
{
    const dReal *ode_pos;
    dQuaternion ode_rot;

    ode_pos = dGeomGetPosition(g);
    dGeomGetQuaternion(g, ode_rot);

    ccdVec3Set(&o->pos, ode_pos[0], ode_pos[1], ode_pos[2]);
    ccdQuatSet(&o->rot, ode_rot[1], ode_rot[2], ode_rot[3], ode_rot[0]);

    ccdQuatInvert2(&o->rot_inv, &o->rot);
}

static 
void ccdGeomToBox(const dGeomID g, ccd_box_t *box)
{
    dVector3 dim;

    ccdGeomToObj(g, (ccd_obj_t *)box);

    dGeomBoxGetLengths(g, dim);
    box->dim[0] = (ccd_real_t)(dim[0] * 0.5);
    box->dim[1] = (ccd_real_t)(dim[1] * 0.5);
    box->dim[2] = (ccd_real_t)(dim[2] * 0.5);
}

static 
void ccdGeomToCap(const dGeomID g, ccd_cap_t *cap)
{
    dReal r, h;
    ccdGeomToObj(g, (ccd_obj_t *)cap);

    dGeomCapsuleGetParams(g, &r, &h);
    cap->radius = r;
    ccdVec3Set(&cap->axis, 0.0, 0.0, h / 2);
    ccdQuatRotVec(&cap->axis, &cap->o.rot);
    ccdVec3Copy(&cap->p1, &cap->axis);
    ccdVec3Copy(&cap->p2, &cap->axis);
    ccdVec3Scale(&cap->p2, -1.0);
    ccdVec3Add(&cap->p1, &cap->o.pos);
    ccdVec3Add(&cap->p2, &cap->o.pos);
}

static 
void ccdGeomToCyl(const dGeomID g, ccd_cyl_t *cyl)
{
    dReal r, h;
    ccdGeomToObj(g, (ccd_obj_t *)cyl);

    dGeomCylinderGetParams(g, &r, &h);
    cyl->radius = r;
    ccdVec3Set(&cyl->axis, 0.0, 0.0, h / 2);
    ccdQuatRotVec(&cyl->axis, &cyl->o.rot);
    ccdVec3Copy(&cyl->p1, &cyl->axis);
    ccdVec3Copy(&cyl->p2, &cyl->axis);
    int cylAxisNormalizationResult = ccdVec3SafeNormalize(&cyl->axis);
    dUVERIFY(cylAxisNormalizationResult == 0, "Invalid cylinder has been passed");
    ccdVec3Scale(&cyl->p2, -1.0);
    ccdVec3Add(&cyl->p1, &cyl->o.pos);
    ccdVec3Add(&cyl->p2, &cyl->o.pos);
}

static 
void ccdGeomToSphere(const dGeomID g, ccd_sphere_t *s)
{
    ccdGeomToObj(g, (ccd_obj_t *)s);
    s->radius = dGeomSphereGetRadius(g);
}

static 
void ccdGeomToConvex(const dGeomID g, ccd_convex_t *c)
{
    ccdGeomToObj(g, (ccd_obj_t *)c);
    c->convex = (dxConvex *)g;
}


static 
void ccdSupportBox(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    const ccd_box_t *o = (const ccd_box_t *)obj;
    ccd_vec3_t dir;

    ccdVec3Copy(&dir, _dir);
    ccdQuatRotVec(&dir, &o->o.rot_inv);

    ccdVec3Set(v, ccdSign(ccdVec3X(&dir)) * o->dim[0],
        ccdSign(ccdVec3Y(&dir)) * o->dim[1],
        ccdSign(ccdVec3Z(&dir)) * o->dim[2]);

    // transform support vertex
    ccdQuatRotVec(v, &o->o.rot);
    ccdVec3Add(v, &o->o.pos);
}

static 
void ccdSupportCap(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    const ccd_cap_t *o = (const ccd_cap_t *)obj;

    ccdVec3Copy(v, _dir);
    ccdVec3Scale(v, o->radius);

    if (ccdVec3Dot(_dir, &o->axis) > 0.0){
        ccdVec3Add(v, &o->p1);
    }else{
        ccdVec3Add(v, &o->p2);
    }

}

static 
void ccdSupportCyl(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    const ccd_cyl_t *cyl = (const ccd_cyl_t *)obj;
    ccd_vec3_t dir;
    ccd_real_t len;
    
    ccd_real_t dot = ccdVec3Dot(_dir, &cyl->axis);
    if (dot > 0.0){
        ccdVec3Copy(v, &cyl->p1);
    } else{
        ccdVec3Copy(v, &cyl->p2);
    }
    // project dir onto cylinder's 'top'/'bottom' plane
    ccdVec3Copy(&dir, &cyl->axis);
    ccdVec3Scale(&dir, -dot);
    ccdVec3Add(&dir, _dir);
    len = CCD_SQRT(ccdVec3Len2(&dir));
    if (!ccdIsZero(len)) {
        ccdVec3Scale(&dir, cyl->radius / len);
        ccdVec3Add(v, &dir);
    }
}

static 
void ccdSupportSphere(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    const ccd_sphere_t *s = (const ccd_sphere_t *)obj;

    ccdVec3Copy(v, _dir);
    ccdVec3Scale(v, s->radius);
    dIASSERT(dFabs(CCD_SQRT(ccdVec3Len2(_dir)) - REAL(1.0)) < 1e-6); // ccdVec3Scale(v, CCD_ONE / CCD_SQRT(ccdVec3Len2(_dir)));

    ccdVec3Add(v, &s->o.pos);
}

static 
void ccdSupportConvex(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    const ccd_convex_t *c = (const ccd_convex_t *)obj;
    ccd_vec3_t dir, p;
    ccd_real_t maxdot, dot;
    size_t i;
    const dReal *curp;

    ccdVec3Copy(&dir, _dir);
    ccdQuatRotVec(&dir, &c->o.rot_inv);

    maxdot = -CCD_REAL_MAX;
    curp = c->convex->points;
    for (i = 0; i < c->convex->pointcount; i++, curp += 3){
        ccdVec3Set(&p, curp[0], curp[1], curp[2]);
        dot = ccdVec3Dot(&dir, &p);
        if (dot > maxdot){
            ccdVec3Copy(v, &p);
            maxdot = dot;
        }
    }

    // transform support vertex
    ccdQuatRotVec(v, &c->o.rot);
    ccdVec3Add(v, &c->o.pos);
}

static 
void ccdCenter(const void *obj, ccd_vec3_t *c)
{
    const ccd_obj_t *o = (const ccd_obj_t *)obj;
    ccdVec3Copy(c, &o->pos);
}

static 
int ccdCollide(
    dGeomID o1, dGeomID o2, int flags, dContactGeom *contact, int skip,
    void *obj1, ccd_support_fn supp1, ccd_center_fn cen1,
    void *obj2, ccd_support_fn supp2, ccd_center_fn cen2)
{
    ccd_t ccd;
    int res;
    ccd_real_t depth;
    ccd_vec3_t dir, pos;
    int max_contacts = (flags & NUMC_MASK);

    if (max_contacts < 1)
        return 0;

    CCD_INIT(&ccd);
    ccd.support1 = supp1;
    ccd.support2 = supp2;
    ccd.center1  = cen1;
    ccd.center2  = cen2;
    ccd.max_iterations = 500;
    ccd.mpr_tolerance = (ccd_real_t)1E-6;

    if (flags & CONTACTS_UNIMPORTANT){
        if (ccdMPRIntersect(obj1, obj2, &ccd)){
            return 1;
        }else{
            return 0;
        }
    }

    res = ccdMPRPenetration(obj1, obj2, &ccd, &depth, &dir, &pos);
    if (res == 0){
        contact->g1 = o1;
        contact->g2 = o2;

        contact->side1 = contact->side2 = -1;

        contact->depth = depth;

        contact->pos[0] = ccdVec3X(&pos);
        contact->pos[1] = ccdVec3Y(&pos);
        contact->pos[2] = ccdVec3Z(&pos);

        ccdVec3Scale(&dir, -1.);
        contact->normal[0] = ccdVec3X(&dir);
        contact->normal[1] = ccdVec3Y(&dir);
        contact->normal[2] = ccdVec3Z(&dir);

        return 1;
    }

    return 0;
}

/*extern */
int dCollideBoxCylinderCCD(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_cyl_t cyl;
    ccd_box_t box;

    ccdGeomToBox(o1, &box);
    ccdGeomToCyl(o2, &cyl);

    return ccdCollide(o1, o2, flags, contact, skip,
        &box, ccdSupportBox, ccdCenter,
        &cyl, ccdSupportCyl, ccdCenter);
}

/*extern */
int dCollideCapsuleCylinder(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_cap_t cap;
    ccd_cyl_t cyl;

    ccdGeomToCap(o1, &cap);
    ccdGeomToCyl(o2, &cyl);

    return ccdCollide(o1, o2, flags, contact, skip,
        &cap, ccdSupportCap, ccdCenter,
        &cyl, ccdSupportCyl, ccdCenter);
}

/*extern */
int dCollideConvexBoxCCD(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_box_t box;
    ccd_convex_t conv;

    ccdGeomToConvex(o1, &conv);
    ccdGeomToBox(o2, &box);

    return ccdCollide(o1, o2, flags, contact, skip,
        &conv, ccdSupportConvex, ccdCenter,
        &box, ccdSupportBox, ccdCenter);
}

/*extern */
int dCollideConvexCapsuleCCD(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_cap_t cap;
    ccd_convex_t conv;

    ccdGeomToConvex(o1, &conv);
    ccdGeomToCap(o2, &cap);

    return ccdCollide(o1, o2, flags, contact, skip,
        &conv, ccdSupportConvex, ccdCenter,
        &cap, ccdSupportCap, ccdCenter);
}

/*extern */
int dCollideConvexSphereCCD(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_sphere_t sphere;
    ccd_convex_t conv;

    ccdGeomToConvex(o1, &conv);
    ccdGeomToSphere(o2, &sphere);

    return ccdCollide(o1, o2, flags, contact, skip,
        &conv, ccdSupportConvex, ccdCenter,
        &sphere, ccdSupportSphere, ccdCenter);
}

/*extern */
int dCollideConvexCylinderCCD(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_cyl_t cyl;
    ccd_convex_t conv;

    ccdGeomToConvex(o1, &conv);
    ccdGeomToCyl(o2, &cyl);

    return ccdCollide(o1, o2, flags, contact, skip,
        &conv, ccdSupportConvex, ccdCenter,
        &cyl, ccdSupportCyl, ccdCenter);
}

/*extern */
int dCollideConvexConvexCCD(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_convex_t c1, c2;

    ccdGeomToConvex(o1, &c1);
    ccdGeomToConvex(o2, &c2);

    return ccdCollide(o1, o2, flags, contact, skip,
        &c1, ccdSupportConvex, ccdCenter,
        &c2, ccdSupportConvex, ccdCenter);
}


/*extern */
int dCollideCylinderCylinder(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_cyl_t cyl1, cyl2;
    
    ccdGeomToCyl(o1, &cyl1);
    ccdGeomToCyl(o2, &cyl2);
    
    int numContacts = collideCylCyl(o1, o2, &cyl1, &cyl2, flags, contact, skip);
    if (numContacts < 0) {
        numContacts = ccdCollide(o1, o2, flags, contact, skip,
                                 &cyl1, ccdSupportCyl, ccdCenter,
                                 &cyl2, ccdSupportCyl, ccdCenter);
    }
    return numContacts;
}

static 
int collideCylCyl(dxGeom *o1, dxGeom *o2, ccd_cyl_t* cyl1, ccd_cyl_t* cyl2, int flags, dContactGeom *contacts, int skip) 
{
    int maxContacts = (flags & NUMC_MASK);
    dAASSERT(maxContacts != 0);

    maxContacts = maxContacts > 8 ? 8 : maxContacts;
    
    dReal axesProd = dFabs(ccdVec3Dot(&cyl1->axis, &cyl2->axis));
    // Check if cylinders' axes are in line
    if (REAL(1.0) - axesProd < 1e-3f) {
        ccd_vec3_t p, proj;
        dReal r1, l1;
        dReal r2, l2;
        dGeomCylinderGetParams(o1, &r1, &l1);
        dGeomCylinderGetParams(o2, &r2, &l2);
        l1 *= 0.5f;
        l2 *= 0.5f;
       
        // Determine the cylinder with smaller radius (minCyl) and bigger radius (maxCyl) and their respective properties: radius, length
        bool r1IsMin;
        dReal rmin, rmax;
        ccd_cyl_t *minCyl, *maxCyl;
        if (r1 <= r2) {
            rmin = r1; rmax = r2;
            minCyl = cyl1; maxCyl = cyl2;
            r1IsMin = true;
        }
        else {
            rmin = r2; rmax = r1;
            minCyl = cyl2; maxCyl = cyl1;
            r1IsMin = false;
        }

        dReal lSum = l1 + l2;

        ccdVec3Copy(&p, &minCyl->o.pos);
        ccdVec3Sub(&p, &maxCyl->o.pos);
        dReal dot = ccdVec3Dot(&p, &maxCyl->axis);
        
        // Maximum possible contact depth
        dReal depth_v = lSum - dFabs(dot) + dSqrt(dMax(0, REAL(1.0) - axesProd * axesProd)) * rmin;
        if (depth_v < 0) {
            return 0;
        }

        // Project the smaller cylinder's center onto the larger cylinder's plane
        ccdVec3Copy(&proj, &maxCyl->axis);
        ccdVec3Scale(&proj, -dot);
        ccdVec3Add(&proj, &p);
        dReal radiiDiff = (dReal)sqrt(ccdVec3Len2(&proj));
        dReal depth_h = r1 + r2 - radiiDiff;

        // Check the distance between cylinders' centers
        if (depth_h < 0) {
            return 0;
        }

        // Check if "vertical" contact depth is less than "horizontal" contact depth
        if (depth_v < depth_h) {
            int contactCount = 0;
            dReal dot2 = -ccdVec3Dot(&p, &minCyl->axis);
            // lmin, lmax - distances from cylinders' centers to potential contact points relative to cylinders' axes
            dReal lmax = r1IsMin ? l2 : l1;
            dReal lmin = r1IsMin ? l1 : l2;
            lmin = dot2 < 0 ? -lmin : lmin;
            lmax = dot < 0 ? -lmax : lmax;
            // Contact normal direction, relative to o1's axis
            dReal normaldir = (dot < 0) != r1IsMin ? REAL(1.0) : -REAL(1.0);
            
            if (rmin + radiiDiff <= rmax) {
                // Case 1: The smaller disc is fully contained within the larger one
                // Simply generate N points on the rim of the smaller disc
                dReal maxContactsRecip = (dReal)(0 < maxContacts ? (2.0 * M_PI / maxContacts) : (2.0 * M_PI)); // The 'else' value does not matter. Just try helping the optimizer.
                for (int i = 0; i < maxContacts; i++) {
                    dReal depth;
                    dReal a = maxContactsRecip * i;
                    if (testAndPrepareDiscContactForAngle(a, rmin, lmin, lSum, minCyl, maxCyl, p, depth)) {
                        contactCount = addCylCylContact(o1, o2, &maxCyl->axis, contacts, &p, normaldir, depth, contactCount, flags, skip);
                        if ((flags & CONTACTS_UNIMPORTANT) != 0) {
                            dIASSERT(contactCount != 0);
                            break;
                        }
                    }
                }
                return contactCount;

            } else {
                // Case 2: Discs intersect
                // Firstly, find intersections assuming the larger cylinder is placed at (0,0,0)
                // http://math.stackexchange.com/questions/256100/how-can-i-find-the-points-at-which-two-circles-intersect
                ccd_vec3_t proj2;
                ccdVec3Copy(&proj2, &proj);
                ccdQuatRotVec(&proj, &maxCyl->o.rot_inv);
                dReal d = dSqrt(ccdVec3X(&proj) * ccdVec3X(&proj) + ccdVec3Y(&proj) * ccdVec3Y(&proj));
                dIASSERT(d != REAL(0.0));
                
                dReal dRecip = REAL(1.0) / d;
                dReal rmaxSquare = rmax * rmax, rminSquare = rmin * rmin, dSquare = d * d;

                dReal minA, diffA, minB, diffB;

                {
                    dReal l = (rmaxSquare - rminSquare + dSquare) * (REAL(0.5) * dRecip);
                    dReal h = dSqrt(rmaxSquare - l * l);
                    dReal divLbyD = l * dRecip, divHbyD = h * dRecip;
                    dReal x1 = divLbyD * ccdVec3X(&proj) + divHbyD * ccdVec3Y(&proj);
                    dReal y1 = divLbyD * ccdVec3Y(&proj) - divHbyD * ccdVec3X(&proj);
                    dReal x2 = divLbyD * ccdVec3X(&proj) - divHbyD * ccdVec3Y(&proj);
                    dReal y2 = divLbyD * ccdVec3Y(&proj) + divHbyD * ccdVec3X(&proj);
                    // Map the intersection points to angles
                    dReal ap1 = dAtan2(y1, x1);
                    dReal ap2 = dAtan2(y2, x2);
                    minA = dMin(ap1, ap2);
                    dReal maxA = dMax(ap1, ap2);
                    // If the segment connecting cylinders' centers does not intersect the arc, change the angles
                    dReal a = dAtan2(ccdVec3Y(&proj), ccdVec3X(&proj));
                    if (a < minA || a > maxA) {
                        a = maxA;
                        maxA = (dReal)(minA + M_PI * 2.0);
                        minA = a;
                    }
                    diffA = maxA - minA;
                }
                
                // Do the same for the smaller cylinder assuming it is placed at (0,0,0) now
                ccdVec3Copy(&proj, &proj2);
                ccdVec3Scale(&proj, -1);
                ccdQuatRotVec(&proj, &minCyl->o.rot_inv);
                
                {
                    dReal l = (rminSquare - rmaxSquare + dSquare) * (REAL(0.5) * dRecip);
                    dReal h = dSqrt(rminSquare - l * l);
                    dReal divLbyD = l * dRecip, divHbyD = h * dRecip;
                    dReal x1 = divLbyD * ccdVec3X(&proj) + divHbyD * ccdVec3Y(&proj);
                    dReal y1 = divLbyD * ccdVec3Y(&proj) - divHbyD * ccdVec3X(&proj);
                    dReal x2 = divLbyD * ccdVec3X(&proj) - divHbyD * ccdVec3Y(&proj);
                    dReal y2 = divLbyD * ccdVec3Y(&proj) + divHbyD * ccdVec3X(&proj);
                    dReal ap1 = dAtan2(y1, x1);
                    dReal ap2 = dAtan2(y2, x2);
                    minB = dMin(ap1, ap2);
                    dReal maxB = dMax(ap1, ap2);
                    dReal a = dAtan2(ccdVec3Y(&proj), ccdVec3X(&proj));
                    if (a < minB || a > maxB) {
                        a = maxB;
                        maxB = (dReal)(minB + M_PI * 2.0);
                        minB = a;
                    }
                    diffB = maxB - minB;
                }

                // Find contact point distribution ratio based on arcs lengths
                dReal ratio = diffA * rmax  / (diffA * rmax + diffB  * rmin);
                dIASSERT(ratio <= REAL(1.0)); 
                dIASSERT(ratio >= REAL(0.0));

                int nMax = (int)dFloor(ratio * maxContacts + REAL(0.5));
                int nMin = maxContacts - nMax;
                dIASSERT(nMax <= maxContacts);

                // Make sure there is at least one point on the smaller radius rim
                if (nMin < 1) {
                    nMin = 1; nMax -= 1;
                }
                // Otherwise transfer one point to the larger radius rim as it is going to fill the rim intersection points
                else if (nMin > 1) {
                    nMin -= 1; nMax += 1;
                }

                // Smaller disc first, skipping the overlapping points
                dReal nMinRecip = 0 < nMin ? diffB / (nMin + 1) : diffB; // The 'else' value does not matter. Just try helping the optimizer.
                for (int i = 1; i <= nMin; i++) {
                    dReal depth;
                    dReal a = minB + nMinRecip * i;
                    if (testAndPrepareDiscContactForAngle(a, rmin, lmin, lSum, minCyl, maxCyl, p, depth)) {
                        contactCount = addCylCylContact(o1, o2, &maxCyl->axis, contacts, &p, normaldir, depth, contactCount, flags, skip);
                        if ((flags & CONTACTS_UNIMPORTANT) != 0) {
                            dIASSERT(contactCount != 0);
                            break;
                        }
                    }
                }

                if (contactCount == 0 || (flags & CONTACTS_UNIMPORTANT) == 0) {
                    // Then the larger disc, + additional point as the start/end points of arcs overlap
                    // (or a single contact at the arc middle point if just one is required)
                    dReal nMaxRecip = nMax > 1 ? diffA / (nMax - 1) : diffA; // The 'else' value does not matter. Just try helping the optimizer.
                    dReal adjustedMinA = nMax == 1 ? minA + REAL(0.5) * diffA : minA;

                    for (int i = 0; i < nMax; i++) {
                        dReal depth;
                        dReal a = adjustedMinA + nMaxRecip * i;
                        if (testAndPrepareDiscContactForAngle(a, rmax, lmax, lSum, maxCyl, minCyl, p, depth)) {
                            contactCount = addCylCylContact(o1, o2, &maxCyl->axis, contacts, &p, normaldir, depth, contactCount, flags, skip);
                            if ((flags & CONTACTS_UNIMPORTANT) != 0) {
                                dIASSERT(contactCount != 0);
                                break;
                            }
                        }
                    }
                }

                return contactCount;
            }
        }
    }
    return -1;
}

static 
bool testAndPrepareDiscContactForAngle(dReal angle, dReal radius, dReal length, dReal lSum, ccd_cyl_t *priCyl, ccd_cyl_t *secCyl, ccd_vec3_t &p, dReal &out_depth)
{
    bool ret = false;

    ccd_vec3_t p2;
    ccdVec3Set(&p, dCos(angle) * radius, dSin(angle) * radius, 0);
    ccdQuatRotVec(&p, &priCyl->o.rot);
    ccdVec3Add(&p, &priCyl->o.pos);
    ccdVec3Copy(&p2, &p);
    ccdVec3Sub(&p2, &secCyl->o.pos);
    dReal depth = lSum - dFabs(ccdVec3Dot(&p2, &secCyl->axis));

    if (depth >= 0) {
        ccdVec3Copy(&p2, &priCyl->axis);
        ccdVec3Scale(&p2, length);
        ccdVec3Add(&p, &p2);

        out_depth = depth;
        ret = true;
    }

    return ret;
}

static 
int addCylCylContact(dxGeom *o1, dxGeom *o2, ccd_vec3_t* axis, dContactGeom *contacts,
               ccd_vec3_t* p, dReal normaldir, dReal depth, int j, int flags, int skip)
{
    dIASSERT(depth >= 0);

    dContactGeom* contact = SAFECONTACT(flags, contacts, j, skip);
    contact->g1 = o1;
    contact->g2 = o2;
    contact->side1 = -1;
    contact->side2 = -1;
    contact->normal[0] = normaldir * ccdVec3X(axis);
    contact->normal[1] = normaldir * ccdVec3Y(axis);
    contact->normal[2] = normaldir * ccdVec3Z(axis);
    contact->depth = depth;
    contact->pos[0] = ccdVec3X(p);
    contact->pos[1] = ccdVec3Y(p);
    contact->pos[2] = ccdVec3Z(p);

    return j + 1;
}
