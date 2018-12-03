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
#include <ccd/quat.h>
#include "config.h"
#include "odemath.h"
#include "collision_libccd.h"
#include "collision_std.h"

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
    ccd_real_t radius, height;
};
typedef struct _ccd_cap_t ccd_cap_t;

struct _ccd_cyl_t {
    ccd_obj_t o;
    ccd_real_t radius, height;
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

static void ccdGeomToObj(const dGeomID g, ccd_obj_t *o)
{
    const dReal *ode_pos;
    dQuaternion ode_rot;

    ode_pos = dGeomGetPosition(g);
    dGeomGetQuaternion(g, ode_rot);

    ccdVec3Set(&o->pos, ode_pos[0], ode_pos[1], ode_pos[2]);
    ccdQuatSet(&o->rot, ode_rot[1], ode_rot[2], ode_rot[3], ode_rot[0]);

    ccdQuatInvert2(&o->rot_inv, &o->rot);
}

static void ccdGeomToBox(const dGeomID g, ccd_box_t *box)
{
    dVector3 dim;

    ccdGeomToObj(g, (ccd_obj_t *)box);

    dGeomBoxGetLengths(g, dim);
    box->dim[0] = dim[0] / 2.;
    box->dim[1] = dim[1] / 2.;
    box->dim[2] = dim[2] / 2.;
}

static void ccdGeomToCap(const dGeomID g, ccd_cap_t *cap)
{
    dReal r, h;
    ccdGeomToObj(g, (ccd_obj_t *)cap);

    dGeomCapsuleGetParams(g, &r, &h);
    cap->radius = r;
    cap->height = h / 2.;
}

static void ccdGeomToCyl(const dGeomID g, ccd_cyl_t *cyl)
{
    dReal r, h;
    ccdGeomToObj(g, (ccd_obj_t *)cyl);

    dGeomCylinderGetParams(g, &r, &h);
    cyl->radius = r;
    cyl->height = h / 2.;
}

static void ccdGeomToSphere(const dGeomID g, ccd_sphere_t *s)
{
    ccdGeomToObj(g, (ccd_obj_t *)s);
    s->radius = dGeomSphereGetRadius(g);
}

static void ccdGeomToConvex(const dGeomID g, ccd_convex_t *c)
{
    ccdGeomToObj(g, (ccd_obj_t *)c);
    c->convex = (dxConvex *)g;
}

static void ccdSupportBox(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
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

static void ccdSupportCap(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    const ccd_cap_t *o = (const ccd_cap_t *)obj;
    ccd_vec3_t dir, pos1, pos2;

    ccdVec3Copy(&dir, _dir);
    ccdQuatRotVec(&dir, &o->o.rot_inv);

    ccdVec3Set(&pos1, CCD_ZERO, CCD_ZERO, o->height);
    ccdVec3Set(&pos2, CCD_ZERO, CCD_ZERO, -o->height);

    ccdVec3Copy(v, &dir);
    ccdVec3Scale(v, o->radius);
    ccdVec3Add(&pos1, v);
    ccdVec3Add(&pos2, v);

    if (ccdVec3Dot(&dir, &pos1) > ccdVec3Dot(&dir, &pos2)){
        ccdVec3Copy(v, &pos1);
    }else{
        ccdVec3Copy(v, &pos2);
    }

    // transform support vertex
    ccdQuatRotVec(v, &o->o.rot);
    ccdVec3Add(v, &o->o.pos);
}

static void ccdSupportCyl(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    const ccd_cyl_t *cyl = (const ccd_cyl_t *)obj;
    ccd_vec3_t dir;
    double zdist, rad;

    ccdVec3Copy(&dir, _dir);
    ccdQuatRotVec(&dir, &cyl->o.rot_inv);

    zdist = dir.v[0] * dir.v[0] + dir.v[1] * dir.v[1];
    zdist = sqrt(zdist);
    if (ccdIsZero(zdist)){
        ccdVec3Set(v, 0., 0., ccdSign(ccdVec3Z(&dir)) * cyl->height);
    }else{
        rad = cyl->radius / zdist;

        ccdVec3Set(v, rad * ccdVec3X(&dir),
            rad * ccdVec3Y(&dir),
            ccdSign(ccdVec3Z(&dir)) * cyl->height);
    }

    // transform support vertex
    ccdQuatRotVec(v, &cyl->o.rot);
    ccdVec3Add(v, &cyl->o.pos);
}

static void ccdSupportSphere(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    const ccd_sphere_t *s = (const ccd_sphere_t *)obj;
    ccd_vec3_t dir;

    ccdVec3Copy(&dir, _dir);
    ccdQuatRotVec(&dir, &s->o.rot_inv);

    ccdVec3Copy(v, &dir);
    ccdVec3Scale(v, s->radius);
    ccdVec3Scale(v, CCD_ONE / CCD_SQRT(ccdVec3Len2(&dir)));

    // transform support vertex
    ccdQuatRotVec(v, &s->o.rot);
    ccdVec3Add(v, &s->o.pos);
}

static void ccdSupportConvex(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v)
{
    const ccd_convex_t *c = (const ccd_convex_t *)obj;
    ccd_vec3_t dir, p;
    ccd_real_t maxdot, dot;
    size_t i;
    dReal *curp;

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

static void ccdCenter(const void *obj, ccd_vec3_t *c)
{
    const ccd_obj_t *o = (const ccd_obj_t *)obj;
    ccdVec3Copy(c, &o->pos);
}

static int ccdCollide(
    dGeomID o1, dGeomID o2, int flags, dContactGeom *contact, int skip,
    void *obj1, ccd_support_fn supp1, ccd_center_fn cen1,
    void *obj2, ccd_support_fn supp2, ccd_center_fn cen2)
{
    ccd_t ccd;
    int res;
    ccd_real_t depth;
    ccd_vec3_t dir, pos;
    int max_contacts = (flags & 0xffff);

    if (max_contacts < 1)
        return 0;

    CCD_INIT(&ccd);
    ccd.support1 = supp1;
    ccd.support2 = supp2;
    ccd.center1  = cen1;
    ccd.center2  = cen2;
    ccd.max_iterations = 500;
    ccd.mpr_tolerance = 1E-6;

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

int dCollideConvexConvexCCD(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
    ccd_convex_t c1, c2;

    ccdGeomToConvex(o1, &c1);
    ccdGeomToConvex(o2, &c2);

    return ccdCollide(o1, o2, flags, contact, skip,
        &c1, ccdSupportConvex, ccdCenter,
        &c2, ccdSupportConvex, ccdCenter);
}
