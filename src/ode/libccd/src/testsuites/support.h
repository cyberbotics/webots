/***
 * libccd
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *
 *  This file is part of libccd.
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see accompanying file BDS-LICENSE for details or see
 *  <http://www.opensource.org/licenses/bsd-license.php>.
 *
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */

/***
 * Some support() functions for some convex shapes.
 */

#ifndef __CCD_SUPPORT_H__
#define __CCD_SUPPORT_H__

#include <ccd/quat.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define CCD_OBJ_BOX 1
#define CCD_OBJ_SPHERE 2
#define CCD_OBJ_CYL 3

#define __CCD_OBJ__ \
    int type; \
    ccd_vec3_t pos; \
    ccd_quat_t quat;

struct _ccd_obj_t {
    __CCD_OBJ__
};
typedef struct _ccd_obj_t ccd_obj_t;

struct _ccd_box_t {
    __CCD_OBJ__
    ccd_real_t x, y, z; //!< Lengths of box's edges
};
typedef struct _ccd_box_t ccd_box_t;

struct _ccd_sphere_t {
    __CCD_OBJ__
    ccd_real_t radius;
};
typedef struct _ccd_sphere_t ccd_sphere_t;

struct _ccd_cyl_t {
    __CCD_OBJ__
    ccd_real_t radius;
    ccd_real_t height;
};
typedef struct _ccd_cyl_t ccd_cyl_t;

#define CCD_BOX(name) \
    ccd_box_t name = { .type = CCD_OBJ_BOX, \
                       .pos  = { .v = { 0., 0., 0. } }, \
                       .quat = { .q = { 0., 0., 0., 1. } }, \
                       .x = 0., \
                       .y = 0., \
                       .z = 0. }

#define CCD_SPHERE(name) \
    ccd_sphere_t name = { .type = CCD_OBJ_SPHERE, \
                          .pos  = { .v = { 0., 0., 0. } }, \
                          .quat = { .q = { 0., 0., 0., 1. } }, \
                          .radius = 0. }

#define CCD_CYL(name) \
    ccd_cyl_t name = { .type = CCD_OBJ_CYL, \
                       .pos  = { .v = { 0., 0., 0. } }, \
                       .quat = { .q = { 0., 0., 0., 1. } }, \
                       .radius = 0., \
                       .height = 0. }

/**
 * Returns supporting vertex via v.
 * Supporting vertex is fathest vertex from object in direction dir.
 */
void ccdSupport(const void *obj, const ccd_vec3_t *dir,
                ccd_vec3_t *v);

/**
 * Returns center of object.
 */
void ccdObjCenter(const void *obj, ccd_vec3_t *center);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __CCD_SUPPORT_H__ */
