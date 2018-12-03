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

#include <stdio.h>
#include <ccd/ccd.h>
#include <ccd/vec3.h>
#include "support.h"

void ccdSupport(const void *_obj, const ccd_vec3_t *_dir,
                ccd_vec3_t *v)
{
    // Support function is made according to Gino van den Bergen's paper
    //  A Fast and Robust CCD Implementation for Collision Detection of
    //  Convex Objects

    ccd_obj_t *obj = (ccd_obj_t *)_obj;
    ccd_vec3_t dir;
    ccd_quat_t qinv;

    ccdVec3Copy(&dir, _dir);
    ccdQuatInvert2(&qinv, &obj->quat);

    ccdQuatRotVec(&dir, &qinv);

    if (obj->type == CCD_OBJ_BOX){
        ccd_box_t *box = (ccd_box_t *)obj;
        ccdVec3Set(v, ccdSign(ccdVec3X(&dir)) * box->x * CCD_REAL(0.5),
                      ccdSign(ccdVec3Y(&dir)) * box->y * CCD_REAL(0.5),
                      ccdSign(ccdVec3Z(&dir)) * box->z * CCD_REAL(0.5));
    }else if (obj->type == CCD_OBJ_SPHERE){
        ccd_sphere_t *sphere = (ccd_sphere_t *)obj;
        ccd_real_t len;

        len = ccdVec3Len2(&dir);
        if (len - CCD_EPS > CCD_ZERO){
            ccdVec3Copy(v, &dir);
            ccdVec3Scale(v, sphere->radius / CCD_SQRT(len));
        }else{
            ccdVec3Set(v, CCD_ZERO, CCD_ZERO, CCD_ZERO);
        }
    }else if (obj->type == CCD_OBJ_CYL){
        ccd_cyl_t *cyl = (ccd_cyl_t *)obj;
        ccd_real_t zdist, rad;

        zdist = dir.v[0] * dir.v[0] + dir.v[1] * dir.v[1];
        zdist = CCD_SQRT(zdist);
        if (ccdIsZero(zdist)){
            ccdVec3Set(v, CCD_ZERO, CCD_ZERO,
                          ccdSign(ccdVec3Z(&dir)) * cyl->height * CCD_REAL(0.5));
        }else{
            rad = cyl->radius / zdist;

            ccdVec3Set(v, rad * ccdVec3X(&dir),
                          rad * ccdVec3Y(&dir),
                          ccdSign(ccdVec3Z(&dir)) * cyl->height * CCD_REAL(0.5));
        }
    }

    // transform support vertex
    ccdQuatRotVec(v, &obj->quat);
    ccdVec3Add(v, &obj->pos);
}

void ccdObjCenter(const void *_obj, ccd_vec3_t *center)
{
    ccd_obj_t *obj = (ccd_obj_t *)_obj;

    ccdVec3Set(center, CCD_ZERO, CCD_ZERO, CCD_ZERO);
    // rotation is not needed
    ccdVec3Add(center, &obj->pos);
}
