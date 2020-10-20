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

#ifndef __CCD_CUSTOM_QUAT_H__
#define __CCD_CUSTOM_QUAT_H__

#include <ccd/quat.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * Rotate vector s by quaternion q and put result into d.
 */
_ccd_inline void ccdQuatRotVec2(ccd_vec3_t *d, const ccd_vec3_t *s, const ccd_quat_t *q);


_ccd_inline void ccdQuatRotVec2(ccd_vec3_t *d, const ccd_vec3_t *s, const ccd_quat_t *q)
{
#ifndef dLIBCCD_USE_SYSTEM
    // original version: 31 mul + 21 add
    // optimized version: 18 mul + 12 add
    // formula: d = s + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * s)
    ccd_real_t cross1_x, cross1_y, cross1_z, cross2_x, cross2_y, cross2_z;
    ccd_real_t x, y, z, w;
    ccd_real_t vx, vy, vz;

    vx = ccdVec3X(s);
    vy = ccdVec3Y(s);
    vz = ccdVec3Z(s);

    w = q->q[3];
    x = q->q[0];
    y = q->q[1];
    z = q->q[2];

    cross1_x = y * vz - z * vy + w * vx;
    cross1_y = z * vx - x * vz + w * vy;
    cross1_z = x * vy - y * vx + w * vz;
    cross2_x = y * cross1_z - z * cross1_y;
    cross2_y = z * cross1_x - x * cross1_z;
    cross2_z = x * cross1_y - y * cross1_x;
    ccdVec3Set(d, vx + 2 * cross2_x, vy + 2 * cross2_y, vz + 2 * cross2_z);
#else
    ccdVec3Copy(d, s);
    ccdQuatRotVec(d, q);
#endif
}

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __CCD_QUAT_H__ */