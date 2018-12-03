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

#ifndef __CCD_QUAT_H__
#define __CCD_QUAT_H__

#include <ccd/compiler.h>
#include <ccd/vec3.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct _ccd_quat_t {
    ccd_real_t q[4]; //!< x, y, z, w
};
typedef struct _ccd_quat_t ccd_quat_t;

#define CCD_QUAT(name, x, y, z, w) \
    ccd_quat_t name = { {x, y, z, w} }

_ccd_inline ccd_real_t ccdQuatLen2(const ccd_quat_t *q);
_ccd_inline ccd_real_t ccdQuatLen(const ccd_quat_t *q);

_ccd_inline void ccdQuatSet(ccd_quat_t *q, ccd_real_t x, ccd_real_t y, ccd_real_t z, ccd_real_t w);
_ccd_inline void ccdQuatCopy(ccd_quat_t *dest, const ccd_quat_t *src);

_ccd_inline int ccdQuatNormalize(ccd_quat_t *q);

_ccd_inline void ccdQuatSetAngleAxis(ccd_quat_t *q,
                                     ccd_real_t angle, const ccd_vec3_t *axis);

_ccd_inline void ccdQuatScale(ccd_quat_t *q, ccd_real_t k);

/**
 * q = q * q2
 */
_ccd_inline void ccdQuatMul(ccd_quat_t *q, const ccd_quat_t *q2);

/**
 * q = a * b
 */
_ccd_inline void ccdQuatMul2(ccd_quat_t *q,
                             const ccd_quat_t *a, const ccd_quat_t *b);

/**
 * Inverts quaternion.
 * Returns 0 on success.
 */
_ccd_inline int ccdQuatInvert(ccd_quat_t *q);
_ccd_inline int ccdQuatInvert2(ccd_quat_t *dest, const ccd_quat_t *src);

/**
 * Rotate vector v by quaternion q.
 */
_ccd_inline void ccdQuatRotVec(ccd_vec3_t *v, const ccd_quat_t *q);

/**** INLINES ****/
_ccd_inline ccd_real_t ccdQuatLen2(const ccd_quat_t *q)
{
    ccd_real_t len;

    len  = q->q[0] * q->q[0];
    len += q->q[1] * q->q[1];
    len += q->q[2] * q->q[2];
    len += q->q[3] * q->q[3];

    return len;
}

_ccd_inline ccd_real_t ccdQuatLen(const ccd_quat_t *q)
{
    return CCD_SQRT(ccdQuatLen2(q));
}

_ccd_inline void ccdQuatSet(ccd_quat_t *q, ccd_real_t x, ccd_real_t y, ccd_real_t z, ccd_real_t w)
{
    q->q[0] = x;
    q->q[1] = y;
    q->q[2] = z;
    q->q[3] = w;
}

_ccd_inline void ccdQuatCopy(ccd_quat_t *dest, const ccd_quat_t *src)
{
    *dest = *src;
}

_ccd_inline int ccdQuatNormalize(ccd_quat_t *q)
{
    ccd_real_t len = ccdQuatLen(q);
    if (len < CCD_EPS)
        return 0;

    ccdQuatScale(q, CCD_ONE / len);
    return 1;
}

_ccd_inline void ccdQuatSetAngleAxis(ccd_quat_t *q,
                                     ccd_real_t angle, const ccd_vec3_t *axis)
{
    ccd_real_t a, x, y, z, n, s;

    a = angle/2;
    x = ccdVec3X(axis);
    y = ccdVec3Y(axis);
    z = ccdVec3Z(axis);
    n = CCD_SQRT(x*x + y*y + z*z);

    // axis==0? (treat this the same as angle==0 with an arbitrary axis)
    if (n < CCD_EPS){
        q->q[0] = q->q[1] = q->q[2] = CCD_ZERO;
        q->q[3] = CCD_ONE;
    }else{
        s = sin(a)/n;

        q->q[3] = cos(a);
        q->q[0] = x*s;
        q->q[1] = y*s;
        q->q[2] = z*s;

        ccdQuatNormalize(q);
    }
}

_ccd_inline void ccdQuatScale(ccd_quat_t *q, ccd_real_t k)
{
    size_t i;
    for (i = 0; i < 4; i++)
        q->q[i] *= k;
}

_ccd_inline void ccdQuatMul(ccd_quat_t *q, const ccd_quat_t *q2)
{
    ccd_quat_t a;
    ccdQuatCopy(&a, q);
    ccdQuatMul2(q, &a, q2);
}

_ccd_inline void ccdQuatMul2(ccd_quat_t *q,
                             const ccd_quat_t *a, const ccd_quat_t *b)
{
    q->q[0] = a->q[3] * b->q[0]
                + a->q[0] * b->q[3]
                + a->q[1] * b->q[2]
                - a->q[2] * b->q[1];
    q->q[1] = a->q[3] * b->q[1]
                + a->q[1] * b->q[3]
                - a->q[0] * b->q[2]
                + a->q[2] * b->q[0];
    q->q[2] = a->q[3] * b->q[2]
                + a->q[2] * b->q[3]
                + a->q[0] * b->q[1]
                - a->q[1] * b->q[0];
    q->q[3] = a->q[3] * b->q[3]
                - a->q[0] * b->q[0]
                - a->q[1] * b->q[1]
                - a->q[2] * b->q[2];
}

_ccd_inline int ccdQuatInvert(ccd_quat_t *q)
{
    ccd_real_t len2 = ccdQuatLen2(q);
    if (len2 < CCD_EPS)
        return -1;

    len2 = CCD_ONE / len2;

    q->q[0] = -q->q[0] * len2;
    q->q[1] = -q->q[1] * len2;
    q->q[2] = -q->q[2] * len2;
    q->q[3] = q->q[3] * len2;

    return 0;
}
_ccd_inline int ccdQuatInvert2(ccd_quat_t *dest, const ccd_quat_t *src)
{
    ccdQuatCopy(dest, src);
    return ccdQuatInvert(dest);
}

_ccd_inline void ccdQuatRotVec(ccd_vec3_t *v, const ccd_quat_t *q)
{
    ccd_real_t w, x, y, z, ww, xx, yy, zz, wx, wy, wz, xy, xz, yz;
    ccd_real_t vx, vy, vz;

    w = q->q[3];
    x = q->q[0];
    y = q->q[1];
    z = q->q[2];
    ww = w*w;
    xx = x*x;
    yy = y*y;
    zz = z*z;
    wx = w*x;
    wy = w*y;
    wz = w*z;
    xy = x*y;
    xz = x*z;
    yz = y*z;

    vx = ww * ccdVec3X(v)
            + xx * ccdVec3X(v)
            - yy * ccdVec3X(v)
            - zz * ccdVec3X(v)
            + 2 * ((xy - wz) * ccdVec3Y(v)
            + (xz + wy) * ccdVec3Z(v));
    vy = ww * ccdVec3Y(v)
            - xx * ccdVec3Y(v)
            + yy * ccdVec3Y(v)
            - zz * ccdVec3Y(v)
            + 2 * ((xy + wz) * ccdVec3X(v)
            + (yz - wx) * ccdVec3Z(v));
    vz = ww * ccdVec3Z(v)
            - xx * ccdVec3Z(v)
            - yy * ccdVec3Z(v)
            + zz * ccdVec3Z(v)
            + 2 * ((xz - wy) * ccdVec3X(v)
            + (yz + wx) * ccdVec3Y(v));
    ccdVec3Set(v, vx, vy, vz);
}

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __CCD_QUAT_H__ */
