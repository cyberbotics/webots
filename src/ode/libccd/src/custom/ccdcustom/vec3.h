/***
 * libccd
 * ---------------------------------
 * Copyright (c)2010,2011 Daniel Fiser <danfis@danfis.cz>
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

#ifndef __CCD_CUSTOM_VEC3_H__
#define __CCD_CUSTOM_VEC3_H__

#include <ccd/vec3.h>


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


#ifdef CCD_SINGLE
# define CCD_ATAN2(x, y) (atan2f((x), (y))) /*!< atan2 of two floats */
#endif /* CCD_SINGLE */

#ifdef CCD_DOUBLE
# define CCD_ATAN2(x, y) (atan2((x), (y))) /*!< atan2 of two floats */
#endif /* CCD_DOUBLE */

/**
 * d = v + w
 */
_ccd_inline void ccdVec3Add2(ccd_vec3_t *d, const ccd_vec3_t *v, const ccd_vec3_t *w);

/**
 * d = s * k;
 */
_ccd_inline void ccdVec3CopyScaled(ccd_vec3_t *d, const ccd_vec3_t *s, ccd_real_t k);

/**
 * d = v + s * k;
 */
_ccd_inline void ccdVec3AddScaled(ccd_vec3_t *d, const ccd_vec3_t *v, const ccd_vec3_t *s, ccd_real_t k);


/**
 * Normalizes given vector to unit length.
 */
_ccd_inline int ccdVec3SafeNormalize(ccd_vec3_t *d);


_ccd_inline void ccdVec3Add2(ccd_vec3_t *d, const ccd_vec3_t *v, const ccd_vec3_t *w)
{
#ifndef dLIBCCD_USE_SYSTEM
    d->v[0] = v->v[0] + w->v[0];
    d->v[1] = v->v[1] + w->v[1];
    d->v[2] = v->v[2] + w->v[2];
#else
    ccdVec3Copy(d, v);
    ccdVec3Add(d, w);
#endif
}

_ccd_inline void ccdVec3CopyScaled(ccd_vec3_t *d, const ccd_vec3_t *s, ccd_real_t k)
{
#ifndef dLIBCCD_USE_SYSTEM
    d->v[0] = s->v[0] * k;
    d->v[1] = s->v[1] * k;
    d->v[2] = s->v[2] * k;
#else
    ccdVec3Copy(d, s);
    ccdVec3Scale(d, k);
#endif
}

_ccd_inline void ccdVec3AddScaled(ccd_vec3_t *d, const ccd_vec3_t *v, const ccd_vec3_t *s, ccd_real_t k)
{
#ifndef dLIBCCD_USE_SYSTEM
    d->v[0] = v->v[0] + s->v[0] * k;
    d->v[1] = v->v[1] + s->v[1] * k;
    d->v[2] = v->v[2] + s->v[2] * k;
#else
    ccdVec3Copy(d, s);
    ccdVec3Scale(d, k);
    ccdVec3Add(d, v);
#endif
}

_ccd_inline int ccdVec3SafeNormalize(ccd_vec3_t *d)
{
    int result = -1;

    ccd_real_t len = CCD_SQRT(ccdVec3Len2(d));
    if (len >= CCD_EPS) {
        ccdVec3Scale(d, CCD_ONE / len);
        result = 0;
    }

    return result;
}


#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __CCD_CUSTOM_VEC3_H__ */