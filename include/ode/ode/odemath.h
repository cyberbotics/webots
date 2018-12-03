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

#ifndef _ODE_ODEMATH_H_
#define _ODE_ODEMATH_H_

#include <ode/common.h>

/*
 * macro to access elements i,j in an NxM matrix A, independent of the
 * matrix storage convention.
 */
#define dACCESS33(A,i,j) ((A)[(i)*4+(j)])

/*
 * Macro to test for valid floating point values
 */
#define dVALIDVEC3(v) (!(dIsNan(v[0]) || dIsNan(v[1]) || dIsNan(v[2])))
#define dVALIDVEC4(v) (!(dIsNan(v[0]) || dIsNan(v[1]) || dIsNan(v[2]) || dIsNan(v[3])))
#define dVALIDMAT3(m) (!(dIsNan(m[0]) || dIsNan(m[1]) || dIsNan(m[2]) || dIsNan(m[3]) || dIsNan(m[4]) || dIsNan(m[5]) || dIsNan(m[6]) || dIsNan(m[7]) || dIsNan(m[8]) || dIsNan(m[9]) || dIsNan(m[10]) || dIsNan(m[11])))
#define dVALIDMAT4(m) (!(dIsNan(m[0]) || dIsNan(m[1]) || dIsNan(m[2]) || dIsNan(m[3]) || dIsNan(m[4]) || dIsNan(m[5]) || dIsNan(m[6]) || dIsNan(m[7]) || dIsNan(m[8]) || dIsNan(m[9]) || dIsNan(m[10]) || dIsNan(m[11]) || dIsNan(m[12]) || dIsNan(m[13]) || dIsNan(m[14]) || dIsNan(m[15]) ))

/* Some vector math */
ODE_PURE_INLINE void dAddVectors3(dReal *res, const dReal *a, const dReal *b)
{
  const dReal res_0 = a[0] + b[0];
  const dReal res_1 = a[1] + b[1];
  const dReal res_2 = a[2] + b[2];
  /* Only assign after all the calculations are over to avoid incurring memory aliasing*/
  res[0] = res_0; res[1] = res_1; res[2] = res_2;
}

ODE_PURE_INLINE void dSubtractVectors3(dReal *res, const dReal *a, const dReal *b)
{
  const dReal res_0 = a[0] - b[0];
  const dReal res_1 = a[1] - b[1];
  const dReal res_2 = a[2] - b[2];
  /* Only assign after all the calculations are over to avoid incurring memory aliasing*/
  res[0] = res_0; res[1] = res_1; res[2] = res_2;
}

ODE_PURE_INLINE void dAddScaledVectors3(dReal *res, const dReal *a, const dReal *b, dReal a_scale, dReal b_scale)
{
  const dReal res_0 = a_scale * a[0] + b_scale * b[0];
  const dReal res_1 = a_scale * a[1] + b_scale * b[1];
  const dReal res_2 = a_scale * a[2] + b_scale * b[2];
  /* Only assign after all the calculations are over to avoid incurring memory aliasing*/
  res[0] = res_0; res[1] = res_1; res[2] = res_2;
}

ODE_PURE_INLINE void dScaleVector3(dReal *res, dReal nScale)
{
  res[0] *= nScale ;
  res[1] *= nScale ;
  res[2] *= nScale ;
}

ODE_PURE_INLINE void dNegateVector3(dReal *res)
{
  res[0] = -res[0];
  res[1] = -res[1];
  res[2] = -res[2];
}

ODE_PURE_INLINE void dCopyVector3(dReal *res, const dReal *a)
{
  const dReal res_0 = a[0];
  const dReal res_1 = a[1];
  const dReal res_2 = a[2];
  /* Only assign after all the calculations are over to avoid incurring memory aliasing*/
  res[0] = res_0; res[1] = res_1; res[2] = res_2;
}

ODE_PURE_INLINE void dCopyScaledVector3(dReal *res, const dReal *a, dReal nScale)
{
  const dReal res_0 = a[0] * nScale;
  const dReal res_1 = a[1] * nScale;
  const dReal res_2 = a[2] * nScale;
  /* Only assign after all the calculations are over to avoid incurring memory aliasing*/
  res[0] = res_0; res[1] = res_1; res[2] = res_2;
}

ODE_PURE_INLINE void dCopyNegatedVector3(dReal *res, const dReal *a)
{
  const dReal res_0 = -a[0];
  const dReal res_1 = -a[1];
  const dReal res_2 = -a[2];
  /* Only assign after all the calculations are over to avoid incurring memory aliasing*/
  res[0] = res_0; res[1] = res_1; res[2] = res_2;
}

ODE_PURE_INLINE void dCopyVector4(dReal *res, const dReal *a)
{
  const dReal res_0 = a[0];
  const dReal res_1 = a[1];
  const dReal res_2 = a[2];
  const dReal res_3 = a[3];
  /* Only assign after all the calculations are over to avoid incurring memory aliasing*/
  res[0] = res_0; res[1] = res_1; res[2] = res_2; res[3] = res_3;
}

ODE_PURE_INLINE void dCopyMatrix4x4(dReal *res, const dReal *a)
{
  dCopyVector4(res + 0, a + 0);
  dCopyVector4(res + 4, a + 4);
  dCopyVector4(res + 8, a + 8);
}

ODE_PURE_INLINE void dCopyMatrix4x3(dReal *res, const dReal *a)
{
  dCopyVector3(res + 0, a + 0);
  dCopyVector3(res + 4, a + 4);
  dCopyVector3(res + 8, a + 8);
}

ODE_PURE_INLINE void dGetMatrixColumn3(dReal *res, const dReal *a, unsigned n)
{
  const dReal res_0 = a[n + 0];
  const dReal res_1 = a[n + 4];
  const dReal res_2 = a[n + 8];
  /* Only assign after all the calculations are over to avoid incurring memory aliasing*/
  res[0] = res_0; res[1] = res_1; res[2] = res_2;
}

ODE_PURE_INLINE dReal dCalcVectorLength3(const dReal *a)
{
  return dSqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

ODE_PURE_INLINE dReal dCalcVectorLengthSquare3(const dReal *a)
{
  return (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

ODE_PURE_INLINE dReal dCalcPointDepth3(const dReal *test_p, const dReal *plane_p, const dReal *plane_n)
{
  return (plane_p[0] - test_p[0]) * plane_n[0] + (plane_p[1] - test_p[1]) * plane_n[1] + (plane_p[2] - test_p[2]) * plane_n[2];
}

ODE_PURE_INLINE void make_sure_plane_normal_has_unit_length(dReal *plane_p)
{
  dReal l = plane_p[0]*plane_p[0] + plane_p[1]*plane_p[1] + plane_p[2]*plane_p[2];
  if (l > 0) {
    l = dRecipSqrt(l);
    plane_p[0] *= l;
    plane_p[1] *= l;
    plane_p[2] *= l;
    plane_p[3] *= l;
  }
  else {
    plane_p[0] = 1;
    plane_p[1] = 0;
    plane_p[2] = 0;
    plane_p[3] = 0;
  }
}

/*
* 3-way dot product. _dCalcVectorDot3 means that elements of `a' and `b' are spaced
* step_a and step_b indexes apart respectively. dCalcVectorDot3() means dDot311.
*/

ODE_PURE_INLINE dReal _dCalcVectorDot3(const dReal *a, const dReal *b, unsigned step_a, unsigned step_b)
{
  return a[0] * b[0] + a[step_a] * b[step_b] + a[2 * step_a] * b[2 * step_b];
}

ODE_PURE_INLINE dReal dCalcVectorDot3    (const dReal *a, const dReal *b) { return _dCalcVectorDot3(a,b,1,1); }
ODE_PURE_INLINE dReal dCalcVectorDot3_13 (const dReal *a, const dReal *b) { return _dCalcVectorDot3(a,b,1,3); }
ODE_PURE_INLINE dReal dCalcVectorDot3_31 (const dReal *a, const dReal *b) { return _dCalcVectorDot3(a,b,3,1); }
ODE_PURE_INLINE dReal dCalcVectorDot3_33 (const dReal *a, const dReal *b) { return _dCalcVectorDot3(a,b,3,3); }
ODE_PURE_INLINE dReal dCalcVectorDot3_14 (const dReal *a, const dReal *b) { return _dCalcVectorDot3(a,b,1,4); }
ODE_PURE_INLINE dReal dCalcVectorDot3_41 (const dReal *a, const dReal *b) { return _dCalcVectorDot3(a,b,4,1); }
ODE_PURE_INLINE dReal dCalcVectorDot3_44 (const dReal *a, const dReal *b) { return _dCalcVectorDot3(a,b,4,4); }

/*
 * cross product, set res = a x b. _dCalcVectorCross3 means that elements of `res', `a'
 * and `b' are spaced step_res, step_a and step_b indexes apart respectively.
 * dCalcVectorCross3() means dCross3111.
 */

ODE_PURE_INLINE void _dCalcVectorCross3(dReal *res, const dReal *a, const dReal *b, unsigned step_res, unsigned step_a, unsigned step_b)
{
  const dReal res_0 = a[  step_a]*b[2*step_b] - a[2*step_a]*b[  step_b];
  const dReal res_1 = a[2*step_a]*b[       0] - a[       0]*b[2*step_b];
  const dReal res_2 = a[       0]*b[  step_b] - a[  step_a]*b[       0];
  /* Only assign after all the calculations are over to avoid incurring memory aliasing*/
  res[         0] = res_0;
  res[  step_res] = res_1;
  res[2*step_res] = res_2;
}

ODE_PURE_INLINE void dCalcVectorCross3    (dReal *res, const dReal *a, const dReal *b) { _dCalcVectorCross3(res, a, b, 1, 1, 1); }
ODE_PURE_INLINE void dCalcVectorCross3_114(dReal *res, const dReal *a, const dReal *b) { _dCalcVectorCross3(res, a, b, 1, 1, 4); }
ODE_PURE_INLINE void dCalcVectorCross3_141(dReal *res, const dReal *a, const dReal *b) { _dCalcVectorCross3(res, a, b, 1, 4, 1); }
ODE_PURE_INLINE void dCalcVectorCross3_144(dReal *res, const dReal *a, const dReal *b) { _dCalcVectorCross3(res, a, b, 1, 4, 4); }
ODE_PURE_INLINE void dCalcVectorCross3_411(dReal *res, const dReal *a, const dReal *b) { _dCalcVectorCross3(res, a, b, 4, 1, 1); }
ODE_PURE_INLINE void dCalcVectorCross3_414(dReal *res, const dReal *a, const dReal *b) { _dCalcVectorCross3(res, a, b, 4, 1, 4); }
ODE_PURE_INLINE void dCalcVectorCross3_441(dReal *res, const dReal *a, const dReal *b) { _dCalcVectorCross3(res, a, b, 4, 4, 1); }
ODE_PURE_INLINE void dCalcVectorCross3_444(dReal *res, const dReal *a, const dReal *b) { _dCalcVectorCross3(res, a, b, 4, 4, 4); }

ODE_PURE_INLINE void dAddVectorCross3(dReal *res, const dReal *a, const dReal *b)
{
  dReal tmp[3];
  dCalcVectorCross3(tmp, a, b);
  dAddVectors3(res, res, tmp);
}

ODE_PURE_INLINE void dSubtractVectorCross3(dReal *res, const dReal *a, const dReal *b)
{
  dReal tmp[3];
  dCalcVectorCross3(tmp, a, b);
  dSubtractVectors3(res, res, tmp);
}

/*
 * set a 3x3 submatrix of A to a matrix such that submatrix(A)*b = a x b.
 * A is stored by rows, and has `skip' elements per row. the matrix is
 * assumed to be already zero, so this does not write zero elements!
 * if (plus,minus) is (+,-) then a positive version will be written.
 * if (plus,minus) is (-,+) then a negative version will be written.
 */

ODE_PURE_INLINE void dSetCrossMatrixPlus(dReal *res, const dReal *a, unsigned skip)
{
  const dReal a_0 = a[0], a_1 = a[1], a_2 = a[2];
  res[1] = -a_2;
  res[2] = +a_1;
  res[skip+0] = +a_2;
  res[skip+2] = -a_0;
  res[2*skip+0] = -a_1;
  res[2*skip+1] = +a_0;
}

ODE_PURE_INLINE void dSetCrossMatrixMinus(dReal *res, const dReal *a, unsigned skip)
{
  const dReal a_0 = a[0], a_1 = a[1], a_2 = a[2];
  res[1] = +a_2;
  res[2] = -a_1;
  res[skip+0] = -a_2;
  res[skip+2] = +a_0;
  res[2*skip+0] = +a_1;
  res[2*skip+1] = -a_0;
}

/*
 * compute the distance between two 3D-vectors
 */

ODE_PURE_INLINE dReal dCalcPointsDistance3(const dReal *a, const dReal *b)
{
  dReal res;
  dReal tmp[3];
  dSubtractVectors3(tmp, a, b);
  res = dCalcVectorLength3(tmp);
  return res;
}

/*
 * special case matrix multiplication, with operator selection
 */

ODE_PURE_INLINE void dMultiplyHelper0_331(dReal *res, const dReal *a, const dReal *b)
{
  const dReal res_0 = dCalcVectorDot3(a, b);
  const dReal res_1 = dCalcVectorDot3(a + 4, b);
  const dReal res_2 = dCalcVectorDot3(a + 8, b);
  /* Only assign after all the calculations are over to avoid incurring memory aliasing*/
  res[0] = res_0; res[1] = res_1; res[2] = res_2;
}

ODE_PURE_INLINE void dMultiplyHelper1_331(dReal *res, const dReal *a, const dReal *b)
{
  const dReal res_0 = dCalcVectorDot3_41(a, b);
  const dReal res_1 = dCalcVectorDot3_41(a + 1, b);
  const dReal res_2 = dCalcVectorDot3_41(a + 2, b);
  /* Only assign after all the calculations are over to avoid incurring memory aliasing*/
  res[0] = res_0; res[1] = res_1; res[2] = res_2;
}

ODE_PURE_INLINE void dMultiplyHelper0_133(dReal *res, const dReal *a, const dReal *b)
{
  dMultiplyHelper1_331(res, b, a);
}

ODE_PURE_INLINE void dMultiplyHelper1_133(dReal *res, const dReal *a, const dReal *b)
{
  const dReal res_0 = dCalcVectorDot3_44(a, b);
  const dReal res_1 = dCalcVectorDot3_44(a + 1, b);
  const dReal res_2 = dCalcVectorDot3_44(a + 2, b);
  /* Only assign after all the calculations are over to avoid incurring memory aliasing*/
  res[0] = res_0; res[1] = res_1; res[2] = res_2;
}

/*
Note: NEVER call any of these functions/macros with the same variable for A and C,
it is not equivalent to A*=B.
*/

ODE_PURE_INLINE void dMultiply0_331(dReal *res, const dReal *a, const dReal *b)
{
  dMultiplyHelper0_331(res, a, b);
}

ODE_PURE_INLINE void dMultiply1_331(dReal *res, const dReal *a, const dReal *b)
{
  dMultiplyHelper1_331(res, a, b);
}

ODE_PURE_INLINE void dMultiply0_133(dReal *res, const dReal *a, const dReal *b)
{
  dMultiplyHelper0_133(res, a, b);
}

ODE_PURE_INLINE void dMultiply0_333(dReal *res, const dReal *a, const dReal *b)
{
  dMultiplyHelper0_133(res + 0, a + 0, b);
  dMultiplyHelper0_133(res + 4, a + 4, b);
  dMultiplyHelper0_133(res + 8, a + 8, b);
}

ODE_PURE_INLINE void dMultiply1_333(dReal *res, const dReal *a, const dReal *b)
{
  dMultiplyHelper1_133(res + 0, b, a + 0);
  dMultiplyHelper1_133(res + 4, b, a + 1);
  dMultiplyHelper1_133(res + 8, b, a + 2);
}

ODE_PURE_INLINE void dMultiply2_333(dReal *res, const dReal *a, const dReal *b)
{
  dMultiplyHelper0_331(res + 0, b, a + 0);
  dMultiplyHelper0_331(res + 4, b, a + 4);
  dMultiplyHelper0_331(res + 8, b, a + 8);
}

ODE_PURE_INLINE void dMultiplyAdd0_331(dReal *res, const dReal *a, const dReal *b)
{
  dReal tmp[3];
  dMultiplyHelper0_331(tmp, a, b);
  dAddVectors3(res, res, tmp);
}

ODE_PURE_INLINE void dMultiplyAdd1_331(dReal *res, const dReal *a, const dReal *b)
{
  dReal tmp[3];
  dMultiplyHelper1_331(tmp, a, b);
  dAddVectors3(res, res, tmp);
}

ODE_PURE_INLINE void dMultiplyAdd0_133(dReal *res, const dReal *a, const dReal *b)
{
  dReal tmp[3];
  dMultiplyHelper0_133(tmp, a, b);
  dAddVectors3(res, res, tmp);
}

ODE_PURE_INLINE void dMultiplyAdd0_333(dReal *res, const dReal *a, const dReal *b)
{
  dReal tmp[3];
  dMultiplyHelper0_133(tmp, a + 0, b);
  dAddVectors3(res+ 0, res + 0, tmp);
  dMultiplyHelper0_133(tmp, a + 4, b);
  dAddVectors3(res + 4, res + 4, tmp);
  dMultiplyHelper0_133(tmp, a + 8, b);
  dAddVectors3(res + 8, res + 8, tmp);
}

ODE_PURE_INLINE void dMultiplyAdd1_333(dReal *res, const dReal *a, const dReal *b)
{
  dReal tmp[3];
  dMultiplyHelper1_133(tmp, b, a + 0);
  dAddVectors3(res + 0, res + 0, tmp);
  dMultiplyHelper1_133(tmp, b, a + 1);
  dAddVectors3(res + 4, res + 4, tmp);
  dMultiplyHelper1_133(tmp, b, a + 2);
  dAddVectors3(res + 8, res + 8, tmp);
}

ODE_PURE_INLINE void dMultiplyAdd2_333(dReal *res, const dReal *a, const dReal *b)
{
  dReal tmp[3];
  dMultiplyHelper0_331(tmp, b, a + 0);
  dAddVectors3(res + 0, res + 0, tmp);
  dMultiplyHelper0_331(tmp, b, a + 4);
  dAddVectors3(res + 4, res + 4, tmp);
  dMultiplyHelper0_331(tmp, b, a + 8);
  dAddVectors3(res + 8, res + 8, tmp);
}

ODE_PURE_INLINE dReal dCalcMatrix3Det( const dReal* mat )
{
    dReal det;

    det = mat[0] * ( mat[5]*mat[10] - mat[9]*mat[6] )
        - mat[1] * ( mat[4]*mat[10] - mat[8]*mat[6] )
        + mat[2] * ( mat[4]*mat[9]  - mat[8]*mat[5] );

    return( det );
}

/**
  Closed form matrix inversion, copied from
  collision_util.h for use in the stepper.

  Returns the determinant.
  returns 0 and does nothing
  if the matrix is singular.
*/
ODE_PURE_INLINE dReal dInvertMatrix3(dReal *dst, const dReal *ma)
{
    dReal det;
    dReal detRecip;

    det = dCalcMatrix3Det( ma );

    /* Setting an arbitrary non-zero threshold
       for the determinant doesn't do anyone
       any favors.  The condition number is the
       important thing.  If all the eigen-values
       of the matrix are small, so is the
       determinant, but it can still be well
       conditioned.
       A single extremely large eigen-value could
       push the determinant over threshold, but
       produce a very unstable result if the other
       eigen-values are small.  So we just say that
       the determinant must be non-zero and trust the
       caller to provide well-conditioned matrices.
       */
    if ( det == 0 )
    {
        return 0;
    }

    detRecip = dRecip(det);

    dst[0] =  ( ma[5]*ma[10] - ma[6]*ma[9]  ) * detRecip;
    dst[1] =  ( ma[9]*ma[2]  - ma[1]*ma[10] ) * detRecip;
    dst[2] =  ( ma[1]*ma[6]  - ma[5]*ma[2]  ) * detRecip;

    dst[4] =  ( ma[6]*ma[8]  - ma[4]*ma[10] ) * detRecip;
    dst[5] =  ( ma[0]*ma[10] - ma[8]*ma[2]  ) * detRecip;
    dst[6] =  ( ma[4]*ma[2]  - ma[0]*ma[6]  ) * detRecip;

    dst[8] =  ( ma[4]*ma[9]  - ma[8]*ma[5]  ) * detRecip;
    dst[9] =  ( ma[8]*ma[1]  - ma[0]*ma[9]  ) * detRecip;
    dst[10] = ( ma[0]*ma[5]  - ma[1]*ma[4]  ) * detRecip;

    return det;
}

/* Include legacy macros here */
#include <ode/odemath_legacy.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * normalize 3x1 and 4x1 vectors (i.e. scale them to unit length)
 */

/* For DLL export*/
ODE_API int  dSafeNormalize3 (dVector3 a);
ODE_API int  dSafeNormalize4 (dVector4 a);
ODE_API void dNormalize3 (dVector3 a); /* Potentially asserts on zero vec*/
ODE_API void dNormalize4 (dVector4 a); /* Potentially asserts on zero vec*/

/*
 * given a unit length "normal" vector n, generate vectors p and q vectors
 * that are an orthonormal basis for the plane space perpendicular to n.
 * i.e. this makes p,q such that n,p,q are all perpendicular to each other.
 * q will equal n x p. if n is not unit length then p will be unit length but
 * q wont be.
 */

ODE_API void dPlaneSpace (const dVector3 n, dVector3 p, dVector3 q);
/* Makes sure the matrix is a proper rotation */
ODE_API void dOrthogonalizeR(dMatrix3 m);

#ifdef __cplusplus
}
#endif

#endif
