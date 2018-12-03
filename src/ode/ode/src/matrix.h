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

/*
 * optimized and unoptimized vector and matrix functions
 * (inlined private versions)
 */

#ifndef _ODE__PRIVATE_MATRIX_H_
#define _ODE__PRIVATE_MATRIX_H_

#include <ode/matrix.h>

#ifdef __cplusplus

template <typename element_type>
ODE_INLINE
void _dSetZero (element_type *a, size_t n)
{
    element_type *acurr = a;
    element_type *const aend = a + n;
    while (acurr != aend) {
        *(acurr++) = 0;
    }
}

template <typename element_type>
ODE_INLINE
void _dSetValue (element_type *a, size_t n, element_type value)
{
    element_type *acurr = a;
    element_type *const aend = a + n;
    while (acurr != aend) {
        *(acurr++) = value;
    }
}

#else // #ifndef __cplusplus

ODE_PURE_INLINE
void _dSetZero (dReal *a, size_t n)
{
    dReal *acurr = a;
    dReal *const aend = a + n;
    while (acurr != aend) {
        *(acurr++) = 0;
    }
}

ODE_PURE_INLINE
void _dSetValue (dReal *a, size_t n, dReal value)
{
    dReal *acurr = a;
    dReal *const aend = a + n;
    while (acurr != aend) {
        *(acurr++) = value;
    }
}

#endif // #ifdef __cplusplus

#ifdef __cplusplus
extern "C" {
#endif

dReal _dDot (const dReal *a, const dReal *b, int n);
void _dMultiply0 (dReal *A, const dReal *B, const dReal *C, int p,int q,int r);
void _dMultiply1 (dReal *A, const dReal *B, const dReal *C, int p,int q,int r);
void _dMultiply2 (dReal *A, const dReal *B, const dReal *C, int p,int q,int r);
int _dFactorCholesky (dReal *A, int n, void *tmpbuf);
void _dSolveCholesky (const dReal *L, dReal *b, int n, void *tmpbuf);
int _dInvertPDMatrix (const dReal *A, dReal *Ainv, int n, void *tmpbuf);
int _dIsPositiveDefinite (const dReal *A, int n, void *tmpbuf);
void _dFactorLDLT (dReal *A, dReal *d, int n, int nskip);
void _dSolveL1 (const dReal *L, dReal *b, int n, int nskip);
void _dSolveL1T (const dReal *L, dReal *b, int n, int nskip);
void _dVectorScale (dReal *a, const dReal *d, int n);
void _dSolveLDLT (const dReal *L, const dReal *d, dReal *b, int n, int nskip);
void _dLDLTAddTL (dReal *L, dReal *d, const dReal *a, int n, int nskip, void *tmpbuf);
void _dLDLTRemove (dReal **A, const int *p, dReal *L, dReal *d, int n1, int n2, int r, int nskip, void *tmpbuf);
void _dRemoveRowCol (dReal *A, int n, int nskip, int r);

ODE_PURE_INLINE size_t _dEstimateFactorCholeskyTmpbufSize(int n)
{
    return dPAD(n) * sizeof(dReal);
}

ODE_PURE_INLINE size_t _dEstimateSolveCholeskyTmpbufSize(int n)
{
    return dPAD(n) * sizeof(dReal);
}

ODE_PURE_INLINE size_t _dEstimateInvertPDMatrixTmpbufSize(int n)
{
    size_t FactorCholesky_size = _dEstimateFactorCholeskyTmpbufSize(n);
    size_t SolveCholesky_size = _dEstimateSolveCholeskyTmpbufSize(n);
    size_t MaxCholesky_size = FactorCholesky_size > SolveCholesky_size ? FactorCholesky_size : SolveCholesky_size;
    return dPAD(n) * (n + 1) * sizeof(dReal) + MaxCholesky_size;
}

ODE_PURE_INLINE size_t _dEstimateIsPositiveDefiniteTmpbufSize(int n)
{
    return dPAD(n) * n * sizeof(dReal) + _dEstimateFactorCholeskyTmpbufSize(n);
}

ODE_PURE_INLINE size_t _dEstimateLDLTAddTLTmpbufSize(int nskip)
{
    return nskip * 2 * sizeof(dReal);
}

ODE_PURE_INLINE size_t _dEstimateLDLTRemoveTmpbufSize(int n2, int nskip)
{
    return n2 * sizeof(dReal) + _dEstimateLDLTAddTLTmpbufSize(nskip);
}

/* For internal use */
#define dSetZero(a, n) _dSetZero(a, n)
#define dSetValue(a, n, value) _dSetValue(a, n, value)
#define dDot(a, b, n) _dDot(a, b, n)
#define dMultiply0(A, B, C, p, q, r) _dMultiply0(A, B, C, p, q, r)
#define dMultiply1(A, B, C, p, q, r) _dMultiply1(A, B, C, p, q, r)
#define dMultiply2(A, B, C, p, q, r) _dMultiply2(A, B, C, p, q, r)
#define dFactorCholesky(A, n, tmpbuf) _dFactorCholesky(A, n, tmpbuf)
#define dSolveCholesky(L, b, n, tmpbuf) _dSolveCholesky(L, b, n, tmpbuf)
#define dInvertPDMatrix(A, Ainv, n, tmpbuf) _dInvertPDMatrix(A, Ainv, n, tmpbuf)
#define dIsPositiveDefinite(A, n, tmpbuf) _dIsPositiveDefinite(A, n, tmpbuf)
#define dFactorLDLT(A, d, n, nskip) _dFactorLDLT(A, d, n, nskip)
#define dSolveL1(L, b, n, nskip) _dSolveL1(L, b, n, nskip)
#define dSolveL1T(L, b, n, nskip) _dSolveL1T(L, b, n, nskip)
#define dVectorScale(a, d, n) _dVectorScale(a, d, n)
#define dSolveLDLT(L, d, b, n, nskip) _dSolveLDLT(L, d, b, n, nskip)
#define dLDLTAddTL(L, d, a, n, nskip, tmpbuf) _dLDLTAddTL(L, d, a, n, nskip, tmpbuf)
#define dLDLTRemove(A, p, L, d, n1, n2, r, nskip, tmpbuf) _dLDLTRemove(A, p, L, d, n1, n2, r, nskip, tmpbuf)
#define dRemoveRowCol(A, n, nskip, r) _dRemoveRowCol(A, n, nskip, r)

#define dEstimateFactorCholeskyTmpbufSize(n) _dEstimateFactorCholeskyTmpbufSize(n)
#define dEstimateSolveCholeskyTmpbufSize(n) _dEstimateSolveCholeskyTmpbufSize(n)
#define dEstimateInvertPDMatrixTmpbufSize(n) _dEstimateInvertPDMatrixTmpbufSize(n)
#define dEstimateIsPositiveDefiniteTmpbufSize(n) _dEstimateIsPositiveDefiniteTmpbufSize(n)
#define dEstimateLDLTAddTLTmpbufSize(nskip) _dEstimateLDLTAddTLTmpbufSize(nskip)
#define dEstimateLDLTRemoveTmpbufSize(n2, nskip) _dEstimateLDLTRemoveTmpbufSize(n2, nskip)

#ifdef __cplusplus
}
#endif

#endif // #ifndef _ODE__PRIVATE_MATRIX_H_
