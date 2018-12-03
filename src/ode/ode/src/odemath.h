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

#ifndef _ODE__PRIVATE_ODEMATH_H_
#define _ODE__PRIVATE_ODEMATH_H_

#include <ode/odemath.h>
#include "error.h"

int  _dSafeNormalize3 (dVector3 a);
int  _dSafeNormalize4 (dVector4 a);

ODE_PURE_INLINE void _dNormalize3(dVector3 a)
{
    int bNormalizationResult = _dSafeNormalize3(a);
    dIVERIFY(bNormalizationResult);
}

ODE_PURE_INLINE void _dNormalize4(dVector4 a)
{
    int bNormalizationResult = _dSafeNormalize4(a);
    dIVERIFY(bNormalizationResult);
}

// For internal use
#define dSafeNormalize3(a) _dSafeNormalize3(a)
#define dSafeNormalize4(a) _dSafeNormalize4(a)
#define dNormalize3(a) _dNormalize3(a)
#define dNormalize4(a) _dNormalize4(a)

#endif
