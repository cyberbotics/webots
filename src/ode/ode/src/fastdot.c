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

/* generated code, do not edit. */

#include <ode/common.h>
#include "config.h"
#include "matrix.h"

dReal _dDot (const dReal *a, const dReal *b, int n)
{
    dReal p0,q0,m0,p1,q1,m1,sum;
    sum = 0;
    n -= 2;
    while (n >= 0) {
        p0 = a[0]; q0 = b[0];
        m0 = p0 * q0;
        p1 = a[1]; q1 = b[1];
        m1 = p1 * q1;
        sum += m0;
        sum += m1;
        a += 2;
        b += 2;
        n -= 2;
    }
    n += 2;
    while (n > 0) {
        sum += (*a) * (*b);
        a++;
        b++;
        n--;
    }
    return sum;
}

#undef dDot

dReal dDot (const dReal *a, const dReal *b, int n)
{
    return _dDot (a, b, n);
}
