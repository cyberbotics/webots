/****************************************************************************

blimp_physics -- A blimp physics model for Webots.

Copyright (C) 2006 Laboratory of Intelligent Systems, EPFL, Lausanne
Authors:    Alexis Guanella            guanella@ini.phys.ethz.ch
            Antoine Beyeler            antoine.beyeler@epfl.ch
            Jean-Christophe Zufferey   jean-christophe.zufferey@epfl.ch
            Dario Floreano             dario.floreano@epfl.ch
Web: http://lis.epfl.ch

The authors of any publication arising from research using this software are
kindly requested to add the following reference:

        Zufferey, J.C., Guanella, A., Beyeler, A., Floreano, D. (2006) Flying over
        the Reality Gap: From Simulated to Real Indoor Airships. Autonomous Robots,
        Springer US.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

******************************************************************************/
/*------------------------------------------------------------------------------

Author:		Alexis Guanella (ag)
                        Antoine Beyeler (ab)

------------------------------------------------------------------------------*/

#ifndef _UTILS_H
#define _UTILS_H

#include <ode/common.h>

//------------------------------------------------------------------------------
// Missing from <ode/common.h>

#if defined(dSINGLE)
#define dAsin(x) ((float)asinf(float(x)))
#elif defined(dDOUBLE)
#define dAsin(x) asin(x)
#else
#error You must #define dSINGLE or dDOUBLE
#endif

typedef dReal Vector6[6];
typedef dReal Matrix33[9];

//------------------------------------------------------------------------------
// Lin. Algebra

/* set a vector/matrix of size n to all zeros, or to a specific value. */
void utils_SetZero(dReal *a, int n);
void utils_SetValue(dReal *a, int n, dReal value);
void utils_Assign(dReal *a, const dReal *b, int n);
void utils_Add(dReal *a, const dReal *b, int n);

/* matrix multiplication. all matrices are stored in standard row format.
 * the digit refers to the argument that is transposed:
 *		A = B  * C   (sizes: A:p*r B:p*q C:q*r)
 */
void utils_Multiply(dReal *A, const dReal *B, const dReal *C, int p, int q, int r);

/* Matrix inversion */
void utils_InvertMatrix33(dReal *m);

#endif  // _UTILS_H
