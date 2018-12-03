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

#include "utils.h"

void utils_SetZero(dReal *a, int n) {
  while (n > 0) {
    *(a++) = 0;
    n--;
  }
}

void utils_SetValue(dReal *a, int n, dReal value) {
  while (n > 0) {
    *(a++) = value;
    n--;
  }
}
void utils_Assign(dReal *a, const dReal *b, int n) {
  while (n > 0) {
    *(a++) = *(b++);
    n--;
  }
}
void utils_Add(dReal *a, const dReal *b, int n) {
  while (n > 0) {
    *(a++) += *(b++);
    n--;
  }
}

void utils_Multiply(dReal *A, const dReal *B, const dReal *C, int p, int q, int r) {
  int i, j, k, qskip, rskip;
  dReal sum;
  const dReal *b, *c, *bb;
  qskip = q;
  rskip = r;

  bb = B;
  for (i = p; i; i--) {
    for (j = 0; j < r; j++) {
      c = C + j;
      b = bb;
      sum = 0;
      for (k = q; k; k--, c += rskip)
        sum += (*(b++)) * (*c);
      *(A++) = sum;
    }
    bb += qskip;
  }
}

void utils_InvertMatrix33(dReal *m) {
  dReal determinant =
    m[0] * m[4] * m[8] + m[1] * m[5] * m[6] + m[2] * m[3] * m[7] - m[6] * m[4] * m[2] - m[7] * m[5] * m[0] - m[8] * m[3] * m[1];

  dReal tmp[9];

  tmp[0] = (m[4] * m[8] - m[5] * m[7]) / determinant;
  tmp[1] = (m[2] * m[7] - m[1] * m[8]) / determinant;
  tmp[2] = (m[1] * m[5] - m[2] * m[4]) / determinant;
  tmp[3] = (m[5] * m[6] - m[3] * m[8]) / determinant;
  tmp[4] = (m[0] * m[8] - m[2] * m[6]) / determinant;
  tmp[5] = (m[2] * m[3] - m[0] * m[5]) / determinant;
  tmp[6] = (m[3] * m[7] - m[4] * m[6]) / determinant;
  tmp[7] = (m[1] * m[6] - m[0] * m[7]) / determinant;
  tmp[8] = (m[0] * m[4] - m[1] * m[3]) / determinant;

  utils_Assign(m, tmp, 9);
}
