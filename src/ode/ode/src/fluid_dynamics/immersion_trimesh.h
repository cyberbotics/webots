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

#ifndef _IMMERSION_TRIMESH_H_
#define _IMMERSION_TRIMESH_H_

#include <ode/common.h>
#include <ode/odemath.h>
#include <ode/matrix.h>
#include "error.h"

ODE_PURE_INLINE void computeTriangleSecondDegreeIntegrals(const dVector3 v, const dVector3 e1, const dVector3 e2, dMatrix3 integrals) {

  static const int indices[3] = { 1, 6, 2 };

  for (int i = 0; i < 3; ++i) {
    const dReal v0 = v[i];
    const dReal a1 = e1[i];
    const dReal a2 = e2[i];

    // x^2, y^2, z^2
    integrals[5 * i] = v0 * (0.5 * v0 + (a1 + a2) / 3.0) + (a1 * a1 + a1 * a2 + a2 * a2) / 12.0;

    const int j = (i + 1) % 3;
    const dReal w0 = v[j];
    const dReal b1 = e1[j];
    const dReal b2 = e2[j];

    // xy, yz, zx
    integrals[indices[i]] = 0.5 * v0 * w0 + (v0 * (b1 + b2) + w0 * (a1 + a2)) / 6.0 +
      (2.0 * (a1 * b1 + a2 * b2) + a1 * b2 + a2 * b1) / 24.0;
  }

  // Ensures symetry
  integrals[4] = integrals[1];
  integrals[8] = integrals[2];
  integrals[9] = integrals[6];
}

static void pickOrthonormalVector3(const dVector3 n, dVector3 u) {
  dAASSERT (n && u);

  dSetZero(u, 3);
  if (n[0] == 0.0) {
    u[0] = 1.0;
    return;
  } else if (n[1] == 0.0) {
    u[1] = 1.0;
    return;
  } else if (n[2] == 0.0) {
    u[2] = 1.0;
    return;
  }

  dReal a, k;
  if (dFabs(n[2]) > M_SQRT1_2) {
    // choose u in y-z plane
    a = n[1] * n[1] + n[2] * n[2];
    k = dRecipSqrt (a);
    u[1] = -n[2] * k;
    u[2] = n[1] * k;
  } else {
    // choose u in xy plane
    a = n[0] * n[0] + n[1] * n[1];
    k = dRecipSqrt (a);
    u[0] = -n[1] * k;
    u[1] = n[0] * k;
  }
}

static void computeTriangleGamma(const dVector3 u, const dMatrix3 integrals, dVector3 gamma) {
  gamma[0] = 0.5 * u[0] * ( integrals[0] - integrals[5] - integrals[10]) + u[1] * integrals[1] + u[2] * integrals[2];
  gamma[1] = 0.5 * u[1] * (integrals[0] + integrals[5] - integrals[10]) + u[0] * integrals[1] + u[2] * integrals[6];
  gamma[2] = 0.5 * u[2] * (-integrals[0] - integrals[5] + integrals[10]) + u[0] * integrals[2] + u[1] * integrals[6];
}

#endif
