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

#ifndef _ODE_ODEMATH_FLUID_DYNAMICS_H_
#define _ODE_ODEMATH_FLUID_DYNAMICS_H_

#include <ode/common.h>
#include <ode/odemath.h>

ODE_PURE_INLINE bool dValidVector3(const dVector3 v) {
  return !(isnan(v[0]) || isnan(v[1]) || isnan(v[2]));
}

ODE_PURE_INLINE void transformVectors3(dVector3 *array, int size, const dReal *rotation, const dReal *translation) {
  for (int i = 0; i < size; ++i) {
    dReal *const position = array[i];
    dMultiplyHelper0_331(position, rotation, position);
    dAddVectors3(position, position, translation);
  }
}

template<class T, dReal (T::*integrand)(dReal) const>
dReal simpson(dReal start, dReal end, int numberOfSubdivisions, const T &object) {
  const dReal stepSize = (end - start) / numberOfSubdivisions;
  dReal a = stepSize + start;
  const dReal halfStepSize = 0.5 * stepSize;
  dReal m = start + halfStepSize;
  dReal result = (object.*integrand)(start) + (object.*integrand)(end) + 4.0 * (object.*integrand)(end - halfStepSize);
  for (int i = 1; i < numberOfSubdivisions; a += stepSize, m += stepSize, ++i)
    result += 4.0 * (object.*integrand)(m) + 2.0 * (object.*integrand)(a);

  return (1.0 / 6.0) * stepSize * result;
}

#endif
