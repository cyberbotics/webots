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

#ifndef _IMMERSION_BOX_H_
#define _IMMERSION_BOX_H_

#include <ode/odemath.h>
#include "../collision_std.h"
#include <ode/matrix.h>
#include "../collision_trimesh_internal.h"
#include <ode/fluid_dynamics/immersion.h>

ODE_PURE_INLINE void projectAreas(dVector3 areas, const dReal *bodyRotation, const dReal *geomRotation) {
  dMatrix3 m;
  dMultiply1(m, bodyRotation, geomRotation, 3, 3, 3);
  dReal s[3] = { 0.0, 0.0, 0.0 };
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const int k = 4 * i;
      s[i] += dFabs(m[j + k]) * areas[j];
    }
  }

  areas[0] = s[0];
  areas[1] = s[1];
  areas[2] = s[2];
}

struct dImmersedVertex {
  dVector3 pos;
  dReal depth;
};

struct sBoxImmersionData {
  sBoxImmersionData(dxBox *box, const dReal *fluidPlane, dImmersionGeom *immersion, int flags) :
    m_vCenter(box->final_posr->pos),
    m_mRotation(box->final_posr->R),
    m_vSide(box->side),
    m_gImmersion(immersion),
    m_vBuoyancyCenter(immersion->buoyancyCenter),
    m_nFlags(flags),
    m_nNumberOfImmersedVertices(0),
    m_bComplement(false)
  {
    dMultiplyHelper1_331(m_vN, m_mRotation, fluidPlane);
    dCopyScaledVector3(m_vHalfSide, m_vSide, 0.5);

    const dVector3 s = { m_vN[0] * m_vHalfSide[0], m_vN[1] * m_vHalfSide[1], m_vN[2] * m_vHalfSide[2] };
    const dReal boxCenterDepth = fluidPlane[3] - dCalcVectorDot3(m_vCenter, fluidPlane);

    int vertexCount = 0;
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
        for (int k = 0; k < 2; ++k) {
          dVector3 v = { i ? 1.0 : -1.0, j ? 1.0 : -1.0, k ? 1.0 : -1.0 };
          const dReal depth = boxCenterDepth - dCalcVectorDot3(v, s);
          dCopyVector3(m_vImmersedVertices[vertexCount].pos, v);
          m_vImmersedVertices[vertexCount].depth = depth;
          if (depth > 0.0)
            ++m_nNumberOfImmersedVertices;
          ++vertexCount;
        }
      }
    }
  }

  int performImmersionChecking();
  void oneImmersedVertex();
  void twoImmersedVertices();
  void threeImmersedVertices();
  void fourImmersedVertices();

  const dReal *m_vCenter, *m_mRotation, *m_vSide;
  dImmersionGeom *m_gImmersion;
  dReal *m_vBuoyancyCenter;
  int m_nFlags;
  int m_nNumberOfImmersedVertices;
  int m_vIndices[8];
  dVector3 m_vN, m_vHalfSide;
  dImmersedVertex m_vImmersedVertices[8];
  bool m_bComplement;
};

#endif
