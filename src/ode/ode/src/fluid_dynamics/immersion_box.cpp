/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
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

standard ODE geometry primitives: public API and pairwise immersion functions.

the rule is that only the low level primitive immersion functions should set
dImmersionGeom::g1 and dImmersionGeom::g2.

*/

#include "immersion_box.h"

#include <ode/objects.h>
#include "../collision_kernel.h"
#include <ode/fluid_dynamics/fluid_dynamics.h>
#include <ode/fluid_dynamics/odemath_fluid_dynamics.h>
#include "immersion_kernel.h"
#include "immersion_outline.h"
#include "immersion_std.h"

#include <algorithm>

#ifdef _MSC_VER
#pragma warning(disable:4291)  // for VC++, no complaints about "no matching operator delete found"
#endif

dReal dGeomBoxGetVolume (dGeomID g)
 {
   dUASSERT (g && g->type == dBoxClass,"argument not a box");

   const dReal *const s = ((dxBox*) g)->side;
   return s[0] * s[1] * s[2];
 }

dReal dGeomBoxGetArea (dGeomID g)
 {
   dUASSERT (g && g->type == dBoxClass,"argument not a box");

   const dReal *const s = ((dxBox*) g)->side;
   return 2.0 * (s[0] * (s[1] + s[2]) + s[1] * s[2]);
 }

void dGeomBoxGetTangentPlane (dGeomID g, dReal x, dReal y, dReal z, dVector4 plane)
 {
   dUASSERT (g && g->type == dBoxClass,"argument not a box");

   const dxBox *const b = (dxBox*) g;
   const dReal *const center = b->final_posr->pos;
   dVector3 p = { x - center[0], y - center[1], z - center[2] };
   const dReal *const R = b->final_posr->R;
   const dReal *const side = b->side;

   static const dReal ZERO_THRESHOLD = 1e-3;
   dReal sign = 0.0;
   int i;
   for (i = 0; i < 3; ++i) {
     const dReal l = dCalcVectorDot3_41(R + i, p);
     if (dFabs(l - 0.5 * side[i]) < ZERO_THRESHOLD) {
       sign = 1.0;
       break;
     } else if (dFabs(l + 0.5 * side[i]) < ZERO_THRESHOLD) {
       sign = -1.0;
       break;
     }
   }

   plane[0] =  sign * R[i];
   plane[1] =  sign * R[i + 4];
   plane[2] =  sign * R[i + 8];
   plane[3] = x * plane[0] + y * plane[1] + z * plane[2];
 }

void dGeomBoxGetFacePlane (dGeomID g, int faceIndex, dVector4 plane)
 {
   dUASSERT (g && g->type == dBoxClass,"argument not a box");
   dUASSERT(abs(faceIndex) <= 3, "Invalid face index: range = [-3, 3]");

   const dxBox *const b = (dxBox*) g;
   const dReal *const center = b->final_posr->pos;
   const dReal *const R = b->final_posr->R;
   const dReal sign = faceIndex >= 0 ? 1.0 : -1.0;
   const int i = abs(faceIndex) - 1;
   plane[0] =  sign * R[i];
   plane[1] =  sign * R[i + 4];
   plane[2] =  sign * R[i + 8];
   plane[3] = dCalcVectorDot3(plane, center) + 0.5 * b->side[i];
 }

void dGeomBoxGetImmersionPlane (dGeomID g, dVector4 plane)
 {
  dGeomBoxGetFacePlane(g, FLUID_PLANE_NORMAL + 1, plane);
 }

void sBoxImmersionData::oneImmersedVertex() {
  const dImmersedVertex *const c = m_vImmersedVertices + m_vIndices[0];
  const dReal depthSquare = c->depth * c->depth;
  const dReal depthCube = depthSquare * c->depth;
  m_gImmersion->volume = depthCube / (m_vN[0] * m_vN[1] * m_vN[2] * 6.0);
  const dReal offset = 0.25 * c->depth;
  for (int i = 0; i < 3; ++i) {
    m_vBuoyancyCenter[i] = c->pos[i] * (m_vHalfSide[i] - offset / m_vN[i]);
    m_gImmersion->projectedAreas[i] = depthSquare / (m_vN[(i + 1) % 3] * m_vN[(i + 2) % 3]);
  }

  // immersion outline
  if (m_gImmersion->outline && (m_nFlags & ~dxImmersionOutlineDisabled))  {
    dVector3 d[3];
    const dReal *const pos = c->pos;
    for (int i = 0; i < 3; ++i)
      d[0][i] = pos[i] * m_vHalfSide[i];
    dCopyVector3(d[1], d[0]);
    dCopyVector3(d[2], d[0]);

    const dReal depth = c->depth;
    for (int i = 0; i < 3; ++i)
      d[i][i] -= pos[i] * depth / m_vN[i];

    transformVectors3(d, 3, m_mRotation, m_vCenter); // turns to absolute coordinates

    dImmersionOutlineID outline = m_gImmersion->outline;
    for (int i = 0; i < 3; ++i) {
      dStraightEdge se = {{0,0,0},{0,0,0}};
      dCopyVector3(se.origin, d[i]);
      dCopyVector3(se.end, d[(i + 1) % 3]);
      outline->appendStraightEdge(se);
    }
  }
}

void sBoxImmersionData::twoImmersedVertices() {
  const dImmersedVertex *c[2] = { m_vImmersedVertices + m_vIndices[0], m_vImmersedVertices + m_vIndices[1] };

  if (c[1]->depth > c[0]->depth)
    std::swap(c[0], c[1]);

  // compute the dimensions of the truncated tetrahedron bases
  dReal a[2], b[2];
  const int edgeDirection = c[0]->pos[0] != c[1]->pos[0] ? 0 : (c[0]->pos[1] != c[1]->pos[1] ? 1 : 2);
  const int aIndex = (edgeDirection + 1) % 3;
  const int bIndex = (aIndex + 1) % 3;
  for (int i = 0; i < 2; ++i) {
    a[i] = c[i]->depth / m_vN[aIndex];
    b[i] = c[i]->depth / m_vN[bIndex];
  }

  const dReal h = m_vSide[edgeDirection];
  const dReal s = a[0] * b[0] + a[1] * b[1];
  const dReal p = s + a[1] * b[0];
  m_gImmersion->volume = h * p / 6.0;
  m_gImmersion->projectedAreas[edgeDirection] = 0.5 * s;
  m_gImmersion->projectedAreas[aIndex] = 0.5 * h * (a[0] + a[1]);
  m_gImmersion->projectedAreas[bIndex] = 0.5 * h * (b[0] + b[1]);

  const dReal deltaA = a[0] - a[1];
  const dReal deltaB = b[0] - b[1];
  const dReal q = (a[0] + a[1]) / (4.0 * p);
  dReal alpha[3];
  alpha[edgeDirection] = h * (1.5 * a[0] * b[0] - 2.0 * b[0] * deltaA + 0.75 * deltaA * deltaB) / p;
  alpha[aIndex] = q * s;
  alpha[bIndex] = q * (b[0] * b[0] + b[1] * b[1]);

  for (int i = 0; i < 3; ++i)
    m_vBuoyancyCenter[i] = c[0]->pos[i] * (m_vHalfSide[i] - alpha[i]);

  // immersion outline
  if (m_gImmersion->outline && (m_nFlags & ~dxImmersionOutlineDisabled))  {
    dVector3 d[4];

    for (int i = 0; i < 2; ++i) {
      const dReal *const pos = c[i]->pos;
      for (int j = 0; j < 3; ++j)
        d[2 * i][j] = pos[j] * m_vHalfSide[j];
    }

    dCopyVector3(d[1], d[0]);
    dCopyVector3(d[3], d[2]);

    for (int i = 0; i < 2; ++i) {
      d[2 * i][aIndex] -= a[i] * c[i]->pos[aIndex];
      d[2 * i + 1][bIndex] -= b[i] * c[i]->pos[bIndex];
    }

    transformVectors3(d, 4, m_mRotation, m_vCenter); // turns to absolute coordinates

    dImmersionOutlineID outline = m_gImmersion->outline;
    for (int i = 0; i < 4; i += 3) {
      for (int j = 1; j < 3; ++j) {
        dStraightEdge se = {{0,0,0},{0,0,0}};
        dCopyVector3(se.origin,d[i]);
        dCopyVector3(se.end, d[j]);
        outline->appendStraightEdge(se);
      }
    }
  }
}

void sBoxImmersionData::threeImmersedVertices() {
  const dImmersedVertex *c[4] = { m_vImmersedVertices + m_vIndices[0], m_vImmersedVertices + m_vIndices[1],
    m_vImmersedVertices + m_vIndices[2], 0 };

  for (int j = 0; j < 2; ++j) {
    const int k = j + 1;
    if (c[k]->depth > c[0]->depth)
      std::swap(c[0], c[k]);
  }

  dReal tetrahedronVolume[3];
  dVector3 tetrahedronCenter[3];
  const dReal nProduct = 6.0 * m_vN[0] * m_vN[1] * m_vN[2];
  for (int i = 0; i < 3; ++i) {
    const dImmersedVertex *const v = c[i];
    const dReal depthCube = v->depth * v->depth * v->depth;
    const dReal d = 0.25 * v->depth;
    tetrahedronVolume[i] = depthCube / nProduct;
    for (int j = 0; j < 3; ++j) {
      const dReal offset = v->pos[j] != c[0]->pos[j] ? -d : d;
      tetrahedronCenter[i][j] = v->pos[j] * (m_vHalfSide[j] - offset / m_vN[j]);
    }
  }

  m_gImmersion->volume = tetrahedronVolume[0] - tetrahedronVolume[1] - tetrahedronVolume[2];
  dCopyScaledVector3(m_vBuoyancyCenter, tetrahedronCenter[0], tetrahedronVolume[0]);
  for (int i = 1; i < 3; ++i)
    dAddScaledVectors3(m_vBuoyancyCenter, m_vBuoyancyCenter, tetrahedronCenter[i], 1.0, -tetrahedronVolume[i]);

  dScaleVector3(m_vBuoyancyCenter, 1.0 / m_gImmersion->volume);

  dVector3 v3;
  dAddVectors3(v3, c[0]->pos, c[1]->pos);
  dAddVectors3(v3, v3, c[2]->pos);
  const int faceIndex = dFabs(v3[0]) == 3.0 ? 0 : (dFabs(v3[1]) == 3.0 ? 1 : 2);
  const int aIndex = (faceIndex + 1) % 3;
  const int bIndex = (aIndex + 1) % 3;
  if (c[0]->pos[aIndex] == c[1]->pos[aIndex])
    std::swap(c[1], c[2]);

  const dReal a[3] = { c[0]->depth / m_vN[faceIndex], c[1]->depth / m_vN[faceIndex], c[2]->depth / m_vN[faceIndex] };
  const dReal b[2] = { c[1]->depth / m_vN[bIndex],  c[2]->depth / m_vN[aIndex] };
  m_gImmersion->projectedAreas[aIndex] = 0.5 * ( (a[0] + a[2]) * m_vSide[bIndex] + a[1] * c[1]->depth / m_vN[bIndex]);
  m_gImmersion->projectedAreas[bIndex] = 0.5 * ( (a[0] + a[1]) * m_vSide[aIndex] + a[2] * c[2]->depth / m_vN[aIndex]);
  m_gImmersion->projectedAreas[faceIndex] = m_vSide[aIndex] * m_vSide[bIndex] - 0.5 * (m_vSide[bIndex] - b[0]) * (m_vSide[aIndex] - b[1]);

  // water outline
  if (m_gImmersion->outline && (m_nFlags & ~dxImmersionOutlineDisabled)) {
    dVector3 d[5];
    for (int i = 0; i < 3; ++i) {
      const dReal *const pos = c[i]->pos;
      for (int j = 0; j < 3; ++j)
        d[i][j] = pos[j] * m_vHalfSide[j];
    }

    dCopyVector3(d[3], d[1]);
    dCopyVector3(d[4], d[2]);

    for (int i = 0; i < 3; ++i) {
      const dReal *const pos = c[i]->pos;
      d[i][faceIndex] -= pos[faceIndex] * c[i]->depth / m_vN[faceIndex];
    }

    const dImmersedVertex *v = c[1];
    const dReal *pos = v->pos;
    d[3][bIndex] -= pos[bIndex] * v->depth / m_vN[bIndex];
    v = c[2];
    pos = v->pos;
    d[4][aIndex] -= pos[aIndex] * v->depth / m_vN[aIndex];

    transformVectors3(d, 5, m_mRotation, m_vCenter); // turns to absolute coordinates

    dStraightEdge se[5];
    for (int i = 0; i < 2; ++i) {
      dCopyVector3(se[i].origin, d[0]);
      dCopyVector3(se[i].end, d[i + 1]);
    }

    int k = 1;
    for (int i = 2; i < 4; ++i) {
      dCopyVector3(se[i].origin, d[3]);
      dCopyVector3(se[i].end, d[k]);
      k = 4;
    }

    dCopyVector3(se[4].origin, d[4]);
    dCopyVector3(se[4].end, d[2]);

    dImmersionOutlineID outline = m_gImmersion->outline;
    for (int i = 0; i < 5; ++i)
      outline->appendStraightEdge(se[i]);
  }
}

void sBoxImmersionData::fourImmersedVertices() {
  const dImmersedVertex *c[4] = { m_vImmersedVertices + m_vIndices[0], m_vImmersedVertices + m_vIndices[1],
    m_vImmersedVertices + m_vIndices[2], m_vImmersedVertices + m_vIndices[3] };

  for (int i = 0; i < 3; ++i) {
    const int k = i + 1;
    if (c[k]->depth > c[0]->depth)
      std::swap(c[0], c[k]);
  }

  dVector3 v3;
  dAddVectors3(v3, c[0]->pos, c[1]->pos);
  dAddVectors3(v3, v3, c[2]->pos);
  dAddVectors3(v3, v3, c[3]->pos);

  if (dFabs(v3[0]) == 2.0) { // case of an hexagonal section (immersed vertices define a tetrahedron)
    dReal tetrahedronVolume[4];
    dVector3 tetrahedronCenter[4];
    const dReal nProduct = 6.0 * m_vN[0] * m_vN[1] * m_vN[2];
    for (int i = 0; i < 4; ++i) {
      const dImmersedVertex *const v = c[i];
      const dReal depthCube = v->depth * v->depth * v->depth;
      tetrahedronVolume[i] = depthCube / nProduct;
      const dReal d = 0.25 * v->depth;
      for (int j = 0; j < 3; ++j) {
        const dReal offset = v->pos[j] != c[0]->pos[j] ? -d : d;
        tetrahedronCenter[i][j] = v->pos[j] * (m_vHalfSide[j] - offset / m_vN[j]);
      }
    }

    m_gImmersion->volume = tetrahedronVolume[0] - tetrahedronVolume[1] - tetrahedronVolume[2] - tetrahedronVolume[3];
    dCopyScaledVector3(m_vBuoyancyCenter, tetrahedronCenter[0], tetrahedronVolume[0] / m_gImmersion->volume);
    for (int i = 1; i < 4; ++i)
      dAddScaledVectors3(m_vBuoyancyCenter, m_vBuoyancyCenter, tetrahedronCenter[i], 1.0, -tetrahedronVolume[i] / m_gImmersion->volume);

    if (c[0]->pos[0] == c[1]->pos[0])
      std::swap(c[1], c[2]);

    if (c[0]->pos[0] == c[1]->pos[0])
      std::swap(c[1], c[3]);

    if (c[0]->pos[1] == c[2]->pos[1])
      std::swap(c[2], c[3]);

    m_gImmersion->projectedAreas[0] = m_vSide[1] * m_vSide[2] - 0.5 * (m_vSide[1] - c[3]->depth / m_vN[1]) * (m_vSide[2] - c[2]->depth / m_vN[2]) +
      0.5 * (c[1]->depth * c[1]->depth / (m_vN[1] *  m_vN[2]));
    m_gImmersion->projectedAreas[1] = m_vSide[0] * m_vSide[2] - 0.5 * (m_vSide[0] - c[3]->depth / m_vN[0]) * (m_vSide[2] - c[1]->depth / m_vN[2]) +
      0.5 * (c[2]->depth * c[2]->depth / (m_vN[0] * m_vN[2]));
    m_gImmersion->projectedAreas[2] = m_vSide[0] * m_vSide[1] - 0.5 * (m_vSide[1] - c[1]->depth / m_vN[1]) * (m_vSide[0] - c[2]->depth / m_vN[0]) +
      0.5 * (c[3]->depth * c[3]->depth / (m_vN[0] * m_vN[1]));

    // water outline
    if (m_gImmersion->outline && (m_nFlags & ~dxImmersionOutlineDisabled)) {
      dVector3 d[6];
      for (int i = 0; i < 6; i += 2) {
        const dReal *const pos = c[(i / 2) + 1]->pos;
        for (int j = 0; j < 3; ++j)
          d[i][j] = pos[j] * m_vHalfSide[j];
        dCopyVector3(d[i + 1], d[i]);
      }

      const dImmersedVertex *v = c[1];
      const dReal *pos = v->pos;
      d[0][2] -= pos[2] * v->depth / m_vN[2];
      d[1][1] -= pos[1] * v->depth / m_vN[1];

      v = c[2];
      pos = v->pos;
      d[2][0] -= pos[0] * v->depth / m_vN[0];
      d[3][2] -= pos[2] * v->depth / m_vN[2];

      v = c[3];
      pos = c[3]->pos;
      d[4][1] -= pos[1] * v->depth / m_vN[1];
      d[5][0] -= pos[0] * v->depth / m_vN[0];

      transformVectors3(d, 6, m_mRotation, m_vCenter); // turns to absolute coordinates

      dStraightEdge se[6];
      dImmersionOutlineID io = m_gImmersion->outline;
      for (int i = 0; i < 6; ++i) {
        dCopyVector3(se[i].origin, d[i]);
        dCopyVector3(se[i].end, d[(i + 1) % 6]);
        io->appendStraightEdge(se[i]);
      }
    }
  } else {
    // case of a parallelogram section (one face is immersed)
    const int faceIndex = dFabs(v3[0]) == 4.0 ? 0 : (dFabs(v3[1]) == 4.0 ? 1 : 2);
    const int aIndex = (faceIndex + 1) % 3;
    const int bIndex = (aIndex + 1) % 3;

    for (int i = 1; i < 3; ++i) {
      if (c[0]->pos[aIndex] == -c[i]->pos[aIndex] && c[0]->pos[bIndex] == -c[i]->pos[bIndex])
        std::swap(c[i], c[3]);
    }

    if (c[0]->pos[aIndex] == c[1]->pos[aIndex])
      std::swap(c[1], c[2]);

    const dReal nFace = m_vN[faceIndex];
    const dReal a[4] = { c[0]->depth / nFace, c[1]->depth / nFace, c[2]->depth / nFace, c[3]->depth / nFace };
    const dReal hPrime = m_vSide[aIndex];
    const dReal h = m_vSide[bIndex];

    const dReal sum = a[0] + a[1] + a[2] + a[3];
    m_gImmersion->volume = 0.25 * h * hPrime * sum;
    const dReal sum3 = 3.0 * sum;
    m_vBuoyancyCenter[aIndex] = c[2]->pos[aIndex] * (m_vHalfSide[aIndex] - hPrime * (sum + a[1] + a[3]) / sum3 );
    m_vBuoyancyCenter[bIndex] = c[2]->pos[bIndex] * (m_vHalfSide[bIndex] - h * (sum + a[0] + a[1]) / sum3);
    m_vBuoyancyCenter[faceIndex] = c[2]->pos[faceIndex] * (m_vHalfSide[faceIndex] - 2.0 * (a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3] + a[0] * a[1] + a[2] * a[3] + a[1] * a[3]  + a[0] * a[2] + 0.5 *
       (a[1] * a[2] + a[0] * a[3])) / (3.0 * sum3));

    m_gImmersion->projectedAreas[faceIndex] = hPrime * h;
    m_gImmersion->projectedAreas[aIndex] = 0.5 * h * sum;
    m_gImmersion->projectedAreas[bIndex] = 0.5 * hPrime * sum;

    // water outline
    if (m_gImmersion->outline && (m_nFlags & ~dxImmersionOutlineDisabled)) {
      dVector3 d[4];
      for (int i = 0; i < 4; ++i) {
        const dReal *const pos = c[i]->pos;
        for (int j = 0; j < 3; ++j)
          d[i][j] = pos[j] * m_vHalfSide[j];

        d[i][faceIndex] -= a[i] * pos[faceIndex];
      }

      transformVectors3(d, 4, m_mRotation, m_vCenter); // turns to absolute coordinates

      dStraightEdge se[4];
      for (int i = 0; i < 2; ++i) {
        dCopyVector3(se[i].origin, d[0]);
        dCopyVector3(se[i].end, d[i + 1]);
      }

      int k = 1;
      for (int i = 2; i < 4; ++i) {
        dCopyVector3(se[i].origin, d[3]);
        dCopyVector3(se[i].end, d[k]);
        k = 2;
      }

      dImmersionOutlineID outline = m_gImmersion->outline;
      for (int i = 0; i < 4; ++i)
        outline->appendStraightEdge(se[i]);
    }
  }
}

int sBoxImmersionData::performImmersionChecking() {
  if (m_nNumberOfImmersedVertices == 0)
    return 0;

  m_bComplement = m_nNumberOfImmersedVertices > 4;
  if (m_bComplement)
    m_nNumberOfImmersedVertices = 8 - m_nNumberOfImmersedVertices;

  for (int i = 0; i < 3; ++i)
    m_vN[i] = dFabs(m_vN[i]);

  dSetZero(m_vBuoyancyCenter, 3);
  m_gImmersion->volume = 0.0;
  m_gImmersion->area = 0.0;

  for (int i = 0; i < 3; ++i)
    m_gImmersion->projectedAreas[i] = 0.0;

  if (m_nNumberOfImmersedVertices > 0) {
    int j = 0;
    for (int i = 0; i < 8; ++i) {
      const bool record = (m_vImmersedVertices[i].depth < 0.0 && m_bComplement) ||
        (m_vImmersedVertices[i].depth > 0.0 && !m_bComplement);
      if (record) {
        m_vImmersedVertices[i].depth = dFabs(m_vImmersedVertices[i].depth);
        m_vIndices[j++] = i;
      }
    }
  }

  switch(m_nNumberOfImmersedVertices) {
  case 0: break;
  case 1: oneImmersedVertex();
           break;
  case 2: twoImmersedVertices();
           break;
  case 3: threeImmersedVertices();
           break;
  case 4: fourImmersedVertices();
           break;
  default: dIASSERT(false);
  }

  if (m_bComplement) {
    const dReal fullVolume = m_vSide[0] * m_vSide[1] * m_vSide[2];
    const dReal volume = fullVolume - m_gImmersion->volume;
    if (volume <= VOLUME_ZERO_THRESHOLD)
      return 0;
    dCopyScaledVector3(m_vBuoyancyCenter, m_vBuoyancyCenter, -m_gImmersion->volume / volume);
    m_gImmersion->volume = volume;
    for (int i = 0; i < 3; ++i)
      m_gImmersion->projectedAreas[i] = m_vSide[(i + 1) % 3] * m_vSide[(i + 2) % 3] - m_gImmersion->projectedAreas[i];
  }

  for (int i = 0; i < 3; ++i)
    m_gImmersion->area += m_gImmersion->projectedAreas[i];

  const dReal *bodyRotation = dBodyGetRotation(dGeomGetBody(m_gImmersion->g1));
  // Compute area projections in body's frame
  projectAreas(m_gImmersion->projectedAreas, bodyRotation, m_mRotation);

  // Absolute coordinates
  dMultiplyHelper0_331(m_vBuoyancyCenter, m_mRotation, m_vBuoyancyCenter);
  dAddVectors3(m_vBuoyancyCenter, m_vBuoyancyCenter, m_vCenter);

  return m_gImmersion->volume > VOLUME_ZERO_THRESHOLD ? 1 : 0;
}

int dImmerseBox (dxBox *box, const dReal *fluidPlane, int flags,
      dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT(box->body, "This box has no body and hence cannot be tested for immersion into a fluid");

  sBoxImmersionData data(box, fluidPlane, immersion, flags);

  return data.performImmersionChecking();
}

int dImmerseBoxBox (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseBoxBox has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideBoxBox(o1, o2, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  dxBox *const box1 = (dxBox *) o1;
  dxBox *const box2 = (dxBox *) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomBoxGetImmersionPlane(box2, fluidPlane);

  return dImmerseBox (box1, fluidPlane, flags, immersion);
}

int dImmerseBoxCapsule (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseBoxCapsule has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideCapsuleBox(o2, o1, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  dxBox *const box = (dxBox *) o1;
  dxCapsule *const capsule = (dxCapsule *) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomCapsuleGetImmersionPlane(capsule, fluidPlane);

  return dImmerseBox (box, fluidPlane, flags, immersion);
}

int dImmerseBoxCylinder (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseBoxCylinder has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideCylinderBox(o2, o1, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  dxBox *const box = (dxBox *) o1;
  dxCylinder *const cylinder = (dxCylinder *) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomCylinderGetImmersionPlane(cylinder, fluidPlane);

  return dImmerseBox (box, fluidPlane, flags, immersion);
}

int dImmerseBoxSphere (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseBoxSphere has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideSphereBox(o2, o1, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  dxBox *const box = (dxBox *) o1;
  dxSphere *const sphere = (dxSphere *) o2;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomSphereGetImmersionPlane(sphere, fluidPlane);

  return dImmerseBox (box, fluidPlane, flags, immersion);
}

int dImmerseBoxTrimesh (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseBoxTrimesh has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideBTL(o2, o1, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomTriMeshGetImmersionPlane(o2, fluidPlane);

  return dImmerseBox ((dxBox *) o1, fluidPlane, flags, immersion);
}

int dImmerseBoxPlane (dxGeom *o1, dxGeom *o2, int flags,
      dImmersionGeom *immersion)
{

  dIASSERT ((flags & NUMI_MASK) >= 1);
  dIASSERT (o1->type == dBoxClass);
  dIASSERT (o2->type == dPlaneClass);
  dUASSERT(o1-> body && o2->fluid, "The first argument of dImmerseBoxPlane has no body or the second has no fluid");

  immersion->g1 = o1;
  immersion->g2 = o2;

  dxBox *const box = (dxBox*) o1;
  const dxPlane *const plane = (dxPlane*) o2;

  return dImmerseBox (box, plane->p, flags, immersion);
}
