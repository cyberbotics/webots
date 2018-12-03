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

#include "immersion_trimesh.h"

#include <ode/objects.h>
#include <ode/fluid_dynamics/immersion.h>
#include <ode/fluid_dynamics/fluid_dynamics.h>
#include <ode/fluid_dynamics/odemath_fluid_dynamics.h>
#include <ode/matrix.h>
#include <ode/odemath.h>
#include "../collision_kernel.h"
#include "../collision_std.h"
#include "immersion_kernel.h"
#include "immersion_outline.h"
#include "immersion_std.h"

#ifdef _MSC_VER
#pragma warning(disable:4291)  // for VC++, no complaints about "no matching operator delete found"
#endif

#if dTRIMESH_ENABLED
#include "../collision_trimesh_internal.h"

dReal dGeomTriMeshPointDepth (dGeomID g, dReal x, dReal y, dReal z)
 {
   dUASSERT (g && g->type == dTriMeshClass,"argument not a trimesh");
   dxTriMesh *const trimesh = (dxTriMesh *)g;
   const unsigned int numberOfTriangles = FetchTriangleCount(trimesh);
   dVector3 v[3];
   dVector3 e1, e2, n;
   dVector3 p = { x, y, z };
   dReal depth = -dInfinity;
#if dTRIMESH_OPCODE
   VertexPointers VP;
   ConversionArea VC;
#endif
   const dReal *const position = dGeomGetPosition(trimesh);
   const dReal *const rotation = dGeomGetRotation(trimesh);
   dSubtractVectors3(p, p, position);
   dMultiply1_331(p, rotation, p);

   for (unsigned int i = 0; i < numberOfTriangles; ++i) {
#if dTRIMESH_OPCODE
     trimesh->Data->Mesh.GetTriangle(VP, i, VC);
     for (int j = 0; j < 3; ++j) {
       v[j][0] = VP.Vertex[j]->x;
       v[j][1] = VP.Vertex[j]->y;
       v[j][2] = VP.Vertex[j]->z;
     }
#endif
#if dTRIMESH_GIMPACT
     FetchTransformedTriangle(trimesh, i, v);
#endif
     dSubtractVectors3(e1, v[1], v[0]);
     dSubtractVectors3(e2, v[2], v[0]);
     dCalcVectorCross3(n, e1, e2);
     dNormalize3(n);
     dVector3 m;
     dSubtractVectors3(m, p, v[0]);
     const dReal distanceToTriangle = - dCalcVectorDot3(n, m);

     // Check if the projection of p on the triangle's plane lies inside the triangle
     dAddScaledVectors3(m, m, n, 1.0, distanceToTriangle);
     dVector3 nPrime;
     dCalcVectorCross3(nPrime, n, e1);
     const dReal a2 = dCalcVectorDot3(m, nPrime) / dCalcVectorDot3(e2, nPrime);
     dAddScaledVectors3(m, m, e2, 1.0, -a2);
     dNormalize3(e1);
     const dReal a1 = dCalcVectorDot3(m, e1);
     static const dReal epsilon = 1e-3;
     const bool projectionLiesInsideTriangle = a1 >= -epsilon && a1 <= 1.0 + epsilon && a2 >= -epsilon && a2 <= 1.0 + epsilon;

     if (projectionLiesInsideTriangle == false)
       continue;

     if ((depth < 0.0 && distanceToTriangle > depth) || (depth > 0.0 && distanceToTriangle > 0.0 && distanceToTriangle < depth))
       depth = distanceToTriangle;

   }
   return depth;
}

dReal dGeomTriMeshGetArea (dGeomID g)
 {
   dUASSERT (g && g->type == dTriMeshClass,"argument not a trimesh");
   dxTriMesh *const trimesh = (dxTriMesh *)g;

   if (!(trimesh->dataFlag & dxTriMesh::areaChanged))
     return trimesh->area;

   dReal area = 0.0;
   const unsigned int numberOfTriangles = FetchTriangleCount(trimesh);
   dVector3 v[3];
   dVector3 e1, e2, n;
#if dTRIMESH_OPCODE
   VertexPointers VP;
   ConversionArea VC;
#endif
   for (unsigned int i = 0; i < numberOfTriangles; ++i) {
#if dTRIMESH_OPCODE
     trimesh->Data->Mesh.GetTriangle(VP, i, VC);
#endif
     for (int j = 0; j < 3; ++j) {
       v[j][0] = VP.Vertex[j]->x;
       v[j][1] = VP.Vertex[j]->y;
       v[j][2] = VP.Vertex[j]->z;
     }
#if dTRIMESH_GIMPACT
     FetchTransformedTriangle(trimesh, i, v);
#endif
     dSubtractVectors3(e1, v[1], v[0]);
     dSubtractVectors3(e2, v[2], v[0]);
     dCalcVectorCross3(n, e1, e2);
     area += dCalcVectorLength3(n);
   }

   trimesh->area = 0.5 * area;
   trimesh->dataFlag ^= dxTriMesh::areaChanged;
   return trimesh->area;
 }

dReal dGeomTriMeshGetVolume (dGeomID g)
 {
   dUASSERT (g && g->type == dTriMeshClass,"argument not a trimesh");
   dxTriMesh *const trimesh = (dxTriMesh *)g;

   if (!(trimesh->dataFlag & dxTriMesh::volumeChanged))
     return trimesh->volume;

   dReal volume = 0.0;
   const unsigned int numberOfTriangles = FetchTriangleCount(trimesh);

   dVector3 v[3];
   dVector3 e1, e2, n;
#if dTRIMESH_OPCODE
   VertexPointers VP;
   ConversionArea VC;
   for (unsigned int i = 0; i < numberOfTriangles; ++i) {
     trimesh->Data->Mesh.GetTriangle(VP, i, VC);
     for (int j = 0; j < 3; ++j){
       v[j][0] = VP.Vertex[j]->x;
       v[j][1] = VP.Vertex[j]->y;
       v[j][2] = VP.Vertex[j]->z;
     }
#endif
#if dTRIMESH_GIMPACT
     FetchTransformedTriangle(trimesh, i, v);
#endif
     dSubtractVectors3(e1, v[1], v[0]);
     dSubtractVectors3(e2, v[2], v[0]);
     dCalcVectorCross3(n, e1, e2);
     for (int j = 0; j < 3; ++j)
       volume += n[j] * (v[0][j] + (e1[j] + e2[j]) / 3.0);
   }
   trimesh->volume = (1.0 / 6.0) * volume;
   trimesh->dataFlag ^= dxTriMesh::volumeChanged;
   return trimesh->volume;
 }

const dReal *dGeomTriMeshGetRelCenterOfMass (dGeomID g)
 {
   dUASSERT (g && g->type == dTriMeshClass,"argument not a trimesh");
   dxTriMesh *const trimesh = (dxTriMesh *)g;

   dReal *const c = trimesh->centerOfMass;

   if (!(trimesh->dataFlag & dxTriMesh::centerOfMassChanged))
     return c;

   const unsigned int numberOfTriangles = FetchTriangleCount(trimesh);
   dVector3 v[3];
   dVector3 e1, e2, n;
   dSetZero(c, 3);
#if dTRIMESH_OPCODE
   VertexPointers VP;
   ConversionArea VC;
#endif
   for(unsigned int i = 0; i < numberOfTriangles; ++i) {
#if dTRIMESH_OPCODE
     trimesh->Data->Mesh.GetTriangle(VP, i, VC);
     for (int j = 0; j < 3; ++j){
       v[j][0] = VP.Vertex[j]->x;
       v[j][1] = VP.Vertex[j]->y;
       v[j][2] = VP.Vertex[j]->z;
     }
#endif
#if dTRIMESH_GIMPACT
     FetchTransformedTriangle(trimesh, i, v);
#endif
     dSubtractVectors3(e1, v[1], v[0]);
     dSubtractVectors3(e2, v[2], v[0]);
     dCalcVectorCross3(n, e1, e2);
     for (int j = 0; j < 3; ++j) {
       const dReal v0 = v[0][j];
       const dReal a1 = e1[j];
       const dReal a2 = e2[j];
       c[j] += n[j] * (v0 * (v0 + (2.0 / 3.0) * (a1 + a2)) + (a1 * a1 + a1 * a2 + a2 * a2) / 6.0);
     }
   }
   const dReal volume = dGeomTriMeshGetVolume(trimesh);
   dScaleVector3(c, 0.25 / volume);
   trimesh->dataFlag ^= dxTriMesh::centerOfMassChanged;
   return c;
 }

void dGeomTriMeshGetCenterOfMass (dGeomID g, dVector3 c)
 {
   const dReal *const relCom = dGeomTriMeshGetRelCenterOfMass(g);
   const dReal *const position = dGeomGetPosition(g);
   const dReal *const rotation = dGeomGetRotation(g);
   dMultiply0_331(c, rotation, relCom);
   dAddVectors3(c, c, position);
 }

const dReal *dGeomTriMeshGetInertiaMatrix (dGeomID g)
 {
   dUASSERT (g && g->type == dTriMeshClass,"argument not a trimesh");
   dxTriMesh *const trimesh = (dxTriMesh *)g;

   dReal *const I = trimesh->inertiaMatrix;

  if (!(trimesh->dataFlag & dxTriMesh::inertiaMatrixChanged))
    return I;

   const unsigned int numberOfTriangles = FetchTriangleCount(trimesh);
   dVector3 v[3];
   dVector3 e1, e2, n;
   dReal J[12];
   dSetZero(J, 12);
#if dTRIMESH_OPCODE
    VertexPointers VP;
    ConversionArea VC;
#endif
   for(unsigned int i = 0; i < numberOfTriangles; ++i) {
#if dTRIMESH_OPCODE
     trimesh->Data->Mesh.GetTriangle(VP, i, VC);
     for (int j = 0; j < 3; ++j){
       v[j][0] = VP.Vertex[j]->x;
       v[j][1] = VP.Vertex[j]->y;
       v[j][2] = VP.Vertex[j]->z;
     }
#endif
#if dTRIMESH_GIMPACT
     FetchTransformedTriangle(trimesh, i, v);
#endif
     dSubtractVectors3(e1, v[1], v[0]);
     dSubtractVectors3(e2, v[2], v[0]);
     dCalcVectorCross3(n, e1, e2);
     for (int j = 0; j < 3; ++j) {
       const dReal v0 = v[0][j];
       const dReal a1 = e1[j];
       const dReal a2 = e2[j];
       const dReal a1Square = a1 * a1;
       const dReal a2Square = a2 * a2;
       const dReal sumSquare = a1Square + a2Square;
       const dReal sum = a1 + a2;
       J[5 * j] += n[j] * (v0 * (v0 * v0 + v0 * sum + 0.5 * (sumSquare + a1 * a2)) + sum * sumSquare / 10.0);
     }
     const dReal x0 = v[0][0], y0 = v[0][1];
     const dReal a1 = e1[0], a2 = e2[0];
     const dReal b1 = e1[1], b2 = e2[1];
     const dReal c1 = e1[2], c2 = e2[2];
     const dReal a1Square = a1 * a1, a2Square = a2 * a2;
     const dReal b1Square = b1 * b1, b2Square = b2 * b2;
     const dReal c1Square = c1 * c1, c2Square = c2 * c2;
     const dReal aProduct = a1 * a2, bProduct = b1 * b2, cProduct = c1 * c2;
     const dReal aSum = a1 + a2, bSum = b1 + b2, cSum = c1 + c2;
     // xy integral
     J[1] += n[0] * (x0 * (x0 * y0 + (2.0 * y0 * aSum + x0 * bSum)) / 3.0
       + (x0 * (a1 * b2 + a2 * b1 + 2.0 * (a1 * b1 + a2 * b2))
          + y0 * (a1Square + aProduct + a2Square)) / 6.0
       + (a1Square * b1 + a2Square * b2) / 10.0
       + (a1Square * b2 + a2Square * b1 + 2.0 * aProduct * bSum) / 30.0);

    // zx integral
     const dReal z0 = v[0][2];
     J[2] += n[2] * (z0 * (x0 * z0 + (2.0 * y0 * cSum + z0 * aSum)) / 3.0
       + (z0 * (c1 * a2 + c2 * a1 + 2.0 * (c1 * a1 + c2 * a2))
         + x0 * (c1Square + cProduct + c2Square)) / 6.0
       + (c1Square * a1 + c2Square * a2) / 10.0
       + (c1Square * a2 + c2Square * a1 + 2.0 * cProduct * aSum) / 30.0);

    // yz integral
     J[6] += n[1] * (y0 * (y0 * z0 + (2.0 * z0 * bSum + y0 * cSum)) / 3.0
       + (y0 * (b1 * c2 + b2 * c1 + 2.0 * (b1 * c1 + b2 * c2))
         + z0 * (b1Square + bProduct + b2Square)) / 6.0
       + (b1Square * c1 + b2Square * c2) / 10.0
       + (b1Square * c2 + b2Square * c1 + 2.0 * bProduct * cSum) / 30.0);
   }

   J[1] *= 0.25;
   J[2] *= 0.25;
   J[6] *= 0.25;

   for (int j = 0; j < 3; ++j)
     J[5 * j] *= 1.0 / 6.0;

   for (int j = 0; j < 3; ++j) {
     const int a = (j + 1) % 3;
     const int b = (a + 1) % 3;
     I[5 * j] = J[5 * a] + J[5 * b];
   }

   I[1] = -J[1];
   I[2] = -J[2];
   I[6] = -J[6];
   I[4] = I[1];
   I[8] = I[2];
   I[9] = I[6];

  // Translate inertia matrix to the CoM
  dMatrix3 aHat, t;
  const dReal *const relCom = dGeomTriMeshGetRelCenterOfMass(trimesh);
  dSetZero(aHat,12);
  dSetCrossMatrixPlus(aHat, relCom, 4);
  dMultiply0_333(t, aHat, aHat);
  const dReal volume = dGeomTriMeshGetVolume(trimesh);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const int k = i * 4 + j;
      I[k] -= volume * t[k];
    }
  }
  // Ensure symetry
  I[4] = I[1];
  I[8] = I[2];
  I[9] = I[6];
  trimesh->dataFlag ^= dxTriMesh::inertiaMatrixChanged;
  return I;
}

void dGeomTriMeshGetBoundingPlane (dGeomID g, int planeIndex, dVector4 plane)
 {
   dUASSERT (g && g->type == dTriMeshClass,"argument not a trimesh");
   dUASSERT(planeIndex >= 0 && planeIndex < 6 , "Invalid plane index: range = [0, 5]");
   const int axisIndex = planeIndex / 2;
   dReal sign = planeIndex % 2 ? -1.0 : 1.0;
   dReal aabb[6];
   dGeomGetAABB(g, aabb);

   plane[0] = 0.0;
   plane[1] = 0.0;
   plane[2] = 0.0;
   plane[axisIndex] = sign;
   plane[3] = sign * aabb[planeIndex];
 }

void dGeomTriMeshGetImmersionPlane (dGeomID g, dVector4 plane)
 {
  dGeomTriMeshGetBoundingPlane(g, 2 * FLUID_PLANE_NORMAL + 1, plane);
 }

int dImmerseTriMesh (dxTriMesh *trimesh, const dReal *fluidPlane, int flags,
      dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT(trimesh->body, "This trimesh has no body and hence cannot be tested for immersion into a fluid");

  dReal aabb[6];
  dGeomGetAABB(trimesh, aabb);
  bool fullyImmersed = true;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      for (int k = 0; k < 2; ++k) {
        const dVector3 aabbVertex = { aabb[i], aabb[2 + j], aabb[4 + k] };
        const dReal depth = fluidPlane[3] - dCalcVectorDot3(fluidPlane, aabbVertex);
        if (depth <= 0.0)
          fullyImmersed = false;
      }
    }
  }

  if (fullyImmersed) {
    immersion->area = dGeomTriMeshGetArea(trimesh);
    immersion->volume = dGeomTriMeshGetVolume(trimesh);
    dGeomTriMeshGetCenterOfMass(trimesh, immersion->buoyancyCenter);
    return 1;
  }

  immersion->area = 0.0;
  immersion->volume = 0.0;
  dSetZero(immersion->buoyancyCenter, 3);

  dVector3 u;
  pickOrthonormalVector3(fluidPlane, u);

  const unsigned int numberOfTriangles = FetchTriangleCount(trimesh);
  dVector3 v[3], e1, e2, n;
  for (unsigned int i = 0; i < numberOfTriangles; ++i) {
    FetchTransformedTriangle(trimesh, i, v);
    dSubtractVectors3(e1, v[1], v[0]);
    dSubtractVectors3(e2, v[2], v[0]);
    dCalcVectorCross3(n, e1, e2);
    const dReal doubleTriangleArea = dCalcVectorLength3(n);
    dReal depth[3];
    int numberOfImmersedVertices = 0;
    int immersedVertexIndex = -1;
    for (int j = 0; j < 3; ++j) {
      depth[j] = fluidPlane[3] - dCalcVectorDot3(v[j], fluidPlane);
      if (depth[j] > 0.0) {
        ++numberOfImmersedVertices;
        immersedVertexIndex = j;
      }
    }

    if (numberOfImmersedVertices == 0)
      continue;

    dReal doubleTriangleVolume = 0.0;
    for (int j = 0; j < 3; ++j)
      doubleTriangleVolume += u[j] * (v[0][j] + (e1[j] + e2[j]) / 3.0);
    const dReal product = dCalcVectorDot3(u, n);
    doubleTriangleVolume *= product;
    dMatrix3 I;
    computeTriangleSecondDegreeIntegrals(v[0], e1, e2, I);
    dVector3 gamma;
    computeTriangleGamma(u, I, gamma);
    dScaleVector3(gamma, product);

    if (numberOfImmersedVertices == 3) {
      immersion->area += doubleTriangleArea;
      immersion->volume += doubleTriangleVolume;
      dAddVectors3(immersion->buoyancyCenter, immersion->buoyancyCenter, gamma);
      continue;
    }

    int j0 = -1;
    if (numberOfImmersedVertices == 1) {
      j0 = immersedVertexIndex;
    } else {
      const int j = (immersedVertexIndex + 1) % 3;
      j0 = depth[j] <= 0.0 ? j : (j + 1) % 3;
    }

    const int j1 = (j0 + 1) % 3;
    const int j2 = (j1 + 1) % 3;
    dVector3 w[3], edge;
    dCopyVector3(w[0], v[j0]);
    dSubtractVectors3(edge, v[j1], w[0]);
    const dReal t1 = depth[j0] / dCalcVectorDot3(edge, fluidPlane);
    dAddScaledVectors3(w[1], w[0], edge, 1.0, t1);
    dSubtractVectors3(edge, v[j2], w[0]);
    const dReal t2 = depth[j0] / dCalcVectorDot3(edge, fluidPlane);
    dAddScaledVectors3(w[2], w[0], edge, 1.0, t2);

    dVector3 sube1, sube2;
    dSubtractVectors3(sube1, w[1], w[0]);
    dSubtractVectors3(sube2, w[2], w[0]);
    dVector3 nPrime;
    dCalcVectorCross3(nPrime, sube1, sube2);
    const dReal doubleTriangleAreaPrime = dCalcVectorLength3(nPrime);

    dReal doubleTriangleVolumePrime = 0.0;
    for (int j = 0; j < 3; ++j)
      doubleTriangleVolumePrime += u[j] * (w[0][j] + (sube1[j] + sube2[j]) / 3.0);
    const dReal productPrime = dCalcVectorDot3(u, nPrime);
    doubleTriangleVolumePrime *= productPrime;

    dMatrix3 IPrime;
    computeTriangleSecondDegreeIntegrals(w[0], sube1, sube2, IPrime);
    dVector3 gammaPrime;
    computeTriangleGamma(u, IPrime, gammaPrime);
    dScaleVector3(gammaPrime, productPrime);

    if (numberOfImmersedVertices == 1) {
      immersion->area += doubleTriangleAreaPrime;
      immersion->volume += doubleTriangleVolumePrime;
    } else {
      immersion->area += doubleTriangleArea - doubleTriangleAreaPrime;
      immersion->volume += doubleTriangleVolume - doubleTriangleVolumePrime;
      dSubtractVectors3(gammaPrime, gamma, gammaPrime);
    }

    dAddVectors3(immersion->buoyancyCenter, immersion->buoyancyCenter, gammaPrime);

    if (immersion->outline && (flags & ~dxImmersionOutlineDisabled)) {
      dStraightEdge se = {{0,0,0},{0,0,0}};
      dCopyVector3(se.origin, w[1]);
      dCopyVector3(se.end, w[2]);
      immersion->outline->appendStraightEdge(se);
    }
  }

  if (immersion->volume <= VOLUME_ZERO_THRESHOLD)
    return 0;

  immersion->area *= 0.5;
  immersion->volume *= 0.5;
  dScaleVector3(immersion->buoyancyCenter, 1.0 / immersion->volume);

  return 1;
}

int dImmerseTrimeshBox (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseTrimeshBox has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideBTL(o1, o2, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomBoxGetImmersionPlane(o2, fluidPlane);
  dxTriMesh *const trimesh = (dxTriMesh *) o1;

  return dImmerseTriMesh (trimesh, fluidPlane, flags, immersion);
}

int trimeshCapsuleCollide(dxGeom *o1, dxGeom *o2) {
   dxTriMesh *const trimesh = (dxTriMesh *) o1;
   const unsigned int numberOfTriangles = FetchTriangleCount(trimesh);

   const dReal *rotation = trimesh->final_posr->R;
   const dReal *translation = trimesh->final_posr->pos;

   dVector3 v;
   for (unsigned int i = 0; i < numberOfTriangles; ++i) {
     GetVertex(trimesh, i, v);
     dMultiply0_331(v, rotation, v);
     dAddVectors3(v, v, translation);
     if (dGeomCapsulePointDepth(o2, v[0], v[1], v[2]) > 0.0)
       return 1;
   }

   return 0;
}

int dImmerseTrimeshCapsule (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseTrimeshCapsule has no body or the second has no fluid");

  //dContactGeom c[1];
  //const int collision = dCollideCCTL(o1, o2, 1, c, sizeof(dContact)); // not working properly for deep contacts
  const int collision = trimeshCapsuleCollide(o1, o2);
  if (collision == 0)
    return 0;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomCapsuleGetImmersionPlane(o2, fluidPlane);
  dxTriMesh *const trimesh = (dxTriMesh *) o1;

  return dImmerseTriMesh (trimesh, fluidPlane, flags, immersion);
}

int dImmerseTrimeshCylinder (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseTrimeshCylinder has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideCylinderTrimesh(o2, o1, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomCylinderGetImmersionPlane(o2, fluidPlane);
  dxTriMesh *const trimesh = (dxTriMesh *) o1;

  return dImmerseTriMesh (trimesh, fluidPlane, flags, immersion);
}

int dImmerseTrimeshSphere (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseTrimeshSphere has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideSTL(o1, o2, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomSphereGetImmersionPlane(o2, fluidPlane);

  return dImmerseTriMesh ((dxTriMesh*) o1, fluidPlane, flags, immersion);
}

int dImmerseTrimeshPlane (dxGeom *o1, dxGeom *o2, int flags,
      dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dIASSERT (o1->type == dTriMeshClass);
  dIASSERT (o2->type == dPlaneClass);
  dUASSERT(o1-> body && o2->fluid, "The first argument of dImmerseTrimeshPlane has no body or the second has no fluid");

  immersion->g1 = o1;
  immersion->g2 = o2;

  dxTriMesh *const trimesh = (dxTriMesh*) o1;
  const dxPlane *const plane = (dxPlane*) o2;

  return dImmerseTriMesh (trimesh, plane->p, flags, immersion);
}

int dImmerseTrimeshTrimesh (dxGeom *o1, dxGeom *o2, int flags, dImmersionGeom *immersion)
{
  dIASSERT ((flags & NUMI_MASK) >= 1);
  dUASSERT (o1->body && o2->fluid, "The first argument of dImmerseTrimeshTrimesh has no body or the second has no fluid");

  dContactGeom c[1];
  const int collision = dCollideTTL(o1, o2, 1, c, sizeof(dContact));
  if (collision == 0)
    return 0;

  immersion->g1 = o1;
  immersion->g2 = o2;

  dVector4 fluidPlane;
  dGeomTriMeshGetImmersionPlane(o2, fluidPlane);

  return dImmerseTriMesh ((dxTriMesh*) o1, fluidPlane, flags, immersion);
}
#endif // dTRIMESH_ENABLED
