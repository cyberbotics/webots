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

#ifndef _ODE_FLUID_DYNAMICS_H_
#define _ODE_FLUID_DYNAMICS_H_

#include <ode/common.h>
#include <ode/fluid_dynamics/common_fluid_dynamics.h>
#include <ode/fluid_dynamics/immersion.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup immerse Immersion Detection
 *
 * ODE has two main components: a dynamics simulation engine and a collision
 * detection engine. The collision engine is given information about the
 * shape of each body. At each time step it figures out which bodies touch
 * each other and passes the resulting contact point information (rigid body vs. rigid body) or immersion information (rigid body vs. fluid) to the user.
 * The user in turn creates contact joints between bodies.
 *
 * Using ODE's collision detection is optional - an alternative collision
 * detection system can be used as long as it can supply the right kinds of
 * contact information.
 */

/* ************************************************************************ */
/* general functions */

/**
 * @brief Set the fluid associated with a placeable geom.
 *
 * @param geom the geom to connect
 * @param fluid  the fluid to attach to the geom
 * @ingroup immerse
 */
ODE_API void dGeomSetFluid (dGeomID geom, dFluidID fluid);

/**
 * @brief Get the fluid associated with a placeable geom.
 * @param geom  the geom to query.
 * @sa dGeomSetFluid
 * @ingroup immerse
 */
ODE_API dFluidID dGeomGetFluid (dGeomID geom);

/**
 * @brief Get the geom area.
 * @param geom  the geom to query.
 * @ingroup immerse
 */
ODE_API dReal dGeomGetArea (dGeomID geom);

/**
 * @brief Get the geom volume.
 * @param geom  the geom to query.
 * @ingroup immerse
 */
ODE_API dReal dGeomGetVolume (dGeomID geom);

/**
 * @brief Get the geom immersion plane.
 * @param geom  the geom to query.
 * @param plane  the returned plane.
 * @ingroup immerse
 */
ODE_API void dGeomGetImmersionPlane(dGeomID geom, dVector4 plane);

/**
 * @brief return 1 if the point is below the immersion plane of the geom, 0 otherwise
 * @param geom  the geom to query.
 * @param x  the X coordinate of the point.
 * @param y  the Y coordinate of the point.
 * @param z  the Z coordinate of the point.
 * @ingroup immerse
 */
ODE_API int dGeomIsBelowImmersionPlane(dGeomID geom, dReal x, dReal y, dReal z);

/**
 * @brief Determines whether the given point is inside the geom
 *
 * @param geom  the geom to query.
 * @param x  the X coordinate of the point.
 * @param y  the Y coordinate of the point.
 * @param z  the Z coordinate of the point.
 *
 * @sa dGeomIsInside
 * @ingroup immerse_geom
 */
ODE_API int dGeomIsInside (dGeomID geom, dReal x, dReal y, dReal z);

/**
 * @brief Get the geom flags
 * @param geom  the geom to query.
 * @ingroup immerse
 */
ODE_API int dGeomGetFlags (dGeomID geom);

/* ************************************************************************ */
/* immersion detection */

/**
 *
 * @brief Given two geoms o1 and o2 that potentially intersect,
 * generate contact information for them.
 *
 * Internally, this just calls the correct class-specific collision
 * functions for o1 and o2.
 *
 * @param o1 The first geom to test.
 * @param o2 The second geom to test.
 *
 * @param flags The flags specify how immmersion should be generated
 * In the future it  may be used to select from different
 * immersion generation strategies.
 *
 * @param contact Points to an dImmersionGeom structure.
 *
 * @returns If the geoms intersect, this function returns 1, else 0.
 *
 * @remarks This function does not care if o1 and o2 are in the same space or not
 * (or indeed if they are in any space at all).
 *
 * @ingroup immerse
 */
ODE_API int dImmerse (dGeomID o1, dGeomID o2, int flags, dImmersionGeom *immersion);

/**
 * @brief Retrieves the volume of a sphere geom.
 *
 * @param box  the box to query.
 *
 * @sa dGeomBoxGetVolume
 * @ingroup immerse_box
 */
ODE_API dReal dGeomBoxGetVolume (dGeomID box);

/**
 * @brief Retrieves the area of a box geom.
 *
 * @param box  the box to query.
 *
 * @sa dGeomBoxGetArea
 * @ingroup immerse_box
 */
ODE_API dReal dGeomBoxGetArea (dGeomID box);

/**
 * @brief Retrieves the tangent plane to a box geom at a given point.
 *
 * @param box  the box to query.
 * @param x       the X coordinate of the point.
 * @param y       the Y coordinate of the point.
 * @param z       the Z coordinate of the point.
 * @param plane   the tangent plane.
 *
 * @sa dGeomBoxGetTangentPlane
 * @ingroup immerse_box
 */

ODE_API void dGeomBoxGetTangentPlane (dGeomID box, dReal x, dReal y, dReal z, dVector4 plane);

/**
 * @brief Retrieves plane equation of a given box face.
 *
 * @param box  the box to query.
 * @param faceIndex the face index (in the range {-1, -2, -3, 1, 2, 3}).
 * @param plane  the face plane.
 *
 * @sa dGeomBoxGetFacePlane
 * @ingroup immerse_box
 */
ODE_API void dGeomBoxGetFacePlane (dGeomID box, int faceIndex, dVector4 plane);

/**
 * @brief Retrieves the immersion plane used for a fluid bounded by a box.
 *
 * @param box  the box to query.
 * @param plane   the immersion plane.
 *
 * @sa dGeomCylinderGetImmersionPlane
 * @ingroup immerse_box
 */
ODE_API void dGeomBoxGetImmersionPlane (dGeomID box, dVector4 plane);

/**
 * @brief Retrieves the volume of a cylinder geom.
 *
 * @param cylinder  the cylinder to query.
 *
 * @sa dGeomCylinderGetVolume
 * @ingroup immerse_capsule
 */
ODE_API dReal dGeomCapsuleGetVolume (dGeomID capsule);

/**
 * @brief Retrieves the area of a capsule geom.
 *
 * @param capsule  the capsule to query.
 *
 * @sa dGeomCapsuleGetArea
 * @ingroup immerse_capsule
 */
ODE_API dReal dGeomCapsuleGetArea (dGeomID capsule);

/**
 * @brief Retrieves the tangent plane to a capsule geom at a given point.
 *
 * @param capsule  the capsule to query.
 * @param x       the X coordinate of the point.
 * @param y       the Y coordinate of the point.
 * @param z       the Z coordinate of the point.
 * @param plane   the tangent plane.
 *
 * @sa dGeomCapsuleGetTangentPlane
 * @ingroup immerse_capsule
 */
ODE_API void dGeomCapsuleGetTangentPlane (dGeomID capsule, dReal x, dReal y, dReal z, dVector4 plane);

/**
 * @brief Retrieves the immersion plane used for a fluid bounded by a capsule.
 *
 * @param capsule  the capsule to query.
 * @param plane   the immersion plane.
 *
 * @sa dGeomCapsuleGetImmersionPlane
 * @ingroup immerse_capsule
 */
ODE_API void dGeomCapsuleGetImmersionPlane (dGeomID capsule, dVector4 plane);

/**
 * @brief Retrieves the volume of a cylinder geom.
 *
 * @param cylinder  the cylinder to query.
 *
 * @sa dGeomCylinderGetVolume
 * @ingroup immerse_cylinder
 */
ODE_API dReal dGeomCylinderGetVolume (dGeomID cylinder);

/**
 * @brief Retrieves the area of a cylinder geom.
 *
 * @param cylinder  the cylinder to query.
 *
 * @sa dGeomCylinderGetArea
 * @ingroup immerse_cylinder
 */
ODE_API dReal dGeomCylinderGetArea (dGeomID cylinder);

/**
 * @brief Retrieves the tangent plane to a cylinder geom at a given point.
 *
 * @param cylinder  the cylinder to query.
 * @param x       the X coordinate of the point.
 * @param y       the Y coordinate of the point.
 * @param z       the Z coordinate of the point.
 * @param plane   the tangent plane.
 *
 * @sa dGeomCylinderGetTangentPlane
 * @ingroup immerse_cylinder
 */
ODE_API void dGeomCylinderGetTangentPlane (dGeomID cylinder, dReal x, dReal y, dReal z, dVector4 plane);

/**
 * @brief Retrieves plane equation of a given cylinder disk.
 *
 * @param cylinder  the cylinder to query.
 * @param faceIndex the disk index (in the range {-1, 1}).
 * @param plane  the disk plane (not normalized).
 *
 * @sa dGeomCylinderGetFacePlane
 * @ingroup immerse_cylinder
 */
ODE_API void dGeomCylinderGetDiskPlane (dGeomID cylinder, int diskIndex, dVector4 plane);

/**
 * @brief Retrieves the immersion plane used for a fluid bounded by a cylinder.
 *
 * @param cylinder  the cylinder to query.
 * @param plane   the immersion plane.
 *
 * @sa dGeomCylinderGetImmersionPlane
 * @ingroup immerse_cylinder
 */
ODE_API void dGeomCylinderGetImmersionPlane (dGeomID cylinder, dVector4 plane);

/**
 * @brief Retrieves the depth of the given point in the cylinder (negative if the point is outside, else min(distance to the caps, distance to the body) >= 0)
 *
 * @param cylinder  the cylinder to query.
 * @param x  the X coordinate of the point.
 * @param y  the Y coordinate of the point.
 * @param z  the Z coordinate of the point.
 *
 * @sa dGeomCylinderIsInside
 * @ingroup immerse_cylinder
 */
ODE_API dReal dGeomCylinderPointDepth (dGeomID cylinder, dReal x, dReal y, dReal z);

/**
 * @brief Retrieves the volume of a sphere geom.
 *
 * @param sphere  the sphere to query.
 *
 * @sa dGeomSphereGetVolume
 * @ingroup immerse_sphere
 */
ODE_API dReal dGeomSphereGetVolume (dGeomID sphere);

/**
 * @brief Retrieves the area of a sphere geom.
 *
 * @param sphere  the sphere to query.
 *
 * @sa dGeomSphereGetArea
 * @ingroup immerse_sphere
 */
ODE_API dReal dGeomSphereGetArea (dGeomID sphere);

/**
 * @brief Retrieves the tangent plane to a sphere geom at a given point.
 *
 * @param sphere  the sphere to query.
 * @param x       the X coordinate of the point.
 * @param y       the Y coordinate of the point.
 * @param z       the Z coordinate of the point.
 * @param plane   the tangent plane.
 *
 * @sa dGeomSphereGetTangentPlane
 * @ingroup immerse_sphere
 */
ODE_API void dGeomSphereGetTangentPlane (dGeomID sphere, dReal x, dReal y, dReal z, dVector4 plane);

/**
 * @brief Retrieves the immersion plane used for a fluid bounded by a sphere.
 *
 * @param cylinder  the sphere to query.
 * @param plane   the immersion plane.
 *
 * @sa dGeomSphereGetImmersionPlane
 * @ingroup immerse_sphere
 */
ODE_API void dGeomSphereGetImmersionPlane (dGeomID sphere, dVector4 plane);

/**
 * @brief Retrieves the volume of a trimesh geom.
 *
 * @param trimesh  the trimesh to query.
 *
 * @sa dGeomTriMeshGetVolume
 * @ingroup immerse_trimesh
 */
ODE_API dReal dGeomTriMeshGetVolume (dGeomID trimesh);

/**
 * @brief Retrieves the area of a trimesh geom.
 *
 * @param trimesh  the trimesh to query.
 *
 * @sa dGeomTriMeshGetArea
 * @ingroup immerse_trimesh
 */
ODE_API dReal dGeomTriMeshGetArea (dGeomID trimesh);

/**
 * @brief Retrieves the center of mass of a trimesh geom.
 *
 * @param trimesh  the trimesh to query.
 * @param c        the trimesh's center of mass.
 *
 * @sa dGeomTriMeshGetCenterOfMass
 * @ingroup immerse_trimesh
 */
ODE_API void dGeomTriMeshGetCenterOfMass (dGeomID trimesh, dVector3 c);

/**
 * @brief Retrieves the center of mass of a trimesh geom expressed in relative coordinates.
 *
 * @param trimesh  the trimesh to query.
 *
 * @sa dGeomTriMeshGetRelCenterOfMass
 * @ingroup immerse_trimesh
 */
ODE_API const dReal *dGeomTriMeshGetRelCenterOfMass (dGeomID trimesh);

/**
 * @brief Retrieves the inertia matrix of a trimesh geom with density 1.
 *
 * @param trimesh  the trimesh to query.
 *
 * @sa dGeomTriMeshGetInertiaMatrix
 * @ingroup immerse_trimesh
 */
ODE_API const dReal *dGeomTriMeshGetInertiaMatrix (dGeomID trimesh);

/**
 * @brief Retrieves the immersion plane used for a fluid bounded by a trimesh.
 *
 * @param cylinder  the trimesh to query.
 * @param plane   the immersion plane.
 *
 * @sa dGeomTriMeshGetImmersionPlane
 * @ingroup immerse_trimesh
 */
ODE_API void dGeomTriMeshGetImmersionPlane (dGeomID trimesh, dVector4 plane);

/**
 * @brief Retrieves plane equation of a given trimesh's bounding plane.
 *
 * @param trimesh  the trimesh to query.
 * @param planeIndex the bounding plane index (in the range {-1, -2, -3, 1, 2, 3}).
 * @param plane  the bounding plane.
 *
 * @sa dGeomTriMeshGetFacePlane
 * @ingroup immerse_trimesh
 */
ODE_API void dGeomTriMeshGetBoundingPlane (dGeomID trimesh, int planeIndex, dVector4 plane);

/**
 * @brief Calculate the depth of the a given point within a trimesh.
 *
 * @param trimesh  the trimesh to query.
 * @param x       the X coordinate of the point.
 * @param y       the Y coordinate of the point.
 * @param z       the Z coordinate of the point.
 *
 * @returns The depth of the point. Points inside the trimesh will have a
 * positive depth, points outside it will have a negative depth, and points
 * on the surface will have a depth of zero.
 *
 * @ingroup collide_trimesh
 */
ODE_API dReal dGeomTriMeshPointDepth (dGeomID trimesh, dReal x, dReal y, dReal z);

typedef int dImmerserFn (dGeomID o1, dGeomID o2,
			 int flags, dImmersionGeom *immersion);
typedef dImmerserFn * dGetImmerserFnFn (int num);

#ifdef __cplusplus
}
#endif

#endif
