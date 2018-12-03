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

#ifndef _ODE_IMMERSION_H_
#define _ODE_IMMERSION_H_

#include <ode/common.h>
#include <ode/fluid_dynamics/common_fluid_dynamics.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  dImmersionProjectedAreas = 0x001,
  dImmersionImmersedArea = 0x002,
  dImmersionExposedArea = 0x003
};

typedef struct dImmersionSurfaceParameters {
  int mode;
  dVector3 dragForceCoefficients; // drag force coefficients along three different axes
  dVector3 dragTorqueCoefficients; // drag torque coefficients along three different axes
  dReal viscousResistanceForceCoefficient, viscousResistanceTorqueCoefficient;
  dReal maxAngularVel, maxLinearVel;
} dImmersionSurfaceParameters;

/**
 * @brief Describe the immersion locus between two geoms.
 *
 *
 * @ingroup collide
 */

typedef struct dStraightEdge {
  dVector3 origin, end;
} dStraightEdge;

typedef struct dCurvedEdge {
  // ellipse half axes
  dVector3 e1, e2;
  // ellipse center
  dVector3 center;
  // min angle, max angle
  dReal minAngle, maxAngle;
} dCurvedEdge;

typedef struct dImmersionGeom {
  dVector3 buoyancyCenter; ///< center of buoyancy
  dReal volume;           ///< immersed volume
  dReal area;          ///< immersed area
  dReal referenceArea; ///< immersed reference area  (depend on relative motion)
  dVector3 projectedAreas; /// projections of the immersed area along the three immersed dGeom axes
  dImmersionOutlineID outline; ///< optional immersion outline info
  dGeomID g1,g2;         ///< the colliding geoms

} dImmersionGeom;

/* immersion info used by immersion links */

typedef struct dImmersion {
  dImmersionSurfaceParameters surface;
  dImmersionGeom geom;
} dImmersion;

#ifdef __cplusplus
}
#endif

#endif
