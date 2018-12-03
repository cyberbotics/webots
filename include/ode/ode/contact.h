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

#ifndef _ODE_CONTACT_H_
#define _ODE_CONTACT_H_

#include <ode/common.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  dContactMu2	  = 0x001,      /**< Use axis dependent friction */
  dContactAxisDep = 0x001,      /**< Same as above */
  dContactFDir1	  = 0x002,      /**< Use FDir for the first friction value */
  dContactBounce  = 0x004,      /**< Restore collision energy anti-parallel to the normal */
  dContactSoftERP = 0x008,      /**< Don't use global erp for penetration reduction */
  dContactSoftCFM = 0x010,      /**< Don't use global cfm for penetration constraint */
  dContactMotion1 = 0x020,      /**< Use a non-zero target velocity for the constraint */
  dContactMotion2 = 0x040,
  dContactMotionN = 0x080,
  dContactSlip1	  = 0x100,      /**< Force-dependent slip. */
  dContactSlip2	  = 0x200,
  dContactRolling = 0x400,      /**< Rolling/Angular friction */

  dContactApprox0   = 0x0000,
  dContactApprox1_1 = 0x1000,
  dContactApprox1_2 = 0x2000,
  dContactApprox1_N = 0x4000,   /**< For rolling friction */
  dContactApprox1   = 0x7000
};

typedef struct dSurfaceParameters {
  /* must always be defined */
  int   mode;
  dReal mu;

  /* only defined if the corresponding flag is set in mode */
  dReal mu2;
  dReal rho;                    /**< Rolling friction */
  dReal rho2;
  dReal rhoN;                   /**< Spinning friction */
  dReal bounce;                 /**< Coefficient of restitution */
  dReal bounce_vel;             /**< Bouncing threshold */
  dReal soft_erp;
  dReal soft_cfm;
  dReal motion1,motion2,motionN;
  dReal slip1,slip2;
} dSurfaceParameters;

/**
 * @brief Describe the contact point between two geoms.
 *
 * If two bodies touch, or if a body touches a static feature in its
 * environment, the contact is represented by one or more "contact
 * points", described by dContactGeom.
 *
 * The convention is that if body 1 is moved along the normal vector by
 * a distance depth (or equivalently if body 2 is moved the same distance
 * in the opposite direction) then the contact depth will be reduced to
 * zero. This means that the normal vector points "in" to body 1.
 *
 * @ingroup collide
 */
typedef struct dContactGeom {
    dVector3 pos;          /*< contact position*/
    dVector3 normal;       /*< normal vector*/
    dReal depth;           /*< penetration depth*/
    dGeomID g1,g2;         /*< the colliding geoms*/
    int side1,side2;       /*< (to be documented)*/
} dContactGeom;

/* contact info used by contact joint */

typedef struct dContact {
  dSurfaceParameters surface;
  dContactGeom geom;
  dVector3 fdir1;
} dContact;

#ifdef __cplusplus
}
#endif

#endif
