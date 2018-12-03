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

// object, body, and world structs.

#ifndef _ODE_FLUID_DYNAMICS_OBJECT_H_
#define _ODE_FLUID_DYNAMICS_OBJECT_H_

#include <ode/common.h>
#include "objects.h"

// some fluid flags

enum {
  dxFluidDisabled =                  1,  // fluid is disabled
};

struct dxFluid : public dObject {
  unsigned flags;  // some dxFluidFlagXXX flags
  dGeomID geom;  // first collision geom associated with fluid
  dxImmersionLink *firstimmersionlink; // list of attached immersion link

  dReal density;
  dReal viscosity;
  dVector3 streamVelocity;
  dxFluid(dxWorld *w);
};

#endif
