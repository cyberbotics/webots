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

internal data structures and functions for immersion detection.

*/

#ifndef _ODE_IMMERSION_KERNEL_H_
#define _ODE_IMMERSION_KERNEL_H_

//****************************************************************************
// constants and macros

// mask for the number-of-immersion field in the dCollide() flags parameter //
#define NUMI_MASK (0xffff)

enum {
  dxImmersionOutlineDisabled =                  2,  // immersion outline info disabled
};

// default fluid plane's normal for a non-dxPlane geometry
#define FLUID_PLANE_NORMAL (2)

// threshold used to avoid buoyancy center computations based on a too small volume
#define VOLUME_ZERO_THRESHOLD 1e-10
//****************************************************************************
// Initialization and finalization functions

void dInitImmersers();
void dFinitImmersers();

#endif
