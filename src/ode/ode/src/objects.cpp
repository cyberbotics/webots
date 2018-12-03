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

// Object, body, and world methods.

#include <ode/common.h>
#include <ode/objects.h>
#include "config.h"
#include "matrix.h"
#include "objects.h"
#include "util.h"

#define dWORLD_DEFAULT_GLOBAL_ERP REAL(0.2)

#if defined(dSINGLE)
#define dWORLD_DEFAULT_GLOBAL_CFM REAL(1e-5)
#elif defined(dDOUBLE)
#define dWORLD_DEFAULT_GLOBAL_CFM REAL(1e-10)
#else
#error dSINGLE or dDOUBLE must be defined
#endif

dObject::~dObject()
{
    // Do nothing - a virtual destructor
}

dxAutoDisable::dxAutoDisable(void *):
    idle_time(REAL(0.0)),
    idle_steps(10),
    average_samples(1), // Default is 1 sample => Instantaneous velocity
    linear_average_threshold(REAL(0.01)*REAL(0.01)), // (magnitude squared)
    angular_average_threshold(REAL(0.01)*REAL(0.01)) // (magnitude squared)
{
}

dxDampingParameters::dxDampingParameters(void *):
    linear_scale(REAL(0.0)),
    angular_scale(REAL(0.0)),
    linear_threshold(REAL(0.01) * REAL(0.01)),
    angular_threshold(REAL(0.01) * REAL(0.01))
{
}

dxQuickStepParameters::dxQuickStepParameters(void *):
    num_iterations(20),
    w(REAL(1.3))
{
}

dxContactParameters::dxContactParameters(void *):
    max_vel(dInfinity),
    min_depth(REAL(0.0))
{
}

dxWorld::dxWorld():
    dBase(),
    firstbody(NULL),
    firstjoint(NULL),
    nb(0),
    nj(0),
    global_erp(dWORLD_DEFAULT_GLOBAL_ERP),
    global_cfm(dWORLD_DEFAULT_GLOBAL_CFM),
    adis(NULL),
    body_flags(0),
    wmem(NULL),
    qs(NULL),
    contactp(NULL),
    dampingp(NULL),
    max_angular_speed(dInfinity),
    userdata(0)
{
    dSetZero (gravity, 4);
}

dxWorld::~dxWorld()
{
    if (wmem)
    {
        wmem->Release();
    }
}
