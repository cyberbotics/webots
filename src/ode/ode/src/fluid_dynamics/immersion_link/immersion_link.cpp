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

#include <ode/ode.h>
#include "config.h"
#include "immersion_link.h"
#include "../objects_fluid_dynamics.h"

dxImmersionLink::dxImmersionLink( dxWorld *w ) :
        dObject( w )
{
    //printf("constructing %p\n", this);
    dIASSERT( w );
    enabled = true;
    fluid = 0;
    group = 0;
    body = 0;
    nextBodyImmersionLink = 0;
    nextFluidImmersionLink = 0;
}

dxImmersionLink::~dxImmersionLink()
{ }

bool dxImmersionLink::isEnabled() const
{
    return ( enabled &&
             (body->invMass > 0.0 ||
             (fluid->density > 0.0)) );
}

// Local Variables:
// mode:c++
// c-basic-offset:4
// End:
