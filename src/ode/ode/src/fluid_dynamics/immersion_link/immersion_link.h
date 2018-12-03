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

#ifndef _ODE_IMMERSION_LINK_H_
#define _ODE_IMMERSION_LINK_H_

#include <ode/fluid_dynamics/immersion.h>
#include <ode/fluid_dynamics/common_fluid_dynamics.h>
#include "../../objects.h"
#include "../../obstack.h"

struct dxImmersionLink : public dObject
{

    dxBody *body;               // body this immersion is connected to
    dxFluid *fluid;             // fluid this immersion is connected to
    dxImmersionLinkGroup *group;  // group this immersion is connected to
    dxImmersionLink *nextBodyImmersionLink;  // next immersion in body's list of connected immersions
    dxImmersionLink *nextFluidImmersionLink;  // next immersion in body's list of connected immersions
    dxImmersionLink *nextGroupImmersionLink;  // next immersion in body's list of connected immersions

    dImmersion immersion;

    bool enabled;

    dxImmersionLink( dxWorld *w );
    virtual ~dxImmersionLink();

   // Test if this immersion should be used in the simulation step
   // (has the enabled flag set, and is attached to at least one dynamic body)
   bool isEnabled() const;

   bool mtTag; // used in clustering algorithm to indicate that the immersion was reattached
};

// immersion group. NOTE: any immersions in the group that have their world destroyed
// will have their world pointer set to 0.

struct dxImmersionLinkGroup : public dBase
{
  dxImmersionLink *firstimmersionlink; // list of attached immersion link
};

#endif

// Local Variables:
// mode:c++
// c-basic-offset:4
// End:
