/****************************************************************************

blimp_physics -- A blimp physics model for Webots.

Copyright (C) 2006 Laboratory of Intelligent Systems, EPFL, Lausanne
Authors:    Alexis Guanella            guanella@ini.phys.ethz.ch
            Antoine Beyeler            antoine.beyeler@epfl.ch
            Jean-Christophe Zufferey   jean-christophe.zufferey@epfl.ch
            Dario Floreano             dario.floreano@epfl.ch
Web: http://lis.epfl.ch

The authors of any publication arising from research using this software are
kindly requested to add the following reference:

        Zufferey, J.C., Guanella, A., Beyeler, A., Floreano, D. (2006) Flying over
        the Reality Gap: From Simulated to Real Indoor Airships. Autonomous Robots,
        Springer US.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

******************************************************************************/
/*------------------------------------------------------------------------------

Author:		Alexis Guanella (ag)
                        Antoine Beyeler (ab)

------------------------------------------------------------------------------*/

#ifndef _BLIMPMODEL_H
#define _BLIMPMODEL_H

//----------------------------------------------------------------------------//
// Includes
#include <ode/common.h>

//----------------------------------------------------------------------------//
// Dynamic model parameters
static const dReal g = REAL(9.81);  // same value in Webots

//----------------------------------------------------------------------------//
// Dynamic model function

void bmod_ComputeGenForces(const dReal *inGenPosA, const dReal *inGenVelAB, const dReal *inPropThrusts, dReal *outGenForceAB);

#endif  // _BLIMPMODEL_H
