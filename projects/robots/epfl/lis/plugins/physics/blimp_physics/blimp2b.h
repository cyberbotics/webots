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

Comments:
Some functions are specific to blimp2b (like b2b_compThrustForce),
therefore we included these function in this file.
BE CAREFUL, because some parameters are in the Webots *.wbt world
file:
        - b2b_mass (once again)
        - I_x, I_y, I_z, I_xy, I_xz, I_yz
        - friction coefficient
        - bounce

Remark that we have a bounce param for the envelope and another one
for front support (idem for friction parameters)

------------------------------------------------------------------------------*/

#ifndef _BLIMP2B_H
#define _BLIMP2B_H

#include <ode/common.h>

//------------------------------------------------------------------------------
// Dynamic model parameters

static const dReal b2b_m = REAL(0.184);  // mass of blimp2b [kg]
static const dReal b2b_addedMass[] =     // added mass parameters [m]
  {REAL(0.033), REAL(0.129), REAL(0.129)};

// Linear damping terms
static const dReal b2b_LinearDamp[] = {REAL(0),  // (here only linear
                                       REAL(0),  // damping for
                                       REAL(0),  // rotation (i=3,4,5)
                                       REAL(0.005), REAL(0.0079), REAL(0.0079)};

// Quadratic damping terms
static const dReal b2b_QuadraticDamp[] = {REAL(0.07),  //(here only
                                          REAL(0.28),  // quadratic
                                          REAL(0.22),  // damping for
                                          REAL(0),     // translation
                                          REAL(0),     // (terms i= 0,1,2)
                                          REAL(0)};

static const dReal b2b_rz = REAL(0.125);  // r is vector CG - CB (buoyancy frame) [m]

//------------------------------------------------------------------------------
// Engines: command [-1.0..+1.0] to thrust [N] converters

void b2b_commandsToThrust(double inFront, double inYaw, double inVert, dReal *propThrusts);
void b2b_compThrustWrench(const dReal *propThrusts, dReal *genForceAB);

#endif  // _BLIMP2B_H
