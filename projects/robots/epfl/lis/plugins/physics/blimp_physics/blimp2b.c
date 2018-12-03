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

#include "blimp2b.h"

static const dReal SinAlphaTimesdAlpha = REAL(0.144);
static const dReal SinBetaTimesdBeta = REAL(0.073);
static const dReal CosBetaTimesdBeta = REAL(0.484);
static const dReal SinGammaTimesdGamma = REAL(0.045);

//------------------------------------------------------------------------------
// Engines: command [-1.0...1.0] to thrust [N] converters

void b2b_commandsToThrust(double inFront, double inYaw, double inVert, dReal *propThrusts) {
  // our thrusters do not have same power in both direction due to propeller shape.
  propThrusts[0] = inFront * (inFront < 0 ? REAL(0.0450) : REAL(0.0400));
  propThrusts[1] = inYaw * (inYaw < 0 ? REAL(0.0250) : REAL(0.0300));
  propThrusts[2] = inVert * (inVert < 0 ? REAL(0.0450) : REAL(0.0250));
}

void b2b_compThrustWrench(const dReal *propThrusts, dReal *genForceAB) {
  genForceAB[0] += +propThrusts[0];
  genForceAB[1] += -propThrusts[1];
  genForceAB[2] += +propThrusts[2];
  genForceAB[3] += +SinBetaTimesdBeta * propThrusts[1];
  genForceAB[4] += +SinAlphaTimesdAlpha * propThrusts[0] - SinGammaTimesdGamma * propThrusts[2];
  genForceAB[5] += +CosBetaTimesdBeta * propThrusts[1];
}
