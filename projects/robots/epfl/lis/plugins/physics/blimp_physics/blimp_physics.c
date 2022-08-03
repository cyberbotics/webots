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

Author:		Antoine Beyeler (ab)

------------------------------------------------------------------------------*/

#include <plugins/physics.h>

#include "blimp2b.h"
#include "blimpmodel.h"
#include "utils.h"

// constants
const char kRobotName[] = "blimp_lis";
const dReal kRotWebotsToAeroBody[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
const dReal kRotAeroToWebotsBody[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

// globals
static dBodyID gRobotBody = NULL;

//-----------------------------------------------------------------
// Physics plug-in implementation

DLLEXPORT void webots_physics_init() {
  // init global variables
  gRobotBody = dWebotsGetBodyFromDEF(kRobotName);
  if (gRobotBody == NULL)
    dWebotsConsolePrintf("!!! blimp_physics :: webots_physics_init :: error : could not get body of robot.\r\n");
  else {
    // disable gravity for the blimp: buoyancy counteract gravity.
    dBodySetGravityMode(gRobotBody, 0);
  }
}

DLLEXPORT void webots_physics_step() {
  Matrix33 rotBodyToFrame;
  Matrix33 rotFrameToBody;
  Matrix33 rotFrameToAeroBody;
  Matrix33 tmp1, tmp2;
  dReal *ode;
  Vector6 genPos, genSpeed, genForce;
  dVector3 propThrusts;
  double *controls;
  int size;

  // we return if we have no robot to actuate.
  if (gRobotBody == NULL)
    return;

  // read control surfaces values
  controls = (double *)dWebotsReceive(&size);
  if (size != 3 * sizeof(double)) {
    // physics plugin and controller run with different frequencies therefore it is not
    // guarantied that control value are present in the buffer for each webots_physics_step()
    return;
  }

  b2b_commandsToThrust(controls[0], controls[1], controls[2], propThrusts);

  // get rotation matrix from webots frame coordinate to aero body coordinate
  ode = (dReal *)dBodyGetRotation(gRobotBody);
  rotBodyToFrame[0] = ode[0];
  rotBodyToFrame[1] = ode[1];
  rotBodyToFrame[2] = ode[2];
  rotBodyToFrame[3] = ode[4];
  rotBodyToFrame[4] = ode[5];
  rotBodyToFrame[5] = ode[6];
  rotBodyToFrame[6] = ode[8];
  rotBodyToFrame[7] = ode[9];
  rotBodyToFrame[8] = ode[10];
  utils_Assign(rotFrameToBody, rotBodyToFrame, 9);
  utils_InvertMatrix33(rotFrameToBody);
  utils_Multiply(rotFrameToAeroBody, kRotWebotsToAeroBody, rotFrameToBody, 3, 3, 3);

  // get the velocity in aero body coordinate...
  utils_Multiply(genSpeed, rotFrameToAeroBody, dBodyGetLinearVel(gRobotBody), 3, 3, 1);
  utils_Multiply(&genSpeed[3], rotFrameToAeroBody, dBodyGetAngularVel(gRobotBody), 3, 3, 1);

  // get the body position...
  utils_Multiply(genPos, kRotWebotsToAeroBody, dBodyGetPosition(gRobotBody), 3, 3, 1);

  // ...and rotation.
  utils_Multiply(tmp1, rotFrameToBody, kRotAeroToWebotsBody, 3, 3, 3);
  utils_Multiply(tmp2, kRotWebotsToAeroBody, tmp1, 3, 3, 3);
  genPos[3] = dAtan2(tmp2[5], tmp2[8]);
  genPos[4] = dAsin(-tmp2[2]);
  genPos[5] = dAtan2(tmp2[1], tmp2[0]);

  // run blimp model
  bmod_ComputeGenForces(genPos, genSpeed, propThrusts, tmp1);

  // convert back to webots coordinate...
  utils_Multiply(genForce, kRotAeroToWebotsBody, tmp1, 3, 3, 1);
  utils_Multiply(&genForce[3], kRotAeroToWebotsBody, &tmp1[3], 3, 3, 1);

  // ...and apply to body.
  dBodyAddRelForce(gRobotBody, genForce[0], genForce[1], genForce[2]);
  dBodyAddRelTorque(gRobotBody, genForce[3], genForce[4], genForce[5]);
}

DLLEXPORT int webots_physics_collide(dGeomID g1, dGeomID g2) {
  // We don't want to handle collisions
  return 0;
}

DLLEXPORT void webots_physics_cleanup() {
}
