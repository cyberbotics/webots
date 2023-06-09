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

#include <ode/odeconfig.h>
#include "config.h"
#include "transmission.h"
#include "joint_internal.h"

namespace {
    inline dReal clamp(dReal x, dReal minX, dReal maxX)
    {
        return x < minX ? minX : (x > maxX ? maxX : x);
    }
}

/*
 * Transmission joint
 */

dxJointTransmission::dxJointTransmission(dxWorld* w) :
    dxJoint(w)
{
    int i;

    flags |= dJOINT_TWOBODIES;
    mode = dTransmissionParallelAxes;

    cfm = world->global_cfm;
    erp = world->global_erp;

    for (i = 0 ; i < 2 ; i += 1) {
        dSetZero( anchors[i], 4 );
        dSetZero( axes[i], 4 );
        axes[i][0] = 1;

        radii[i] = 0;
    }

    backlash = 0;
    ratio = 1;
    update = 1;
}

void
dxJointTransmission::getSureMaxInfo( SureMaxInfo* info )
{
    info->max_m = 1;
}

void
dxJointTransmission::getInfo1( dxJoint::Info1* info )
{
    // If there's backlash in the gears then constraint must be
    // unilateral, that is the driving gear can only push the driven
    // gear in one direction.  In order to push it in the other it
    // first needs to traverse the backlash gap.

    info->m = 1;
    info->nub = backlash > 0 ? 0 : 1;
}

void
dxJointTransmission::getInfo2( dReal worldFPS,
                               dReal /*worldERP*/,
                               const Info2Descr* info )
 {
    dVector3 a[2], n[2], l[2], r[2], c[2], s, t, O, d, z, u, v;
    dReal theta, delta, nn, na_0, na_1, cosphi, sinphi, m;
    const dReal *p[2], *omega[2];
    int i;

    // Transform all needed quantities to the global frame.

    for (i = 0 ; i < 2 ; i += 1) {
        dBodyGetRelPointPos(node[i].body,
                            anchors[i][0], anchors[i][1], anchors[i][2],
                            a[i]);

        dBodyVectorToWorld(node[i].body, axes[i][0], axes[i][1], axes[i][2],
                           n[i]);

        p[i] = dBodyGetPosition(node[i].body);
        omega[i] = dBodyGetAngularVel(node[i].body);
    }

    if (update) {
        // Make sure both gear reference frames end up with the same
        // handedness.

        if (dCalcVectorDot3(n[0], n[1]) < 0) {
            dNegateVector3(axes[0]);
            dNegateVector3(n[0]);
        }
    }

    // Calculate the mesh geometry based on the current mode.

    switch (mode) {
    case dTransmissionParallelAxes:
        // Simply calculate the contact point as the point on the
        // baseline that will yield the correct ratio.

        dIASSERT (ratio > 0);

        dSubtractVectors3(d, a[1], a[0]);
        dAddScaledVectors3(c[0], a[0], d, 1, ratio / (1 + ratio));
        dCopyVector3(c[1], c[0]);

        dNormalize3(d);

        for (i = 0 ; i < 2 ; i += 1) {
            dCalcVectorCross3(l[i], d, n[i]);
        }

        break;
    case dTransmissionIntersectingAxes:
        // Calculate the line of intersection between the planes of the
        // gears.

        dCalcVectorCross3(l[0], n[0], n[1]);
        dCopyVector3(l[1], l[0]);

        nn = dCalcVectorDot3(n[0], n[1]);
        dIASSERT(fabs(nn) != 1);

        na_0 = dCalcVectorDot3(n[0], a[0]);
        na_1 = dCalcVectorDot3(n[1], a[1]);

        dAddScaledVectors3(O, n[0], n[1],
                           (na_0 - na_1 * nn) / (1 - nn * nn),
                           (na_1 - na_0 * nn) / (1 - nn * nn));

        // Find the contact point as:
        //
        // c = ((r_a - O) . l) l + O
        //
        // where r_a the anchor point of either gear and l, O the tangent
        // line direction and origin.

        for (i = 0 ; i < 2 ; i += 1) {
            dSubtractVectors3(d, a[i], O);
            m = dCalcVectorDot3(d, l[i]);
            dAddScaledVectors3(c[i], O, l[i], 1, m);
        }

        break;
    case dTransmissionChainDrive:
        dSubtractVectors3(d, a[0], a[1]);
        m = dCalcVectorLength3(d);

        dIASSERT(m > 0);

        // Calculate the angle of the contact point relative to the
        // baseline.

        cosphi = clamp((radii[1] - radii[0]) / m, REAL(-1.0), REAL(1.0)); // Force into range to fix possible computation errors
        sinphi = dSqrt (REAL(1.0) - cosphi * cosphi);

        dNormalize3(d);

        for (i = 0 ; i < 2 ; i += 1) {
            // Calculate the contact radius in the local reference
            // frame of the chain.  This has axis x pointing along the
            // baseline, axis y pointing along the sprocket axis and
            // the remaining axis normal to both.

            u[0] = radii[i] * cosphi;
            u[1] = 0;
            u[2] = radii[i] * sinphi;

            // Transform the contact radius into the global frame.

            dCalcVectorCross3(z, d, n[i]);

            v[0] = dCalcVectorDot3(d, u);
            v[1] = dCalcVectorDot3(n[i], u);
            v[2] = dCalcVectorDot3(z, u);

            // Finally calculate contact points and l.

            dAddVectors3(c[i], a[i], v);
            dCalcVectorCross3(l[i], v, n[i]);
            dNormalize3(l[i]);

            // printf ("%d: %f, %f, %f\n",
            //      i, l[i][0], l[i][1], l[i][2]);
        }

        break;
    default:
        dSetZero(c[0], 3);
        dSetZero(c[1], 3);

        break;
    }

    if (update) {
        // We need to calculate an initial reference frame for each
        // wheel which we can measure the current phase against.  This
        // frame will have the initial contact radius as the x axis,
        // the wheel axis as the z axis and their cross product as the
        // y axis.

        for (i = 0 ; i < 2 ; i += 1) {
            dSubtractVectors3 (r[i], c[i], a[i]);
            radii[i] = dCalcVectorLength3(r[i]);
            dIASSERT(radii[i] > 0);

            dBodyVectorFromWorld(node[i].body, r[i][0], r[i][1], r[i][2],
                                 reference[i]);
            dNormalize3(reference[i]);
            dCopyVector3(reference[i] + 8, axes[i]);
            dCalcVectorCross3(reference[i] + 4, reference[i] + 8, reference[i]);

            // printf ("%f\n", dDOT(r[i], n[i]));
            // printf ("(%f, %f, %f,\n %f, %f, %f,\n %f, %f, %f)\n",
            //      reference[i][0],reference[i][1],reference[i][2],
            //      reference[i][4],reference[i][5],reference[i][6],
            //      reference[i][8],reference[i][9],reference[i][10]);

            phase[i] = 0;
        }

        ratio = radii[0] / radii[1];
        update = 0;
    }

    for (i = 0 ; i < 2 ; i += 1) {
        dReal phase_hat;

        dSubtractVectors3 (r[i], c[i], a[i]);

        // Transform the (global) contact radius into the gear's
        // reference frame.

        dBodyVectorFromWorld (node[i].body, r[i][0], r[i][1], r[i][2], s);
        dMultiply0_331(t, reference[i], s);

        // Now simply calculate its angle on the plane relative to the
        // x-axis which is the initial contact radius.  This will be
        // an angle between -pi and pi that is coterminal with the
        // actual phase of the wheel.  To find the real phase we
        // estimate it by adding omega * dt to the old phase and then
        // find the closest angle to that, that is coterminal to
        // theta.

        theta = atan2(t[1], t[0]);
        phase_hat = phase[i] + dCalcVectorDot3(omega[i], n[i]) / worldFPS;

        if (phase_hat > M_PI_2) {
            if (theta < 0) {
                theta += (dReal)(2 * M_PI);
            }

            theta += (dReal)(floor(phase_hat / (2 * M_PI)) * (2 * M_PI));
        } else if (phase_hat < -M_PI_2) {
            if (theta > 0) {
                theta -= (dReal)(2 * M_PI);
            }

            theta += (dReal)(ceil(phase_hat / (2 * M_PI)) * (2 * M_PI));
        }

        if (phase_hat - theta > M_PI) {
            phase[i] = theta + (dReal)(2 * M_PI);
        } else if (phase_hat - theta < -M_PI) {
            phase[i] = theta - (dReal)(2 * M_PI);
        } else {
            phase[i] = theta;
        }

        dIASSERT(fabs(phase_hat - phase[i]) < M_PI);
    }

    // Calculate the phase error.  Depending on the mode the condition
    // is that the distances traveled by each contact point must be
    // either equal (chain and sprockets) or opposite (gears).

    if (mode == dTransmissionChainDrive) {
        delta = (dCalcVectorLength3(r[0]) * phase[0] -
                 dCalcVectorLength3(r[1]) * phase[1]);
    } else {
        delta = (dCalcVectorLength3(r[0]) * phase[0] +
                 dCalcVectorLength3(r[1]) * phase[1]);
    }

    // When in chain mode a torque reversal, signified by the change
    // in sign of the wheel phase difference, has the added effect of
    // switching the active chain branch.  We must therefore reflect
    // the contact points and tangents across the baseline.

    if (mode == dTransmissionChainDrive && delta < 0) {
        dVector3 d;

        dSubtractVectors3(d, a[0], a[1]);

        for (i = 0 ; i < 2 ; i += 1) {
            dVector3 nn;
            dReal a;

            dCalcVectorCross3(nn, n[i], d);
            a = dCalcVectorDot3(nn, nn);
            dIASSERT(a > 0);

            dAddScaledVectors3(c[i], c[i], nn,
                               1, -2 * dCalcVectorDot3(c[i], nn) / a);
            dAddScaledVectors3(l[i], l[i], nn,
                               -1, 2 * dCalcVectorDot3(l[i], nn) / a);
        }
    }

    // Do not add the constraint if there's backlash and we're in the
    // backlash gap.

    if (backlash == 0 || fabs(delta) > backlash) {
        // The constraint is satisfied if the absolute velocity of the
        // contact point projected onto the tangent of the wheels is equal
        // for both gears.  This velocity can be calculated as:
        //
        // u = v + omega x r_c
        //
        // The constraint therefore becomes:
        // (v_1 + omega_1 x r_c1) . l = (v_2 + omega_2 x r_c2) . l <=>
        // (v_1 . l + (r_c1 x l) . omega_1 = v_2 . l + (r_c2 x l) . omega_2

        for (i = 0 ; i < 2 ; i += 1) {
            dSubtractVectors3 (r[i], c[i], p[i]);
        }

        dCalcVectorCross3(info->J1a, r[0], l[0]);
        dCalcVectorCross3(info->J2a, l[1], r[1]);

        dCopyVector3(info->J1l, l[0]);
        dCopyNegatedVector3(info->J2l, l[1]);

        if (delta > 0) {
            if (backlash > 0) {
                info->lo[0] = -dInfinity;
                info->hi[0] = 0;
            }

            info->c[0] = -worldFPS * erp * (delta - backlash);
        } else {
            if (backlash > 0) {
                info->lo[0] = 0;
                info->hi[0] = dInfinity;
            }

            info->c[0] = -worldFPS * erp * (delta + backlash);
        }
    }

    info->cfm[0] = cfm;

    // printf ("%f, %f, %f, %f, %f\n", delta, phase[0], phase[1], -phase[1] / phase[0], ratio);

    // Cache the contact point (in world coordinates) to avoid
    // recalculation if requested by the user.

    dCopyVector3(contacts[0], c[0]);
    dCopyVector3(contacts[1], c[1]);
}

void dJointSetTransmissionAxis( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    int i;

    dUASSERT( joint, "bad joint argument" );
    dUASSERT(joint->mode == dTransmissionParallelAxes ||
             joint->mode == dTransmissionChainDrive ,
             "axes must be set individualy in current mode" );

    for (i = 0 ; i < 2 ; i += 1) {
        if (joint->node[i].body) {
            dBodyVectorFromWorld(joint->node[i].body, x, y, z, joint->axes[i]);
            dNormalize3(joint->axes[i]);
        }
    }

    joint->update = 1;
}

void dJointSetTransmissionAxis1( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT(joint->mode == dTransmissionIntersectingAxes,
             "can't set individual axes in current mode" );

    if (joint->node[0].body) {
        dBodyVectorFromWorld(joint->node[0].body, x, y, z, joint->axes[0]);
        dNormalize3(joint->axes[0]);
    }

    joint->update = 1;
}

void dJointSetTransmissionAxis2( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT(joint->mode == dTransmissionIntersectingAxes,
             "can't set individual axes in current mode" );

    if (joint->node[1].body) {
        dBodyVectorFromWorld(joint->node[1].body, x, y, z, joint->axes[1]);
        dNormalize3(joint->axes[1]);
    }

    joint->update = 1;
}

void dJointSetTransmissionAnchor1( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );

    if (joint->node[0].body) {
        dBodyGetPosRelPoint(joint->node[0].body, x, y, z, joint->anchors[0]);
    }

    joint->update = 1;
}

void dJointSetTransmissionAnchor2( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );

    if (joint->node[1].body) {
        dBodyGetPosRelPoint(joint->node[1].body, x, y, z, joint->anchors[1]);
    }

    joint->update = 1;
}

void dJointGetTransmissionContactPoint1( dJointID j, dVector3 result )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );

    dCopyVector3(result, joint->contacts[0]);
}

void dJointGetTransmissionContactPoint2( dJointID j, dVector3 result )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );

    dCopyVector3(result, joint->contacts[1]);
}

void dJointGetTransmissionAxis( dJointID j, dVector3 result )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );
    dUASSERT(joint->mode == dTransmissionParallelAxes,
             "axes must be queried individualy in current mode" );

    if (joint->node[0].body) {
        dBodyVectorToWorld(joint->node[0].body,
                           joint->axes[0][0],
                           joint->axes[0][1],
                           joint->axes[0][2],
                           result);
    }
}

void dJointGetTransmissionAxis1( dJointID j, dVector3 result )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );

    if (joint->node[0].body) {
        dBodyVectorToWorld(joint->node[0].body,
                           joint->axes[0][0],
                           joint->axes[0][1],
                           joint->axes[0][2],
                           result);
    }
}

void dJointGetTransmissionAxis2( dJointID j, dVector3 result )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );

    if (joint->node[1].body) {
        dBodyVectorToWorld(joint->node[1].body,
                           joint->axes[1][0],
                           joint->axes[1][1],
                           joint->axes[1][2],
                           result);
    }
}

void dJointGetTransmissionAnchor1( dJointID j, dVector3 result )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );

    if (joint->node[0].body) {
        dBodyGetRelPointPos(joint->node[0].body,
                            joint->anchors[0][0],
                            joint->anchors[0][1],
                            joint->anchors[0][2],
                            result);
    }
}

void dJointGetTransmissionAnchor2( dJointID j, dVector3 result )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );

    if (joint->node[1].body) {
        dBodyGetRelPointPos(joint->node[1].body,
                            joint->anchors[1][0],
                            joint->anchors[1][1],
                            joint->anchors[1][2],
                            result);
    }
}

void dJointSetTransmissionParam( dJointID j, int parameter, dReal value )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );

    switch ( parameter ) {
        case dParamCFM:
            joint->cfm = value;
            break;
        case dParamERP:
            joint->erp = value;
            break;
    }
}

dReal dJointGetTransmissionParam( dJointID j, int parameter )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );

    switch ( parameter ) {
        case dParamCFM:
            return joint->cfm;
        case dParamERP:
            return joint->erp;
        default:
            return 0;
    }
}

void dJointSetTransmissionMode( dJointID j, int mode )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( mode == dTransmissionParallelAxes ||
              mode == dTransmissionIntersectingAxes ||
              mode == dTransmissionChainDrive, "invalid joint mode" );

    joint->mode = mode;
}

int dJointGetTransmissionMode( dJointID j )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );

    return joint->mode;
}

void dJointSetTransmissionRatio( dJointID j, dReal ratio )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( joint->mode == dTransmissionParallelAxes,
              "can't set ratio explicitly in current mode" );
    dUASSERT( ratio > 0, "ratio must be positive" );

    joint->ratio = ratio;
}

dReal dJointGetTransmissionRatio( dJointID j )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );

    return joint->ratio;
}

dReal dJointGetTransmissionAngle1( dJointID j )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );

    return joint->phase[0];
}

dReal dJointGetTransmissionAngle2( dJointID j )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );

    return joint->phase[1];
}

dReal dJointGetTransmissionRadius1( dJointID j )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );

    return joint->radii[0];
}

dReal dJointGetTransmissionRadius2( dJointID j )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );

    return joint->radii[1];
}

void dJointSetTransmissionRadius1( dJointID j, dReal radius )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( joint->mode == dTransmissionChainDrive,
              "can't set wheel radius explicitly in current mode" );

    joint->radii[0] = radius;
}

void dJointSetTransmissionRadius2( dJointID j, dReal radius )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( joint->mode == dTransmissionChainDrive,
              "can't set wheel radius explicitly in current mode" );

    joint->radii[1] = radius;
}

dReal dJointGetTransmissionBacklash( dJointID j )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );

    return joint->backlash;
}

void dJointSetTransmissionBacklash( dJointID j, dReal backlash )
{
    dxJointTransmission* joint = static_cast<dxJointTransmission*>(j);
    dUASSERT( joint, "bad joint argument" );

    joint->backlash = backlash;
}

dJointType
dxJointTransmission::type() const
{
    return dJointTypeTransmission;
}

size_t
dxJointTransmission::size() const
{
    return sizeof( *this );
}
