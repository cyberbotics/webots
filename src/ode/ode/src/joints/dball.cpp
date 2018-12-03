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
#include "dball.h"
#include "joint_internal.h"

/*
 * Double Ball joint: tries to maintain a fixed distance between two anchor
 * points.
 */

dxJointDBall::dxJointDBall(dxWorld *w) :
    dxJoint(w)
{
    dSetZero(anchor1, 3);
    dSetZero(anchor2, 3);
    targetDistance = 0;
    erp = world->global_erp;
    cfm = world->global_cfm;
}

void
dxJointDBall::getSureMaxInfo( SureMaxInfo* info )
{
    info->max_m = 1;
}
void
dxJointDBall::getInfo1( dxJoint::Info1 *info )
{
    info->m = 1;
    info->nub = 1;
}

void
dxJointDBall::getInfo2( dReal worldFPS, dReal /*worldERP*/, const Info2Descr *info )
{
    info->cfm[0] = this->cfm;

    dVector3 globalA1, globalA2;
    dBodyGetRelPointPos(node[0].body, anchor1[0], anchor1[1], anchor1[2], globalA1);
    if (node[1].body)
        dBodyGetRelPointPos(node[1].body, anchor2[0], anchor2[1], anchor2[2], globalA2);
    else
        dCopyVector3(globalA2, anchor2);

    dVector3 q;
    dSubtractVectors3(q, globalA1, globalA2);

#ifdef dSINGLE
    const dReal MIN_LENGTH = REAL(1e-7);
#else
    const dReal MIN_LENGTH = REAL(1e-12);
#endif

    if (dCalcVectorLength3(q) < MIN_LENGTH) {
        // too small, let's choose an arbitrary direction
        // heuristic: difference in velocities at anchors
        dVector3 v1, v2;
        dBodyGetPointVel(node[0].body, globalA1[0], globalA1[1], globalA1[2], v1);
        if (node[1].body)
            dBodyGetPointVel(node[1].body, globalA2[0], globalA2[1], globalA2[2], v2);
        else
            dSetZero(v2, 3);
        dSubtractVectors3(q, v1, v2);

        if (dCalcVectorLength3(q) < MIN_LENGTH) {
            // this direction is as good as any
            q[0] = 1;
            q[1] = 0;
            q[2] = 0;
        }
    }
    dNormalize3(q);

    info->J1l[0] = q[0];
    info->J1l[1] = q[1];
    info->J1l[2] = q[2];

    dVector3 relA1;
    dBodyVectorToWorld(node[0].body,
                       anchor1[0], anchor1[1], anchor1[2],
                       relA1);

    dMatrix3 a1m;
    dSetZero(a1m, 12);
    dSetCrossMatrixMinus(a1m, relA1, 4);

    dMultiply1_331(info->J1a, a1m, q);

    if (node[1].body) {
        info->J2l[0] = -q[0];
        info->J2l[1] = -q[1];
        info->J2l[2] = -q[2];

        dVector3 relA2;
        dBodyVectorToWorld(node[1].body,
                           anchor2[0], anchor2[1], anchor2[2],
                           relA2);
        dMatrix3 a2m;
        dSetZero(a2m, 12);
        dSetCrossMatrixPlus(a2m, relA2, 4);
        dMultiply1_331(info->J2a, a2m, q);
    }

    const dReal k = worldFPS * this->erp;
    info->c[0] = k * (targetDistance - dCalcPointsDistance3(globalA1, globalA2));
}

void
dxJointDBall::updateTargetDistance()
{
    dVector3 p1, p2;

    if (node[0].body)
        dBodyGetRelPointPos(node[0].body, anchor1[0], anchor1[1], anchor1[2], p1);
    else
        dCopyVector3(p1, anchor1);
    if (node[1].body)
        dBodyGetRelPointPos(node[1].body, anchor2[0], anchor2[1], anchor2[2], p2);
    else
        dCopyVector3(p2, anchor2);

    targetDistance = dCalcPointsDistance3(p1, p2);
}

void dJointSetDBallAnchor1( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointDBall* joint = static_cast<dxJointDBall*>(j);
    dUASSERT( joint, "bad joint argument" );

    if ( joint->flags & dJOINT_REVERSE ) {
        if (joint->node[1].body)
            dBodyGetPosRelPoint(joint->node[1].body, x, y, z, joint->anchor2);
        else {
            joint->anchor2[0] = x;
            joint->anchor2[1] = y;
            joint->anchor2[2] = z;
        }
    } else {
        if (joint->node[0].body)
            dBodyGetPosRelPoint(joint->node[0].body, x, y, z, joint->anchor1);
        else {
            joint->anchor1[0] = x;
            joint->anchor1[1] = y;
            joint->anchor1[2] = z;
        }
    }

    joint->updateTargetDistance();
}

void dJointSetDBallAnchor2( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointDBall* joint = static_cast<dxJointDBall*>(j);
    dUASSERT( joint, "bad joint argument" );

    if ( joint->flags & dJOINT_REVERSE ) {
        if (joint->node[0].body)
            dBodyGetPosRelPoint(joint->node[0].body, x, y, z, joint->anchor1);
        else {
            joint->anchor1[0] = x;
            joint->anchor1[1] = y;
            joint->anchor1[2] = z;
        }
    } else {
        if (joint->node[1].body)
            dBodyGetPosRelPoint(joint->node[1].body, x, y, z, joint->anchor2);
        else {
            joint->anchor2[0] = x;
            joint->anchor2[1] = y;
            joint->anchor2[2] = z;
        }
    }

    joint->updateTargetDistance();
}

dReal dJointGetDBallDistance(dJointID j)
{
    dxJointDBall* joint = static_cast<dxJointDBall*>(j);
    dUASSERT( joint, "bad joint argument" );

    return joint->targetDistance;
}

void dJointGetDBallAnchor1( dJointID j, dVector3 result )
{
    dxJointDBall* joint = static_cast<dxJointDBall*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );

    if ( joint->flags & dJOINT_REVERSE ) {
        if (joint->node[1].body)
            dBodyGetRelPointPos(joint->node[1].body, joint->anchor2[0], joint->anchor2[1], joint->anchor2[2], result);
        else
            dCopyVector3(result, joint->anchor2);
    } else {
        if (joint->node[0].body)
            dBodyGetRelPointPos(joint->node[0].body, joint->anchor1[0], joint->anchor1[1], joint->anchor1[2], result);
        else
            dCopyVector3(result, joint->anchor1);
    }
}

void dJointGetDBallAnchor2( dJointID j, dVector3 result )
{
    dxJointDBall* joint = static_cast<dxJointDBall*>(j);
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );

    if ( joint->flags & dJOINT_REVERSE ) {
        if (joint->node[0].body)
            dBodyGetRelPointPos(joint->node[0].body, joint->anchor1[0], joint->anchor1[1], joint->anchor1[2], result);
        else
            dCopyVector3(result, joint->anchor1);
    } else {
        if (joint->node[1].body)
            dBodyGetRelPointPos(joint->node[1].body, joint->anchor2[0], joint->anchor2[1], joint->anchor2[2], result);
        else
            dCopyVector3(result, joint->anchor2);
    }
}

void dJointSetDBallParam( dJointID j, int parameter, dReal value )
{
    dxJointDBall* joint = static_cast<dxJointDBall*>(j);
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

dReal dJointGetDBallParam( dJointID j, int parameter )
{
    dxJointDBall* joint = static_cast<dxJointDBall*>(j);
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

dJointType
dxJointDBall::type() const
{
    return dJointTypeDBall;
}

size_t
dxJointDBall::size() const
{
    return sizeof( *this );
}

void
dxJointDBall::setRelativeValues()
{
    updateTargetDistance();
}
