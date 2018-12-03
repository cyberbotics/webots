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
#include "dhinge.h"
#include "joint_internal.h"

/*
 * Double Hinge joint
 */

dxJointDHinge::dxJointDHinge(dxWorld* w) :
    dxJointDBall(w)
{
    dSetZero(axis1, 3);
    dSetZero(axis2, 3);
}

void
dxJointDHinge::getSureMaxInfo( SureMaxInfo* info )
{
    info->max_m = 4;
}

void
dxJointDHinge::getInfo1( dxJoint::Info1* info )
{
    info->m = 4;
    info->nub = 4;
}

void
dxJointDHinge::getInfo2( dReal worldFPS, dReal worldERP, const Info2Descr* info )
{
    dxJointDBall::getInfo2( worldFPS, worldERP, info ); // sets row0

    const int skip = info->rowskip;
    const int row1 = skip;
    const int row2 = 2*skip;
    const int row3 = 3*skip;

    dVector3 globalAxis1;
    dBodyVectorToWorld(node[0].body, axis1[0], axis1[1], axis1[2], globalAxis1);

    // angular constraints, perpendicular to axis
    dVector3 p, q;
    dPlaneSpace(globalAxis1, p, q);
    info->J1a[row1+0] = p[0];
    info->J1a[row1+1] = p[1];
    info->J1a[row1+2] = p[2];
    info->J1a[row2+0] = q[0];
    info->J1a[row2+1] = q[1];
    info->J1a[row2+2] = q[2];

    if ( node[1].body ) {
        info->J2a[row1+0] = -p[0];
        info->J2a[row1+1] = -p[1];
        info->J2a[row1+2] = -p[2];
        info->J2a[row2+0] = -q[0];
        info->J2a[row2+1] = -q[1];
        info->J2a[row2+2] = -q[2];
    }

    dVector3 globalAxis2;
    if ( node[1].body )
        dBodyVectorToWorld(node[1].body, axis2[0], axis2[1], axis2[2], globalAxis2);
    else
        dCopyVector3(globalAxis2, axis2);

    // similar to the hinge joint
    dVector3 u;
    dCalcVectorCross3(u, globalAxis1, globalAxis2);
    const dReal k = worldFPS * this->erp;

    info->c[1] = k * dCalcVectorDot3( u, p );
    info->c[2] = k * dCalcVectorDot3( u, q );

    /*
     * Constraint along the axis: translation along it should couple angular movement.
     * This is just the ball-and-socket derivation, projected onto the hinge axis,
     * producing a single constraint at the end.
     *
     * The choice of "ball" position can be arbitrary; we could place it at the center
     * of one of the bodies, canceling out its rotational jacobian; or we could make
     * everything symmetrical by just placing at the midpoint between the centers.
     *
     * I like symmetry, so I'll use the second approach here. I'll call the midpoint h.
     *
     * Of course, if the second body is NULL, the first body is pretty much locked
     * along this axis, and the linear constraint is enough.
     */

    info->J1l[row3+0] = globalAxis1[0];
    info->J1l[row3+1] = globalAxis1[1];
    info->J1l[row3+2] = globalAxis1[2];

    if ( node[1].body ) {

        dVector3 h;
        dAddScaledVectors3(h, node[0].body->posr.pos, node[1].body->posr.pos, -0.5, 0.5);

        dVector3 omega;
        dCalcVectorCross3(omega, h, globalAxis1);
        info->J1a[row3+0] = omega[0];
        info->J1a[row3+1] = omega[1];
        info->J1a[row3+2] = omega[2];

        info->J2l[row3+0] = -globalAxis1[0];
        info->J2l[row3+1] = -globalAxis1[1];
        info->J2l[row3+2] = -globalAxis1[2];

        info->J2a[row3+0] = omega[0];
        info->J2a[row3+1] = omega[1];
        info->J2a[row3+2] = omega[2];
    }

    // error correction: both anchors should lie on the same plane perpendicular to the axis
    dVector3 globalA1, globalA2;
    dBodyGetRelPointPos(node[0].body, anchor1[0], anchor1[1], anchor1[2], globalA1);
    if ( node[1].body )
        dBodyGetRelPointPos(node[1].body, anchor2[0], anchor2[1], anchor2[2], globalA2);
    else
        dCopyVector3(globalA2, anchor2);

    dVector3 d;
    dSubtractVectors3(d, globalA1, globalA2); // displacement error
    info->c[3] = -k * dCalcVectorDot3(globalAxis1, d);
}

void dJointSetDHingeAxis( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointDHinge* joint = static_cast<dxJointDHinge*>(j);
    dUASSERT( joint, "bad joint argument" );

    dBodyVectorFromWorld(joint->node[0].body, x, y, z, joint->axis1);
    if (joint->node[1].body)
        dBodyVectorFromWorld(joint->node[1].body, x, y, z, joint->axis2);
    else {
        joint->axis2[0] = x;
        joint->axis2[1] = y;
        joint->axis2[2] = z;
    }
    dNormalize3(joint->axis1);
    dNormalize3(joint->axis2);
}

void dJointGetDHingeAxis( dJointID j, dVector3 result )
{
    dxJointDHinge* joint = static_cast<dxJointDHinge*>(j);
    dUASSERT( joint, "bad joint argument" );

    dBodyVectorToWorld(joint->node[0].body, joint->axis1[0], joint->axis1[1], joint->axis1[2], result);
}

void dJointSetDHingeAnchor1( dJointID j, dReal x, dReal y, dReal z )
{
    dJointSetDBallAnchor1(j, x, y, z);
}

void dJointSetDHingeAnchor2( dJointID j, dReal x, dReal y, dReal z )
{
    dJointSetDBallAnchor2(j, x, y, z);
}

dReal dJointGetDHingeDistance(dJointID j)
{
    return dJointGetDBallDistance(j);
}

void dJointGetDHingeAnchor1( dJointID j, dVector3 result )
{
    dJointGetDBallAnchor1(j, result);
}

void dJointGetDHingeAnchor2( dJointID j, dVector3 result )
{
    dJointGetDBallAnchor2(j, result);
}

void dJointSetDHingeParam( dJointID j, int parameter, dReal value )
{
    dJointSetDBallParam(j, parameter, value);
}

dReal dJointGetDHingeParam( dJointID j, int parameter )
{
    return dJointGetDBallParam(j, parameter);
}

dJointType
dxJointDHinge::type() const
{
    return dJointTypeDHinge;
}

size_t
dxJointDHinge::size() const
{
    return sizeof( *this );
}
