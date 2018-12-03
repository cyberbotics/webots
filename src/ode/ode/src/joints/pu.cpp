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
#include "pu.h"
#include "joint_internal.h"

//****************************************************************************
// Prismatic and Universal

dxJointPU::dxJointPU( dxWorld *w ) :
    dxJointUniversal( w )
{
    // Default Position
    //               Y                ^ Axis2
    //              ^                 |
    //             /                  |     ^ Axis1
    // Z^         /                   |    /
    //  |        / Body 2             |   /         Body 1
    //  |       /  +---------+        |  /          +-----------+
    //  |      /  /         /|        | /          /           /|
    //  |     /  /         / +        _/     -    /           / +
    //  |    /  /         /-/--------(_)----|--- /-----------/-------> AxisP
    //  |   /  +---------+ /                 -  +-----------+ /
    //  |  /   |         |/                     |           |/
    //  | /    +---------+                      +-----------+
    //  |/
    //  .-----------------------------------------> X
    //             |----------------->
    //             Anchor2           <--------------|
    //                               Anchor1
    //

    // Setting member variables which are w.r.t body2
    dSetZero( axis1, 4 );
    axis1[1] = 1;

    // Setting member variables which are w.r.t body2
    dSetZero( anchor2, 4 );
    dSetZero( axis2, 4 );
    axis2[2] = 1;

    dSetZero( axisP1, 4 );
    axisP1[0] = 1;

    dSetZero( qrel1, 4 );
    dSetZero( qrel2, 4 );

    limotP.init( world );
    limot1.init( world );
    limot2.init( world );
}

dReal dJointGetPUPosition( dJointID j )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, PU );

    dVector3 q;
    // get the offset in global coordinates
    dMultiply0_331( q, joint->node[0].body->posr.R, joint->anchor1 );

    if ( joint->node[1].body )
    {
        dVector3 anchor2;

        // get the anchor2 in global coordinates
        dMultiply0_331( anchor2, joint->node[1].body->posr.R, joint->anchor2 );

        q[0] = (( joint->node[0].body->posr.pos[0] + q[0] ) -
            ( joint->node[1].body->posr.pos[0] + anchor2[0] ) );
        q[1] = (( joint->node[0].body->posr.pos[1] + q[1] ) -
            ( joint->node[1].body->posr.pos[1] + anchor2[1] ) );
        q[2] = (( joint->node[0].body->posr.pos[2] + q[2] ) -
            ( joint->node[1].body->posr.pos[2] + anchor2[2] ) );
    }
    else
    {
        //N.B. When there is no body 2 the joint->anchor2 is already in
        //     global coordinates

        q[0] = (( joint->node[0].body->posr.pos[0] + q[0] ) -
            ( joint->anchor2[0] ) );
        q[1] = (( joint->node[0].body->posr.pos[1] + q[1] ) -
            ( joint->anchor2[1] ) );
        q[2] = (( joint->node[0].body->posr.pos[2] + q[2] ) -
            ( joint->anchor2[2] ) );

        if ( joint->flags & dJOINT_REVERSE )
        {
            q[0] = -q[0];
            q[1] = -q[1];
            q[2] = -q[2];
        }
    }

    dVector3 axP;
    // get prismatic axis in global coordinates
    dMultiply0_331( axP, joint->node[0].body->posr.R, joint->axisP1 );

    return dCalcVectorDot3( axP, q );
}

dReal dJointGetPUPositionRate( dJointID j )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, PU );

    if ( joint->node[0].body )
    {
        // We want to find the rate of change of the prismatic part of the joint
        // We can find it by looking at the speed difference between body1 and the
        // anchor point.

        // r will be used to find the distance between body1 and the anchor point
        dVector3 r;
        dVector3 anchor2 = {0,0,0};
        if ( joint->node[1].body )
        {
            // Find joint->anchor2 in global coordinates
            dMultiply0_331( anchor2, joint->node[1].body->posr.R, joint->anchor2 );

            r[0] = ( joint->node[0].body->posr.pos[0] -
                ( anchor2[0] + joint->node[1].body->posr.pos[0] ) );
            r[1] = ( joint->node[0].body->posr.pos[1] -
                ( anchor2[1] + joint->node[1].body->posr.pos[1] ) );
            r[2] = ( joint->node[0].body->posr.pos[2] -
                ( anchor2[2] + joint->node[1].body->posr.pos[2] ) );
        }
        else
        {
            //N.B. When there is no body 2 the joint->anchor2 is already in
            //     global coordinates
            // r = joint->node[0].body->posr.pos -  joint->anchor2;
            dSubtractVectors3( r, joint->node[0].body->posr.pos, joint->anchor2 );
        }

        // The body1 can have velocity coming from the rotation of
        // the rotoide axis. We need to remove this.

        // N.B. We do vel = r X w instead of vel = w x r to have vel negative
        //      since we want to remove it from the linear velocity of the body
        dVector3 lvel1;
        dCalcVectorCross3( lvel1, r, joint->node[0].body->avel );

        // lvel1 += joint->node[0].body->lvel;
        dAddVectors3( lvel1, lvel1, joint->node[0].body->lvel );

        // Since we want rate of change along the prismatic axis
        // get axisP1 in global coordinates and get the component
        // along this axis only
        dVector3 axP1;
        dMultiply0_331( axP1, joint->node[0].body->posr.R, joint->axisP1 );

        if ( joint->node[1].body )
        {
            // Find the contribution of the angular rotation to the linear speed
            // N.B. We do vel = r X w instead of vel = w x r to have vel negative
            //      since we want to remove it from the linear velocity of the body
            dVector3 lvel2;
            dCalcVectorCross3( lvel2, anchor2, joint->node[1].body->avel );

            // lvel1 -=  lvel2 + joint->node[1].body->lvel;
            dVector3 tmp;
            dAddVectors3( tmp, lvel2, joint->node[1].body->lvel );
            dSubtractVectors3( lvel1, lvel1, tmp );

            return dCalcVectorDot3( axP1, lvel1 );
        }
        else
        {
            dReal rate = dCalcVectorDot3( axP1, lvel1 );
            return ( (joint->flags & dJOINT_REVERSE) ? -rate : rate);
        }
    }

    return 0.0;
}

void
dxJointPU::getSureMaxInfo( SureMaxInfo* info )
{
    info->max_m = 6;
}

void
dxJointPU::getInfo1( dxJoint::Info1 *info )
{
    info->m = 3;
    info->nub = 3;

    // powered needs an extra constraint row

    // see if we're at a joint limit.
    limotP.limit = 0;
    if (( limotP.lostop > -dInfinity || limotP.histop < dInfinity ) &&
        limotP.lostop <= limotP.histop )
    {
        // measure joint position
        dReal pos = dJointGetPUPosition( this );
        limotP.testRotationalLimit( pos );  // N.B. The function is ill named
    }

    if ( limotP.limit || limotP.fmax > 0 ) info->m++;

    bool limiting1 = ( limot1.lostop >= -M_PI || limot1.histop <= M_PI ) &&
        limot1.lostop <= limot1.histop;
    bool limiting2 = ( limot2.lostop >= -M_PI || limot2.histop <= M_PI ) &&
        limot2.lostop <= limot2.histop;

    // We need to call testRotationLimit() even if we're motored, since it
    // records the result.
    limot1.limit = 0;
    limot2.limit = 0;
    if ( limiting1 || limiting2 )
    {
        dReal angle1, angle2;
        getAngles( &angle1, &angle2 );
        if ( limiting1 )
            limot1.testRotationalLimit( angle1 );
        if ( limiting2 )
            limot2.testRotationalLimit( angle2 );
    }

    if ( limot1.limit || limot1.fmax > 0 ) info->m++;
    if ( limot2.limit || limot2.fmax > 0 ) info->m++;
}

void
dxJointPU::getInfo2( dReal worldFPS, dReal worldERP, const Info2Descr *info )
{
    const int s1 = info->rowskip;
    const int s2 = 2 * s1;
    const dReal k = worldFPS * worldERP;

    // ======================================================================
    // The angular constraint
    //
    dVector3 ax1, ax2; // Global axes of rotation
    getAxis(this, ax1, axis1);
    getAxis2(this,ax2, axis2);

    dVector3 uniPerp;  // Axis perpendicular to axes of rotation
    dCalcVectorCross3(uniPerp,ax1,ax2);
    dNormalize3( uniPerp );
    dCopyVector3( info->J1a , uniPerp );
    if ( node[1].body )
    {
        dCopyNegatedVector3( info->J2a , uniPerp );
    }
    // Corrective velocity attempting to keep uni axes perpendicular
    dReal val = dCalcVectorDot3( ax1, ax2 );
    // Small angle approximation :
    // theta = asin(val)
    // theta is approximately val when val is near zero.
    info->c[0] = -k * val;

    // ==========================================================================
    // Handle axes orthogonal to the prismatic
    dVector3 an1, an2; // Global anchor positions
    dVector3 axP, sep; // Prismatic axis and separation vector
    getAnchor(this,an1,anchor1);
    getAnchor2(this,an2,anchor2);
    if (flags & dJOINT_REVERSE) {
        getAxis2(this, axP, axisP1);
    } else {
        getAxis(this, axP, axisP1);
    }
    dSubtractVectors3(sep,an2,an1);

    dVector3 p,q;
    dPlaneSpace(axP,p,q);

    dCopyVector3(( info->J1l ) + s1, p );
    dCopyVector3(( info->J1l ) + s2, q );
    // Make the anchors be body local
    // Aliasing isn't a problem here.
    dSubtractVectors3(an1,an1,node[0].body->posr.pos);
    dCalcVectorCross3(( info->J1a ) + s1, an1, p );
    dCalcVectorCross3(( info->J1a ) + s2, an1, q );

    if (node[1].body) {
        dCopyNegatedVector3(( info->J2l ) + s1, p );
        dCopyNegatedVector3(( info->J2l ) + s2, q );
        dSubtractVectors3(an2,an2,node[1].body->posr.pos);
        dCalcVectorCross3(( info->J2a ) + s1, p, an2 );
        dCalcVectorCross3(( info->J2a ) + s2, q, an2 );
    }

    info->c[1] = k * dCalcVectorDot3( p, sep );
    info->c[2] = k * dCalcVectorDot3( q, sep );

    // ==========================================================================
    // Handle the limits/motors
    int row = 3 + limot1.addLimot( this, worldFPS, info, 3, ax1, 1 );
    row += limot2.addLimot( this, worldFPS, info, row, ax2, 1 );

    if (  node[1].body || !(flags & dJOINT_REVERSE) )
        limotP.addTwoPointLimot( this, worldFPS, info, row, axP, an1, an2 );
    else
    {
        axP[0] = -axP[0];
        axP[1] = -axP[1];
        axP[2] = -axP[2];
        limotP.addTwoPointLimot ( this, worldFPS, info, row, axP, an1, an2  );
    }
}

void dJointSetPUAnchor( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, PU );
    setAnchors( joint, x, y, z, joint->anchor1, joint->anchor2 );
    joint->computeInitialRelativeRotations();
}

/**
 * This function initialize the anchor and the relative position of each body
 * as if body2 was at its current position + [dx,dy,dy].
 * Ex:
 * <PRE>
 * dReal offset = 1;
 * dVector3 dir;
 * dJointGetPUAxis3(jId, dir);
 * dJointSetPUAnchor(jId, 0, 0, 0);
 * // If you request the position you will have: dJointGetPUPosition(jId) == 0
 * dJointSetPUAnchorDelta(jId, 0, 0, 0, dir[X]*offset, dir[Y]*offset, dir[Z]*offset);
 * // If you request the position you will have: dJointGetPUPosition(jId) == -offset
 * </PRE>

 * @param j The PU joint for which the anchor point will be set
 * @param x The X position of the anchor point in world frame
 * @param y The Y position of the anchor point in world frame
 * @param z The Z position of the anchor point in world frame
 * @param dx A delta to be added to the X position as if the anchor was set
 *           when body1 was at current_position[X] + dx
 * @param dx A delta to be added to the Y position as if the anchor was set
 *           when body1 was at current_position[Y] + dy
 * @param dx A delta to be added to the Z position as if the anchor was set
 *           when body1 was at current_position[Z] + dz
 * @note Should have the same meaning as dJointSetSliderAxisDelta
 */
void dJointSetPUAnchorDelta( dJointID j, dReal x, dReal y, dReal z,
                            dReal dx, dReal dy, dReal dz )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, PU );

    if ( joint->node[0].body )
    {
        joint->node[0].body->posr.pos[0] += dx;
        joint->node[0].body->posr.pos[1] += dy;
        joint->node[0].body->posr.pos[2] += dz;
    }

    setAnchors( joint, x, y, z, joint->anchor1, joint->anchor2 );

    if ( joint->node[0].body )
    {
        joint->node[0].body->posr.pos[0] -= dx;
        joint->node[0].body->posr.pos[1] -= dy;
        joint->node[0].body->posr.pos[2] -= dz;
    }

    joint->computeInitialRelativeRotations();
}

/**
 * \brief This function initialize the anchor and the relative position of each body
 * such that dJointGetPUPosition will return the dot product of axis and [dx,dy,dy].
 *
 * The body 1 is moved to [-dx, -dy, -dx] then the anchor is set. This will be the
 * position 0 for the prismatic part of the joint. Then the body 1 is moved to its
 * original position.
 *
 * Ex:
 * <PRE>
 * dReal offset = 1;
 * dVector3 dir;
 * dJointGetPUAxis3(jId, dir);
 * dJointSetPUAnchor(jId, 0, 0, 0);
 * // If you request the position you will have: dJointGetPUPosition(jId) == 0
 * dJointSetPUAnchorDelta(jId, 0, 0, 0, dir[X]*offset, dir[Y]*offset, dir[Z]*offset);
 * // If you request the position you will have: dJointGetPUPosition(jId) == offset
 * </PRE>

 * @param j The PU joint for which the anchor point will be set
 * @param x The X position of the anchor point in world frame
 * @param y The Y position of the anchor point in world frame
 * @param z The Z position of the anchor point in world frame
 * @param dx A delta to be added to the X position as if the anchor was set
 *           when body1 was at current_position[X] + dx
 * @param dx A delta to be added to the Y position as if the anchor was set
 *           when body1 was at current_position[Y] + dy
 * @param dx A delta to be added to the Z position as if the anchor was set
 *           when body1 was at current_position[Z] + dz
 * @note Should have the same meaning as dJointSetSliderAxisDelta
 */
void dJointSetPUAnchorOffset( dJointID j, dReal x, dReal y, dReal z,
                             dReal dx, dReal dy, dReal dz )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, PU );

    if (joint->flags & dJOINT_REVERSE)
    {
        dx = -dx;
        dy = -dy;
        dz = -dz;
    }

    if ( joint->node[0].body )
    {
        joint->node[0].body->posr.pos[0] -= dx;
        joint->node[0].body->posr.pos[1] -= dy;
        joint->node[0].body->posr.pos[2] -= dz;
    }

    setAnchors( joint, x, y, z, joint->anchor1, joint->anchor2 );

    if ( joint->node[0].body )
    {
        joint->node[0].body->posr.pos[0] += dx;
        joint->node[0].body->posr.pos[1] += dy;
        joint->node[0].body->posr.pos[2] += dz;
    }

    joint->computeInitialRelativeRotations();
}

void dJointSetPUAxis1( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, PU );
    if ( joint->flags & dJOINT_REVERSE )
        setAxes( joint, x, y, z, NULL, joint->axis2 );
    else
        setAxes( joint, x, y, z, joint->axis1, NULL );
    joint->computeInitialRelativeRotations();
}

void dJointSetPUAxis2( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, PU );
    if ( joint->flags & dJOINT_REVERSE )
        setAxes( joint, x, y, z, joint->axis1, NULL );
    else
        setAxes( joint, x, y, z, NULL, joint->axis2 );
    joint->computeInitialRelativeRotations();
}

void dJointSetPUAxisP( dJointID id, dReal x, dReal y, dReal z )
{
    dJointSetPUAxis3( id, x, y, z );
}

void dJointSetPUAxis3( dJointID j, dReal x, dReal y, dReal z )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, PU );

    setAxes( joint, x, y, z, joint->axisP1, 0 );

    joint->computeInitialRelativeRotations();
}

void dJointGetPUAngles( dJointID j, dReal *angle1, dReal *angle2 )
{
    dxJointUniversal* joint = ( dxJointUniversal* ) j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, PU );
    if ( joint->flags & dJOINT_REVERSE )
        joint->getAngles( angle2, angle1 );
    else
        joint->getAngles( angle1, angle2 );
}

dReal dJointGetPUAngle1( dJointID j )
{
    dxJointUniversal* joint = ( dxJointUniversal* ) j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, PU );
    if ( joint->flags & dJOINT_REVERSE )
        return joint->getAngle2();
    else
        return joint->getAngle1();
}

dReal dJointGetPUAngle2( dJointID j )
{
    dxJointUniversal* joint = ( dxJointUniversal* ) j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, PU );
    if ( joint->flags & dJOINT_REVERSE )
        return joint->getAngle1();
    else
        return joint->getAngle2();
}

dReal dJointGetPUAngle1Rate( dJointID j )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, PU );

    if ( joint->node[0].body )
    {
        dVector3 axis;

        if ( joint->flags & dJOINT_REVERSE )
            getAxis2( joint, axis, joint->axis2 );
        else
            getAxis( joint, axis, joint->axis1 );

        dReal rate = dCalcVectorDot3( axis, joint->node[0].body->avel );
        if ( joint->node[1].body ) rate -= dCalcVectorDot3( axis, joint->node[1].body->avel );
        return rate;
    }
    return 0;
}

dReal dJointGetPUAngle2Rate( dJointID j )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, PU );

    if ( joint->node[0].body )
    {
        dVector3 axis;

        if ( joint->flags & dJOINT_REVERSE )
            getAxis( joint, axis, joint->axis1 );
        else
            getAxis2( joint, axis, joint->axis2 );

        dReal rate = dCalcVectorDot3( axis, joint->node[0].body->avel );
        if ( joint->node[1].body ) rate -= dCalcVectorDot3( axis, joint->node[1].body->avel );
        return rate;
    }
    return 0;
}

void dJointSetPUParam( dJointID j, int parameter, dReal value )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, PU );

    switch ( parameter & 0xff00 )
    {
    case dParamGroup1:
        joint->limot1.set( parameter, value );
        break;
    case dParamGroup2:
        joint->limot2.set( parameter & 0xff, value );
        break;
    case dParamGroup3:
        joint->limotP.set( parameter & 0xff, value );
        break;
    }
}

void dJointGetPUAnchor( dJointID j, dVector3 result )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );
    checktype( joint, PU );

    if ( joint->node[1].body )
        getAnchor2( joint, result, joint->anchor2 );
    else
    {
        // result[i] = joint->anchor2[i];
        dCopyVector3( result, joint->anchor2 );
    }
}

void dJointGetPUAxis1( dJointID j, dVector3 result )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );
    checktype( joint, PU );
    if ( joint->flags & dJOINT_REVERSE )
        getAxis2( joint, result, joint->axis2 );
    else
        getAxis( joint, result, joint->axis1 );
}

void dJointGetPUAxis2( dJointID j, dVector3 result )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );
    checktype( joint, PU );
    if ( joint->flags & dJOINT_REVERSE )
        getAxis( joint, result, joint->axis1 );
    else
        getAxis2( joint, result, joint->axis2 );
}

/**
 * @brief Get the prismatic axis
 * @ingroup joints
 *
 * @note This function was added for convenience it is the same as
 *       dJointGetPUAxis3
 */
void dJointGetPUAxisP( dJointID id, dVector3 result )
{
    dJointGetPUAxis3( id, result );
}

void dJointGetPUAxis3( dJointID j, dVector3 result )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    dUASSERT( result, "bad result argument" );
    checktype( joint, PU );
    getAxis( joint, result, joint->axisP1 );
}

dReal dJointGetPUParam( dJointID j, int parameter )
{
    dxJointPU* joint = ( dxJointPU* ) j;
    dUASSERT( joint, "bad joint argument" );
    checktype( joint, PU );

    switch ( parameter & 0xff00 )
    {
    case dParamGroup1:
        return joint->limot1.get( parameter );
        break;
    case dParamGroup2:
        return joint->limot2.get( parameter & 0xff );
        break;
    case dParamGroup3:
        return joint->limotP.get( parameter & 0xff );
        break;
    }

    return 0;
}

dJointType
dxJointPU::type() const
{
    return dJointTypePU;
}

size_t
dxJointPU::size() const
{
    return sizeof( *this );
}

void
dxJointPU::setRelativeValues()
{
    dVector3 anchor;
    dJointGetPUAnchor(this, anchor);
    setAnchors( this, anchor[0], anchor[1], anchor[2], anchor1, anchor2 );

    dVector3 ax1, ax2, ax3;
    dJointGetPUAxis1(this, ax1);
    dJointGetPUAxis2(this, ax2);
    dJointGetPUAxis3(this, ax3);

    if ( flags & dJOINT_REVERSE )
    {
        setAxes( this, ax1[0], ax1[1], ax1[2], NULL, axis2 );
        setAxes( this, ax2[0], ax2[1], ax2[2], axis1, NULL );
    }
    else
    {
        setAxes( this, ax1[0], ax1[1], ax1[2], axis1, NULL );
        setAxes( this, ax2[0], ax2[1], ax2[2], NULL, axis2 );
    }

    setAxes( this, ax3[0], ax3[1], ax3[2], axisP1, NULL );

    computeInitialRelativeRotations();
}
