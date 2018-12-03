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

#ifndef _ODE_JOINT_TRANSMISSION_
#define _ODE_JOINT_TRANSMISSION_

#include "joint.h"

struct dxJointTransmission : public dxJoint
{
    int mode, update;
    dVector3 contacts[2], axes[2], anchors[2];
    dMatrix3 reference[2];
    dReal phase[2], radii[2], backlash;
    dReal ratio;        // transmission ratio
    dReal erp;          // error reduction
    dReal cfm;          // constraint force mix in

    dxJointTransmission(dxWorld *w);

    virtual void getSureMaxInfo( SureMaxInfo* info );
    virtual void getInfo1( Info1* info );
    virtual void getInfo2( dReal worldFPS, dReal worldERP, const Info2Descr* info );
    virtual dJointType type() const;
    virtual size_t size() const;
};

#endif
