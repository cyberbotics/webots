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

#include <ode/ode.h>
#include "config.h"
#include "objects.h"
#include "joints/joint.h"
#include "util.h"

#include <new>

#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
#define dMAX(A,B)  ((B)>(A) ? (B) : (A))

//****************************************************************************
// Malloc based world stepping memory manager

dxWorldProcessMemoryManager g_WorldProcessMallocMemoryManager(dAlloc, dRealloc, dFree);
dxWorldProcessMemoryReserveInfo g_WorldProcessDefaultReserveInfo(dWORLDSTEP_RESERVEFACTOR_DEFAULT, dWORLDSTEP_RESERVESIZE_DEFAULT);

//****************************************************************************
// dxWorldProcessContext

dxWorldProcessContext::dxWorldProcessContext():
  m_pmaIslandsArena(NULL),
  m_pmaStepperArena(NULL)
{
    // Do nothing
}

dxWorldProcessContext::~dxWorldProcessContext()
{
  if (m_pmaIslandsArena)
  {
    dxWorldProcessMemArena::FreeMemArena(m_pmaIslandsArena);
  }

  if (m_pmaStepperArena)
  {
    dxWorldProcessMemArena::FreeMemArena(m_pmaStepperArena);
  }
}

bool dxWorldProcessContext::IsStructureValid() const
{
  return (!m_pmaIslandsArena || m_pmaIslandsArena->IsStructureValid()) && (!m_pmaStepperArena || m_pmaStepperArena->IsStructureValid());
}

void dxWorldProcessContext::CleanupContext()
{
  if (m_pmaIslandsArena)
  {
    m_pmaIslandsArena->ResetState();
  }

  if (m_pmaStepperArena)
  {
    m_pmaStepperArena->ResetState();
  }
}

dxWorldProcessMemArena *dxWorldProcessContext::ReallocateIslandsMemArena(size_t nMemoryRequirement,
    const dxWorldProcessMemoryManager *pmmMemortManager, float fReserveFactor, unsigned uiReserveMinimum)
{
    dxWorldProcessMemArena *pmaExistingArena = GetIslandsMemArena();
    dxWorldProcessMemArena *pmaNewMemArena = dxWorldProcessMemArena::ReallocateMemArena(pmaExistingArena, nMemoryRequirement, pmmMemortManager, fReserveFactor, uiReserveMinimum);
    SetIslandsMemArena(pmaNewMemArena);
    return pmaNewMemArena;
}

dxWorldProcessMemArena *dxWorldProcessContext::ReallocateStepperMemArena(size_t nMemoryRequirement,
  const dxWorldProcessMemoryManager *pmmMemortManager, float fReserveFactor, unsigned uiReserveMinimum)
{
    dxWorldProcessMemArena *pmaExistingArena = GetStepperMemArena();
    dxWorldProcessMemArena *pmaNewMemArena = dxWorldProcessMemArena::ReallocateMemArena(pmaExistingArena, nMemoryRequirement, pmmMemortManager, fReserveFactor, uiReserveMinimum);
    SetStepperMemArena(pmaNewMemArena);
    return pmaNewMemArena;
}

//****************************************************************************
// Auto disabling

void dInternalHandleAutoDisabling (dxWorld *world, dReal stepSize)
{
    dxBody *bb;
    for ( bb=world->firstbody; bb; bb=(dxBody*)bb->next )
    {
        // don't freeze objects mid-air (patch 1586738)
        if ( bb->firstjoint == NULL ) continue;

        // nothing to do unless this body is currently enabled and has
        // the auto-disable flag set
        if ( (bb->flags & (dxBodyAutoDisable|dxBodyDisabled)) != dxBodyAutoDisable ) continue;

        // if sampling / threshold testing is disabled, we can never sleep.
        if ( bb->adis.average_samples == 0 ) continue;

        //
        // see if the body is idle
        //

#ifndef dNODEBUG
        // sanity check
        if ( bb->average_counter >= bb->adis.average_samples )
        {
            dUASSERT( bb->average_counter < bb->adis.average_samples, "buffer overflow" );

            // something is going wrong, reset the average-calculations
            bb->average_ready = 0; // not ready for average calculation
            bb->average_counter = 0; // reset the buffer index
        }
#endif // dNODEBUG

        // sample the linear and angular velocity
        bb->average_lvel_buffer[bb->average_counter][0] = bb->lvel[0];
        bb->average_lvel_buffer[bb->average_counter][1] = bb->lvel[1];
        bb->average_lvel_buffer[bb->average_counter][2] = bb->lvel[2];
        bb->average_avel_buffer[bb->average_counter][0] = bb->avel[0];
        bb->average_avel_buffer[bb->average_counter][1] = bb->avel[1];
        bb->average_avel_buffer[bb->average_counter][2] = bb->avel[2];
        bb->average_counter++;

        // buffer ready test
        if ( bb->average_counter >= bb->adis.average_samples )
        {
            bb->average_counter = 0; // fill the buffer from the beginning
            bb->average_ready = 1; // this body is ready now for average calculation
        }

        int idle = 0; // Assume it's in motion unless we have samples to disprove it.

        // enough samples?
        if ( bb->average_ready )
        {
            idle = 1; // Initial assumption: IDLE

            // the sample buffers are filled and ready for calculation
            dVector3 average_lvel, average_avel;

            // Store first velocity samples
            average_lvel[0] = bb->average_lvel_buffer[0][0];
            average_avel[0] = bb->average_avel_buffer[0][0];
            average_lvel[1] = bb->average_lvel_buffer[0][1];
            average_avel[1] = bb->average_avel_buffer[0][1];
            average_lvel[2] = bb->average_lvel_buffer[0][2];
            average_avel[2] = bb->average_avel_buffer[0][2];

            // If we're not in "instantaneous mode"
            if ( bb->adis.average_samples > 1 )
            {
                // add remaining velocities together
                for ( unsigned int i = 1; i < bb->adis.average_samples; ++i )
                {
                    average_lvel[0] += bb->average_lvel_buffer[i][0];
                    average_avel[0] += bb->average_avel_buffer[i][0];
                    average_lvel[1] += bb->average_lvel_buffer[i][1];
                    average_avel[1] += bb->average_avel_buffer[i][1];
                    average_lvel[2] += bb->average_lvel_buffer[i][2];
                    average_avel[2] += bb->average_avel_buffer[i][2];
                }

                // make average
                dReal r1 = dReal( 1.0 ) / dReal( bb->adis.average_samples );

                average_lvel[0] *= r1;
                average_avel[0] *= r1;
                average_lvel[1] *= r1;
                average_avel[1] *= r1;
                average_lvel[2] *= r1;
                average_avel[2] *= r1;
            }

            // threshold test
            dReal av_lspeed, av_aspeed;
            av_lspeed = dCalcVectorDot3( average_lvel, average_lvel );
            if ( av_lspeed > bb->adis.linear_average_threshold )
            {
                idle = 0; // average linear velocity is too high for idle
            }
            else
            {
                av_aspeed = dCalcVectorDot3( average_avel, average_avel );
                if ( av_aspeed > bb->adis.angular_average_threshold )
                {
                    idle = 0; // average angular velocity is too high for idle
                }
            }
        }

        // if it's idle, accumulate steps and time.
        // these counters won't overflow because this code doesn't run for disabled bodies.
        if (idle) {
            bb->adis_stepsleft--;
            bb->adis_timeleft -= stepSize;
        }
        else {
            // Reset countdowns
            bb->adis_stepsleft = bb->adis.idle_steps;
            bb->adis_timeleft = bb->adis.idle_time;
        }

        // disable the body if it's idle for a long enough time
        if ( bb->adis_stepsleft <= 0 && bb->adis_timeleft <= 0 )
        {
            bb->flags |= dxBodyDisabled; // set the disable flag

            // disabling bodies should also include resetting the velocity
            // should prevent jittering in big "islands"
            bb->lvel[0] = 0;
            bb->lvel[1] = 0;
            bb->lvel[2] = 0;
            bb->avel[0] = 0;
            bb->avel[1] = 0;
            bb->avel[2] = 0;
        }
    }
}

//****************************************************************************
// body rotation

// return sin(x)/x. this has a singularity at 0 so special handling is needed
// for small arguments.

static inline dReal sinc (dReal x)
{
    // if |x| < 1e-4 then use a taylor series expansion. this two term expansion
    // is actually accurate to one LS bit within this range if double precision
    // is being used - so don't worry!
    if (dFabs(x) < 1.0e-4) return REAL(1.0) - x*x*REAL(0.166666666666666666667);
    else return dSin(x)/x;
}

// given a body b, apply its linear and angular rotation over the time
// interval h, thereby adjusting its position and orientation.

void dxStepBody (dxBody *b, dReal h)
{
    // cap the angular velocity
    if (b->flags & dxBodyMaxAngularSpeed) {
        const dReal max_ang_speed = b->max_angular_speed;
        const dReal aspeed = dCalcVectorDot3( b->avel, b->avel );
        if (aspeed > max_ang_speed*max_ang_speed) {
            const dReal coef = max_ang_speed/dSqrt(aspeed);
            dScaleVector3(b->avel, coef);
        }
    }
    // end of angular velocity cap

    // handle linear velocity
    for (unsigned int j=0; j<3; j++) b->posr.pos[j] += h * b->lvel[j];

    if (b->flags & dxBodyFlagFiniteRotation) {
        dVector3 irv;	// infitesimal rotation vector
        dQuaternion q;	// quaternion for finite rotation

        if (b->flags & dxBodyFlagFiniteRotationAxis) {
            // split the angular velocity vector into a component along the finite
            // rotation axis, and a component orthogonal to it.
            dVector3 frv;		// finite rotation vector
            dReal k = dCalcVectorDot3 (b->finite_rot_axis,b->avel);
            frv[0] = b->finite_rot_axis[0] * k;
            frv[1] = b->finite_rot_axis[1] * k;
            frv[2] = b->finite_rot_axis[2] * k;
            irv[0] = b->avel[0] - frv[0];
            irv[1] = b->avel[1] - frv[1];
            irv[2] = b->avel[2] - frv[2];

            // make a rotation quaternion q that corresponds to frv * h.
            // compare this with the full-finite-rotation case below.
            h *= REAL(0.5);
            dReal theta = k * h;
            q[0] = dCos(theta);
            dReal s = sinc(theta) * h;
            q[1] = frv[0] * s;
            q[2] = frv[1] * s;
            q[3] = frv[2] * s;
        }
        else {
            // make a rotation quaternion q that corresponds to w * h
            dReal wlen = dSqrt (b->avel[0]*b->avel[0] + b->avel[1]*b->avel[1] +
                b->avel[2]*b->avel[2]);
            h *= REAL(0.5);
            dReal theta = wlen * h;
            q[0] = dCos(theta);
            dReal s = sinc(theta) * h;
            q[1] = b->avel[0] * s;
            q[2] = b->avel[1] * s;
            q[3] = b->avel[2] * s;
        }

        // do the finite rotation
        dQuaternion q2;
        dQMultiply0 (q2,q,b->q);
        for (unsigned int j=0; j<4; j++) b->q[j] = q2[j];

        // do the infitesimal rotation if required
        if (b->flags & dxBodyFlagFiniteRotationAxis) {
            dReal dq[4];
            dWtoDQ (irv,b->q,dq);
            for (unsigned int j=0; j<4; j++) b->q[j] += h * dq[j];
        }
    }
    else {
        // the normal way - do an infitesimal rotation
        dReal dq[4];
        dWtoDQ (b->avel,b->q,dq);
        for (unsigned int j=0; j<4; j++) b->q[j] += h * dq[j];
    }

    // normalize the quaternion and convert it to a rotation matrix
    dNormalize4 (b->q);
    dQtoR (b->q,b->posr.R);

    // notify all attached geoms that this body has moved
    for (dxGeom *geom = b->geom; geom; geom = dGeomGetBodyNext (geom))
        dGeomMoved (geom);

    // notify the user
    if (b->moved_callback != NULL) {
        b->moved_callback(b);
    }

    // damping
    if (b->flags & dxBodyLinearDamping) {
        const dReal lin_threshold = b->dampingp.linear_threshold;
        const dReal lin_speed = dCalcVectorDot3( b->lvel, b->lvel );
        if ( lin_speed > lin_threshold) {
            const dReal k = 1 - b->dampingp.linear_scale;
            dScaleVector3(b->lvel, k);
        }
    }
    if (b->flags & dxBodyAngularDamping) {
        const dReal ang_threshold = b->dampingp.angular_threshold;
        const dReal ang_speed = dCalcVectorDot3( b->avel, b->avel );
        if ( ang_speed > ang_threshold) {
            const dReal k = 1 - b->dampingp.angular_scale;
            dScaleVector3(b->avel, k);
        }
    }
}

//****************************************************************************
// island processing

// This estimates dynamic memory requirements for dxProcessIslands
static size_t EstimateIslandsProcessingMemoryRequirements(dxWorld *world)
{
    size_t res = 0;

    size_t islandcounts = dEFFICIENT_SIZE((size_t)(unsigned)world->nb * 2 * sizeof(int));
    res += islandcounts;

    size_t bodiessize = dEFFICIENT_SIZE((size_t)(unsigned)world->nb * sizeof(dxBody*));
    size_t jointssize = dEFFICIENT_SIZE((size_t)(unsigned)world->nj * sizeof(dxJoint*));
    res += bodiessize + jointssize;

    size_t sesize = (bodiessize < jointssize) ? bodiessize : jointssize;
    res += sesize;

    return res;
}

static size_t BuildIslandsAndEstimateStepperMemoryRequirements(
    dxWorldProcessIslandsInfo &islandsInfo, dxWorldProcessMemArena *memarena,
    dxWorld *world, dReal stepSize, dmemestimate_fn_t stepperEstimate)
{
  const unsigned int sizeelements = 2;
    size_t maxreq = 0;

    // handle auto-disabling of bodies
    dInternalHandleAutoDisabling (world,stepSize);

    unsigned int nb = world->nb, nj = world->nj;
    // Make array for island body/joint counts
    unsigned int *islandsizes = memarena->AllocateArray<unsigned int>(2 * (size_t)nb);
    unsigned int *sizescurr;

    // make arrays for body and joint lists (for a single island) to go into
    dxBody **body = memarena->AllocateArray<dxBody *>(nb);
    dxJoint **joint = memarena->AllocateArray<dxJoint *>(nj);

    BEGIN_STATE_SAVE(memarena, stackstate) {
        // allocate a stack of unvisited bodies in the island. the maximum size of
        // the stack can be the lesser of the number of bodies or joints, because
        // new bodies are only ever added to the stack by going through untagged
        // joints. all the bodies in the stack must be tagged!
        unsigned int stackalloc = (nj < nb) ? nj : nb;
        dxBody **stack = memarena->AllocateArray<dxBody *>(stackalloc);

        {
            // set all body/joint tags to 0
            for (dxBody *b=world->firstbody; b; b=(dxBody*)b->next) b->tag = 0;
            for (dxJoint *j=world->firstjoint; j; j=(dxJoint*)j->next) j->tag = 0;
        }

        sizescurr = islandsizes;
        dxBody **bodystart = body;
        dxJoint **jointstart = joint;
        for (dxBody *bb=world->firstbody; bb; bb=(dxBody*)bb->next) {
            // get bb = the next enabled, untagged body, and tag it
            if (!bb->tag) {
                if (!(bb->flags & dxBodyDisabled)) {
                    bb->tag = 1;

                    dxBody **bodycurr = bodystart;
                    dxJoint **jointcurr = jointstart;

                    // tag all bodies and joints starting from bb.
                    *bodycurr++ = bb;

                    unsigned int stacksize = 0;
                    dxBody *b = bb;

                    while (true) {
                        // traverse and tag all body's joints, add untagged connected bodies
                        // to stack
                        for (dxJointNode *n=b->firstjoint; n; n=n->next) {
                            dxJoint *njoint = n->joint;
                            if (!njoint->tag) {
                                if (njoint->isEnabled()) {
                                    njoint->tag = 1;
                                    *jointcurr++ = njoint;

                                    dxBody *nbody = n->body;
                                    // Body disabled flag is not checked here. This is how auto-enable works.
                                    if (nbody && nbody->tag <= 0) {
                                        nbody->tag = 1;
                                        // Make sure all bodies are in the enabled state.
                                        nbody->flags &= ~dxBodyDisabled;
                                        stack[stacksize++] = nbody;
                                    }
                                } else {
                                    njoint->tag = -1; // Used in Step to prevent search over disabled joints (not needed for QuickStep so far)
                                }
                            }
                        }
                        dIASSERT(stacksize <= (unsigned int)world->nb);
                        dIASSERT(stacksize <= (unsigned int)world->nj);

                        if (stacksize == 0) {
                            break;
                        }

                        b = stack[--stacksize];	// pop body off stack
                        *bodycurr++ = b;	// put body on body list
                    }

                    unsigned int bcount = (unsigned int)(bodycurr - bodystart);
                    unsigned int jcount = (unsigned int)(jointcurr - jointstart);
                    dIASSERT((size_t)(bodycurr - bodystart) <= (size_t)UINT_MAX);
                    dIASSERT((size_t)(jointcurr - jointstart) <= (size_t)UINT_MAX);

          sizescurr[0] = bcount;
          sizescurr[1] = jcount;
          sizescurr += sizeelements;

                    size_t islandreq = stepperEstimate(bodystart, bcount, jointstart, jcount);
                    maxreq = (maxreq > islandreq) ? maxreq : islandreq;

                    bodystart = bodycurr;
                    jointstart = jointcurr;
                } else {
                    bb->tag = -1; // Not used so far (assigned to retain consistency with joints)
                }
            }
        }
    } END_STATE_SAVE(memarena, stackstate);

# ifndef dNODEBUG
    // if debugging, check that all objects (except for disabled bodies,
    // unconnected joints, and joints that are connected to disabled bodies)
    // were tagged.
    {
        for (dxBody *b=world->firstbody; b; b=(dxBody*)b->next) {
            if (b->flags & dxBodyDisabled) {
                if (b->tag > 0) dDebug (0,"disabled body tagged");
            }
            else {
                if (b->tag <= 0) dDebug (0,"enabled body not tagged");
            }
        }
        for (dxJoint *j=world->firstjoint; j; j=(dxJoint*)j->next) {
            if ( (( j->node[0].body && (j->node[0].body->flags & dxBodyDisabled)==0 ) ||
                (j->node[1].body && (j->node[1].body->flags & dxBodyDisabled)==0) )
                &&
                j->isEnabled() ) {
                    if (j->tag <= 0) dDebug (0,"attached enabled joint not tagged");
            }
            else {
                if (j->tag > 0) dDebug (0,"unattached or disabled joint tagged");
            }
        }
    }
# endif

    size_t islandcount = ((size_t)(sizescurr - islandsizes) / sizeelements);
    islandsInfo.AssignInfo(islandcount, islandsizes, body, joint);

    return maxreq;
}

// this groups all joints and bodies in a world into islands. all objects
// in an island are reachable by going through connected bodies and joints.
// each island can be simulated separately.
// note that joints that are not attached to anything will not be included
// in any island, an so they do not affect the simulation.
//
// this function starts new island from unvisited bodies. however, it will
// never start a new islands from a disabled body. thus islands of disabled
// bodies will not be included in the simulation. disabled bodies are
// re-enabled if they are found to be part of an active island.

void dxProcessIslands (dxWorld *world, const dxWorldProcessIslandsInfo &islandsInfo,
  dReal stepSize, dstepper_fn_t stepper)
{
  const unsigned int sizeelements = 2;

  dxStepWorkingMemory *wmem = world->wmem;
  dIASSERT(wmem != NULL);

  dxWorldProcessContext *context = wmem->GetWorldProcessingContext();
  dIASSERT(context != NULL);

  size_t islandcount = islandsInfo.GetIslandsCount();
  unsigned int const *islandsizes = islandsInfo.GetIslandSizes();
  dxBody *const *body = islandsInfo.GetBodiesArray();
  dxJoint *const *joint = islandsInfo.GetJointsArray();

  dxWorldProcessMemArena *stepperarena = context->GetStepperMemArena();

  dxBody *const *bodystart = body;
  dxJoint *const *jointstart = joint;

  unsigned int const *const sizesend = islandsizes + islandcount * sizeelements;
  for (unsigned int const *sizescurr = islandsizes; sizescurr != sizesend; sizescurr += sizeelements) {
    unsigned int bcount = sizescurr[0];
    unsigned int jcount = sizescurr[1];

    BEGIN_STATE_SAVE(stepperarena, stepperstate) {
      // now do something with body and joint lists
      stepper (stepperarena,world,bodystart,bcount,jointstart,jcount,stepSize);
    } END_STATE_SAVE(stepperarena, stepperstate);

    bodystart += bcount;
    jointstart += jcount;
  }
}

//****************************************************************************
// World processing context management

dxWorldProcessMemArena *dxWorldProcessMemArena::ReallocateMemArena (
    dxWorldProcessMemArena *oldarena, size_t memreq,
    const dxWorldProcessMemoryManager *memmgr, float rsrvfactor, unsigned rsrvminimum)
{
    dxWorldProcessMemArena *arena = oldarena;
    bool allocsuccess = false;

    size_t nOldArenaSize;
    void *pOldArenaBuffer;

    do {
        size_t oldmemsize = oldarena ? oldarena->GetMemorySize() : 0;
        if (oldarena == NULL || oldmemsize < memreq) {
            nOldArenaSize = oldarena ? dxWorldProcessMemArena::MakeArenaSize(oldmemsize) : 0;
            pOldArenaBuffer = oldarena ? oldarena->m_pArenaBegin : NULL;

            if (!dxWorldProcessMemArena::IsArenaPossible(memreq)) {
                break;
            }

            size_t arenareq = dxWorldProcessMemArena::MakeArenaSize(memreq);
            size_t arenareq_with_reserve = AdjustArenaSizeForReserveRequirements(arenareq, rsrvfactor, rsrvminimum);
            size_t memreq_with_reserve = memreq + (arenareq_with_reserve - arenareq);

            if (oldarena != NULL) {
                oldarena->m_pArenaMemMgr->m_fnFree(pOldArenaBuffer, nOldArenaSize);
                oldarena = NULL;

                // Zero variables to avoid another freeing on exit
                pOldArenaBuffer = NULL;
                nOldArenaSize = 0;
            }

            // Allocate new arena
            void *pNewArenaBuffer = memmgr->m_fnAlloc(arenareq_with_reserve);
            if (pNewArenaBuffer == NULL) {
                break;
            }

            arena = (dxWorldProcessMemArena *)dEFFICIENT_PTR(pNewArenaBuffer);

            void *blockbegin = dEFFICIENT_PTR(arena + 1);
            void *blockend = dOFFSET_EFFICIENTLY(blockbegin, memreq_with_reserve);

             arena->m_pAllocBegin = blockbegin;
             arena->m_pAllocEnd = blockend;
             arena->m_pArenaBegin = pNewArenaBuffer;
             arena->m_pAllocCurrent = blockbegin;
             arena->m_pArenaMemMgr = memmgr;
        }

        allocsuccess = true;
    } while (false);

    if (!allocsuccess) {
        if (pOldArenaBuffer != NULL) {
            dIASSERT(oldarena != NULL);
            oldarena->m_pArenaMemMgr->m_fnFree(pOldArenaBuffer, nOldArenaSize);
        }
        arena = NULL;
    }

    return arena;
}

void dxWorldProcessMemArena::FreeMemArena (dxWorldProcessMemArena *arena)
{
    size_t memsize = arena->GetMemorySize();
    size_t arenasize = dxWorldProcessMemArena::MakeArenaSize(memsize);

    void *pArenaBegin = arena->m_pArenaBegin;
    arena->m_pArenaMemMgr->m_fnFree(pArenaBegin, arenasize);
}

size_t dxWorldProcessMemArena::AdjustArenaSizeForReserveRequirements(size_t arenareq, float rsrvfactor, unsigned rsrvminimum)
{
    float scaledarena = arenareq * rsrvfactor;
    size_t adjustedarena = (scaledarena < SIZE_MAX) ? (size_t)scaledarena : SIZE_MAX;
    size_t boundedarena = (adjustedarena > rsrvminimum) ? adjustedarena : (size_t)rsrvminimum;
    return dEFFICIENT_SIZE(boundedarena);
}

bool dxReallocateWorldProcessContext (dxWorld *world, dxWorldProcessIslandsInfo &islandsInfo,
    dReal stepSize, dmemestimate_fn_t stepperEstimate)
{
  dxStepWorkingMemory *wmem = AllocateOnDemand(world->wmem);
  if (wmem == NULL) return false;

  dxWorldProcessContext *context = wmem->SureGetWorldProcessingContext();
  if (context == NULL) return false;
  dIASSERT (context->IsStructureValid());

  const dxWorldProcessMemoryReserveInfo *reserveinfo = wmem->SureGetMemoryReserveInfo();
  const dxWorldProcessMemoryManager *memmgr = wmem->SureGetMemoryManager();

  size_t islandsreq = EstimateIslandsProcessingMemoryRequirements(world);
  dIASSERT(islandsreq == dEFFICIENT_SIZE(islandsreq));

  dxWorldProcessMemArena *stepperarena = NULL;
  dxWorldProcessMemArena *islandsarena = context->ReallocateIslandsMemArena(islandsreq, memmgr, 1.0f, reserveinfo->m_uiReserveMinimum);

  if (islandsarena != NULL)
  {
    size_t stepperreq = BuildIslandsAndEstimateStepperMemoryRequirements(islandsInfo, islandsarena, world, stepSize, stepperEstimate);
    dIASSERT(stepperreq == dEFFICIENT_SIZE(stepperreq));

    stepperarena = context->ReallocateStepperMemArena(stepperreq, memmgr, reserveinfo->m_fReserveFactor, reserveinfo->m_uiReserveMinimum);
  }

  return stepperarena != NULL;
}

void dxCleanupWorldProcessContext (dxWorld *world)
{
  dxStepWorkingMemory *wmem = world->wmem;
  if (wmem != NULL)
  {
    dxWorldProcessContext *context = wmem->GetWorldProcessingContext();
    if (context != NULL)
    {
      context->CleanupContext();
      dIASSERT(context->IsStructureValid());
    }
  }
}

dxWorldProcessMemArena *dxAllocateTemporaryWorldProcessMemArena(
    size_t memreq, const dxWorldProcessMemoryManager *memmgr/*=NULL*/, const dxWorldProcessMemoryReserveInfo *reserveinfo/*=NULL*/)
{
    const dxWorldProcessMemoryManager *surememmgr = memmgr ? memmgr : &g_WorldProcessMallocMemoryManager;
    const dxWorldProcessMemoryReserveInfo *surereserveinfo = reserveinfo ? reserveinfo : &g_WorldProcessDefaultReserveInfo;
    dxWorldProcessMemArena *arena = dxWorldProcessMemArena::ReallocateMemArena(NULL, memreq, surememmgr, surereserveinfo->m_fReserveFactor, surereserveinfo->m_uiReserveMinimum);
    return arena;
}

void dxFreeTemporaryWorldProcessMemArena(dxWorldProcessMemArena *arena)
{
    dxWorldProcessMemArena::FreeMemArena(arena);
}
