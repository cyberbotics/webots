
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

#ifndef _ODE_OBJECTS_FLUID_DYNAMICS_H_
#define _ODE_OBJECTS_FLUID_DYNAMICS_H_

#include <ode/fluid_dynamics/common_fluid_dynamics.h>
#include <ode/fluid_dynamics/immersion.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup fluids Fluids
 *
 *
 *
 *
 *
 *
 **/

/**
 * @brief Applies fluid static and dynamics forces (Archimedean, forces, drag forces)
 * to all bodies in the world according to the immersionLinks generated after collision detection
 * @ingroup fluids
 * @param world the simulated world
 * @remarks this step requires that collision detection was performed
 */
ODE_API int dFluidDynamicsStep (dWorldID w);

/**
 * @brief Retrieves the world attached to te given body.
 * @remarks
 *
 * @ingroup fluids
 */
ODE_API dWorldID dFluidGetWorld (dFluidID);

/**
 * @brief Create a body in given world.
 * @remarks
 * Default mass parameters are at position (0,0,0).
 * @ingroup fluids
 */
ODE_API dFluidID dFluidCreate (dWorldID);

/**
 * @brief Destroy a fluid.
 * @remarks
 * All immersion links that are attached to this fluid will be put into limbo:
 * i.e. unattached and not affecting the simulation, but they will NOT be
 * deleted.
 * @ingroup fluids
 */
ODE_API void dFluidDestroy (dFluidID);

/**
 * @brief Set the fluid's user-data pointer.
 * @ingroup fluids
 * @param data arbitraty pointer
 */
ODE_API void  dFluidSetData (dFluidID, void *data);

/**
 * @brief Get the fluid's user-data pointer.
 * @ingroup fluids
 * @return a pointer to the user's data.
 */
ODE_API void *dFluidGetData (dFluidID);

/**
 * @brief Set the fluid's density.
 * @ingroup fluids
 * @param zero or positive number
 */
ODE_API void dFluidSetDensity (dFluidID, dReal density);

/**
 * @brief Get the fluid's dynamic viscosity.
 * @ingroup fluids
 * @return the viscosity value.
 */
ODE_API dReal dFluidGetViscosity (dFluidID);

/**
 * @brief Set the fluid's viscosity.
 * @ingroup fluids
 * @param zero or positive number
 */
ODE_API void dFluidSetViscosity (dFluidID, dReal viscosity);

/**
 * @brief Get the fluid's density.
 * @ingroup fluids
 * @return the density value.
 */
ODE_API dReal dFluidGetDensity (dFluidID);

/**
 * @brief Set the fluid's stream linear velocity.
 * @ingroup fluids
 */
ODE_API void dFluidSetStreamVel (dFluidID, const dReal *velocity);

/**
 * @brief Get the fluid's stream linear velocity.
 * @ingroup fluids
 * @return the stream linear velocity.
 */
ODE_API const dReal* dFluidGetStreamVel (dFluidID);

/**
 * @brief Manually enable a fluid.
 * @param dFluidID identification of fluid.
 * @ingroup fluids
 */
ODE_API void dFluidEnable (dFluidID);

/**
 * @brief Manually disable a fluid.
 * @ingroup fluids
 * @remarks
 * A disabled fluid that is connected through a immersion link to an enabled fluid will
 * be automatically re-enabled at the next simulation step.
 */
ODE_API void dFluidDisable (dFluidID);

/**
 * @brief Check wether a fluid is enabled.
 * @ingroup fluids
 * @return 1 if a fluid is currently enabled or 0 if it is disabled.
 */
ODE_API int dFluidIsEnabled (dFluidID);

/**
 * @brief Return the first geom associated with the fluid.
 *
 * You can traverse through the geoms by repeatedly calling
 * dFluidGetNextGeom().
 *
 * @return the first geom attached to this fluid, or 0.
 * @ingroup fluids
 */
ODE_API dGeomID dFluidGetFirstGeom (dFluidID f);

/**
 * @brief returns the next geom associated with the same fluid.
 * @param g a geom attached to some fluid.
 * @return the next geom attached to the same fluid, or 0.
 * @sa dFluidGetFirstGeom
 * @ingroup fluids
 */
ODE_API dGeomID dFluidGetNextGeom (dGeomID f);

/**
 * @defgroup immersion links Immersion links
 * An immersion link is structure that contains
 * information about the (possibly partial) immersion of body into a fluid
 */

/**
 * @brief Create a new immersion link.
 * @ingroup immersion links
 * @remarks
 * The immersion link is initially in "limbo" (i.e. it has no effect on the simulation)
 * because it does not connect to any bodies and fluids.
 * @param dImmersionLinkGroupID set to 0 to allocate the immersion link normally.
 * If it is nonzero the immersion link is allocated in the given immersion link group.
 */
ODE_API dImmersionLinkID dImmersionLinkCreate (dWorldID, dImmersionLinkGroupID, const dImmersion *);

/**
 * @brief Destroy an immersion link.
 * @ingroup immersion links
 *
 * disconnects it from its attached body and fluid and removing it from the world.
 * However, if the immersion link is a member of a group then this function has no
 * effect - to destroy that immersion link the group must be emptied or destroyed.
 */
ODE_API void dImmersionLinkDestroy (dImmersionLinkID);

/**
 * @brief Create a immersion link group
 * @ingroup immersion links
 */
ODE_API dImmersionLinkGroupID dImmersionLinkGroupCreate ();

/**
 * @brief Destroy a immersion link group.
 * @ingroup immersion links
 *
 * All immersion links in the immersion link group will be destroyed.
 */
ODE_API void dImmersionLinkGroupDestroy (dImmersionLinkGroupID);

/**
 * @brief Empty a immersion link group.
 * @ingroup immersion links
 *
 * All immersion links in the immersion link group will be destroyed,
 * but the immersion link group itself will not be destroyed.
 */
ODE_API void dImmersionLinkGroupEmpty (dImmersionLinkGroupID);

/**
 * @brief Attach the immersion link to some new body and fluid.
 * @ingroup immersion links
 *
 * If the immersion link is already attached, it will be detached from the old bodies
 * first.
 * Setting either the body or the fluid to zero puts the immersion link into "limbo", i.e. it will
 * have no effect on the simulation.
 */
ODE_API void dImmersionLinkAttach (dImmersionLinkID, dBodyID, dFluidID);

/**
 * @brief Manually enable a immersion link.
 * @param dImmersionLinkID identification of immersion link.
 * @ingroup immersion links
 */
ODE_API void dImmersionLinkEnable (dImmersionLinkID);

/**
 * @brief Manually disable a immersion link.
 * @ingroup immersion links
 * @remarks
 * A disabled immersion link will not affect the simulation
 */
ODE_API void dImmersionLinkDisable (dImmersionLinkID);

/**
 * @brief Check wether a immersion link is enabled.
 * @ingroup immersion links
 * @return 1 if a immersion link is currently enabled or 0 if it is disabled.
 */
ODE_API int dImmersionLinkIsEnabled (const dImmersionLinkID);

/**
 * @brief Set the user-data pointer
 * @ingroup immersion links
 */
ODE_API void dImmersionLinkSetData (dImmersionLinkID, void *data);

/**
 * @brief Get the user-data pointer
 * @ingroup immersion links
 */
ODE_API void *dImmersionLinkGetData (const dImmersionLinkID);

/**
 * @brief Return the body that this immersion link connects.
 * @ingroup immersion links
 * @remarks
 * If the body IDs is zero, the immersion link is in ``limbo'' and has no effect on
 * the simulation.
 */
ODE_API dBodyID dImmersionLinkGetBody (const dImmersionLinkID);

/**
 * @brief Return the fluid that this immersion link connects.
 * @ingroup immersion links
 * @remarks
 * If the fluid ID is zero, the immersion link is in ``limbo'' and has no effect on
 * the simulation.
 */
ODE_API dBodyID dImmersionLinkGetFluid (const dImmersionLinkID);

/**
 * @ingroup immersion links
 */
ODE_API dImmersionLinkID dLinkingImmersionLink (const dBodyID, const dFluidID);

/**
 * @brief Utility function
 * @return 1 if the the body and the fluid are connected together by
 * a immersion link, otherwise return 0.
 * @ingroup immersion links
 */
ODE_API int dAreLinked (const dBodyID, const dFluidID);

/**
 * @brief Get the number of immersion links that are attached to this body.
 * @ingroup bodies
 * @return number of immersion links
 */
ODE_API int dBodyGetNumImmersionLinks (dBodyID b);

/**
 * @brief Return an immersion link attached to this body, given by index.
 * @ingroup bodies
 * @param index valid range is  0 to n-1 where n is the value returned by
 * dBodyGetNumImmersionLinks().
 */
ODE_API dImmersionLinkID dBodyGetImmersionLink (dBodyID, int index);

/**
 * @brief Get the number of immersion links that are attached to this fluid.
 * @ingroup fluids
 * @return number of immersion links
 */
ODE_API int dFluidGetNumImmersionLinks (dFluidID b);

/**
 * @brief Return an immersion link attached to this fluid, given by index.
 * @ingroup fluids
 * @param index valid range is  0 to n-1 where n is the value returned by
 * dFluidGetNumImmersionLinks().
 */
ODE_API dImmersionLinkID dFluidGetImmersionLink (dFluidID, int index);

/**
 * @brief Return a pointer to an allocated immersion outline
 * @ingroup fluids
 */
ODE_API dImmersionOutlineID dImmersionOutlineCreate ();

/**
 * @brief Destroys an allocated immersion outline
 * @ingroup fluids
 */
ODE_API void dImmersionOutlineDestroy (dImmersionOutlineID);

/**
 * @brief Return the size of the straight edge array in the given immersion outline.
 * @ingroup fluids
 */
ODE_API int dImmersionOutlineGetStraightEdgesSize (dImmersionOutlineID);

/**
 * @brief Return a straight edge of the immersion outline, given by index.
 * @ingroup fluids
 * @param index valid range is  0 to n-1 where n is the value returned by
 * dImmersionOutlineGetStraightEdgesSize().
 */
ODE_API void dImmersionOutlineGetStraightEdge (dImmersionOutlineID, int index, dStraightEdge *);

/**
 * @brief Return the straight edge end of the immersion outline straight edge given by index.
 * @ingroup fluids
 * @param index valid range is  0 to n-1 where n is the value returned by
 * dImmersionOutlineGetStraightEdgesSize().
 */
ODE_API const dReal *dImmersionOutlineGetStraightEdgeEnd (dImmersionOutlineID, int index);

/**
 * @brief Return the straight edge origin of the immersion outline straight edge given by index.
 * @ingroup fluids
 * @param index valid range is  0 to n-1 where n is the value returned by
 * dImmersionOutlineGetStraightEdgesSize().
 */
ODE_API const dReal *dImmersionOutlineGetStraightEdgeOrigin (dImmersionOutlineID, int index);

/**
 * @brief Return the size of the curved edge array in the given immersion outline.
 * @ingroup fluids
 */
ODE_API int dImmersionOutlineGetCurvedEdgesSize (dImmersionOutlineID);

/**
 * @brief Return a curved edge of the immersion outline, given by index.
 * @ingroup fluids
 * @param index valid range is  0 to n-1 where n is the value returned by
 * dImmersionOutlineGetCurvedEdgesSize().
 */
ODE_API void dImmersionOutlineGetCurvedEdge (dImmersionOutlineID, int index, dCurvedEdge *);

#ifdef __cplusplus
}
#endif

#endif
