#ifndef _ODE_MT_ODE_MT_H_
#define _ODE_MT_ODE_MT_H_

#include "ode/common.h"
#include "util.h"
#include "util_MT.h"
#include "objects.h"
#include "ode/collision_space.h"
#include "threading/threadManager.h"

dxThreadManager *dThreadManager();

/* multi-threaded utility functions for collision detection and LCP solver */
typedef void dSpaceCollideFunction(dSpaceID, void *, dNearCallback);
typedef int dWorldStepFunction(dxWorld *, dReal);

dWorldID dWorldCreate_MT(dWorldCreateFunction *_worldFunc);
dSpaceID dSpaceCreate_MT(dSpaceID _space, dSpaceCreateFunction *_spaceFunc);

typedef void dInitODEFunction();
typedef void dCloseODEFunction();
typedef int dInitODE2Function(unsigned int);
void dInitODE_MT(dInitODEFunction *_initFunc);
int dInitODE2_MT(unsigned int uiInitFlags, dInitODE2Function *_initFunc);
void dCloseODE_MT(dCloseODEFunction *_closeFunc);

void dSpaceDestroy_MT(dxSpace *space, dSpaceDestroyFunction *_spaceDestroyFunc);
void dWorldDestroy_MT(dxWorld *world, dWorldDestroyFunction *_worldDestroyFunc);

typedef dxBody* dBodyCreateFunction(dxWorld *);
typedef void dBodyDestroyFunction(dxBody *);
typedef dxFluid* dFluidCreateFunction(dxWorld *);
typedef void dFluidDestroyFunction(dxFluid *);
typedef void dGeomDestroyFunction(dxGeom *);
typedef void dSpaceAddFunction (dxSpace *, dxGeom *);

dxBody* dBodyCreate_MT(dxWorld *_world, dBodyCreateFunction *_bodyCreateFunc);
void dBodyDestroy_MT(dxBody *_body, dBodyDestroyFunction *_bodyDestroyFunc);
dxFluid* dFluidCreate_MT(dxWorld *_world, dFluidCreateFunction *_fluidCreateFunc);
void dFluidDestroy_MT(dxFluid *_fluid, dFluidDestroyFunction *_fluidDestroyFunc);
void dGeomDestroy_MT(dxGeom *_geom, dGeomDestroyFunction *_geomDestroyFunc);
void dSpaceAdd_MT (dxSpace *_space, dxGeom *_geom, dSpaceAddFunction *_spaceAddFunc);

void dGeomSetPosition_MT (dxGeom *g, dReal x, dReal y, dReal z, dGeomSetPositionFunction *_geomSetPositionFunc);
void dGeomSetRotation_MT (dxGeom *g, const dMatrix3 R, dGeomSetRotationFunction *_geomSetRotationFunc);
void dGeomSetQuaternion_MT (dxGeom *g, const dQuaternion quat, dGeomSetQuaternionFunction *_geomSetQuaternionFunc);
void dGeomBoxSetLengths_MT (dxGeom *g, dReal lx, dReal ly, dReal lz, dGeomBoxSetLengthsFunction *_geomBoxSetLengthsFunc);
void dGeomCapsuleSetParams_MT (dxGeom *g, dReal radius, dReal length, dGeomCapsuleSetParamsFunction *_geomCapsuleSetParamsFunc);
void dGeomCylinderSetParams_MT (dxGeom *g, dReal radius, dReal length, dGeomCylinderSetParamsFunction *_geomCylinderSetParamsFunc);
void dGeomTriMeshSetData_MT (dxGeom *g, dTriMeshDataID data, dGeomTriMeshSetDataFunction *_geomTriMeshSetDataFunc);
void dGeomSphereSetRadius_MT (dxGeom *g, dReal radius, dGeomSphereSetRadiusFunction *_geomSphereSetRadiusFunc);

void dGeomPlaneSetParams_MT (dGeomID g, dReal a, dReal b, dReal c, dReal d, dGeomPlaneSetParamsFunction *_geomPlaneSetParamsFunc);
void dGeomOffset_MT(dxGeom *g);
//*/
#endif
