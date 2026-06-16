#ifndef _ODE_MT_UTIL_MT_H_
#define _ODE_MT_UTIL_MT_H_

#include "ode/common.h"
#include "ode/objects.h"
#include "ode/collision_space.h"
#include "ode/collision_trimesh.h"

#ifdef dNODEBUG
#define ODE_PRINT(...) do {} while(0)
#define ODE_INFO(...) do {} while(0)
#define ODE_IMPORTANT(...) do {} while(0)
#else
// Debug logging is disabled, but the arguments are still referenced in dead code that
// the optimizer removes. This keeps variables used only for debug logging from being
// reported as unused, without evaluating the arguments at runtime or producing output.
inline void ode_debug_noop(...) {}
#define ODE_PRINT(...) do { if (false) ode_debug_noop(__VA_ARGS__); } while(0)
#define ODE_INFO(...) do { if (false) ode_debug_noop(__VA_ARGS__); } while(0)
#define ODE_IMPORTANT(...) do { if (false) ode_debug_noop(__VA_ARGS__); } while(0)
// Display the debug messages through ODE's messaging facility (the same channel used by
// dDEBUGMSG). Disabled in release builds (dNODEBUG), where these expand to no-ops.
//#define ODE_PRINT(...) do { dMessage(d_ERR_UASSERT, __VA_ARGS__); } while(0)
//#define ODE_INFO(...) do { dMessage(d_ERR_UASSERT, __VA_ARGS__); } while(0)
//#define ODE_IMPORTANT(...) do { dMessage(d_ERR_UASSERT, __VA_ARGS__); } while(0)
#endif

typedef dWorldID dWorldCreateFunction();
typedef dSpaceID dSpaceCreateFunction(dSpaceID);
typedef void dAddGeomToSpaceFunction(dxSpace *, dxGeom *);
typedef void dSpaceDestroyFunction(dxSpace *);
typedef void dWorldDestroyFunction(dxWorld *);

typedef void dGeomSetPositionFunction (dxGeom *g, dReal x, dReal y, dReal z);
typedef void dGeomSetRotationFunction (dxGeom *g, const dMatrix3 R);
typedef void dGeomSetQuaternionFunction (dxGeom *g, const dQuaternion quat);
typedef void dGeomBoxSetLengthsFunction (dxGeom *g, dReal x, dReal y, dReal z);
typedef void dGeomCapsuleSetParamsFunction (dxGeom *g, dReal radius, dReal length);
typedef void dGeomCylinderSetParamsFunction (dxGeom *g, dReal radius, dReal length);
typedef void dGeomTriMeshSetDataFunction (dxGeom *g, dTriMeshDataID data);
typedef void dGeomSphereSetRadiusFunction (dxGeom *g, dReal radius);

typedef void dGeomPlaneSetParamsFunction (dGeomID g, dReal a, dReal b, dReal c, dReal d);

class util_MT
{
private:
    static int getBodyIndex(dxBody *_b);
    static void buildBodyIndices(dWorldID);

public:
    // Function pointers to overrided ODE functions
    static dWorldCreateFunction *worldFunc;
    static dSpaceCreateFunction *spaceFunc;
    static dSpaceDestroyFunction *spaceDestroyFunc;// = &dSpaceDestroy;
    static dWorldDestroyFunction *worldDestroyFunc;// = &dWorldDestroy;
    static dAddGeomToSpaceFunction *addGeomToSpaceFunc;// = &dSpaceAdd;

public:
    static void copyWorldParameters(dWorldID _destWorld, dWorldID _srcWorld);
    static void cleanTags(dxWorld* _world, class dxClusteredWorldAndSpace* _cwas);

    static dReal odeMax(dReal val1, dReal val2, dReal val3);
    static dReal odeMin(dReal val1, dReal val2, dReal val3);

#ifndef dNODEBUG
    static void checkClusterNodes(int clusterCount, struct dxClusterNode** clusterAABBs);
    static void checkBodyAndGeomConsistency(int clusterCount, struct ClusterNode** clusterArray);
#endif
    static void printWorldJoints(dxWorld *_world);
    static void printJoints(dxBody *body);
    static void printBodyInfo(dxBody *_body);

    static int findBodyCount(dxWorld *world);
    static int findJointCount(dxWorld *world);
    static int findJointCount(dxBody *body);
    static void printSpaceBodies(dxSpace *space);
    static void printAABBList(struct dxClusterAABB* _aabb);

};

#endif
