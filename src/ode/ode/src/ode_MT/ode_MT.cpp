#include "ode_MT.h"
#include "util_MT.h"
#include "ode/objects.h"
#include "clustering/cluster.h"
#include "threading/threadManagerPthread.h"
#include "threading/threadManagerOpenMP.h"
#include "heuristics/heuristicsManager.h"
#include "clustering/clusterManager.h"

static bool bMTmode = true;
static int threadCount = 4;
static bool bMainThread = true;
static bool bPhysicsStepsReverse = false;

static bool bRefreshClusters = true;
static bool bODEInitialized = false;

// Collision & LCP Data
static void *colData;
static dNearCallback *colCallback;
static dSpaceUpdateFunction *spaceUpdateFunc;
static dReal LCPstepsize = 0.0f;

static dWorldStepFunction *stepFunc;

static dWorldID currentWorld;
static dSpaceID currentSpace;

struct threadCluster
{
  int tid;
  dxMaintainedArray<int> clusterIDs;
  int clusterCount;
  int bodyCount;
  int geomCount;
};

static dxMaintainedArray<threadCluster> threadClusters;

static int findLeastLoadedThread()
{
    int minVal = 99999;
    int minIndex = -1;
    for (int t=0; t<threadCount; t++)
    {
        if (threadClusters[t].geomCount < minVal)
        {
            minVal = threadClusters[t].geomCount;
            minIndex = t;
        }
    }
    return minIndex;
}

static void clusterChangeCallback();

#ifdef ODE_THREADMODE_PTHREAD
static dxThreadManager *threadManager = new dxThreadManagerPthread();
#else
static dxThreadManager *threadManager = new dxThreadManagerOpenMP();
#endif
static dxClusterManager* clusterManager = new dxClusterManager(&clusterChangeCallback);
static dxHeuristicsManager *heuristicsManager = new dxHeuristicsManager(threadCount, 10, clusterManager, threadManager);

dxThreadManager *dThreadManager() {
  return threadManager;
}

static void clusterChangeCallback()
{
    // cluster count has changed, update thread load balancing
    ODE_IMPORTANT("Cluster count changed!\n");
    for (int t=0; t<threadCount; t++)
    {
        threadClusters[t].tid = t;
        threadClusters[t].clusterCount = 0;
        threadClusters[t].bodyCount = 0;
        threadClusters[t].geomCount = 0;
    }
    for (int j=0; j<clusterManager->currentCWAS->activeClusterCount; j++)
    {
        int k = clusterManager->currentCWAS->activeClusters[j];

        if (clusterManager->currentCWAS->worlds[k] == NULL || clusterManager->currentCWAS->spaces[k] == 0)
            continue;

        int tid = findLeastLoadedThread();
        threadClusters[tid].clusterIDs[threadClusters[tid].clusterCount++] = k;
        threadClusters[tid].bodyCount += clusterManager->currentCWAS->worlds[k]->nb;
        threadClusters[tid].geomCount += clusterManager->currentCWAS->spaces[k]->count;
    }
}

static void dHashSpaceCollide(int tid, void *data, dNearCallback *callback)
{
    dSpaceCollide(clusterManager->currentCWAS->spaces[tid], colData, colCallback);

#ifdef TEST_COLLISIONS
    ODE_INFO("Cluster %d: dHashSpaceCollide: %d out of %d objects(%d) with static(%d)\n", kid, n, m, clusterAABBs[kid]->count + 1, bigboxCount);
#endif
}

bool clusterCollide(int tid)
{
  ODE_PRINT("ClusterCollide with id: %d\n", tid);
  bool bError = false;
  if (bMTmode) {
      for (int j = 0; j < threadClusters[tid].clusterCount; ++j)
      {
          int kid = threadClusters[tid].clusterIDs[j];
          // first of all, see if any static clusters need to be added or removed
          clusterManager->currentCWAS->checkClusterConsistency(kid);
          clusterManager->currentCWAS->spaces[kid]->cleanGeoms();
          bError = clusterManager->currentCWAS->updateClusterAABBsAndTable(kid);
          if (bError == false)
            dHashSpaceCollide(kid, colData, colCallback);
      }
  }
  else if (tid < 1 && !bMainThread) dSpaceCollide(currentSpace, colData, colCallback);
  return bError;
}

void clusterLCP(int tid)
{
  ODE_PRINT("Performing child LCP %d\n", tid);
  ODE_PRINT("Original Joint count: %d\n", currentWorld->nj);
  ODE_PRINT("Cluster Joint count: %d\n", clusterManager->currentCWAS->worlds[tid]->nj);
  // in reverse order, the collider runs after the lcp solver
  // hence, any errors from the collider in the previous frame
  // will have been sorted out by the CheckHashSpaceConsistency function
  // at the beginning of this frame. Therefore, no need to check for collider error
  // hypothetically.
  //if (bError == false) // FIXME: remove that line when sure it makes no problem
  {
      for (int j = 0; j < threadClusters[tid].clusterCount; ++j)
      {
          int kid = threadClusters[tid].clusterIDs[j];
          if (bMTmode) stepFunc(clusterManager->currentCWAS->worlds[kid], LCPstepsize);
          else if (tid < 1 && !bMainThread) stepFunc(currentWorld, LCPstepsize);
      }

  }
}

void* simLoopChild(int tid, void* args)
{
    if (tid >= threadCount || clusterManager->currentCWAS == NULL || tid >= clusterManager->currentCWAS->clusterCount)
        return NULL;

    int i = tid;
    if (bPhysicsStepsReverse == false)
    {
        bool bError = clusterCollide(i);
        if (bError == false)
            clusterLCP(i);
    } else
    {
        clusterLCP(i);
        if (spaceUpdateFunc != NULL)
          spaceUpdateFunc(tid);
        clusterCollide(i);
    }
  return NULL;
}

int dAreGeomsSame(dGeomID g1, dGeomID g2)
{
    ODE_PRINT("Calling dAreGeomsSame for %p and %p\n", g1, g2);
    if (clusterManager->currentCWAS == NULL)
        return g1 == g2;

    std::map<int, dxClusteredWorldAndSpace::geomMap>& geomDuplicationMap = clusterManager->currentCWAS->getGeomDuplicationMap();
    for (int k=0; k<clusterManager->currentCWAS->getClusterCount(); k++)
    {
        if (geomDuplicationMap[k][g1] == NULL && geomDuplicationMap[k][g2] == NULL)
           continue;

        if (geomDuplicationMap[k][g1] == g2 || geomDuplicationMap[k][g2] == g1 || geomDuplicationMap[k][g1] == geomDuplicationMap[k][g2])
            return 1;
    }

    return g1 == g2;
}

dxClusteredWorldAndSpace* dGetClusteredWorldAndSpace(dWorldID _world, dSpaceID _space)
{
    return clusterManager->getCWAS(_world, _space);
}

ODE_API int dClusterGetCount(dWorldID _world, dSpaceID _space)
{
    return clusterManager->getCWAS(_world, _space)->clusterCount;
}
ODE_API float dClusterGetGridStep(dWorldID _world, dSpaceID _space)
{
    return clusterManager->getCWAS(_world, _space)->gridstep;
}
ODE_API void dClusterGetCenter(dWorldID _world, dSpaceID _space, float &_x, float &_y, float &_z)
{
    _x = clusterManager->getCWAS(_world, _space)->hsAxis[0];
    _y = clusterManager->getCWAS(_world, _space)->hsAxis[1];
    _z = clusterManager->getCWAS(_world, _space)->hsAxis[2];
}
ODE_API dxClusterNode** dClusterGetClusterAABBs(dWorldID _world, dSpaceID _space)
{
    return clusterManager->getCWAS(_world, _space)->clusterAABBs.data();
}

ODE_API int dThreadGetSpacesCount(int threadID) {
    if (threadID >= 0 && bMTmode)
        return threadClusters[threadID].clusterCount;
    return 1;
}

ODE_API dSpaceID dThreadGetSpaceID(int threadID, int index) {
    if (threadID >= 0 && bMTmode) {
        int kid = threadClusters[threadID].clusterIDs[index];
        return clusterManager->currentCWAS->spaces[kid];
    }
    return currentSpace;
}

ODE_API void dGeomSetDynamicFlag(dGeomID geom)
{
    geom->is_dynamic = true;
}

void dSpaceCollideAndWorldStep (dSpaceID _space, void *_data, dNearCallback *_callback, dxWorld *_world, dReal _stepsize, dWorldStepFunction *_stepFunc)
{
    ODE_PRINT("dSpaceCollideAndWorldStep\n");
    bPhysicsStepsReverse = false;
    ODE_PRINT("StepWorld: %p, StepSpace: %p\n", _world, _space);
#ifdef ODE_MT

    LCPstepsize = _stepsize;
    stepFunc = _stepFunc;
    colData = _data;
    colCallback = _callback;
    currentSpace = _space;
    currentWorld = _world;

    if (bMTmode)
    {
        clusterManager->currentCWAS = clusterManager->getCWAS(_world, _space);

        clusterManager->simLoop(bRefreshClusters);
        threadManager->simLoop();
        heuristicsManager->simLoop(_world, _space);

        bRefreshClusters = false;
    } else {
        dSpaceCollide(_space, _data, _callback);
        _stepFunc(_world, _stepsize);
    }

#else
    dSpaceCollide(_space, _data, _callback);
    _stepFunc(_world, _stepsize);
#endif
}

void dWorldStepAndSpaceCollide (dSpaceID _space, void *_data, dNearCallback *_callback, dxWorld *_world, dReal _stepsize, dWorldStepFunction *_stepFunc, dSpaceUpdateFunction* _spaceUpdateFunc)
{
    ODE_PRINT("dWorldStepAndSpaceCollide\n");
    bPhysicsStepsReverse = true;
#ifdef ODE_MT

    LCPstepsize = _stepsize;
    stepFunc = _stepFunc;
    spaceUpdateFunc = _spaceUpdateFunc;
    colData = _data;
    colCallback = _callback;
    currentSpace = _space;
    currentWorld = _world;

    if (bMTmode)
    {
        clusterManager->currentCWAS = clusterManager->getCWAS(_world, _space);

        clusterManager->simLoop(bRefreshClusters);
        threadManager->simLoop();
        heuristicsManager->simLoop(_world, _space);

        bRefreshClusters = false;
    } else {
        _stepFunc(_world, _stepsize);
        if (_spaceUpdateFunc != NULL)
          _spaceUpdateFunc(-1);
        dSpaceCollide(_space, _data, _callback);
    }

#else
    _stepFunc(_world, _stepsize);
    if (_spaceUpdateFunc != NULL)
      _spaceUpdateFunc(-1);
    dSpaceCollide(_space, _data, _callback);
#endif
}

static void initODE()
{

    if (!bODEInitialized)
    {
        bODEInitialized = true;
        threadManager->init(threadCount, &simLoopChild);
        heuristicsManager->reset();
    }
}

void dInitODE_MT(dInitODEFunction *_initFunc)
{
    ODE_INFO("dInitODE\n");

    initODE();

    _initFunc();
}

int dInitODE2_MT(unsigned int uiInitFlags, dInitODE2Function *_initFunc)
{
    ODE_INFO("dInitODE2\n");

    initODE();

    return _initFunc(uiInitFlags);
}

void dCloseODE_MT(dCloseODEFunction *_closeFunc)
{
    ODE_INFO("dCloseODE\n");
	_closeFunc();
}

static void resetWorldsAndSpaces()
{
    ODE_PRINT("Resetting worlds and spaces...\n");
    bRefreshClusters = true;

    currentSpace->cleanGeoms();
    for (int i = 0; i < clusterManager->currentCWAS->clusterCount; ++i)
        if (clusterManager->currentCWAS && clusterManager->currentCWAS->spaces[i]) clusterManager->currentCWAS->spaces[i]->cleanGeoms();

    clusterManager->currentCWAS->recombineClusters();
    clusterManager->removeCWAS(currentWorld, currentSpace);
    currentWorld = NULL;
    currentSpace = NULL;
    clusterManager->currentCWAS = NULL;

    heuristicsManager->reset();
}

void dToggleODE_MT(int numThreads)
{
    if (numThreads == threadCount)
        return;

#ifdef ODE_MT
    bool mode = numThreads > 0;
    if (mode)
        ODE_IMPORTANT("Switching to multi-threaded mode (%d threads)...\n", numThreads);
    else
        ODE_IMPORTANT("Switching to single-threaded mode...\n");

    if (clusterManager->currentCWAS)
        // recombine clusters
        resetWorldsAndSpaces();

    threadManager->reinit(numThreads);
    threadCount = numThreads;

    bMTmode = mode;
#endif
}

dWorldID dWorldCreate_MT(dWorldCreateFunction *_worldFunc)
{
    if (bMTmode && !bRefreshClusters && clusterManager->currentCWAS)
        resetWorldsAndSpaces();

    util_MT::worldFunc = _worldFunc;
    dWorldID newWorld = _worldFunc();

    return newWorld;
}

dSpaceID dSpaceCreate_MT(dSpaceID _space, dSpaceCreateFunction *_spaceFunc)
{
    if (bMTmode && !bRefreshClusters && clusterManager->currentCWAS)
        resetWorldsAndSpaces();

    util_MT::spaceFunc = _spaceFunc;
    dSpaceID newSpace = _spaceFunc(_space);

    return newSpace;
}

void dSpaceDestroy_MT(dxSpace *space, dSpaceDestroyFunction *_spaceDestroyFunc)
{
    ODE_PRINT("Removing space %p\n", space);

    if (bMTmode && !bRefreshClusters && clusterManager->currentCWAS)
        resetWorldsAndSpaces();

    util_MT::spaceDestroyFunc = _spaceDestroyFunc;

    clusterManager->removeCWAS(NULL, space);
    return _spaceDestroyFunc(space);
}

void dWorldDestroy_MT(dxWorld *world, dWorldDestroyFunction *_worldDestroyFunc)
{
    ODE_INFO("Removing world %p\n", world);

    if (bMTmode && !bRefreshClusters && clusterManager->currentCWAS)
        resetWorldsAndSpaces();

    util_MT::worldDestroyFunc = _worldDestroyFunc;

    clusterManager->removeCWAS(world, NULL);
    return _worldDestroyFunc(world);
}

dxBody* dBodyCreate_MT(dxWorld *_world, dBodyCreateFunction *_bodyCreateFunc)
{
    if (bMTmode && !bRefreshClusters && clusterManager->currentCWAS)
        resetWorldsAndSpaces();

    return _bodyCreateFunc(_world);
}

void dBodyDestroy_MT(dxBody *_body, dBodyDestroyFunction *_bodyDestroyFunc)
{
    ODE_PRINT("Destroying body %p\n", _body);

    if (bMTmode && !bRefreshClusters && clusterManager->currentCWAS)
        resetWorldsAndSpaces();

    return _bodyDestroyFunc(_body);
}

dxFluid* dFluidCreate_MT(dxWorld *_world, dFluidCreateFunction *_fluidCreateFunc)
{
    if (bMTmode && !bRefreshClusters && clusterManager->currentCWAS && clusterManager->currentCWAS->clusterCount > 0)
    {
        resetWorldsAndSpaces();
        //bMTmode = true;
    }
    return _fluidCreateFunc(_world);
}

void dFluidDestroy_MT(dxFluid *_fluid, dFluidDestroyFunction *_fluidDestroyFunc)
{
    ODE_PRINT("Destroying fluid %p\n", _fluid);

    if (bMTmode && !bRefreshClusters && clusterManager->currentCWAS && clusterManager->currentCWAS->clusterCount > 0)
    {
        resetWorldsAndSpaces();
        //bMTmode = true;
    }
    //*/
    return _fluidDestroyFunc(_fluid);
}

void dGeomDestroy_MT(dxGeom *_geom, dGeomDestroyFunction *_geomDestroyFunc)
{
    ODE_PRINT("Destroying geom %p\n", _geom);

    if (bMTmode && !bRefreshClusters && clusterManager->currentCWAS)
        resetWorldsAndSpaces();

    return _geomDestroyFunc(_geom);
}

void dSpaceAdd_MT(dxSpace *_space, dxGeom *_geom, dSpaceAddFunction *_spaceAddFunc)
{
    ODE_PRINT("Adding geom %p to space %p\n", _geom, _space);

    util_MT::addGeomToSpaceFunc = _spaceAddFunc;

    if (bMTmode && !bRefreshClusters && clusterManager->currentCWAS)
        resetWorldsAndSpaces();

    return _spaceAddFunc(_space, _geom);
}

void dWorldRefreshParameters(dWorldID w)
{
    dxClusteredWorldAndSpace* clusterWorlds = clusterManager->currentCWAS;
    if (clusterWorlds == NULL || clusterWorlds->originalWorld != w)
        return;

    if (clusterWorlds)
        for (int i = 0; i < clusterWorlds->clusterCount; ++i)
            if (clusterWorlds->worlds[i] != NULL)
                util_MT::copyWorldParameters(clusterWorlds->worlds[i], w);
}

void dGeomSetPosition_MT (dxGeom *g, dReal x, dReal y, dReal z, dGeomSetPositionFunction *_geomSetPositionFunc)
{
  _geomSetPositionFunc(g, x, y, z);
  g->recomputeAABB();

  if (clusterManager->currentCWAS)
    clusterManager->currentCWAS->propagateGeomPosition(g, x, y, z, _geomSetPositionFunc);
}

void dGeomSetRotation_MT (dxGeom *g, const dMatrix3 R, dGeomSetRotationFunction *_geomSetRotationFunc)
{
  _geomSetRotationFunc(g, R);
  g->recomputeAABB();

  if (clusterManager->currentCWAS)
    clusterManager->currentCWAS->propagateGeomRotation(g, R, _geomSetRotationFunc);
}

void dGeomSetQuaternion_MT (dxGeom *g, const dQuaternion quat, dGeomSetQuaternionFunction *_geomSetQuaternionFunc)
{
  if (clusterManager->currentCWAS)
    clusterManager->currentCWAS->propagateGeomQuaternion(g, quat, _geomSetQuaternionFunc);
  return _geomSetQuaternionFunc(g, quat);
}

void dGeomBoxSetLengths_MT (dxGeom *g, dReal lx, dReal ly, dReal lz, dGeomBoxSetLengthsFunction *_geomBoxSetLengthsFunc)
{
  _geomBoxSetLengthsFunc(g, lx, ly, lz);
  g->recomputeAABB();

  if (clusterManager->currentCWAS)
    clusterManager->currentCWAS->propagateGeomBoxSetLengths(g, lx, ly, lz, _geomBoxSetLengthsFunc);
}

void dGeomCapsuleSetParams_MT (dxGeom *g, dReal radius, dReal length, dGeomCapsuleSetParamsFunction *_geomCapsuleSetParamsFunc) {

  _geomCapsuleSetParamsFunc(g, radius, length);
  g->recomputeAABB();

  if (clusterManager->currentCWAS)
    clusterManager->currentCWAS->propagateGeomCapsuleSetParams(g, radius, length, _geomCapsuleSetParamsFunc);
}

void dGeomCylinderSetParams_MT (dxGeom *g, dReal radius, dReal length, dGeomCylinderSetParamsFunction *_geomCylinderSetParamsFunc) {

  _geomCylinderSetParamsFunc(g, radius, length);
  g->recomputeAABB();

  if (clusterManager->currentCWAS)
    clusterManager->currentCWAS->propagateGeomCylinderSetParams(g, radius, length, _geomCylinderSetParamsFunc);
}

void dGeomTriMeshSetData_MT (dxGeom *g, dTriMeshDataID data, dGeomTriMeshSetDataFunction *_geomTriMeshSetDataFunc)
{
  _geomTriMeshSetDataFunc(g, data);
  g->recomputeAABB();

  if (clusterManager->currentCWAS)
    clusterManager->currentCWAS->propagateGeomTriMeshSetData(g, data, _geomTriMeshSetDataFunc);
}

void dGeomSphereSetRadius_MT (dxGeom *g, dReal radius, dGeomSphereSetRadiusFunction *_geomSphereSetRadiusFunc) {

  _geomSphereSetRadiusFunc(g, radius);
  g->recomputeAABB();

  if (clusterManager->currentCWAS)
    clusterManager->currentCWAS->propagateGeomSphereSetRadius(g, radius, _geomSphereSetRadiusFunc);
}

void dGeomPlaneSetParams_MT (dGeomID g, dReal a, dReal b, dReal c, dReal d, dGeomPlaneSetParamsFunction *_geomPlaneSetParamsFunc)
{
    if (clusterManager->currentCWAS)
        clusterManager->currentCWAS->propagatePlaneParams(g, a, b, c, d, _geomPlaneSetParamsFunc);
    return _geomPlaneSetParamsFunc(g, a, b, c, d);
}

void dGeomOffset_MT (dxGeom *g) {
  g->recomputeAABB();

  if (clusterManager->currentCWAS)
    clusterManager->currentCWAS->propagateOffsetChange(g);
}
