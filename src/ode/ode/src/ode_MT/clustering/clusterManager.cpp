#include "clusterManager.h"
#include "ode_MT/util_MT.h"
#include "collision_kernel.h"
#include "joints/joints.h"
#include "ode/objects.h"
#include "util.h"

dxClusterManager *dxClusterManager::mInstance = NULL;

struct CWASNode
{
    dxClusteredWorldAndSpace* cwas;
    CWASNode* next;

    CWASNode(clusterChangeCallbackFunc* callback)
    {
        cwas = new dxClusteredWorldAndSpace(callback);
        next = NULL;
    }

    ~CWASNode()
    {
        delete cwas;
        cwas = NULL;
    }
};

dxClusteredWorldAndSpace* dxClusterManager::getCWAS2(dWorldID world, dSpaceID space)
{
    dxClusteredWorldAndSpace *retval = NULL;
    for (CWASNode* node = clusters; node; node = node->next)
    {
        retval = node->cwas;
        if (world != NULL && retval->originalWorld == world)
            break;
        if (space != NULL && retval->originalSpace == space)
            break;
    }
    return retval;
}

dxClusteredWorldAndSpace* dxClusterManager::addCWAS(dWorldID newWorld, dSpaceID newSpace)
{
    CWASNode *newWorldNode = new CWASNode(clusterChangeCallback);
    newWorldNode->next = NULL;

    if (clusters != NULL)
    {
        newWorldNode->next = clusters;
    }
    clusters = newWorldNode;

    newWorldNode->cwas->originalWorld = newWorld;
    newWorldNode->cwas->originalSpace = newSpace;

    return newWorldNode->cwas;
}

static void dWorldDestroy2 (dxWorld *w)
{
  // delete all bodies and joints
  dAASSERT (w);

  if (w->wmem) {
    w->wmem->Release();
  }
  delete w;
}

/*
static void dSpaceDestroy2 (dxSpace *space)
{
  dAASSERT (space);
  dUASSERT (dGeomIsSpace(space),"argument not a space");
  dGeomDestroy (space);
}
*/

void dxClusterManager::removeCWAS(dWorldID world, dSpaceID space)
{
    CWASNode *cworld = NULL, *pworld = NULL;
    for (cworld = clusters; cworld; cworld = cworld->next)
    {
        if (world != NULL && cworld->cwas->originalWorld == world)
            break;
        if (space != NULL && cworld->cwas->originalSpace == space)
            break;
        pworld = cworld;
    }

    if (cworld == NULL) return;

    for (int i = 0; i < cworld->cwas->clusterCount; ++i)
        if (cworld->cwas->worlds[i]) dWorldDestroy2(cworld->cwas->worlds[i]);

    if (pworld) pworld->next = cworld->next;
    if (cworld == clusters) clusters = cworld->next;

    delete cworld;
    currentCWAS = NULL;
}

dxClusterManager::dxClusterManager(clusterChangeCallbackFunc* callback)
{
    //threadCount = numThreads;
    clusters = NULL;
    currentCWAS = NULL;
    clusterChangeCallback = callback;
    mInstance = this;
}

dxClusterManager::~dxClusterManager()
{
    mInstance = NULL;
}

dxClusteredWorldAndSpace* dxClusterManager::getCWAS(dWorldID _world, dSpaceID _space)
{
    currentCWAS = getCWAS2(_world, _space);
    if (currentCWAS == NULL)
    {
        ODE_INFO("Creating new clustered world and space...\n");
        currentCWAS = addCWAS(_world, _space);
    }
    return currentCWAS;
}

void dxClusterManager::simLoop(bool bRefreshClusters)
{
    currentCWAS->update(bRefreshClusters);
}
