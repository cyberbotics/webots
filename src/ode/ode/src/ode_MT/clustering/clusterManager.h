#ifndef _ODE_MT_CLUSTERMANAGER_H_
#define _ODE_MT_CLUSTERMANAGER_H_

#include "cluster.h"
#include "collision_kernel.h"

class dxClusterManager
{
private:
    struct CWASNode *clusters;
    clusterChangeCallbackFunc* clusterChangeCallback;

private:
    dxClusteredWorldAndSpace *getCWAS2(dWorldID world, dSpaceID space);
    dxClusteredWorldAndSpace *addCWAS(dWorldID newWorld, dSpaceID newSpace);
    static dxClusterManager* mInstance;

public:
        dxClusteredWorldAndSpace* currentCWAS;
        static dxClusterManager* instance() { return mInstance; }
        dxClusterManager(clusterChangeCallbackFunc*);
        ~dxClusterManager();

        dxClusteredWorldAndSpace* getCWAS(dWorldID, dSpaceID);
        void removeCWAS(dWorldID world, dSpaceID space);
        void simLoop(bool);
};

#endif
