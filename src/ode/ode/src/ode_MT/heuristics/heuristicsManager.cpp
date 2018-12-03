#include "heuristicsManager.h"
#include "ode_MT/clustering/clusterManager.h"
#include "ode_MT/threading/threadManager.h"
#include "ode_MT/threading/threadManagerPthread.h"
#include "ode_MT/util_MT.h"
#include "heuristic.cpp"

void dxHeuristicsManager::alertCallback(dxHeuristic<int>* _heuristic)
{
    ODE_INFO("Alert callback!\n");
    //threadManager->reinit(clusterCountHeuristic->getValue());
    if (_heuristic == clusterCountHeuristic)
    {
        ODE_PRINT("Cluster Count Heuristic\n");

        int newThreadCount = clusterCountHeuristic->getNewValue();
        if (newThreadCount > MAX_THREADS)
            newThreadCount = MAX_THREADS;
        if (dynamic_cast<dxThreadManagerPthread*>(threadManager) == NULL)
            dToggleODE_MT(newThreadCount);

    } else if (_heuristic == objectCountHeuristic)
    {
        ODE_PRINT("Object Count Heuristic\n");

    } else if (_heuristic == clusterCheckHeuristic)
    {
        static int maxCount = clusterManager->currentCWAS->getActiveClusterCount();
        int geomCount = clusterManager->currentCWAS->getGeomCount();
        int clusterCount = clusterManager->currentCWAS->getActiveClusterCount();

        ODE_PRINT("ClusterCheck Heuristic Alert! Cluster Count %d (max %d, entities: %d)\n", clusterCount, maxCount, geomCount);
        if (clusterCount > maxCount)
            maxCount = clusterCount;

        float curSize = clusterManager->currentCWAS->getGridSize();

        // if cluster count has changed, maintain the best grid size
        if (clusterCount > mBestClusterCount)
        {
            ODE_PRINT("Updating best size to %1.3f with cluster count %d and max count %d\n", curSize, clusterCount, geomCount);
            mBestGridSize = curSize;
            mBestClusterCount = clusterCount;
        }

        if (clusterCount < geomCount && curSize < 20.0f)
            clusterManager->currentCWAS->setGridSize(curSize + 1.0f);
        else
        {
            if (curSize >= 20.0f)
                clusterManager->currentCWAS->setGridSize(mBestGridSize);
            _heuristic->pause();
        }

    }

}

dxHeuristicsManager::dxHeuristicsManager(int _numThreads, int _syncInterval, dxClusterManager* _clusterManager, dxThreadManager* _threadManager)
{
    resetVariables();

    clusterManager = _clusterManager;
    threadManager = _threadManager;

    userThreadCount = _numThreads;
    syncInterval = _syncInterval;

    objectCountHeuristic = new dxHeuristic<int>(0);
    clusterCountHeuristic = new dxHeuristic<int>(0);
    clusterCheckHeuristic = new dxHeuristic<int>(0);
    //sceneSizeHeuristic = new dxHeuristic<int>(0);

    //clusterCountHeuristic->addAlert(dxHeuristic<int>::ALERTTYPE_VALUECHANGE, this, 0);
    //objectCountHeuristic->addAlert(dxHeuristic<int>::ALERTTYPE_VALUECHANGE, this, 0);
    clusterCheckHeuristic->addAlert(dxHeuristic<int>::ALERTTYPE_FIXEDINTERVAL, this, 1);
}

void dxHeuristicsManager::resetVariables()
{
  recommendedThreadCount = 0;
  frameCount = 0;
  frameCounter = 0;
  mBestClusterCount = 0;
  mBestGridSize = 5.0f;
}

void dxHeuristicsManager::reset()
{
    objectCountHeuristic->reset();
    clusterCountHeuristic->reset();
    clusterCheckHeuristic->reset();

    resetVariables();
}

dxHeuristicsManager::~dxHeuristicsManager()
{
    //threadCount = numThreads;
    delete objectCountHeuristic;
    delete clusterCountHeuristic;
    delete clusterCheckHeuristic;

}

void dxHeuristicsManager::simLoop(struct dxWorld* _world, struct dxSpace* _space)
{
    if (frameCount++ >= syncInterval)
    {
        objectCountHeuristic->update(frameCounter, clusterManager->getCWAS(_world, _space)->getGeomCount());
        clusterCountHeuristic->update(frameCounter, clusterManager->getCWAS(_world, _space)->getActiveClusterCount());
        clusterCheckHeuristic->update(frameCounter, 0);

        frameCount = 0;
        //ODE_INFO("Object Count Heuristic: %d\n", objectCountHeuristic->getNewValue());
        //ODE_INFO("Cluster Count Heuristic: %d\n\n", clusterCountHeuristic->getNewValue());
    }
    frameCounter++;
}
