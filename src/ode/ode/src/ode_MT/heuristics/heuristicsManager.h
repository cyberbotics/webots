#ifndef _ODE_MT_HEURISTICMANAGER_H_
#define _ODE_MT_HEURISTICMANAGER_H_

#include "heuristic.h"

//template<typename T> class dxHeuristic;

class HeuristicAlertCallback
{
public:
    virtual void alertCallback(dxHeuristic<int>* _heuristic) = 0;
};

class dxHeuristicsManager : public HeuristicAlertCallback
{
private:
        int userThreadCount;
        int recommendedThreadCount;
        int frameCount;
        int frameCounter;
        int syncInterval;

        int mBestClusterCount;
        float mBestGridSize;

        class dxClusterManager* clusterManager;
        class dxThreadManager* threadManager;

        class dxHeuristic<int>* objectCountHeuristic;
        class dxHeuristic<int>* clusterCountHeuristic;
        class dxHeuristic<int>* clusterCheckHeuristic;

        void alertFunc();
        void alertCallback(dxHeuristic<int>* _heuristic);

        void resetVariables();

public:
        dxHeuristicsManager(int _numThreads, int _syncInterval, class dxClusterManager* _clusterManager, class dxThreadManager* _threadManager);
        ~dxHeuristicsManager();

        void reset();
        void simLoop(struct dxWorld* _world, struct dxSpace* _space);

};

#endif
