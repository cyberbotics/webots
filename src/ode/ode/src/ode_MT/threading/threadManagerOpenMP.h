#ifndef _ODE_MT_THREADMANAGEROPENMP_H_
#define _ODE_MT_THREADMANAGEROPENMP_H_

#include "threadManager.h"

class dxThreadManagerOpenMP : public dxThreadManager
{

public:
    dxThreadManagerOpenMP();
    ~dxThreadManagerOpenMP();

    void init(int numThreads, voidFunc threadFunc);
    void simLoop();
    void reinit(int numThreads);
    int currentThreadId() const;
};

#endif
