#ifndef _ODE_MT_THREADMANAGER_H_
#define _ODE_MT_THREADMANAGER_H_

#define MAX_THREADS 16

typedef void* voidFunc(int, void*);

struct threadInfo
{
    int tid;
    class dxThreadManager* mgr;
    voidFunc* threadFunc;
    void* threadArgs;
};

class dxThreadManager
{
protected:
    int threadCount;

    struct threadInfo threadArgs[MAX_THREADS];
    voidFunc* threadFunc;

public:
    virtual void init(int numThreads, voidFunc threadFunc) = 0;
    virtual void simLoop() = 0;
    virtual void reinit(int numThreads) = 0;
    virtual int currentThreadId() const = 0; // returns a value between 0 and MAX_THREAD - 1
    int getThreadCount() { return threadCount; }
};

#endif
