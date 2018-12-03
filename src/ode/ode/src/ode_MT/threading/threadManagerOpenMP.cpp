#include "threadManagerOpenMP.h"
#include "ode_MT/util_MT.h"
#ifdef ODE_THREADMODE_OPENMP
#include <omp.h>
#endif
//#include <pthread.h>

//pthread_attr_t gomp_thread_attr;

void dxThreadManagerOpenMP::simLoop()
{
#ifdef ODE_THREADMODE_OPENMP
    #pragma omp parallel
    threadFunc(omp_get_thread_num(), NULL);
#endif
}

void dxThreadManagerOpenMP::init(int numThreads, voidFunc tf)
{
    threadFunc = tf;
    threadCount = numThreads;
#ifdef ODE_THREADMODE_OPENMP
    omp_set_num_threads(threadCount);
#endif
}

void dxThreadManagerOpenMP::reinit(int numThreads)
{
    threadCount = numThreads;
#ifdef ODE_THREADMODE_OPENMP
    omp_set_num_threads(threadCount);
#endif
}

int dxThreadManagerOpenMP::currentThreadId() const
{
#ifdef ODE_THREADMODE_OPENMP
  #pragma message ("To implement: dxThreadManagerPthread::currentThreadId()")
  // probably:
  // return omp_get_thread_num();
#endif
  return 0;
}

dxThreadManagerOpenMP::dxThreadManagerOpenMP()
{

}

dxThreadManagerOpenMP::~dxThreadManagerOpenMP()
{

}
