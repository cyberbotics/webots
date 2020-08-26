
#include "threadManagerPthread.h"
#include "ode_MT/util_MT.h"

// On macOS, we need to implement our own version of Posix barriers and spin locks

//#ifdef __APPLE__

static pthread_t tids[MAX_THREADS];

/* initialize the barrier */
int ode_barrier_init(ode_barrier_t* barrier, ode_barrierattr_t* barrierattr, int n)
{
  barrier->count = 0;
  barrier->limit = 0;
  pthread_mutex_init(&barrier->bmutex, NULL);
  pthread_cond_init(&barrier->bcond, NULL);
  int error = pthread_mutex_lock(&barrier->bmutex);
  if (error)
    return error;
  if (barrier->limit != 0) { /* don't initialize barrier twice! */
    pthread_mutex_unlock(&barrier->bmutex);
    return 1;
  }
  barrier->limit = n;
  return pthread_mutex_unlock(&barrier->bmutex);
}

int ode_barrier_reinit(ode_barrier_t* barrier, ode_barrierattr_t* barrierattr, int n)
{
  int error = pthread_mutex_lock(&barrier->bmutex);
  if (error)
    return error;
  barrier->limit = n;
  return pthread_mutex_unlock(&barrier->bmutex);
}

int ode_barrier_destroy(ode_barrier_t *barrier)
{
  pthread_mutex_destroy(&barrier->bmutex);
  pthread_cond_destroy(&barrier->bcond);
  return 0;
}

/* wait at barrier until all n threads arrive */
int ode_barrier_wait(ode_barrier_t *barrier)
{
  int berror = pthread_mutex_lock(&barrier->bmutex);
  if (berror)
    return berror;
  if (barrier->limit <= 0)
  { /* barrier not initialized?! */
    pthread_mutex_unlock(&barrier->bmutex);
    return 1;
  }
  barrier->count++;
  if (barrier->count < barrier->limit)
    berror = pthread_cond_wait(&barrier->bcond, &barrier->bmutex);
  else
  {
    barrier->count = 0;
    berror = pthread_cond_broadcast(&barrier->bcond);
  }
    if (barrier->count > 0)
    {
        // Bugfix for Mac OSX. A called process on OSX wakes up the calling thread on return
        // If its a case of process finishing, go back to sleep and wait for the barrier
        pthread_cond_wait(&barrier->bcond, &barrier->bmutex);
    }
  berror = pthread_mutex_unlock(&barrier->bmutex);

  return berror;
}

void* dxThreadManagerPthread::simLoopChild(void *arg)
{
    int tid = ((threadInfo*)arg)->tid; //gettid();
    dxThreadManagerPthread* mgr = (dxThreadManagerPthread*)((threadInfo*)arg)->mgr;

    //pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    while (1)
    {
        //pthread_cleanup_push(simLoopCleanup, &threadArgs[tid]);
        ode_barrier_wait(&mgr->simLoopBarrier1);
        ODE_PRINT("Running thread %d\n", tid);
        // if thread has been cancelled, return
        if (pthread_mutex_trylock(&mgr->threadCancelMutex[tid]) == 0)
        {
            pthread_mutex_unlock(&mgr->threadCancelMutex[tid]);
            ODE_IMPORTANT("Exiting thread %d\n", tid);
            pthread_exit(0);
        }
        // only if thread mutex is locked, thread is allowed execution for this frame
        else if (pthread_mutex_trylock(&mgr->threadSkipMutex[tid]) == 0)
        {
            pthread_mutex_unlock(&mgr->threadSkipMutex[tid]);
            ODE_IMPORTANT("Skipping thread %d\n", tid);
        } else {
            ((threadInfo*)arg)->threadFunc(tid, ((threadInfo*)arg)->threadArgs);
        }
        ode_barrier_wait(&mgr->simLoopBarrier2);
    }
    return NULL;
}

void dxThreadManagerPthread::simLoop()
{
    //if (bMTmode)
        ode_barrier_wait(&simLoopBarrier1);
    /*
    if (!bMTmode && bMainThread)
    {
        dSpaceCollide(_space, _data, _callback);
        _stepFunc(_world, _stepsize);
    }
    //*/

    //if (bMTmode)
        ode_barrier_wait(&simLoopBarrier2);
}

void dxThreadManagerPthread::init(int numThreads, voidFunc _threadFunc)
{
    threadCount = numThreads;
    threadFunc = _threadFunc;
    ode_barrier_init(&simLoopBarrier1, NULL, threadCount + 1);
    ode_barrier_init(&simLoopBarrier2, NULL, threadCount + 1);
    //ode_barrier_init(&debugBarrier, NULL, 1);
    ode_barrier_init(&colBarrier, NULL, 1);
    pthread_spin_init(&bigbox_spinlock, PTHREAD_PROCESS_PRIVATE);
    pthread_spin_init(&task_spinlock, PTHREAD_PROCESS_PRIVATE);

    // Spawn the threads needed for cluster processing
    for (int i = 0; i < threadCount; ++i)
    {
        threadArgs[i].tid = i;
        threadArgs[i].mgr = this;
        threadArgs[i].threadFunc = threadFunc;
        //threadArgs[i].clusterCount = 0;
        pthread_mutex_init(&threadCancelMutex[i], NULL);
        pthread_mutex_lock(&threadCancelMutex[i]);
        pthread_mutex_init(&threadSkipMutex[i], NULL);
        // skip the first execution of all threads
        //pthread_mutex_lock(&threadSkipMutex[i]);

        pthread_create(&tids[i], NULL, &simLoopChild, &threadArgs[i]);

    }

    // we need to wait here for all threads to initialize
    ode_barrier_wait(&simLoopBarrier1);
    ode_barrier_wait(&simLoopBarrier2);
    for (int i=0; i<threadCount; i++)
        pthread_mutex_lock(&threadSkipMutex[i]);
}

void dxThreadManagerPthread::reinit(int numThreads)
{
    if (numThreads > threadCount)
    {
        // first, adjust barrier limit
       ode_barrier_reinit(&simLoopBarrier1, NULL, numThreads + 1);
       ode_barrier_reinit(&simLoopBarrier2, NULL, numThreads + 1);

        // all threads must skip this frame
        for (int i=0; i<threadCount; i++)
            pthread_mutex_unlock(&threadSkipMutex[i]);

        // if new threadcount is higher, we need to spawn more overlords
        for (int i = threadCount; i < numThreads; i++)
        {
            threadArgs[i].tid = i;
            threadArgs[i].mgr = this;
            threadArgs[i].threadFunc = threadFunc;
            //threadArgs[i].clusterCount = 0;

            pthread_mutex_init(&threadCancelMutex[i], NULL);
            pthread_mutex_lock(&threadCancelMutex[i]);
            pthread_mutex_init(&threadSkipMutex[i], NULL);
            //pthread_mutex_lock(&threadSkipMutex[i]);
            ODE_IMPORTANT("Creating thread %d\n", i);
            pthread_create(&tids[i], NULL, &simLoopChild, &threadArgs[i]);
        }

        ode_barrier_wait(&simLoopBarrier1);
        ode_barrier_wait(&simLoopBarrier2);
        for (int i=0; i<numThreads; i++)
            pthread_mutex_lock(&threadSkipMutex[i]);

    }
    else if (numThreads < threadCount)
    {
        // for the non-destroyed thread, next frame is going to be empty
        for (int i=0; i<numThreads; i++)
            pthread_mutex_unlock(&threadSkipMutex[i]);

        // if new threadcount is lower, destroy extra threads
        for (int i = numThreads; i < threadCount; i++)
        {
            ODE_IMPORTANT("Destroying thread %d\n", i);
            pthread_mutex_unlock(&threadCancelMutex[i]);
        }

        ode_barrier_wait(&simLoopBarrier1);

        // first, adjust barrier limit
        ode_barrier_reinit(&simLoopBarrier1, NULL, numThreads + 1);
        ode_barrier_reinit(&simLoopBarrier2, NULL, numThreads + 1);

        ode_barrier_wait(&simLoopBarrier2);
        for (int i=0; i<numThreads; i++)
            pthread_mutex_lock(&threadSkipMutex[i]);
    }

    threadCount = numThreads;
}

int dxThreadManagerPthread::currentThreadId() const
{
  pthread_t t = pthread_self();
  for (int i = 0; i < threadCount; i++)
    if (pthread_equal(t, tids[i]))
      return i;
  return 0;
}

dxThreadManagerPthread::dxThreadManagerPthread()
{

}

dxThreadManagerPthread::~dxThreadManagerPthread()
{

}
