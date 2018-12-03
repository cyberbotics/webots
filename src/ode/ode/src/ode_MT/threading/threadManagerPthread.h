#ifndef _ODE_MT_THREADMANAGERPTHREAD_H_
#define _ODE_MT_THREADMANAGERPTHREAD_H_

#include "threadManager.h"
#include <pthread.h>
#ifdef __APPLE__
#include <sys/errno.h> // for EINVAL on macOS
#endif

struct ode_barrier_t
{
	int limit;
	int count;
	pthread_mutex_t bmutex;
	pthread_cond_t bcond;
};

struct ode_barrierattr_t
{
};

#ifdef __APPLE__
typedef int pthread_spinlock_t;

static inline int pthread_spin_init(pthread_spinlock_t *lock, int pshared) {
  __asm__ __volatile__ ("" ::: "memory");
  *lock = 0;
  return 0;
}

static inline int pthread_spin_destroy(pthread_spinlock_t *lock) {
  return 0;
}

static inline int pthread_spin_lock(pthread_spinlock_t *lock) {
  while (1) {
    int i;
    for (i=0; i < 10000; i++) {
      if (__sync_bool_compare_and_swap(lock, 0, 1)) {
        return 0;
      }
    }
    sched_yield();
  }
}

static inline int pthread_spin_trylock(pthread_spinlock_t *lock) {
  if (__sync_bool_compare_and_swap(lock, 0, 1)) {
    return 0;
  }
  return EBUSY;
}

static inline int pthread_spin_unlock(pthread_spinlock_t *lock) {
  __asm__ __volatile__ ("" ::: "memory");
  *lock = 0;
  return 0;
}

#endif

class dxThreadManagerPthread : public dxThreadManager
{
private:
    pthread_mutex_t threadCancelMutex[MAX_THREADS];
    pthread_mutex_t threadSkipMutex[MAX_THREADS];

    // Barriers
    ode_barrier_t simLoopBarrier1;
    ode_barrier_t simLoopBarrier2;
    //static ode_barrier_t debugBarrier;
    ode_barrier_t colBarrier;

    // Mutexes
    pthread_spinlock_t bigbox_spinlock;
    pthread_spinlock_t task_spinlock;

    static void* simLoopChild(void * arg);

public:
    dxThreadManagerPthread();
    ~dxThreadManagerPthread();

    void init(int numThreads, voidFunc threadFunc);
    void simLoop();
    void reinit(int numThreads);
    int currentThreadId() const;
};

#endif
