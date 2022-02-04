/*
 *   LinuxMotionTimer.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _LINUX_MOTION_MANAGER_H_
#define _LINUX_MOTION_MANAGER_H_

#include <pthread.h>
#include "MotionManager.h"
#include <time.h>

namespace Robot
{
  class LinuxMotionTimer
  {
    private:
      pthread_t m_Thread;// thread structure
      unsigned long m_Interval_ns;
      MotionManager *m_Manager;// reference to the motion manager class.
      bool m_TimerRunning;
      bool m_FinishTimer;

    protected:
      static void *TimerProc(void *param);// thread function

    public:
      LinuxMotionTimer(MotionManager* manager);
      ~LinuxMotionTimer();

      void Start();
      void Stop();
      bool IsRunning();
  };
}

#endif
