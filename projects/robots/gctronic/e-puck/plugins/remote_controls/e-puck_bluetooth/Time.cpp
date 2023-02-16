// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "Time.hpp"

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#include <unistd.h>
#endif

Time::Time() {
  mInitTime = currentTime();
}

Time::~Time() {
}

int Time::elapsedTime() const {
  return (int)(currentTime() - mInitTime);
}

unsigned int Time::currentTime() {
#ifdef _WIN32
  SYSTEMTIME tim;
  GetSystemTime(&tim);
  return (((tim.wHour * 60 + tim.wMinute) * 60 + tim.wSecond) * 1000 + tim.wMilliseconds) + 0.5;
#else
  timeval tim;
  gettimeofday(&tim, NULL);
  return ((tim.tv_sec) * 1000 + tim.tv_usec / 1000.0) + 0.5;
#endif
}

void Time::wait(int duration) {
#ifdef _WIN32
  Sleep(duration);
#else
  // because usleep cannot handle values
  // bigger than 1 second (1'000'000 nanoseconds)
  int seconds = duration / 1000;
  int milliSeconds = duration - 1000 * seconds;
  for (int i = 0; i < seconds; i++)
    usleep(1000000);
  usleep(1000 * milliSeconds);
#endif
}
