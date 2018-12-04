/*
 * Description:  Helper function to return simulation real time
 */

#ifndef TIME_HPP
#define TIME_HPP

class Time {
public:
  Time();
  virtual ~Time();

  int elapsedTime() const;  // returns milliseconds

  static void wait(int duration);  // duration in milliseconds

private:
  static unsigned int currentTime();
  unsigned int mInitTime;
};

#endif
