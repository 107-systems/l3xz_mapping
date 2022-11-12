#ifndef TIMER_HPP
#define TIMER_HPP

#include <stdio.h>
#include <sys/time.h>

class Timer
{
  public:
    Timer() { reset(); }
    ~Timer() {}

    void reset();
    
    double getSeconds();

  private:
    struct timeval start, stop;
};
#endif
