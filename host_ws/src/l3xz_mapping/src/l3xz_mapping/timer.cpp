#include <l3xz_mapping/timer.hpp>

void Timer::reset() { gettimeofday(&start, 0); }

double Timer::getSeconds()
{
    gettimeofday(&stop, 0);
    long seconds = stop.tv_sec - start.tv_sec;
    long microseconds = stop.tv_usec - start.tv_usec;

    return static_cast<double>(seconds) + static_cast<double>(microseconds) * 1e-6;
}
