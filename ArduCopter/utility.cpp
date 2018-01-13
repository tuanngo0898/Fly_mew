#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <time.h>
#include "utility.h"

struct timespec Utility::start_time;

Utility::Utility()
{
}

Utility::~Utility()
{

}

string Utility::intToString(int number)
{
    std::stringstream ss;
    ss << number;
    return ss.str();
}

void Utility::initTime()
{
    clock_gettime(CLOCK_MONOTONIC, &start_time);
}

uint32_t Utility::millis()
{
    return millis64() & 0xFFFFFFFF;
}

uint64_t Utility::millis64()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) -
        (start_time.tv_sec +
            (start_time.tv_nsec*1.0e-9)));
}