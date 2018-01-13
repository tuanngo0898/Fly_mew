#ifndef UTILITY_H
#define UTILITY_H

#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <stdint.h>
#include <time.h>

using namespace std;

class Utility
{
public:
    Utility();
    ~Utility();

    static string intToString(int number);
    static void initTime();
    static uint32_t millis();
    static uint64_t millis64();
private:    
    static struct timespec start_time;    
};

#endif // UTILITY_H
