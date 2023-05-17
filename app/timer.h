#pragma once

#include <chrono>
#include <map>
#include <string>
#include <iostream>

class Timer {
public:
    Timer();
    double report();
    void start();
    void stop();
    void clear();
private:
    double duration;
    std::chrono::steady_clock _clock;
    std::chrono::steady_clock::time_point _startingPoint;
};

extern std::map<std::string, Timer> myClock;