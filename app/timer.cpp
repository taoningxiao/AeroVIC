#include "timer.h"

Timer::Timer() {
    duration = 0;
}

double Timer::report() {
    return duration;
}

void Timer::start() {
    _startingPoint = _clock.now();
}

void Timer::stop() {
    auto end = std::chrono::steady_clock::now();
    auto count = std::chrono::duration_cast<std::chrono::microseconds>(
        end - _startingPoint).count();
    duration += count / 1000000.0;
}

void Timer::clear() {
    duration = 0;
}

std::map<std::string, Timer> myClock;