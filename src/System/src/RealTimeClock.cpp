/**
 * @file RealTimeClock.cpp
 * @authors Lorenzo Moretti
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifdef __linux__

#include <chrono>
#include <cstdint>
#include <ctime>
#include <thread>

#include <BipedalLocomotion/System/RealTimeClock.h>

using namespace BipedalLocomotion::System;

std::chrono::nanoseconds RealTimeClock::now()
{
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return std::chrono::nanoseconds(static_cast<int64_t>(ts.tv_sec * 1e9 + ts.tv_nsec));
}

void RealTimeClock::sleepFor(const std::chrono::nanoseconds& sleepDuration)
{
    // Convert nanoseconds to seconds and nanoseconds
    std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(sleepDuration);
    std::chrono::nanoseconds remaining_nanoseconds = sleepDuration - seconds;

    const timespec ts{.tv_sec = seconds.count(), .tv_nsec = remaining_nanoseconds.count()};
    clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
}

void RealTimeClock::sleepUntil(const std::chrono::nanoseconds& sleepTime)
{
    // Convert nanoseconds to seconds and nanoseconds
    std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(sleepTime);
    std::chrono::nanoseconds remaining_nanoseconds = sleepTime - seconds;

    const timespec ts{.tv_sec = seconds.count(), .tv_nsec = remaining_nanoseconds.count()};
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
}

void RealTimeClock::yield()
{
    std::this_thread::yield();
}

IClock& RealTimeClockFactory::createClock()
{
    // Create the singleton. Meyers' implementation. It is automatically threadsafe
    static RealTimeClock clock;
    return clock;
}

#endif // __linux__
