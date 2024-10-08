/**
 * @file RealTimeClock.h
 * @authors Lorenzo Moretti
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifdef __linux__

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_REALTIME_CLOCK_H
#define BIPEDAL_LOCOMOTION_SYSTEM_REALTIME_CLOCK_H

#include <BipedalLocomotion/System/IClock.h>

#include <chrono>

namespace BipedalLocomotion
{
namespace System
{

/**
 * RealTimeClock implements the IClock interface using `<chrono>` from c++std library.
 */
class RealTimeClock final : public IClock
{
public:
    /**
     * Get the system current time
     * @return the current time since some unspecified point in the past, that does not change after
     * system startup.
     * @note `BipedalLocomotion::clock().now().count()` returns a double containing the seconds
     * since some unspecified point in the past that does not change after system startup.
     */
    std::chrono::nanoseconds now() final;

    /**
     * Blocks the execution of the current thread for at least the specified sleepDuration.
     * @param time duration to sleep
     * @note std::this_tread::sleep_for() function is used.
     */
    void sleepFor(const std::chrono::nanoseconds& sleepDuration) final;

    /**
     * Blocks the execution of the current thread until specified sleepTime has been reached.
     * @param time to block until
     * @note sleepTime is the duration since some unspecified point in the past that does not change
     * after system startup. Therefore, it is suggested to first call `now()` and then add the
     * desired sleep duration.
     */
    void sleepUntil(const std::chrono::nanoseconds& sleepTime) final;

    /**
     * Provides a hint to the implementation to reschedule the execution of threads, allowing other
     * threads to run.
     */
    void yield() final;
};

class RealTimeClockFactory final : public ClockFactory
{
public:
    /**
     * Create the std clock as a singleton
     * @return the reference to a System::RealTimeClock
     */
    IClock& createClock() final;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_REALTIME_CLOCK_H

#endif // __linux__
